#!/usr/bin/env python3
"""pico-flexray TS-CAPTURE logger.

Enables hardware frame timestamps (record format 3) on the pico-flexray
firmware, reads the bulk-IN stream and writes one CSV row per FlexRay frame.
Also keeps the raw USB bytes (.bin) so the CSV can be rebuilt with --replay.

  live:    python3 fr_ts_logger.py -o capture.csv
  replay:  python3 fr_ts_logger.py --replay capture.csv.bin -o capture_again.csv

The console output (banner, status/device-counter lines, stop summary) is also
written to <output>.console.log (flushed per line). USB reads run in a
separate thread with 64 KB reads so the device always has a transfer to fill.

CSV columns: t_us (device time, us, 10 ns resolution), channel, slot_id,
cycle, payload_len, header_flags, payload_hex, then null_frame, sync, startup,
t_us_raw, t_fine, seq, rec_flags, ts_hw, rec_format, host_time_s.
On exit the device is put back to record format 1 (stock), so SavvyCAN and
flexray_stream_recorder.py work as before.

--emulate (DDE emulator firmware only): once the capture is running, the
emulator's FR1 transmitter is enabled (vendor request 0xa3: 0 = clear, then 1).
The enable is repeated every couple of seconds as a keepalive (the firmware
switches TX off by itself if it hears nothing for 20 s), the emulator counters
(vendor request 0xa4) are printed with every status line as "emu: ...", and TX
is disabled again on exit, Ctrl-C or any error. With the v1b firmware the 0xa4
stats also carry the first fault (code, slot, cycle, time, counters at the fault)
and per-reason late counters; the first fault is printed once as "EMU FAULT ...".
v1d (stats v4, 256 bytes; older firmware answers 64/128/192 and still works): the
emu line also shows KL15 (source 0xF1/0x103, edges, time since off), the 0x5C NM
state and the time left before the 60 s hard ceiling, and the 0x10A/0x142/0x68 counts.

--can (V2 board): PT-CAN frames (MCP251863, listen-only) are logged to
<output>.can.csv and, together with the FlexRay frames, to a SavvyCAN CSV
<output>.savvy.csv (FR1 = bus 0, FR2 = bus 1, PT-CAN = bus 2). Status lines show
"can=N (bus2:N ids=K)". On a V1 board the request is refused and ignored.

--can-tx / --can-tx-ids LIST (v1e DDE emulator firmware, V2 board; implies --can):
enable the PT-CAN DDE/EGS frame emulation (vendor request 0xa5: 0 = clear, then the
ID mask; repeated every 2 s as keepalive, the firmware stops by itself 20 s after the
last one). --can-tx = every emulated ID; --can-tx-ids takes hex IDs and/or set names,
comma separated: step1 (3F9), step2 (+3FD), step3 (+39A), step4 (+396,518), step5/all,
dde, egs. The firmware only transmits while PT-CAN traffic is present (silence >200 ms
stops it). The counters (vendor request 0xa6) are printed with every status line as
"cantx: ...". TX is disabled on exit, Ctrl-C or any error. Default: no CAN TX at all.

--fake-start cas|fallback (v1g DDE emulator firmware, with --emulate): the emulator
also shows a running engine when you start the car (vendor request 0xa7, repeated
every second; the firmware turns it off by itself 3 s after the last one, which
"stops" the engine). Off by default.
  cas:      the car's own start: ignition on, foot on the brake, press START. The
            emulator answers CAS's starter request (ST_KL = D) with crank -> run -> idle.
  fallback: CAS does not ask: ignition on (ST_KL = A), hold the brake, then type
            "s" + Enter in this window (the brake must be pressed when it arrives).
            "x" + Enter stops the fake engine in both modes, "?" prints the
            engine status. Every engine phase change is printed as "ENGINE: ...".
Ctrl-C (or the end of --duration) turns the fake engine off before TX is disabled.

Gears (v1h.1/v1h.2 firmware, with --emulate): once the fake engine runs, the emulator reads
the gear lever (PT-CAN 0x197, V2 board only) and the brake (0x173) and reports P / R /
N / D like the real gearbox. Every gear change or refusal is printed as "GEAR: ..."
(vendor request 0xab, read 5 times a second). --no-gear switches shifting off (fixed P
as in v1h; sent with the 0xa7 keepalive, wIndex bit 8).

Versions (v1g.2): the logger prints its own version first ("fr_ts_logger.py version
v1g.2"), then asks the firmware for its version and prints a big WARNING if the two are
not from the same release (older firmware answers "dev"). --version prints it and exits.
"""
import argparse
import csv
import queue
import signal
import sys
import threading
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import frts  # noqa: E402


class _Tee:
    """Write to the terminal and to the console log; flush both on every write
    so Ctrl-C, a crash or a pulled cable still leaves a complete log."""

    def __init__(self, term, log):
        self.term = term
        self.log = log

    def write(self, text):
        self.term.write(text)
        self.term.flush()
        try:
            self.log.write(text)
            self.log.flush()
        except ValueError:  # log already closed at interpreter exit
            pass
        return len(text)

    def flush(self):
        self.term.flush()
        try:
            self.log.flush()
        except ValueError:
            pass

    def isatty(self):
        return self.term.isatty()


def start_console_log(path, argv):
    """Mirror stdout/stderr into <path> (line-flushed). Returns the file."""
    f = open(path, "a", buffering=1)
    f.write("==== fr_ts_logger %s %s  argv: %s\n" % (frts.LOGGER_VERSION, time.strftime("%Y-%m-%d %H:%M:%S %Z"),
                                                     " ".join(argv)))
    sys.stdout = _Tee(sys.__stdout__, f)
    sys.stderr = _Tee(sys.__stderr__, f)
    return f


class Logger:
    def __init__(self, out_csv, can_csv=None, fine_hz=100_000_000, quiet=False, savvy_csv=None):
        self.f = open(out_csv, "w", newline="")
        self.w = csv.DictWriter(self.f, fieldnames=frts.CSV_COLUMNS)
        self.w.writeheader()
        self.cf = None
        if can_csv:
            self.cf = open(can_csv, "w", newline="")
            self.cw = csv.DictWriter(self.cf, fieldnames=frts.CAN_CSV_COLUMNS)
            self.cw.writeheader()
        self.sf = None
        if savvy_csv:
            self.sf = open(savvy_csv, "w", newline="")
            self.sf.write(frts.SAVVY_HEADER + "\n")
        self.parser = frts.RecordParser()
        self.clock = frts.DeviceClock(fine_hz)
        self.seq = frts.SeqTracker()
        self.n_fr = 0
        self.n_can = 0
        self.n_can_tx = 0
        self.can_tx_ids = {}
        self.can_by_bus = {}
        self.can_ids = set()
        self.n_hw = 0
        self.by_ch = {}
        self.fmt_seen = set()
        self.quiet = quiet

    def feed(self, data, host_time=None):
        for rec in self.parser.feed(data):
            if rec["kind"] == "can":
                if rec["can_flags"] & frts.CAN_FLAG_TX:
                    # v1f: our own PT-CAN TX (dir=Tx); counted apart from what we heard
                    self.n_can_tx += 1
                    self.can_tx_ids[rec["can_id"]] = self.can_tx_ids.get(rec["can_id"], 0) + 1
                else:
                    self.n_can += 1
                    self.can_by_bus[rec["bus"]] = self.can_by_bus.get(rec["bus"], 0) + 1
                    self.can_ids.add(rec["can_id"])
                t = self.can_time(rec["t_us_raw"])
                if self.cf:
                    self.cw.writerow(frts.can_csv_row(rec, host_time, t))
                if self.sf:
                    self.sf.write(frts.savvy_can_row(rec, t) + "\n")
                continue
            self.n_fr += 1
            self.fmt_seen.add(rec["fmt"])
            hw = bool(rec["flags"] & frts.FLAG_TS_HW)
            self.n_hw += hw
            if rec["t_us_raw"] is None:
                t = None
            else:
                t = self.clock.time(rec["t_us_raw"], rec["t_fine"], hw)
            self.seq.add(rec["seq"])
            ch = frts.channel_name(rec["source"])
            self.by_ch[ch] = self.by_ch.get(ch, 0) + 1
            self.w.writerow(frts.fr_csv_row(rec, t, host_time))
            if self.sf:
                self.sf.write(frts.savvy_fr_row(rec, t) + "\n")

    def can_time(self, t_us_raw):
        """CAN records carry time_us_32 (same clock as FlexRay): put them on the FlexRay
        device time axis (unwrapped around the latest FlexRay timestamp)."""
        c = self.clock
        if c.last_raw is None:
            return float(c.unwrap(t_us_raw))
        delta = ((t_us_raw - c.last_raw + 0x80000000) & 0xFFFFFFFF) - 0x80000000
        return float(c.t64 + delta)

    def can_summary(self):
        tx = (" own-tx=%d (ids=%d)" % (self.n_can_tx, len(self.can_tx_ids))) if self.n_can_tx else ""
        if not self.n_can:
            return "can=0" + tx
        return "can=%d (%s ids=%d)%s" % (self.n_can, " ".join("bus%d:%d" % kv for kv in sorted(self.can_by_bus.items())),
                                         len(self.can_ids), tx)

    def summary(self):
        return ("frames=%d (%s) hw_ts=%d (%.1f%%) seq_gaps=%d seq_missing=%d %s "
                "clock_anomalies=%d bytes_skipped=%d formats=%s" % (
                    self.n_fr, " ".join("%s:%d" % kv for kv in sorted(self.by_ch.items())),
                    self.n_hw, 100.0 * self.n_hw / max(1, self.n_fr), self.seq.gaps, self.seq.missing,
                    self.can_summary(), self.clock.anomalies, self.parser.bytes_skipped,
                    ",".join(str(x) for x in sorted(self.fmt_seen)) or "-"))

    def close(self):
        self.f.close()
        if self.cf:
            self.cf.close()
        if self.sf:
            self.sf.close()


def replay(args):
    lg = Logger(args.output, args.can_csv, args.fine_hz, savvy_csv=args.savvy_csv)
    with open(args.replay, "rb") as f:
        while True:
            chunk = f.read(args.replay_chunk)
            if not chunk:
                break
            lg.feed(chunk)
    lg.close()
    print("replay done: " + lg.summary())
    print("wrote " + args.output)
    return 0


def fmt_info(i):
    return ("armed=%d fmt=%d valid=%d seq_next=%d usb_fifo_dropped=%d notify_dropped=%d "
            "ring_overflow=%d len_mismatch=%d parse_fail=%d crc_fail=%d ts_matched=%d ts_missing=%d ts_skipped=%d" % (
                i["armed"], i["record_format"], i["valid_frames"], i["seq_next"], i["usb_fifo_dropped"],
                i["notify_dropped"], i["ring_overflow"], i["len_mismatch"], i["parse_fail"], i.get("crc_fail", 0),
                i["ts_matched"], i["ts_missing"], i["ts_skipped"]))


def live(args):
    try:
        import usb.core
        import usb.util
    except ImportError:
        print("pyusb is missing: pip install pyusb   (and libusb-1.0)", file=sys.stderr)
        return 2

    dev = usb.core.find(idVendor=frts.VID, idProduct=frts.PID)
    if dev is None:
        print("pico-flexray not found (USB %04x:%04x). Is it plugged in and running the TS-CAPTURE firmware?"
              % (frts.VID, frts.PID), file=sys.stderr)
        return 1
    try:
        dev.set_configuration()
    except usb.core.USBError as e:
        print("note: set_configuration: %s (continuing)" % e)

    def ctrl_out(req, value):
        dev.ctrl_transfer(0x40, req, value, 0, None, timeout=1000)

    def get_info():
        return frts.parse_ts_info(dev.ctrl_transfer(0xC0, frts.REQ_GET_TS_INFO, 0, 0, 64, timeout=1000))

    # v1g.2: which firmware is this? Old firmware answers "dev".
    try:
        raw = dev.ctrl_transfer(0xC0, frts.REQ_GET_FW_VERSION, 0, 0, 64, timeout=1000)
        fw = bytes(raw or b"").decode("ascii", "replace").rstrip("\x00").strip()
    except (usb.core.USBError, TypeError, ValueError):
        fw = ""   # no answer: treat as unknown (warning below), never stop the capture
    print("firmware version: %s" % (fw or "(no answer)"))
    if not frts.fw_version_matches(fw):
        print("\n" + "!" * 78 + "\n"
              "WARNING: VERSION MISMATCH - firmware %r does not belong to logger %s.\n"
              "Flash the .uf2 from the same zip as this logger (or use the logger from the\n"
              "firmware's zip). Results of this run may be incomplete.\n" % (fw or "unknown", frts.LOGGER_VERSION)
              + "!" * 78 + "\n")

    try:
        ctrl_out(frts.REQ_SET_RECORD_FORMAT, 3)
    except usb.core.USBError as e:
        print("ERROR: the device refused record format 3 (%s).\n"
              "It is not running the TS-CAPTURE firmware (pico_flexray_ts_capture_*.uf2)." % e, file=sys.stderr)
        return 1
    info = get_info()
    fine_hz = info["fine_hz"] or args.fine_hz
    print("TS-CAPTURE firmware: record format %d, fine clock %d Hz (%.1f ns), device us now %d"
          % (info["record_format"], fine_hz, 1e9 / fine_hz, info["now_us"]))
    print("device: " + fmt_info(info))
    if args.can:
        try:
            # bit1: v1f firmware also streams our own PT-CAN TX frames (CSV dir=Tx); older
            # firmware only looks at bit0
            ctrl_out(frts.REQ_SET_CAN_RECORDS, frts.CAN_RECORDS_RX | frts.CAN_RECORDS_TX)
            print("CAN records enabled (V2 board; own TX frames logged as dir=Tx on v1f+ firmware)")
        except usb.core.USBError:
            print("note: CAN records not supported by this board (main board build) - ignored")
    ctrl_out(frts.REQ_RESET_COMMS, 0)

    lg = Logger(args.output, args.can_csv, fine_hz, savvy_csv=args.savvy_csv)
    raw = open(args.raw, "wb") if args.raw else None
    stop = {"flag": False}

    def on_sig(_s, _f):
        stop["flag"] = True
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    t_start = time.time()
    next_status = t_start + args.status_interval
    last_n = 0
    last_t = t_start
    usb_errors = 0
    # USB reads run in their own thread so a bulk-IN transfer is (almost) always
    # pending while the main thread parses and writes CSV. With a single
    # synchronous loop the device had no IN transfer to send into while Python
    # was busy, which cut the usable USB rate (run 4: ~217 KB/s ceiling, 4.5 %
    # of frames dropped on the device as usb_fifo_dropped -> seq gaps).
    q = queue.Queue()
    rd = {"errors": 0, "fatal": None, "bytes": 0, "qmax": 0}

    def reader():
        while not stop["flag"]:
            try:
                data = dev.read(frts.EP_IN, args.read_size, timeout=args.read_timeout_ms)
            except usb.core.USBTimeoutError:
                continue
            except usb.core.USBError as e:
                rd["errors"] += 1
                print("USB error: %s" % e, file=sys.stderr)
                if rd["errors"] > 20:
                    rd["fatal"] = "too many USB errors, stopping"
                    stop["flag"] = True
                    break
                time.sleep(0.1)
                continue
            if data:
                b = bytes(data)
                rd["bytes"] += len(b)
                q.put((b, time.time()))
                rd["qmax"] = max(rd["qmax"], q.qsize())

    th = threading.Thread(target=reader, name="usb-reader", daemon=True)
    print("logging to %s%s - Ctrl-C to stop" % (args.output, (" (raw: %s)" % args.raw) if args.raw else ""))
    th.start()

    def emu_set(v):
        ctrl_out(frts.REQ_SET_EMU, v)

    emu_fault_seen = {"key": None}

    def emu_line():
        try:
            d = frts.parse_emu_stats(dev.ctrl_transfer(0xC0, frts.REQ_GET_EMU, 0, 0, frts.EMU_STATS_LEN, timeout=1000))
        except usb.core.USBError:
            return "(emu stats read failed)"
        ff = frts.fmt_emu_first_fault(d)
        if ff:   # printed once per first-fault record
            key = (d["ff_code"], d["ff_us"])
            if key != emu_fault_seen["key"]:
                emu_fault_seen["key"] = key
                print("EMU %s (logger t=%+.1fs)" % (ff, time.time() - t_start - d["ff_age_s"]))
        return frts.fmt_emu(d)

    def emu_diag_line():
        # v1h: echo-miss diagnostics (USB 0xaa); older firmware stalls the request
        try:
            return frts.fmt_emu_diag(frts.parse_emu_diag(dev.ctrl_transfer(0xC0, frts.REQ_GET_EMUDIAG, 0, 0,
                                                                           frts.EMU_DIAG_LEN, timeout=1000)))
        except usb.core.USBError:
            return "(not supported by this firmware)"

    cantx = {"on": False, "prev": None, "fault": None}

    def cantx_set(mask):
        dev.ctrl_transfer(0x40, frts.REQ_SET_CANTX, mask & 0xFFFF, (mask >> 16) & 0xFFFF, None, timeout=1000)

    def cantx_line(delta=False):
        try:
            d = frts.parse_cantx_stats(dev.ctrl_transfer(0xC0, frts.REQ_GET_CANTX, 0, 0,
                                                         frts.CANTX_STATS_LEN, timeout=1000))
        except usb.core.USBError:
            return "(PT-CAN TX stats read failed)"
        if d and d["fault"] and d["fault"] != cantx["fault"]:
            print("CANTX FAULT: %s%s - PT-CAN TX stopped, controller back to Listen Only (logger t=%.1fs)"
                  % (frts.CANTX_FAULTS[d["fault"]] if d["fault"] < len(frts.CANTX_FAULTS) else d["fault"],
                     (" (ID 0x%03X)" % d["fault_id"]) if d["fault"] == 1 else "", time.time() - t_start))
        if d:
            cantx["fault"] = d["fault"]
        line = frts.fmt_cantx(d, cantx["prev"] if delta else None)
        if d and delta:
            cantx["prev"] = d["tx"]
        return line

    # v1g fake engine start
    eng = {"on": False, "mode": frts.ENGINE_MODES.get(args.fake_start or "off", 0), "last": None, "next": 0.0,
           "poll": 0.0}
    cmdq = queue.Queue()

    def eng_set(cmd=frts.ENGINE_CMD_NONE, mode=None):
        # v1h: wIndex = start profile (older firmware ignores wIndex)
        dev.ctrl_transfer(0x40, frts.REQ_SET_ENGINE, frts.engine_wvalue(eng["mode"] if mode is None else mode, cmd),
                          frts.ENGINE_PROFILES[args.start_profile] | (frts.GEAR_OFF_WINDEX if args.no_gear else 0),
                          None, timeout=1000)

    def eng_status():
        try:
            return frts.parse_engine_status(dev.ctrl_transfer(0xC0, frts.REQ_GET_ENGINE, 0, 0,
                                                              frts.ENGINE_STATUS_V2_LEN, timeout=1000))
        except usb.core.USBError:
            return None

    def eng_poll(force=False):
        d = eng_status()
        if not d:
            return
        key = (d["phase"], d["rejects"], d["e6_b3"])
        if force or key != eng["last"]:
            if eng["last"] is not None and d["rejects"] != eng["last"][1]:
                print("ENGINE: start refused - %s (logger t=%.1fs)"
                      % (frts.ENGINE_REJECTS[d["last_reject"]] if d["last_reject"] < len(frts.ENGINE_REJECTS)
                         else d["last_reject"], time.time() - t_start))
            print("ENGINE: %s (logger t=%.1fs)" % (frts.fmt_engine(d), time.time() - t_start))
            eng["last"] = key
        # v1h: what the CAS did with our start - loud, once per new flag
        for msg in frts.engine_cas_news(d, eng.get("cas")):
            print("ENGINE: *** %s *** (logger t=%.1fs)" % (msg, time.time() - t_start))
        eng["cas"] = (d["starts"], d.get("cas_flags", 0))

    # v1h.1 gear: print every gear event the firmware logged ("GEAR: P -> R (brake on) ...")
    gear = {"ok": True, "seen": None, "next": 0.0}

    def gear_status():
        if not gear["ok"]:
            return None
        try:
            return frts.parse_gear_status(dev.ctrl_transfer(0xC0, frts.REQ_GET_GEAR, 0, 0, frts.GEAR_STATUS_LEN,
                                                            timeout=1000))
        except usb.core.USBError:
            gear["ok"] = False   # older firmware stalls the request
            print("gear: status not supported by this firmware")
            return None

    def gear_poll():
        d = gear_status()
        if not d:
            return
        if gear["seen"] is None:   # first read: say where we are, then only new events
            gear["seen"] = d["ev_seq"]
            print("GEAR: %s (logger t=%.1fs)" % (frts.fmt_gear(d), time.time() - t_start))
            return
        news, gear["seen"] = frts.gear_news(d, gear["seen"])
        for m in news:
            print("%s (logger t=%.1fs)" % (m, time.time() - t_start))

    def stdin_reader():
        for line in sys.stdin:
            cmdq.put(line.strip().lower())

    emu_on = False
    next_keepalive = 0.0
    try:
        if args.can_tx_mask:
            time.sleep(0.5)   # capture running first
            try:
                cantx_set(0)  # clear any latched fault
                cantx_set(args.can_tx_mask)
            except usb.core.USBError as e:
                print("ERROR: the device refused PT-CAN TX (%s).\n"
                      "It is not running the v1e (or later) DDE emulator firmware for the V2 board "
                      "(pico_flexray_dde_emu_v1e_ignon_v2.uf2); V1 boards have no CAN." % e, file=sys.stderr)
                stop["flag"] = True
                raise SystemExit(1)
            cantx["on"] = True
            next_keepalive = time.time() + 2.0
            print("PT-CAN TX ENABLED for %s (sends only while PT-CAN is awake). Ctrl-C disables it."
                  % " ".join("0x%03X" % i for i in frts.cantx_ids_of(args.can_tx_mask)))
            print("cantx: " + cantx_line(True))
        if args.emulate:
            time.sleep(0.5)   # capture running first
            try:
                emu_set(0)    # clear any latched fault
                emu_set(1)
            except usb.core.USBError as e:
                print("ERROR: the device refused the emulator enable (%s).\n"
                      "It is not running the DDE emulator firmware (pico_flexray_dde_emu_*.uf2)." % e,
                      file=sys.stderr)
                stop["flag"] = True
                raise SystemExit(1)
            emu_on = True
            next_keepalive = time.time() + 2.0
            print("EMULATOR TX ENABLED (it starts sending after 8 clean bus cycles). Ctrl-C disables it.")
            print("emu: " + emu_line())
        if args.fake_start:
            try:
                eng_set()
            except usb.core.USBError as e:
                print("ERROR: the device refused the fake engine start (%s).\n"
                      "It is not running the v1g (or later) DDE emulator firmware "
                      "(pico_flexray_dde_emu_v1g_ignon*.uf2)." % e, file=sys.stderr)
                stop["flag"] = True
                raise SystemExit(1)
            eng["on"] = True
            eng["next"] = time.time() + 1.0
            print("FAKE START PROFILE: %s - %s" % (args.start_profile, frts.ENGINE_PROFILE_TEXT[args.start_profile]))
            if args.fake_start == "cas":
                print("FAKE ENGINE START ARMED (cas): ignition on, foot on the brake, press START. "
                      "Type x + Enter to stop the engine, ? + Enter for status.")
            else:
                print("FAKE ENGINE START ARMED (fallback): ignition on (not START), foot on the brake, "
                      "then type s + Enter. x + Enter stops, ? + Enter for status.")
            threading.Thread(target=stdin_reader, name="stdin", daemon=True).start()
            eng_poll(True)
        while True:
            now = time.time()
            if args.duration and now - t_start >= args.duration:
                stop["flag"] = True
            if stop["flag"] and (not th.is_alive()) and q.empty():
                break
            try:
                b, ht = q.get(timeout=0.2)
            except queue.Empty:
                b = None
            if b:
                if raw:
                    raw.write(b)
                lg.feed(b, ht)
            if stop["flag"] and not th.is_alive():
                continue
            now = time.time()
            if (emu_on or cantx["on"]) and now >= next_keepalive:
                next_keepalive = now + 2.0
                if emu_on:
                    try:
                        emu_set(1)
                    except usb.core.USBError:
                        print("emu: keepalive failed (USB)", file=sys.stderr)
                    emu_line()   # prints "EMU FAULT ..." as soon as a first fault appears
                if cantx["on"]:
                    try:
                        cantx_set(args.can_tx_mask)
                    except usb.core.USBError:
                        print("cantx: keepalive failed (USB)", file=sys.stderr)
                    cantx_line()   # prints "CANTX FAULT ..." as soon as a fault appears
            if emu_on and now >= gear["next"]:
                gear["next"] = now + 0.2
                gear_poll()
            if eng["on"]:
                while not cmdq.empty():
                    c = cmdq.get()
                    try:
                        if c in ("s", "start"):
                            eng_set(frts.ENGINE_CMD_START)
                            print("ENGINE: start command sent (logger t=%.1fs)%s" % (now - t_start,
                                  "" if eng["mode"] == 2 else " - cas mode ignores it, press START in the car"))
                        elif c in ("x", "stop"):
                            eng_set(frts.ENGINE_CMD_STOP)
                            print("ENGINE: stop command sent (logger t=%.1fs)" % (now - t_start))
                        elif c in ("?", "status", ""):
                            eng_poll(True)
                        else:
                            print("ENGINE: commands: s = start (fallback), x = stop, ? = status")
                    except usb.core.USBError:
                        print("ENGINE: command failed (USB)", file=sys.stderr)
                if now >= eng["next"]:
                    eng["next"] = now + 1.0
                    try:
                        eng_set()
                    except usb.core.USBError:
                        print("engine: keepalive failed (USB)", file=sys.stderr)
                if now >= eng["poll"]:
                    eng["poll"] = now + 0.2
                    eng_poll()
            if now >= next_status:
                rate = (lg.n_fr - last_n) / max(1e-6, now - last_t)
                last_n, last_t = lg.n_fr, now
                next_status = now + args.status_interval
                try:
                    di = fmt_info(get_info())
                except usb.core.USBError:
                    di = "(info read failed)"
                print("[%6.1fs] %.0f frames/s | %s | host queue %d (max %d)"
                      % (now - t_start, rate, lg.summary(), q.qsize(), rd["qmax"]))
                print("         device: " + di)
                if emu_on:
                    print("         emu: " + emu_line())
                    print("         emu diag: " + emu_diag_line())
                if cantx["on"]:
                    print("         cantx: " + cantx_line(True))
                if eng["on"]:
                    print("         engine: " + frts.fmt_engine(eng_status()))
                if emu_on and gear["ok"]:
                    print("         gear: " + frts.fmt_gear(gear_status()))
                if raw:
                    raw.flush()
                lg.f.flush()
        if rd["fatal"]:
            print(rd["fatal"], file=sys.stderr)
    finally:
        if eng["on"]:
            # fake engine off first (the firmware would also drop it 3 s after the last keepalive)
            for _ in range(3):
                try:
                    eng_set(frts.ENGINE_CMD_STOP, mode=0)
                    print("FAKE ENGINE OFF")
                    break
                except Exception:
                    time.sleep(0.1)
            d = eng_status()
            print("engine at stop: " + frts.fmt_engine(d))
            if gear["ok"]:
                gear_poll()
                print("gear at stop: " + frts.fmt_gear(gear_status()))
        if args.can_tx_mask and not cantx["on"]:
            try:
                cantx_set(0)   # enable was refused or never sent: best effort, no warning
            except Exception:
                pass
        if cantx["on"]:
            # PT-CAN TX off first, before anything else can fail
            for _ in range(3):
                try:
                    cantx_set(0)
                    print("PT-CAN TX DISABLED")
                    break
                except Exception:
                    time.sleep(0.1)
            else:
                print("WARNING: could not disable PT-CAN TX over USB - unplug the pico! "
                      "(the firmware also stops by itself 20 s after the last keepalive)", file=sys.stderr)
            print("cantx at stop: " + cantx_line(True))
        if args.emulate:
            # TX off first, before anything else can fail
            for _ in range(3):
                try:
                    emu_set(0)
                    print("EMULATOR TX DISABLED")
                    break
                except Exception:
                    time.sleep(0.1)
            else:
                print("WARNING: could not disable emulator TX over USB - unplug the pico! "
                      "(the firmware also stops by itself 20 s after the last keepalive)", file=sys.stderr)
            print("emu at stop: " + emu_line())
            print("emu diag at stop: " + emu_diag_line())
        try:
            info = get_info()
            print("device at stop: " + fmt_info(info))
        except Exception:
            pass
        for req, val in ((frts.REQ_SET_RECORD_FORMAT, 1),) + (((frts.REQ_SET_CAN_RECORDS, 0),) if args.can else ()):
            try:
                ctrl_out(req, val)
            except Exception:
                pass
        if raw:
            raw.close()
        lg.close()
    print("done: " + lg.summary())
    print("wrote " + args.output)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", action="version", version="fr_ts_logger.py " + frts.LOGGER_VERSION)
    ap.add_argument("-o", "--output", default="fr_ts_capture.csv", help="CSV output (default %(default)s)")
    ap.add_argument("--raw", default=None,
                    help="raw USB byte dump (default: <output>.bin; use --no-raw to disable)")
    ap.add_argument("--no-raw", action="store_true", help="do not keep the raw byte dump")
    ap.add_argument("--replay", metavar="BIN", help="rebuild the CSV from a raw dump instead of USB")
    ap.add_argument("--duration", type=float, default=0, help="stop after this many seconds (0 = Ctrl-C)")
    ap.add_argument("--status-interval", type=float, default=5.0, help="status line period, s")
    ap.add_argument("--can", action="store_true", help="also log CAN records (V2 board only)")
    ap.add_argument("--emulate", action="store_true",
                    help="DDE emulator firmware: enable FR1 TX while logging, disable on exit")
    ap.add_argument("--can-tx", action="store_true",
                    help="v1e V2 DDE emulator: enable PT-CAN DDE/EGS frame TX, all IDs (implies --can; off by default)")
    ap.add_argument("--can-tx-ids", default=None, metavar="LIST",
                    help="like --can-tx but only these: hex IDs and/or sets, comma separated "
                         "(step1=3F9, step2=+3FD, step3=+39A, step4=+396,518, step5/all, dde, egs), e.g. 3F9 or 3F9,3FD,39A")
    ap.add_argument("--fake-start", choices=("cas", "fallback"), default=None,
                    help="v1g: show a running engine on start (needs --emulate): cas = car's START button "
                         "(ST_KL D), fallback = ignition on + brake + typed 's'")
    ap.add_argument("--can-csv", default=None, help="CAN CSV output (default <output>.can.csv with --can)")
    ap.add_argument("--savvy-csv", default=None,
                    help="combined SavvyCAN CSV: FR1 = bus 0, FR2 = bus 1, PT-CAN = bus 2 "
                         "(default <output>.savvy.csv with --can in live mode; --no-savvy-csv to disable)")
    ap.add_argument("--no-savvy-csv", action="store_true", help="do not write the combined SavvyCAN CSV")
    ap.add_argument("--fine-hz", type=int, default=100_000_000,
                    help="fine counter rate for --replay (live mode reads it from the device)")
    ap.add_argument("--console-log", default=None,
                    help="copy of the console output (default: <output>.console.log; --no-console-log to disable)")
    ap.add_argument("--no-console-log", action="store_true", help="do not write the console log file")
    ap.add_argument("--read-size", type=int, default=65536,
                    help="bytes per USB bulk read, multiple of 64 (default %(default)s)")
    ap.add_argument("--read-timeout-ms", type=int, default=100, help="USB bulk read timeout, ms")
    ap.add_argument("--replay-chunk", type=int, default=65536, help=argparse.SUPPRESS)   # host tests: odd chunk sizes
    ap.add_argument("--no-gear", action="store_true",
                    help="v1h.1, with --fake-start: gear shifting off (the emulator reports fixed P as v1h)")
    ap.add_argument("--start-profile", choices=("short", "realistic", "run8", "default"), default="short",
                    help="v1h, with --fake-start: how long the fake start takes. short (default) = running "
                         "~0.17 s after START (shorter than any real start: the CAS may refuse it, the "
                         "logger then says so); realistic = ~0.84 s (just inside the fastest real start); "
                         "run8 = exactly the v1g.2 start that passed run 8 (~0.94 s); default = firmware's choice")
    args = ap.parse_args(argv)
    args.can_tx_mask = 0
    if args.can_tx_ids is not None or args.can_tx:
        if args.replay:
            ap.error("--can-tx/--can-tx-ids are for live capture only")
        try:
            args.can_tx_mask = frts.cantx_mask(args.can_tx_ids if args.can_tx_ids is not None else "all")
        except ValueError as e:
            ap.error(str(e))
        args.can = True
    if args.fake_start and (args.replay or not args.emulate):
        ap.error("--fake-start needs --emulate (live capture with the DDE emulator firmware)")
    if args.no_gear and not args.fake_start:
        print("note: --no-gear without --fake-start: the engine never runs, so the gear stays P anyway")
    if args.read_size % 64:
        ap.error("--read-size must be a multiple of 64")
    if args.raw is None and not args.no_raw and not args.replay:
        args.raw = args.output + ".bin"
    if args.no_raw:
        args.raw = None
    if args.can and not args.can_csv:
        args.can_csv = args.output + ".can.csv"
    if args.replay and not args.can_csv:
        args.can_csv = None
    if args.savvy_csv is None and args.can and not args.replay and not args.no_savvy_csv:
        args.savvy_csv = args.output + ".savvy.csv"
    if args.no_savvy_csv:
        args.savvy_csv = None
    if args.console_log is None and not args.no_console_log:
        args.console_log = args.output + ".console.log"
    if args.no_console_log:
        args.console_log = None
    if args.console_log:
        start_console_log(args.console_log, sys.argv if argv is None else ["fr_ts_logger.py"] + list(argv))
    # v1g.2: say which logger this is, first thing (the checker looks for this line)
    print("=" * 78)
    print("fr_ts_logger.py version %s  (own TX frames logged as dir=Tx with --can)" % frts.LOGGER_VERSION)
    print("=" * 78)
    if args.console_log:
        print("console log: %s" % args.console_log)
    return replay(args) if args.replay else live(args)


if __name__ == "__main__":
    sys.exit(main())
