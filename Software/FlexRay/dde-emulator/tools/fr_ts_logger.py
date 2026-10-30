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
is disabled again on exit, Ctrl-C or any error.
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
    f.write("==== fr_ts_logger %s  argv: %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S %Z"), " ".join(argv)))
    sys.stdout = _Tee(sys.__stdout__, f)
    sys.stderr = _Tee(sys.__stderr__, f)
    return f


class Logger:
    def __init__(self, out_csv, can_csv=None, fine_hz=100_000_000, quiet=False):
        self.f = open(out_csv, "w", newline="")
        self.w = csv.DictWriter(self.f, fieldnames=frts.CSV_COLUMNS)
        self.w.writeheader()
        self.cf = None
        if can_csv:
            self.cf = open(can_csv, "w", newline="")
            self.cw = csv.DictWriter(self.cf, fieldnames=frts.CAN_CSV_COLUMNS)
            self.cw.writeheader()
        self.parser = frts.RecordParser()
        self.clock = frts.DeviceClock(fine_hz)
        self.seq = frts.SeqTracker()
        self.n_fr = 0
        self.n_can = 0
        self.n_hw = 0
        self.by_ch = {}
        self.fmt_seen = set()
        self.quiet = quiet

    def feed(self, data, host_time=None):
        for rec in self.parser.feed(data):
            if rec["kind"] == "can":
                self.n_can += 1
                if self.cf:
                    self.cw.writerow(frts.can_csv_row(rec, host_time))
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

    def summary(self):
        return ("frames=%d (%s) hw_ts=%d (%.1f%%) seq_gaps=%d seq_missing=%d can=%d "
                "clock_anomalies=%d bytes_skipped=%d formats=%s" % (
                    self.n_fr, " ".join("%s:%d" % kv for kv in sorted(self.by_ch.items())),
                    self.n_hw, 100.0 * self.n_hw / max(1, self.n_fr), self.seq.gaps, self.seq.missing,
                    self.n_can, self.clock.anomalies, self.parser.bytes_skipped,
                    ",".join(str(x) for x in sorted(self.fmt_seen)) or "-"))

    def close(self):
        self.f.close()
        if self.cf:
            self.cf.close()


def replay(args):
    lg = Logger(args.output, args.can_csv, args.fine_hz)
    with open(args.replay, "rb") as f:
        while True:
            chunk = f.read(65536)
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
            ctrl_out(frts.REQ_SET_CAN_RECORDS, 1)
            print("CAN records enabled (V2 board)")
        except usb.core.USBError:
            print("note: CAN records not supported by this board (main board build) - ignored")
    ctrl_out(frts.REQ_RESET_COMMS, 0)

    lg = Logger(args.output, args.can_csv, fine_hz)
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

    def emu_line():
        try:
            return frts.fmt_emu(frts.parse_emu_stats(
                dev.ctrl_transfer(0xC0, frts.REQ_GET_EMU, 0, 0, 64, timeout=1000)))
        except usb.core.USBError:
            return "(emu stats read failed)"

    emu_on = False
    next_keepalive = 0.0
    try:
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
            if emu_on and now >= next_keepalive:
                next_keepalive = now + 2.0
                try:
                    emu_set(1)
                except usb.core.USBError:
                    print("emu: keepalive failed (USB)", file=sys.stderr)
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
                if raw:
                    raw.flush()
                lg.f.flush()
        if rd["fatal"]:
            print(rd["fatal"], file=sys.stderr)
    finally:
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
    ap.add_argument("--can-csv", default=None, help="CAN CSV output (default <output>.can.csv with --can)")
    ap.add_argument("--fine-hz", type=int, default=100_000_000,
                    help="fine counter rate for --replay (live mode reads it from the device)")
    ap.add_argument("--console-log", default=None,
                    help="copy of the console output (default: <output>.console.log; --no-console-log to disable)")
    ap.add_argument("--no-console-log", action="store_true", help="do not write the console log file")
    ap.add_argument("--read-size", type=int, default=65536,
                    help="bytes per USB bulk read, multiple of 64 (default %(default)s)")
    ap.add_argument("--read-timeout-ms", type=int, default=100, help="USB bulk read timeout, ms")
    args = ap.parse_args(argv)
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
    if args.console_log is None and not args.no_console_log:
        args.console_log = args.output + ".console.log"
    if args.no_console_log:
        args.console_log = None
    if args.console_log:
        start_console_log(args.console_log, sys.argv if argv is None else ["fr_ts_logger.py"] + list(argv))
        print("console log: %s" % args.console_log)
    return replay(args) if args.replay else live(args)


if __name__ == "__main__":
    sys.exit(main())
