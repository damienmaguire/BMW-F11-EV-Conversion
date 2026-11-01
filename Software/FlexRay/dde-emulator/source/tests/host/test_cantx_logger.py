#!/usr/bin/env python3
"""v1e: fr_ts_logger.py --can-tx/--can-tx-ids against a fake USB device (no pyusb needed).
Checks: mask parsing/sets, clear -> mask -> keepalives -> 0 on exit, stats line, and that a
firmware that stalls 0xa5 (V1 / older builds) makes the logger stop with an error."""
import csv
import os
import struct
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../../tools/ts_capture"))
import frts  # noqa: E402

usb = types.ModuleType("usb")
usb.core = types.ModuleType("usb.core")
usb.util = types.ModuleType("usb.util")


class USBError(Exception):
    pass


class USBTimeoutError(USBError):
    pass


usb.core.USBError = USBError
usb.core.USBTimeoutError = USBTimeoutError
sys.modules["usb"], sys.modules["usb.core"], sys.modules["usb.util"] = usb, usb.core, usb.util


class FakeDev:
    def __init__(self, cantx_ok=True, stats=None, engine_ok=True):
        self.log = []
        self.cantx_ok = cantx_ok
        self.stats = stats
        self.engine_ok = engine_ok
        self.eng_phase = 0

    def set_configuration(self):
        pass

    def ctrl_transfer(self, bm, req, value, index, data_or_len, timeout=None):
        self.log.append((bm, req, value, index))
        if req in (frts.REQ_SET_CANTX, frts.REQ_GET_CANTX) and not self.cantx_ok:
            raise USBError("Pipe error")
        if bm == 0xC0 and req == frts.REQ_GET_TS_INFO:
            return bytes(frts.TS_INFO_STRUCT.pack(*([1, 3, 0, 1, 100_000_000] + [0] * 13)))
        if bm == 0xC0 and req == frts.REQ_GET_CANTX:
            return self.stats
        if req in (frts.REQ_SET_ENGINE, frts.REQ_GET_ENGINE) and not self.engine_ok:
            raise USBError("Pipe error")
        if req == frts.REQ_SET_ENGINE and value >> 8 == frts.ENGINE_CMD_START:
            self.eng_phase = 2   # pretend the start went through
        if req == frts.REQ_SET_ENGINE and value >> 8 == frts.ENGINE_CMD_STOP:
            self.eng_phase = 4
        if bm == 0xC0 and req == frts.REQ_GET_ENGINE:
            run = self.eng_phase == 2
            return frts.ENGINE_STATUS_STRUCT.pack(1, 2, self.eng_phase, 0xA, 0, 1, 0, 0x82 if run else 0x80,
                                                  780 if run else 0, 2098, 1 if self.eng_phase else 0,
                                                  1 if self.eng_phase == 4 else 0, 0, 100, 20, 1, 0xC8, 0x26, 0, 1234)
        if bm == 0xC0 and req == frts.REQ_GET_EMU:
            return bytes(frts.EMU_STATS_LEN)
        return None

    def read(self, ep, size, timeout=None):
        import time
        time.sleep(timeout / 1000.0)
        raise USBTimeoutError("timeout")


def can_rec(can_id, flags, t_us, data, bus=2):
    """One CAN record as the firmware sends it (src/pico_can_record.h)."""
    body = struct.pack("<BBBBII", 0x43, bus, flags, len(data), can_id, t_us) + data
    return struct.pack("<H", 0x8000 | len(body)) + body


def run(argv, dev):
    usb.core.find = lambda **kw: dev
    import importlib
    import fr_ts_logger
    importlib.reload(fr_ts_logger)
    try:
        return fr_ts_logger.main(argv)
    except SystemExit as e:
        return e.code


def main():
    fails = 0

    def check(c, msg):
        nonlocal fails
        print(("  ok   " if c else "  FAIL ") + msg)
        fails += not c

    check(frts.cantx_mask("3F9") == 1 and frts.cantx_mask("step1") == 1, "3F9 = step1 = bit 0")
    check(frts.cantx_mask("3F9,3FD,39A") == 7 == frts.cantx_mask("step3"), "3F9,3FD,39A = step3")
    check(frts.cantx_mask("all") == (1 << 19) - 1, "all = 19 IDs")
    check(frts.cantx_mask("dde") | frts.cantx_mask("egs") == frts.cantx_mask("all")
          and not frts.cantx_mask("dde") & frts.cantx_mask("egs"), "dde + egs = all, disjoint")
    for bad in ("137", "598", "351", "618", "27A", "80", "x"):
        try:
            frts.cantx_mask(bad)
            check(False, "%s refused" % bad)
        except ValueError:
            check(True, "%s refused" % bad)

    raw = open(sys.argv[1], "rb").read() if len(sys.argv) > 1 else bytes(frts.CANTX_STATS_STRUCT.size)
    if len(sys.argv) > 1:   # blob from test_ptcan_emu.c "stats" (known values)
        d = frts.parse_cantx_stats(raw)
        want = dict(version=1, state=2, fault=1, flags=0xA5, mask=0x7FFFF, fault_id=0x3FD, st_kl=0xA, n_frames=19,
                    tec=7, rec=9, opmod=6, rx_frames=123456, n12f=789, wakes=3, ms_wake=1000, ms_rx=2, ms_on=-1,
                    ms_off=45930, ms_fnm=1950, tx_total=99999, fifo_full=11, tx_fail=12, mode_switches=13)
        want.update(v2=True, loop_gap_max_us=4321, loop_gaps_20ms=5, late_max_us=131000, late_sends=77, resyncs=1,
                    tx_logged=36000, tx_log_lost=2, console_dropped=9, burst_max=8)
        bad = [k for k in want if d.get(k) != want[k]]
        d1 = frts.parse_cantx_stats(raw[:178])   # v1e firmware / short answer: v1 fields only
        check(d1 and not d1["v2"] and d1["tx_total"] == 99999 and "loop-gap" not in frts.fmt_cantx(d1),
              "178-byte (v1) stats still parse")
        check("late 77 (max 131.0ms)" in frts.fmt_cantx(d), "v2 status line: " + frts.fmt_cantx(d)[:60] + "...")
        check(not bad and tuple(d["ids"]) == frts.CANTX_IDS and d["tx"] == [1000 + i for i in range(19)],
              "ptcan_stats_t layout C vs frts.py, ID order = firmware table %s" % (bad or ""))
    tmp = tempfile.mkdtemp()
    dev = FakeDev(stats=raw)
    rc = run(["-o", os.path.join(tmp, "t.csv"), "--duration", "4.5", "--status-interval", "1",
              "--can-tx-ids", "step2", "--no-console-log"], dev)
    sets = [(v | i << 16) for bm, r, v, i in dev.log if r == frts.REQ_SET_CANTX]
    check(rc == 0, "logger run rc=0 (%s)" % rc)
    check(sets[:2] == [0, 3], "clear first, then mask 0x3 (%s)" % sets[:3])
    check(sets[-1] == 0, "TX disabled on exit")
    check(sets.count(3) >= 2, "keepalive repeated (%d)" % sets.count(3))
    check(any(r == frts.REQ_SET_CAN_RECORDS and v == 3 for _, r, v, _ in dev.log),
          "--can-tx implies --can (CAN records + own TX records: wValue 3)")
    check(any(r == frts.REQ_GET_CANTX and i == 0 for bm, r, v, i in dev.log), "stats read")
    check(any(r == frts.REQ_GET_CANTX for _, r, _, _ in dev.log), "stats read")

    # v1f: own TX records (flag bit5) -> dir=Tx rows, counted apart from RX
    import fr_ts_logger
    lg = fr_ts_logger.Logger(os.path.join(tmp, "tx.csv"), can_csv=os.path.join(tmp, "tx.can.csv"),
                             savvy_csv=os.path.join(tmp, "tx.savvy.csv"), quiet=True)
    lg.feed(can_rec(0x12F, 0, 1000, bytes([0, 0, 0xA, 0, 0, 0, 0, 0])) +
            can_rec(0x0A5, frts.CAN_FLAG_TX, 1500, bytes(range(8))) +
            can_rec(0x173, 0, 1700, bytes([0, 0, 0, 0, 0, 0, 0, 3])))
    lg.close() if hasattr(lg, "close") else None
    rows = list(csv.DictReader(open(os.path.join(tmp, "tx.can.csv"))))
    check([(r["can_id"], r["dir"]) for r in rows] == [("0x12F", "Rx"), ("0xA5", "Tx"), ("0x173", "Rx")],
          "CAN CSV dir column %s" % [(r["can_id"], r["dir"]) for r in rows])
    sv = open(os.path.join(tmp, "tx.savvy.csv")).read().splitlines()
    check(len(sv) == 4 and ",Tx,2," in sv[2] and ",Rx,2," in sv[1], "SavvyCAN Dir Tx")
    check(lg.n_can == 2 and lg.n_can_tx == 1 and "own-tx=1" in lg.can_summary(), lg.can_summary())

    dev = FakeDev(cantx_ok=False)
    rc = run(["-o", os.path.join(tmp, "t2.csv"), "--duration", "2", "--can-tx", "--no-console-log"], dev)
    check(rc == 1, "firmware without 0xa5 -> error exit (%s)" % rc)

    # v1g: --fake-start fallback: mode keepalive every 1 s, typed s / x, engine off on exit
    check(frts.engine_wvalue("cas") == 1 and frts.engine_wvalue("fallback", 1) == 0x102
          and frts.engine_wvalue(0, 2) == 0x200, "SET_ENGINE wValue = mode | cmd << 8")
    d = frts.parse_engine_status(frts.ENGINE_STATUS_STRUCT.pack(1, 1, 2, 0xC, 0, 1, 0, 0x82, 779, 2098, 1, 0, 0,
                                                                5000, 30, 1, 0xC8, 0x26, 0, 99))
    check(d and d["rpm"] == 779 and "phase run rpm 779 E6 82/26" in frts.fmt_engine(d), frts.fmt_engine(d))

    class SlowStdin:
        def __iter__(self):
            import time
            time.sleep(1.2)
            yield "s\n"
            time.sleep(1.5)
            yield "?\n"
            yield "x\n"
    old_stdin = sys.stdin
    sys.stdin = SlowStdin()
    dev = FakeDev()
    rc = run(["-o", os.path.join(tmp, "t3.csv"), "--duration", "4.5", "--status-interval", "1", "--emulate",
              "--fake-start", "fallback", "--no-console-log"], dev)
    sys.stdin = old_stdin
    eng = [v for bm, r, v, i in dev.log if r == frts.REQ_SET_ENGINE]
    check(rc == 0, "fake-start run rc=0 (%s)" % rc)
    check(eng[0] == 2 and eng.count(2) >= 3, "fallback mode keepalive every second (%s)" % eng)
    check(0x102 in eng and 0x202 in eng and eng.index(0x102) < eng.index(0x202), "typed s -> start, x -> stop")
    check(eng[-1] == 0x200, "engine off (mode 0 + stop) on exit")
    emu = [v for bm, r, v, i in dev.log if r == frts.REQ_SET_EMU]
    last_eng = max(k for k, e in enumerate(dev.log) if e[1] == frts.REQ_SET_ENGINE)
    last_emu = max(k for k, e in enumerate(dev.log) if e[1] == frts.REQ_SET_EMU)
    check(emu[-1] == 0 and last_eng < last_emu, "fake engine off before emulator TX off")
    rc = run(["-o", os.path.join(tmp, "t4.csv"), "--duration", "1", "--fake-start", "cas", "--no-console-log"], FakeDev())
    check(rc == 2, "--fake-start without --emulate refused (%s)" % rc)
    sys.stdin = iter(())
    dev = FakeDev(engine_ok=False)
    rc = run(["-o", os.path.join(tmp, "t5.csv"), "--duration", "2", "--emulate", "--fake-start", "cas",
              "--no-console-log"], dev)
    sys.stdin = old_stdin
    check(rc == 1 and [v for bm, r, v, i in dev.log if r == frts.REQ_SET_EMU][-1] == 0,
          "firmware without 0xa7 -> error exit, emulator TX off (%s)" % rc)
    print("cantx logger: " + ("ok" if not fails else "%d FAILED" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
