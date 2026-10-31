#!/usr/bin/env python3
"""v1e: fr_ts_logger.py --can-tx/--can-tx-ids against a fake USB device (no pyusb needed).
Checks: mask parsing/sets, clear -> mask -> keepalives -> 0 on exit, stats line, and that a
firmware that stalls 0xa5 (V1 / older builds) makes the logger stop with an error."""
import os
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
    def __init__(self, cantx_ok=True, stats=None):
        self.log = []
        self.cantx_ok = cantx_ok
        self.stats = stats

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
        return None

    def read(self, ep, size, timeout=None):
        import time
        time.sleep(timeout / 1000.0)
        raise USBTimeoutError("timeout")


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
        bad = [k for k in want if d[k] != want[k]]
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
    check(any(r == frts.REQ_SET_CAN_RECORDS and v == 1 for _, r, v, _ in dev.log), "--can-tx implies --can")
    check(any(r == frts.REQ_GET_CANTX for _, r, _, _ in dev.log), "stats read")

    dev = FakeDev(cantx_ok=False)
    rc = run(["-o", os.path.join(tmp, "t2.csv"), "--duration", "2", "--can-tx", "--no-console-log"], dev)
    check(rc == 1, "firmware without 0xa5 -> error exit (%s)" % rc)
    print("cantx logger: " + ("ok" if not fails else "%d FAILED" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
