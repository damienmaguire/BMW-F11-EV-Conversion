#!/usr/bin/env python3
"""Checks fr_ts_logger.py --emulate against a fake USB device (no hardware):
TX is enabled only after the capture started (0xa3: 0 then 1), kept alive,
the emulator counters are printed, and TX is disabled on exit, also when the
run is stopped by Ctrl-C (SIGINT) and before the record format is reset."""
import os, signal, struct, sys, tempfile, threading, time, types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ts_capture"))


class USBError(Exception):
    pass


class USBTimeoutError(USBError):
    pass


class FakeDev:
    def __init__(self):
        self.log = []

    def set_configuration(self):
        pass

    def ctrl_transfer(self, rt, req, val, idx, data_or_len, timeout=0):
        self.log.append((time.time(), rt, req, val))
        if rt == 0xC0 and req == 0xA2:
            return bytes(struct.pack("<4B14I4x", 1, 3, 16, 1, 100_000_000, *([0] * 13)))
        if rt == 0xC0 and req == 0xA4:
            return bytes(struct.pack("<4B5I6I3h2H2BI", 1, 3, 1, 0, 10, 20, 10, 10, 0,
                                     0, 0, 50, 0, 0, 400, -5, 30, 2, 180, 31, 7, 4, 1))
        return None

    def read(self, ep, n, timeout=0):
        time.sleep(timeout / 1000.0)
        raise USBTimeoutError("timeout")


def run(stop_by_sigint):
    dev = FakeDev()
    core = types.ModuleType("usb.core")
    core.USBError, core.USBTimeoutError = USBError, USBTimeoutError
    core.find = lambda **kw: dev
    usb = types.ModuleType("usb")
    usb.core, usb.util = core, types.ModuleType("usb.util")
    sys.modules.update({"usb": usb, "usb.core": core, "usb.util": usb.util})
    import fr_ts_logger
    out = os.path.join(tempfile.mkdtemp(), "x.csv")
    argv = ["-o", out, "--emulate", "--status-interval", "1", "--no-console-log"]
    if not stop_by_sigint:
        argv += ["--duration", "5"]
    else:
        threading.Timer(4.0, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
    t0 = time.time()
    rc = fr_ts_logger.main(argv)
    reqs = [(r[2], r[3]) for r in dev.log if r[1] == 0x40]
    print("rc", rc, "out-requests:", reqs)
    ok = True
    i_fmt = reqs.index((0xA0, 3))
    i_en0 = reqs.index((0xA3, 0))
    ok &= i_fmt < i_en0 and reqs[i_en0 + 1] == (0xA3, 1)                 # capture first, then 0, 1
    ok &= reqs.count((0xA3, 1)) >= 2                                      # keepalive
    last_off = max(i for i, r in enumerate(reqs) if r == (0xA3, 0))
    ok &= last_off > i_en0 + 1 and reqs.index((0xA0, 1)) > last_off       # disabled, before format reset
    ok &= all(r != (0xA3, 1) for r in reqs[last_off:])                    # nothing re-enables
    ok &= any(r[1] == 0xC0 and r[2] == 0xA4 for r in dev.log)             # stats read
    ok &= time.time() - t0 < 10
    return ok


if __name__ == "__main__":
    a = run(False)
    b = run(True)
    print("test_logger_emulate: %s" % ("all OK" if a and b else "FAIL (duration %s, sigint %s)" % (a, b)))
    sys.exit(0 if a and b else 1)
