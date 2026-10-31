#!/usr/bin/env python3
"""Checks fr_ts_logger.py --emulate against a fake USB device (no hardware):
TX is enabled only after the capture started (0xa3: 0 then 1), kept alive,
the emulator counters are printed, and TX is disabled on exit, also when the
run is stopped by Ctrl-C (SIGINT) and before the record format is reset.
v1b: the 0xa4 stats are read with 128 bytes (v2); a first fault in them is printed
exactly once as "EMU FAULT ..."; a v1 device (64 bytes) still works.
v1d: the stats are read with 256 bytes (v4); v1/v1b/v1c devices (64/128/192) still work;
the v4 KL15/NM part is printed; with --can a V2 device's CAN records are counted
("can=N (bus2:N ids=K)") and land as bus 2 in the .can.csv and the SavvyCAN CSV."""
import contextlib, io, os, signal, struct, sys, tempfile, threading, time, types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ts_capture"))


class USBError(Exception):
    pass


class USBTimeoutError(USBError):
    pass


V1_FMT = "<4B5I6I3h2H2BI"
V2_FMT = "<4B6Ii4B5H2BhH3I"
V3_FMT = "<6I6H7HH3H2BHH"
V4_FMT = "<3I3H3H2B3H2I4h2H2BH2I"


class FakeDev:
    def __init__(self, v2=True, v3=False, v4=False, can=False):
        self.log = []
        self.v2 = v2
        self.v3 = v3 or v4
        self.v4 = v4
        self.can = can
        self.can_on = False
        self.n_stats = 0
        self.stream = None
        if can:
            from fr_ts_synth import encode_v3
            fr = encode_v3(1, 0x07, 1000, 0, 0, 0x26, 1, 0x04, bytes(16))
            def canrec(cid, t, data):
                body = struct.pack("<4BII", 0x43, 2, 0, len(data), cid, t) + data
                return struct.pack("<H", 0x8000 | len(body)) + body
            self.stream = fr + canrec(0x12F, 1500, bytes(8)) + canrec(0x3C, 2500, b"\x01\x02") + \
                encode_v3(2, 0x07, 3000, 0, 1, 0x26, 1, 0x04, bytes(16)) + canrec(0x12F, 3500, bytes(8))

    def set_configuration(self):
        pass

    def ctrl_transfer(self, rt, req, val, idx, data_or_len, timeout=0):
        self.log.append((time.time(), rt, req, val, data_or_len))
        if rt == 0x40 and req == 0xA1:
            if not self.can:
                raise USBError("stall")       # V1 board: CAN records request not supported
            self.can_on = bool(val)
        if rt == 0xC0 and req == 0xA2:
            return bytes(struct.pack("<4B14I4x", 1, 3, 16, 1, 100_000_000, *([0] * 13)))
        if rt == 0xC0 and req == 0xA4:
            self.n_stats += 1
            if not self.v2:
                return bytes(struct.pack(V1_FMT, 1, 3, 1, 0, 10, 20, 10, 10, 0,
                                         0, 0, 50, 0, 0, 400, -5, 30, 2, 180, 31, 7, 4, 1))[:data_or_len]
            fault = 1 if self.n_stats >= 2 else 0          # a fault appears from the 2nd read on
            if self.v3:
                fault = 0
            head = struct.pack(V1_FMT, 4 if self.v4 else (3 if self.v3 else 2), 4 if fault else 3, 1, fault, 2, 2, 2, 1, 0,
                               0, 1, 6, 0, 0, 400, 15, 180, 4, 338, 34, 7, 4, 1)
            tail = struct.pack(V2_FMT, fault, 0x3D if fault else 0, 53 if fault else 0, 3 if fault else 0,
                               1000000 if fault else 0, 7, 1, 6, 0, 0, 759 if fault else 0,
                               fault, 0, 0, fault, 0, 0, 1, 0, 0, 0x3D, 3, 140, 0,
                               1000000 if fault else 0, 3000000, 0)
            if self.v3:   # v1c firmware: 192 bytes
                dyn = struct.pack(V3_FMT, 50, 11, 250, 10, 50, 10, 0, 0, 0, 0, 0, 0, *([0] * 7), 0,
                                  138, 148, 163, 7, 15, 250, 0)
                if self.v4:   # v1d firmware: 256 bytes
                    v4 = struct.pack(V4_FMT, 39, 3009, 13, 0, 0, 0, 129, 240, 252,
                                     0x1F | 64, 0x20, 1, 0, 0, 0xFFFFFFFF, 41234, 3, 9, 130, 125, 0, 812,
                                     31, 2, 0, 12345, 0)
                    return bytes(head + tail + dyn + v4)[:data_or_len]
                return bytes(head + tail + dyn)[:min(data_or_len, 192)]
            return bytes(head + tail)[:min(data_or_len, 128)]   # v1b firmware: 128 bytes max
        return None

    def read(self, ep, n, timeout=0):
        if self.stream is not None and self.can_on:
            data, self.stream = self.stream, None
            return data
        time.sleep(timeout / 1000.0)
        raise USBTimeoutError("timeout")


def run(stop_by_sigint, v2=True, v3=False, v4=False, can=False, can_arg=False):
    dev = FakeDev(v2, v3, v4, can)
    core = types.ModuleType("usb.core")
    core.USBError, core.USBTimeoutError = USBError, USBTimeoutError
    core.find = lambda **kw: dev
    usb = types.ModuleType("usb")
    usb.core, usb.util = core, types.ModuleType("usb.util")
    sys.modules.update({"usb": usb, "usb.core": core, "usb.util": usb.util})
    import fr_ts_logger
    out = os.path.join(tempfile.mkdtemp(), "x.csv")
    argv = ["-o", out, "--emulate", "--status-interval", "1", "--no-console-log"]
    if can_arg:
        argv += ["--can"]
    if not stop_by_sigint:
        argv += ["--duration", "5"]
    else:
        threading.Timer(4.0, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
    t0 = time.time()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = fr_ts_logger.main(argv)
    text = buf.getvalue()
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
    lens = [r[4] for r in dev.log if r[1] == 0xC0 and r[2] == 0xA4]
    ok &= all(n == 256 for n in lens)                                     # v4 request length
    n_ff = text.count("EMU FAULT ")
    if can_arg:
        ok &= (0xA1, 1) in reqs
        if can:
            ok &= "CAN records enabled" in text and "can=3 (bus2:3 ids=2)" in text and reqs[-1] == (0xA1, 0)
            cc = open(out + ".can.csv").read().splitlines()
            sv = open(out + ".savvy.csv").read().splitlines()
            ok &= len(cc) == 4 and cc[1].startswith("1500.00,1500,2,0x12F")
            ok &= sv[0].startswith("Time Stamp,ID,Extended,Dir,Bus,LEN") and len(sv) == 6
            ok &= sv[2] == "1500,0000012F,false,Rx,2,8,00,00,00,00,00,00,00,00" and sv[3] == "2500,0000003C,false,Rx,2,2,01,02"
            ok &= sv[1].startswith("1000,00000026,false,Rx,0,17,01,") and sv[4].startswith("3000,00000026,false,Rx,1,17,")
            print([l for l in text.splitlines() if l.startswith("done:")][0])
        else:
            ok &= "not supported by this board" in text and "can=0" in text
    if v4:
        want = ("kl15=ON(src=F1 f1b3=0x20 on=1 off=0) nm=TX ceil_left=41.2s dyn2[10A=39 142=3009 68=13] "
                "ms2[10A=129 142=240 68=252] ph142=2 abs=12345 isr_late=3..9us r_min=130/125")
        ok &= want in text and "CEILING" not in text and "build2" not in text and "TOPUP" not in text
        ok &= "dyn[E6=50 FB5=11" in text
        print([l for l in text.splitlines() if "kl15=" in l][0])
    elif v3:
        ok &= n_ff == 0 and "dyn[E6=50 FB5=11 FB7=250 FB9=10 E8L9=50 E8L5=10] ms[E6=138 E8=148 FB=163] ign=on" in text
        ok &= "skip[" not in text and "parks=" not in text
        print([l for l in text.splitlines() if "dyn[" in l][0])
    elif v2:
        ok &= n_ff == 1 and dev.n_stats >= 3                              # printed once, not per read
        line = [l for l in text.splitlines() if l.startswith("EMU FAULT ")][0]
        ok &= "slot=0x3D cycle=53" in line and "detail=759" in line and "sent=7" in line
        ok &= "first FAULT 1" in text and "late[busy=0 fire=0 short=1" in text
        print(line)
    else:
        ok &= n_ff == 0 and "state=ACTIVE" in text
    return ok


if __name__ == "__main__":
    a = run(False)
    b = run(True)
    c = run(False, v2=False)
    d = run(False, v3=True)
    e = run(False, v4=True)
    f = run(False, v4=True, can=True, can_arg=True)
    g = run(False, v4=True, can=False, can_arg=True)
    allok = a and b and c and d and e and f and g
    print("test_logger_emulate: %s" % ("all OK" if allok else
                                       "FAIL (duration %s, sigint %s, v1 device %s, v1c device %s, v1d device %s, "
                                       "v1d V2 CAN %s, v1d V1 --can %s)" % (a, b, c, d, e, f, g)))
    sys.exit(0 if allok else 1)
