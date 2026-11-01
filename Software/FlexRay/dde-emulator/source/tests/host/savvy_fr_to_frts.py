#!/usr/bin/env python3
"""v1g test helper: SavvyCAN FlexRay CSV (LEN 17 = cycle + 16 bytes) -> fr_ts_logger-style
CSV with the columns check_emu_capture.py reads, for the slots given (hex), plus an empty
<out>.can.csv. Used to run the --engine checks on a real DDE start capture."""
import sys


def main(src, dst, slots="74,26,28,29,3D,E6"):
    keep = {int(x, 16) for x in slots.split(",")}
    n = 0
    with open(dst, "w") as out:
        out.write("t_us,channel,slot_id,cycle,payload_len,header_flags,payload_hex\n")
        for line in open(src):
            f = line.rstrip().split(",")
            if len(f) < 8 or f[0] == "Time Stamp" or "NULL" in line:
                continue
            sid = int(f[1], 16)
            if sid not in keep:
                continue
            ln = int(f[5])
            b = f[6:6 + ln]
            if len(b) < ln or "" in b:
                continue
            out.write("%s,FR%d,%d,%d,%d,0x03,%s\n" % (f[0], int(f[4]) + 1, sid, int(b[0], 16), ln - 1, "".join(b[1:])))
            n += 1
    open(dst + ".can.csv", "w").write("t_us,can_id,ide,dir,data_hex\n1,0x100,0,Rx,00\n")
    print("%d frames" % n)


if __name__ == "__main__":
    main(*sys.argv[1:])
