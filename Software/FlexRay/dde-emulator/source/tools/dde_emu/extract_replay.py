#!/usr/bin/env python3
"""Make a replay vector file for tests/host/test_dde_emu_replay.c from a ts-capture CSV
(fr_ts_logger.py / frts.py export). FR1 frames only; the emulator's own IDs
(0x26/0x28/0x29/0x3D) are removed because the replay generates its own echoes.

  extract_replay.py f11_emu1.csv out.txt [--seconds 1.6]

tests/host/emu_run1_fr1_1600ms.txt = car test 1 (f11_emu1.csv), --seconds 1.6.
"""
import argparse
import csv

OURS = {0x26, 0x28, 0x29, 0x3D}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=0.0, help="only the first N s after the first frame (0 = all)")
    a = ap.parse_args()
    rows = []
    with open(a.csv, newline="") as f:
        for d in csv.DictReader(f):
            if d["channel"] != "FR1":
                continue
            t = float(d["t_us"])
            if rows and a.seconds and t - rows[0][0] > a.seconds * 1e6:
                break
            rows.append((t, int(d["slot_id"]), int(d["cycle"]), int(d["payload_len"]), int(d["header_flags"], 16)))
    t0 = rows[0][0]
    with open(a.out, "w") as f:
        if a.seconds:
            f.write("# F11 emulator car test 1 (f11_emu1.csv), FR1, first 1.6 s after bus wake, our own IDs\n")
            f.write("# 0x26/0x28/0x29/0x3D removed (the replay generates its own echoes).\n")
            f.write("# t_ticks(10 ns, from first frame)  slot_id  cycle  payload_bytes  header_flags\n")
        for t, i, c, n, h in rows:
            if i not in OURS:
                f.write("%d %d %d %d %d\n" % (round((t - t0) * 100), i, c, n, h))


if __name__ == "__main__":
    main()
