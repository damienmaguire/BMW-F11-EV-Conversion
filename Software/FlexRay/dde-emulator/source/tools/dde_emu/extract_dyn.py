#!/usr/bin/env python3
"""Dynamic-segment vectors for tests/host/test_dde_emu_core.c (v1c) from a ts-capture CSV.

  extract_dyn.py f11_run5.csv out.txt            all frames of both channels (tracker test)
  extract_dyn.py f11_run5.csv out.txt --dde-only only the DDE's 0xE6/0xE8/0xFB, v1d also 0x10A/0x142/0x68
                                                 (builder test; tests/host/dde_dyn_run5.txt). 0x142 (every
                                                 20 ms) only in the first --max-142-s seconds (default 40).

Line: t_ticks(10 ns, from the first frame) channel(1/2) slot_id cycle payload_bytes ts_hw payload_hex|-
Header "# ign_ticks N": first 0x103 (ign-on indicator) on FR1.
"""
import argparse
import csv

DYN = {0xE6, 0xE8, 0xFB, 0x10A, 0x142, 0x68}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--dde-only", action="store_true")
    ap.add_argument("--max-142-s", type=float, default=40.0)
    a = ap.parse_args()
    rows = []
    ign = None
    with open(a.csv, newline="") as f:   # time base: the first frame of the capture
        t0 = float(next(csv.DictReader(f))["t_us"])
    with open(a.csv, newline="") as f:
        for d in csv.DictReader(f):
            t = float(d["t_us"])
            ch = 1 if d["channel"] == "FR1" else 2
            sid = int(d["slot_id"])
            if ign is None and ch == 1 and sid == 0x103:
                ign = t
            if a.dde_only and not (ch == 2 and sid in DYN):
                continue
            if a.dde_only and sid == 0x142 and t - t0 > a.max_142_s * 1e6:
                continue
            hw = int(d.get("ts_hw") or 1)
            pl = d["payload_hex"] if (ch == 2 and sid in DYN) else "-"
            rows.append((t, ch, sid, int(d["cycle"]), int(d["payload_len"]), hw, pl))
    rows.sort(key=lambda r: r[0])
    with open(a.out, "w") as f:
        f.write("# dynamic-segment vectors from %s (extract_dyn.py%s)\n" % (a.csv.split("/")[-1], " --dde-only" if a.dde_only else ""))
        f.write("# ign_ticks %d\n" % (round((ign - t0) * 100) if ign is not None else -1))
        f.write("# t_ticks channel slot_id cycle payload_bytes ts_hw payload_hex\n")
        for t, ch, sid, c, n, hw, pl in rows:
            f.write("%d %d %d %d %d %d %s\n" % (round((t - t0) * 100), ch, sid, c, n, hw, pl))


if __name__ == "__main__":
    main()
