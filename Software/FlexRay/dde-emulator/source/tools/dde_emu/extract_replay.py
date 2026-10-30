#!/usr/bin/env python3
"""Make a replay vector file for tests/host/test_dde_emu_replay.c from a ts-capture CSV
(fr_ts_logger.py / frts.py export). FR1 frames by default; the emulator's own IDs
(statics 0x26/0x28/0x29/0x3D and, since v1c, the dynamic 0xE6/0xE8/0xFB) are removed
because the replay generates its own echoes.

  extract_replay.py f11_emu1.csv out.txt [--seconds 1.6]
  extract_replay.py f11_run5.csv out.txt --channels FR1,FR2 --dde-out dde.txt

Columns: t_ticks(10 ns, from the first frame) slot_id cycle payload_bytes header_flags ts_hw
(ts_hw 0 = no hardware frame-start mark matched this record; the dynamic replay feeds
such a header without a matching mark). --channels FR1,FR2 (run 5, MITM capture):
the real DDE's frames other than ours are kept as bus frames (the car saw them via the
repeater); --dde-out writes the real DDE's 0xE6/0xE8/0xFB frames (same time base) for
the placement comparison.

tests/host/emu_run1_fr1_1600ms.txt = car test 1 (f11_emu1.csv), --seconds 1.6.
"""
import argparse
import csv

OURS_STATIC = {0x26, 0x28, 0x29, 0x3D}
OURS_DYN = {0xE6, 0xE8, 0xFB}
OURS = OURS_STATIC | OURS_DYN


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=0.0, help="only the first N s after the first frame (0 = all)")
    ap.add_argument("--channels", default="FR1", help="comma list (FR1,FR2 for a MITM capture)")
    ap.add_argument("--dde-out", help="write the FR2 0xE6/0xE8/0xFB frames here")
    a = ap.parse_args()
    chans = set(a.channels.split(","))
    rows = []
    dde = []
    with open(a.csv, newline="") as f:
        for d in csv.DictReader(f):
            ch = d["channel"]
            t = float(d["t_us"])
            if rows and a.seconds and t - rows[0][0] > a.seconds * 1e6:
                break
            sid = int(d["slot_id"])
            hw = int(d.get("ts_hw") or 1)
            if ch == "FR2" and sid in OURS_DYN and a.dde_out:
                dde.append((t, sid, int(d["cycle"]), int(d["payload_len"]), hw))
            if ch not in chans:
                continue
            rows.append((t, sid, int(d["cycle"]), int(d["payload_len"]), int(d["header_flags"], 16), hw))
    rows.sort(key=lambda r: r[0])
    t0 = rows[0][0]
    with open(a.out, "w") as f:
        if a.seconds:
            f.write("# F11 emulator car test 1 (f11_emu1.csv), FR1, first 1.6 s after bus wake, our own IDs\n")
            f.write("# 0x26/0x28/0x29/0x3D removed (the replay generates its own echoes).\n")
            f.write("# t_ticks(10 ns, from first frame)  slot_id  cycle  payload_bytes  header_flags\n")
        for t, i, c, n, h, hw in rows:
            if i not in OURS:
                if a.seconds:   # committed 1.6 s file keeps its 5-column format
                    f.write("%d %d %d %d %d\n" % (round((t - t0) * 100), i, c, n, h))
                else:
                    f.write("%d %d %d %d %d %d\n" % (round((t - t0) * 100), i, c, n, h, hw))
    if a.dde_out:
        with open(a.dde_out, "w") as f:
            f.write("# real DDE dynamic frames: t_ticks(same base) slot_id cycle payload_bytes ts_hw\n")
            for t, i, c, n, hw in dde:
                f.write("%d %d %d %d %d\n" % (round((t - t0) * 100), i, c, n, hw))


if __name__ == "__main__":
    main()
