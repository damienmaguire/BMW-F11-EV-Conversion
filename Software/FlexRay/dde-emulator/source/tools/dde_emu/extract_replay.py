#!/usr/bin/env python3
"""Make a replay vector file for tests/host/test_dde_emu_replay.c from a ts-capture CSV
(fr_ts_logger.py / frts.py export). FR1 frames by default; the emulator's own IDs
(statics 0x26/0x28/0x29/0x3D, since v1c the dynamic 0xE6/0xE8/0xFB, since v1d 0x10A/0x142/0x68
and 0x5C in cycles c%16 == 1) are removed because the replay generates its own echoes.

  extract_replay.py f11_emu1.csv out.txt [--seconds 1.6]
  extract_replay.py f11_run5.csv out.txt --channels FR1,FR2 --dde-out dde.txt

Columns: t_ticks(10 ns, from the first frame) slot_id cycle payload_bytes header_flags ts_hw
(ts_hw 0 = no hardware frame-start mark matched this record; the dynamic replay feeds
such a header without a matching mark). --channels FR1,FR2 (run 5, MITM capture):
the real DDE's frames other than ours are kept as bus frames (the car saw them via the
repeater); --dde-out writes the real DDE's 0xE6/0xE8/0xFB frames (same time base) for
the placement comparison.
v1d: a 7th column b3 is written for non-null 0xF1 rows: payload byte 2 (KL15: 0x20 on, 0xE0 off),
which the replay hands to dde_emu_on_frame() like core 0 does. Rows without it keep 6 columns.

tests/host/emu_run1_fr1_1600ms.txt = car test 1 (f11_emu1.csv), --seconds 1.6.
"""
import argparse
import csv

OURS_STATIC = {0x26, 0x28, 0x29, 0x3D}
OURS_DYN = {0xE6, 0xE8, 0xFB, 0x10A, 0x142, 0x68}
OURS = OURS_STATIC | OURS_DYN


def ours(sid, cyc):
    return sid in OURS or (sid == 0x5C and cyc % 16 == 1)


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
            cyc = int(d["cycle"])
            if ch == "FR2" and ours(sid, cyc) and sid not in OURS_STATIC and a.dde_out:
                dde.append((t, sid, cyc, int(d["payload_len"]), hw))
            if ch not in chans:
                continue
            b3 = -1
            ph = d.get("payload_hex") or ""
            if sid == 0xF1 and d.get("null_frame", "0") == "0" and len(ph) >= 6:
                b3 = int(ph[4:6], 16)
            rows.append((t, sid, cyc, int(d["payload_len"]), int(d["header_flags"], 16), hw, b3))
    rows.sort(key=lambda r: r[0])
    t0 = rows[0][0]
    with open(a.out, "w") as f:
        if a.seconds:
            f.write("# F11 emulator car test 1 (f11_emu1.csv), FR1, first 1.6 s after bus wake, our own IDs\n")
            f.write("# 0x26/0x28/0x29/0x3D removed (the replay generates its own echoes).\n")
            f.write("# t_ticks(10 ns, from first frame)  slot_id  cycle  payload_bytes  header_flags\n")
        for t, i, c, n, h, hw, b3 in rows:
            if not ours(i, c):
                if a.seconds:   # committed 1.6 s file keeps its 5-column format
                    f.write("%d %d %d %d %d\n" % (round((t - t0) * 100), i, c, n, h))
                elif b3 >= 0:
                    f.write("%d %d %d %d %d %d %d\n" % (round((t - t0) * 100), i, c, n, h, hw, b3))
                else:
                    f.write("%d %d %d %d %d %d\n" % (round((t - t0) * 100), i, c, n, h, hw))
    if a.dde_out:
        with open(a.dde_out, "w") as f:
            f.write("# real DDE dynamic frames (0x5C c%16==1, 0x68, 0xE6, 0xE8, 0xFB, 0x10A, 0x142): t_ticks(same base) slot_id cycle payload_bytes ts_hw\n")
            for t, i, c, n, hw in dde:
                f.write("%d %d %d %d %d\n" % (round((t - t0) * 100), i, c, n, hw))


if __name__ == "__main__":
    main()
