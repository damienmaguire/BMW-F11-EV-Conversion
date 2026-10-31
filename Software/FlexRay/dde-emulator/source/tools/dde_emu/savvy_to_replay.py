#!/usr/bin/env python3
"""Make replay vectors (tests/host/test_dde_emu_replay.c) from a SavvyCAN FlexRay CSV of the
MITM board (e.g. F11_mitm_ignonoff.csv): Bus 0 = vehicle side, Bus 1 = the real DDE.

  savvy_to_replay.py F11_mitm_ignonoff.csv out.txt --dde-out dde.txt

SavvyCAN columns: Time Stamp (us), ID (hex), Extended, Dir, Bus, LEN, D1..Dn, flags
(FR / FR+NULL). D1 is the cycle counter, D2.. the payload (LEN - 1 bytes). Output has the
format of extract_replay.py: t_ticks(10 ns, from the first row) slot_id cycle payload_bytes
header_flags ts_hw [b3]; 0xF1 rows get b3 = payload[2] (KL15). Our IDs are removed from
the vehicle side (the replay generates its own); --dde-out gets the real DDE's 0x5C (c%16 == 1)
/ 0x68 / 0xE6 / 0xE8 / 0xFB / 0x10A / 0x142. Timestamps have 1 us resolution (the replay
uses a 1.2 us static tolerance for this file). The vehicle side of that capture is lossy (a
2.6 s outage at 4.5 s, ~25 ms gaps every 5 s, only one 0xF1): the replay uses it for the
KL15 / NM behaviour, not for slot placement.
"""
import argparse
import csv

OURS_STATIC = {0x26, 0x28, 0x29, 0x3D}
OURS_DYN = {0xE6, 0xE8, 0xFB, 0x10A, 0x142, 0x68}


def ours(sid, cyc):
    return sid in OURS_STATIC or sid in OURS_DYN or (sid == 0x5C and cyc % 16 == 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--dde-out")
    ap.add_argument("--vehicle-bus", default="0")
    ap.add_argument("--dde-bus", default="1")
    ap.add_argument("--start-s", type=float, default=0.0,
                    help="drop rows before this many s after the first row (F11_mitm_ignonoff.csv: 7.05, after "
                         "the 2.6 s logger outage at 4.48 s that would look like a new bus wake)")
    a = ap.parse_args()
    rows, dde = [], []
    t0 = None
    with open(a.csv, newline="") as f:
        r = csv.reader(f)
        next(r)
        for d in r:
            if len(d) < 7:
                continue
            t = int(d[0])
            if t0 is None:
                t0 = t
            if t - t0 < a.start_s * 1e6:
                continue
            sid = int(d[1], 16)
            bus = d[4]
            n = int(d[5])
            data = [int(x, 16) for x in d[6:6 + n]]
            if not data:
                continue
            cyc = data[0] & 63
            pl = data[1:]
            null = "NULL" in "".join(d[6 + n:])
            if bus == a.vehicle_bus:
                if ours(sid, cyc):
                    continue
                b3 = pl[2] if sid == 0xF1 and not null and len(pl) >= 3 else -1
                rows.append((t, sid, cyc, len(pl), 4 if null else 0, b3))
            elif bus == a.dde_bus and ours(sid, cyc) and sid not in OURS_STATIC and not null:
                dde.append((t, sid, cyc, len(pl)))
    rows.sort(key=lambda x: x[0])
    with open(a.out, "w") as f:
        f.write("# %s bus %s (vehicle side), SavvyCAN 1 us timestamps\n" % (a.csv.split("/")[-1], a.vehicle_bus))
        for t, i, c, n, h, b3 in rows:
            if b3 >= 0:
                f.write("%d %d %d %d %d 1 %d\n" % ((t - t0) * 100, i, c, n, h, b3))
            else:
                f.write("%d %d %d %d %d 1\n" % ((t - t0) * 100, i, c, n, h))
    if a.dde_out:
        with open(a.dde_out, "w") as f:
            f.write("# real DDE (bus %s): t_ticks(same base) slot_id cycle payload_bytes ts_hw\n" % a.dde_bus)
            for t, i, c, n in sorted(dde):
                f.write("%d %d %d %d 1\n" % ((t - t0) * 100, i, c, n))


if __name__ == "__main__":
    main()
