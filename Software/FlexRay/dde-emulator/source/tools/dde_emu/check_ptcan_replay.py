#!/usr/bin/env python3
"""v1e PT-CAN emulation: replay real PT-CAN captures through the C core
(tests/host/test_ptcan_emu.c + src/ptcan_emu_core.c) and check it against the
real DDE/EGS frames.

  check_ptcan_replay.py BIN ddecan   CAN_CSV   full check vs the real DDE+EGS (mask = all)
  check_ptcan_replay.py BIN noddecan CAN_CSV   DDE off / EGS on: DDE set must run clean,
                                               the full set must stop on a foreign 0x3FD/...
  check_ptcan_replay.py BIN noegs    CAN_CSV   EGS off / DDE on: EGS set clean, full set faults
  check_ptcan_replay.py BIN vectors  CAN_CSV OUT   write E2E vectors (real frames) for 'e2e'
"""
import csv
import subprocess
import sys

TABLE = [0x3F9, 0x3FD, 0x39A, 0x396, 0x518, 0x0A5, 0x0D9, 0x0DC, 0x0F3, 0x08F,
         0x2C4, 0x281, 0x1B9, 0x381, 0x3B3, 0x397, 0x3FB, 0x512, 0x592]
E2E = {0x3F9, 0x3FD, 0x0A5, 0x0D9, 0x0DC, 0x0F3, 0x08F}
EGS = {0x3FD, 0x39A, 0x396, 0x518}
DDE = set(TABLE) - EGS
MASK_ALL = (1 << len(TABLE)) - 1
MASK_EGS = sum(1 << i for i, x in enumerate(TABLE) if x in EGS)
MASK_DDE = MASK_ALL & ~MASK_EGS

fails = []


def check(ok, msg):
    print(("  ok   " if ok else "  FAIL ") + msg)
    if not ok:
        fails.append(msg)


def load(path):
    out = []
    for r in csv.DictReader(open(path)):
        if r.get("ide", "0") == "1":
            continue
        out.append((int(float(r["t_us"])), int(r["can_id"], 16), bytes.fromhex(r["data_hex"])))
    out.sort(key=lambda x: x[0])
    return out


def kl15_edges(frames):
    on = off = None
    kl = False
    ons, offs = [], []
    for t, i, p in frames:
        if i != 0x12F or len(p) < 3:
            continue
        n = p[2] & 15
        if n >= 0xE:
            continue
        new = n >= 0xA or (n == 9 and kl)
        if new and not kl:
            ons.append(t)
        if kl and not new:
            offs.append(t)
        kl = new
    return ons, offs


def run(binary, frames, exclude, mask, tag, extra_s=5.0):
    rx = "/tmp/pte_rx_%s.txt" % tag
    with open(rx, "w") as f:
        for t, i, p in frames:
            if i in exclude:
                continue
            f.write("%d %03x %s\n" % (t, i, p.hex()))
    out = "/tmp/pte_out_%s.txt" % tag
    r = subprocess.run([binary, "replay", rx, "%x" % mask, out, str(extra_s)], capture_output=True, text=True)
    print("  " + r.stdout.strip().replace("\n", "\n  "))
    check(r.returncode == 0, "%s: replay ran clean (no TX while silent, no forbidden ID)" % tag)
    tx, states, end = [], [], ""
    for line in open(out):
        a = line.split()
        if a[0] == "T":
            tx.append((int(a[1]), int(a[2], 16), bytes.fromhex(a[3])))
        elif a[0] == "S":
            states.append((int(a[1]), a[2]))
        elif a[0] == "E":
            end = line.strip()
    return tx, states, end


def by_id(frames):
    d = {}
    for t, i, p in frames:
        d.setdefault(i, []).append((t, p))
    return d


def ddecan(binary, path):
    fr = load(path)
    W = fr[0][0]
    ons, offs = kl15_edges(fr)
    ON, OFF = ons[0], offs[0]
    real = by_id([x for x in fr if x[1] in TABLE])
    last_rx = fr[-1][0]
    print("ddecan: W = first frame, KL15 ON = W+%.3f s, OFF = W+%.3f s, last frame OFF+%.2f s"
          % ((ON - W) / 1e6, (OFF - W) / 1e6, (last_rx - OFF) / 1e6))
    tx, states, end = run(binary, fr, set(TABLE), MASK_ALL, "ddecan")
    ours = by_id(tx)
    check(tx and tx[0][0] - W <= 500000, "first TX W+%.3f s (<= 0.5 s after wake)" % ((tx[0][0] - W) / 1e6))
    check(all(t >= W for t, _, _ in tx), "no TX before the wake")
    check(max(t for t, _, _ in tx) <= last_rx + 200000, "no TX after the bus went silent (last TX %.3f s after the last RX)"
          % ((max(t for t, _, _ in tx) - last_rx) / 1e6))
    print("  %-6s %6s %6s %9s %9s %9s %9s %7s" % ("ID", "real", "ours", "first_r", "first_o", "last_r", "last_o", "match"))
    for i in TABLE:
        r, o = real.get(i, []), ours.get(i, [])
        # content: each of our frames vs the latest real frame of the ID at or before it (+5 ms)
        m = n = 0
        k = 0
        sk = 2 if i in E2E else 0
        for t, p in o:
            while k + 1 < len(r) and r[k + 1][0] <= t + 5000:
                k += 1
            if r and r[k][0] <= t + 5000:
                n += 1
                m += p[sk:] == r[k][1][sk:]
        pct = 100.0 * m / max(1, n)
        def rel(t):
            return "W%+.3f" % ((t - W) / 1e6) if t < OFF else "O%+.2f" % ((t - OFF) / 1e6)
        print("  0x%03x %6d %6d %9s %9s %9s %9s %6.1f%%" % (i, len(r), len(o), rel(r[0][0]) if r else "-",
              rel(o[0][0]) if o else "-", rel(r[-1][0]) if r else "-", rel(o[-1][0]) if o else "-", pct))
        if i == 0x592:   # we send only the MIL-off frame: compare with the real 40 22 00 1x timing instead
            rl = [t for t, p in r if p[:3] == b"\x40\x22\x00"]
            check(all(p == bytes.fromhex("40220014ffffffff") for _, p in o), "0x592: only `40 22 00 14 ff ff ff ff` sent")
            check(abs(len(o) - len(rl)) <= 1, "0x592: %d frames, real CC-34 frames %d" % (len(o), len(rl)))
            continue
        check(len(o) > 0 and abs(o[0][0] - r[0][0]) <= 30000, "0x%03x first frame within 30 ms of the real one (%+.0f ms)"
              % (i, (o[0][0] - r[0][0]) / 1e3 if o else 0))
        lim = {0x512: 700000, 0x518: 700000}.get(i, 250000 if pte_period(i) <= 100 else 1100000)
        check(abs(o[-1][0] - r[-1][0]) <= lim, "0x%03x last frame within %.2f s of the real one (%+.3f s)"
              % (i, lim / 1e6, (o[-1][0] - r[-1][0]) / 1e6))
        need = {0x397: 90, 0x3FB: 30, 0x281: 40, 0x3F9: 80, 0x0A5: 80}.get(i, 97)
        check(pct >= need, "0x%03x content matches the real frame %.1f%% (>= %d%%)" % (i, pct, need))
        ratio = len(o) / max(1, len(r))
        lo = {0x3FB: 0.5, 0x397: 0.4, 0x3F9: 0.6, 0x381: 0.5, 0x3B3: 0.5}.get(i, 0.95)
        check(lo <= ratio <= 1.05, "0x%03x frame count %d vs real %d" % (i, len(o), len(r)))
    # 0x3F9 P-lock (bits 28-29 = 2) from W+0.61 s on
    bad = [t for t, p in ours.get(0x3F9, []) if t > W + 610000 and ((p[3] >> 4) & 3) != 2]
    check(not bad, "0x3F9 bits 28-29 = 2 (P) in every frame after W+0.61 s")
    l518 = ours[0x518][-1][0] - OFF
    l512 = ours[0x512][-1][0] - OFF
    check(29.0e6 <= l518 <= 29.91e6, "0x518 released OFF+%.2f s (real OFF+%.2f)" % (l518 / 1e6, (real[0x518][-1][0] - OFF) / 1e6))
    check(45.2e6 <= l512 <= 45.93e6, "0x512 released OFF+%.2f s (real OFF+%.2f)" % (l512 / 1e6, (real[0x512][-1][0] - OFF) / 1e6))
    ps = [t for t, s in states if s == "PREP_SLEEP"]
    lastnm = max(t for t, i, _ in fr if 0x500 <= i <= 0x57F and i not in (0x512, 0x518))
    check(bool(ps) and abs(ps[0] - (lastnm + 1950000)) <= 2000,
          "PREP_SLEEP at OFF+%.2f s = last foreign NM (OFF+%.2f) + 1.95 s; real DDE cyclics last at OFF+%.2f"
          % (((ps[0] if ps else 0) - OFF) / 1e6, (lastnm - OFF) / 1e6, (real[0x0A5][-1][0] - OFF) / 1e6))
    after = [x for x in tx if x[0] > (ps[0] if ps else 1 << 62)]
    check(not after, "no TX after PREP_SLEEP (%d)" % len(after))
    print("  states: " + " ".join("%s@%+.2f" % (s, (t - W) / 1e6) for t, s in states))
    print("  " + end)

    # step 1: 0x3F9 only
    tx1, _, _ = run(binary, fr, set(TABLE), 1, "ddecan_step1")
    check(tx1 and {i for _, i, _ in tx1} == {0x3F9}, "step 1 (mask 0x3F9) sends 0x3F9 only (%d frames)" % len(tx1))


def pte_period(i):
    return {0x0A5: 20, 0x0D9: 20, 0x0DC: 20, 0x0F3: 20, 0x08F: 30, 0x3FD: 100, 0x2C4: 100, 0x281: 100}.get(i, 1000)


def absent_run(binary, path, tag, clean_mask, absent):
    fr = load(path)
    W = fr[0][0]
    ons, offs = kl15_edges(fr)
    present = sorted({i for _, i, _ in fr if i in TABLE})
    print("%s: emulated IDs present on the bus: %s; KL15 on %s off %s" % (
        tag, " ".join("0x%03x" % i for i in present), ["W+%.2f" % ((t - W) / 1e6) for t in ons],
        ["W+%.2f" % ((t - W) / 1e6) for t in offs]))
    check(not (set(present) & absent), "%s: none of the %s IDs on the bus" % (tag, "DDE" if absent == DDE else "EGS"))
    if tag == "noegs":
        clean_mask = MASK_ALL if not present else clean_mask
    tx, states, end = run(binary, fr, set(), clean_mask, tag + "_clean")
    check("fault=0" in end, "%s: %s set runs without a fault (%s)" % (tag, {MASK_DDE: "DDE", MASK_EGS: "EGS", MASK_ALL: "full"}.get(clean_mask, hex(clean_mask)), end))
    check(tx and tx[0][0] - W <= 500000, "%s: first TX W+%.3f s" % (tag, (tx[0][0] - W) / 1e6))
    if offs and clean_mask & (1 << TABLE.index(0x512)):
        o = [t for t, i, _ in tx if i == 0x512]
        if fr[-1][0] - offs[0] > 46e6:
            check(o and 45.2e6 <= o[-1] - offs[0] <= 45.93e6, "%s: 0x512 released OFF+%.2f s" % (tag, (o[-1] - offs[0]) / 1e6))
    ps = [t for t, s in states if s == "PREP_SLEEP"]
    print("  states: " + " ".join("%s@%+.2f" % (s, (t - W) / 1e6) for t, s in states))
    print("  PREP_SLEEP %s; last frame W+%.2f s" % ("never" if not ps else "W+%.2f" % ((ps[0] - W) / 1e6), (fr[-1][0] - W) / 1e6))
    tx2, st2, end2 = run(binary, fr, set(), MASK_ALL, tag + "_all")
    if present:
        first_foreign = min(t for t, i, _ in fr if i in TABLE)
        late = [x for x in tx2 if x[0] > first_foreign]
        check("fault=1" in end2 and not late,
              "%s: full set latches FOREIGN at the first real emulated ID and stops (%s)" % (tag, end2))
    else:
        # neither DDE nor EGS on the bus (run 5 noegs: DDE off too) = the v1e target setup
        check("fault=0" in end2, "%s: no emulated ID on the bus, full set runs clean (%s)" % (tag, end2))
        ids = sorted({i for _, i, _ in tx2})
        check(len(ids) == len(TABLE), "%s: all %d IDs sent" % (tag, len(ids)))
        print("  full set states: " + " ".join("%s@%+.2f" % (s, (t - W) / 1e6) for t, s in st2))


def vectors(path, out):
    fr = load(path)
    n = 0
    with open(out, "w") as f:
        for t, i, p in fr:
            if i in E2E:
                f.write("%03x %s\n" % (i, p.hex()))
                n += 1
    print("wrote %d E2E vectors to %s" % (n, out))


def main():
    binary, mode, path = sys.argv[1], sys.argv[2], sys.argv[3]
    if mode == "vectors":
        vectors(path, sys.argv[4])
        return 0
    if mode == "ddecan":
        ddecan(binary, path)
    elif mode == "noddecan":
        absent_run(binary, path, "noddecan", MASK_DDE, DDE)
    elif mode == "noegs":
        absent_run(binary, path, "noegs", MASK_EGS, EGS)
    print("%s: %s" % (mode, "all checks passed" if not fails else "%d FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
