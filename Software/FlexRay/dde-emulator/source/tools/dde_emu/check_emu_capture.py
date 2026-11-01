#!/usr/bin/env python3
"""Check a car capture taken with the v1f/v1g emulator (fr_ts_logger.py --can --can-tx).

  check_emu_capture.py CAPTURE.csv            (reads CAPTURE.csv.can.csv next to it, if there)
  check_emu_capture.py CAPTURE.csv --engine   (v1g: also report the fake start/stop)
  check_emu_capture.py CAPTURE.csv --engine --gear   (v1h.1: also the gear timeline)

What it reports (and the pass criteria printed in the README):
  1. Our own PT-CAN TX (dir=Tx rows, v1f firmware): per emulated ID the frames sent vs the
     slots due between its first and last frame, the longest gap, and 0x397 (average
     spacing, e3 blips, the df/de/dd walk after KL15 off).
  2. Foreign copies: an emulated ID heard as dir=Rx (the real DDE/EGS still on the bus).
  0. (v1g.2) The capture itself: from CAPTURE.csv.console.log (the logger's own log),
     FAIL if any FlexRay frame was lost before USB (usb_fifo_dropped, notify_dropped,
     ring_overflow > 0), if the logger was older than v1f (no "own TX frames logged as
     dir=Tx" line - it never asks for our own frames), or if the logger warned about a
     firmware/logger VERSION MISMATCH. No console log = FAIL (cannot tell).
  3. KL15 (0x12F ST_KL) and the brake signals: PT-CAN 0x173 byte p[7] bits 0-1 and 0x12F
     byte p[6] bits 0-1, FlexRay 0x74 p[6] bits 0-1 on even cycles. p[n] = 0-based byte
     index, as in Python (p[7] = the 8th byte). v1g.2 fix: v1f/v1g read 0x173 p[6] and
     0x12F p[5], one byte too early, and so never saw the brake on PT-CAN.
  4. --engine: the v1g start/stop as sent (0x0A5 rpm, 0x0D9 crank nibble) and as seen on
     FlexRay (0x28 rpm, 0xE6 b3 state, ST_KL C/D). Per start: ST_KL D -> first rpm ->
     0xE6 81 -> 82 -> CAS releases the starter (D -> C), idle rpm, every E2E CRC of our
     0x26/0x28/0x29/0x3D/0xE6 frames, 0x592 (MIL) never changed.
  v1h additions:
     - 0x397: the stock-like ~380 ms spacing is only checked while the engine is NOT
       running; while running (our 0x0A5 rpm > 0) the real DDE sends 0x397 every 60 ms,
       so that part must be ~60 ms instead (run 8 was marked FAIL for this by mistake).
     - --engine, per start: crank nibble (0x0D9 CounterHi 3) and first rpm latency after
       ST_KL D (real DDE: crank +51..59 ms, rpm +94 ms; the v1h "short" profile is much
       quicker on purpose), 0x3FB p[4] f1->f0 during crank (Fred: the gateway copies it to
       FlexRay 0xE9 b5), 0x1B3 p[4] bit4 (Ian: KL50 / starter relay) during D - reported,
       not failed - and 0xE6 0x92 within 20 ms of ST_KL 8 (real: +19 ms; run8 profile: no).
     - CAS refusal: D released before our 0x82 = "CAS REFUSED / GAVE UP"; D still held
       more than 300 ms after 0x82 = "CAS did not accept the running report".
     - The start profile is read from the console log ("FAKE START PROFILE: ...").
  v1h.1 additions:
     - --gear: the gear timeline. The lever (0x197 p[2] high nibble, P button p[3] = D5),
       the brake (0x173), every change of what we send (0x3FD p[2], 0x39A p[3]/p[4],
       0x3F9 bits 28-29 + p[6] gear nibble, 0x0DC p[2]/p[5], FlexRay 0xE6 b4/b7 and 0x28
       c3 b11/b14), CAS 0x12F p[5] and FlexRay 0x74 p[5], and the firmware's own "GEAR:"
       lines from the console log (changes and refusals). Per lever push: what we did and
       how long after. PASS lines: our 0x3FD / 0x39A / 0x3F9 / 0xE6 agree on the gear
       outside the transitions, R/N/D only while our engine runs, and the P pawl
       (0x39A E2) always together with 0x3F9 bits 28-29 = 2.
  v1h.2 additions:
     - --flicker: single-frame flicker of our own gear / engine-status frames 0x39A,
       0x3FD, 0x3F9, 0x0DC and 0x397 (run 9: a stale core1 -> core0 snapshot put the stock
       "P, engine off" content into single frames). A flicker = frame k differs from frame
       k-1 and frame k+1 equals frame k-1 (whole payload; E2E counter + CRC bytes 0-1 of
       0x3F9/0x3FD/0x0DC ignored). One FAIL line per ID, any flicker fails, there is no
       settle window. The only exemption is the real DDE's own 0x397 "e3 blip" (byte p[1]
       e2 <-> e3 for one frame, the other bytes unchanged: the recorded run 4 blip table
       has 60 ms blips 60 ms apart), which the stock 0x397 model sends on purpose; those
       are counted and printed, not failed.
Exit code 0 = all PASS lines passed (other missing data is reported, not failed).
  --console PATH   use this console log instead of CAPTURE.csv.console.log
"""
import bisect
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import dde_emu_model as model   # E2E CRC tables (optional)
except Exception:   # pragma: no cover
    model = None

EMU_IDS = [0x3F9, 0x3FD, 0x39A, 0x396, 0x518, 0x0A5, 0x0D9, 0x0DC, 0x0F3, 0x08F,
           0x2C4, 0x281, 0x1B9, 0x381, 0x3B3, 0x397, 0x3FB, 0x512, 0x592]
PERIOD_MS = {0x0A5: 20, 0x0D9: 20, 0x0DC: 20, 0x0F3: 20, 0x08F: 30, 0x3FD: 100, 0x2C4: 100, 0x281: 100}
fails = []


def res(ok, msg):
    print(("  PASS " if ok else "  FAIL ") + msg)
    if not ok:
        fails.append(msg)


def console_profile(path):
    """v1h: the fake start profile named in the logger's console log (last session), or None."""
    if not path or not os.path.exists(path):
        return None
    prof = None
    for l in open(path, errors="replace"):
        if l.startswith("==== fr_ts_logger"):
            prof = None
        if l.startswith("FAKE START PROFILE:"):
            prof = l.split(":", 1)[1].split()[0]
    return prof


def check_console(path):
    """v1g.2: the logger's console log. Only the last session in the file counts (the
    logger appends). Returns nothing; adds FAIL lines."""
    print("\n0. capture health (console log %s)" % path)
    if not path or not os.path.exists(path):
        res(False, "console log not found - cannot check USB drops or the logger version")
        return
    lines = open(path, errors="replace").read().splitlines()
    starts = [k for k, l in enumerate(lines) if l.startswith("==== fr_ts_logger")]
    if len(starts) > 1:
        print("       %d logger sessions in this file: checking the last one (line %d)" % (len(starts), starts[-1] + 1))
    sess = lines[starts[-1]:] if starts else lines
    head = sess[0] if sess else ""
    ver = [l.split("version", 1)[1].split()[0] for l in sess if l.startswith("fr_ts_logger.py version")]
    fw = [l.split(":", 1)[1].strip() for l in sess if l.startswith("firmware version:")]
    print("       logger: %s | firmware: %s" % (ver[0] if ver else "older than v1g.2 (no version line)",
                                                 fw[0] if fw else "not reported (logger older than v1g.2)"))
    res(any("own TX frames logged as dir=Tx" in l for l in sess),
        "logger is v1f or newer (banner says \"own TX frames logged as dir=Tx\")" if any(
            "own TX frames logged as dir=Tx" in l for l in sess) else
        "OLD LOGGER: no \"own TX frames logged as dir=Tx\" banner (%s) - use the logger from the firmware zip"
        % head[:60])
    if any("VERSION MISMATCH" in l for l in sess):
        res(False, "the logger warned: firmware/logger VERSION MISMATCH")
    last = {}
    for l in sess:
        if "usb_fifo_dropped=" in l:
            for kv in l.split():
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    if v.isdigit():
                        last[k] = int(v)
    if not last:
        res(False, "no device counter line (usb_fifo_dropped=...) in the console log")
        return
    lost = {k: last.get(k, 0) for k in ("usb_fifo_dropped", "notify_dropped", "ring_overflow")}
    res(not any(lost.values()), "FlexRay frames lost before USB: " + " ".join("%s=%d" % kv for kv in lost.items())
        + " (of %d valid) [need 0]" % last.get("valid", 0))
    txl = [l for l in sess if "tx-log " in l]
    if txl:
        w = txl[-1].split("tx-log ", 1)[1].split()
        if len(w) >= 3 and w[1] == "lost" and w[2].isdigit() and int(w[2]):
            print("  WARN own-TX log lost %s records (TEF overflow / queue full): Tx row counts may be short" % w[2])


def load_can(path):
    rx, tx = [], []
    for r in csv.DictReader(open(path)):
        if r.get("ide", "0") == "1" or not r.get("t_us"):
            continue
        row = (float(r["t_us"]), int(r["can_id"], 16), bytes.fromhex(r["data_hex"]))
        (tx if r.get("dir", "Rx") == "Tx" else rx).append(row)
    rx.sort()
    tx.sort()
    return rx, tx


def load_fr(path, slots):
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                sid = int(r["slot_id"], 0)
            except (KeyError, ValueError):
                continue
            if sid in slots and r.get("t_us") and r.get("payload_hex"):
                out.append((float(r["t_us"]), sid, int(r["cycle"]), bytes.fromhex(r["payload_hex"])))
    out.sort()
    return out


def runs_of(ts, brk=1e6):
    runs, a = [], 0
    for k in range(1, len(ts) + 1):
        if k == len(ts) or ts[k] - ts[k - 1] > brk:
            runs.append(ts[a:k])
            a = k
    return runs


def edges(seq):
    """[(t, value)] -> only the changes."""
    out, last = [], None
    for t, v in seq:
        if v != last:
            out.append((t, v))
            last = v
    return out


def v1h_start_checks(td, t82, trel, by, rx, T, prof):
    """v1h per-start checks after ST_KL D at td (FlexRay time base = CAN time base: one clock)."""
    ms = lambda x: "%+.0f ms" % ((x - td) / 1e3) if x is not None else "never"  # noqa: E731
    d9 = [(t, p[1] >> 4) for t, p in by.get(0x0D9, []) if len(p) > 1 and t >= td - 5e3]
    t_crank = next((t for t, v in d9 if v == 3), None)
    a5 = [(t, (p[5] | p[6] << 8) / 4.0) for t, p in by.get(0x0A5, []) if len(p) > 6 and t >= td - 5e3]
    t_rpm = next((t for t, v in a5 if v > 0), None)
    print("  latency after ST_KL D: crank nibble (0x0D9 CounterHi 3) %s (real DDE +51..59), first PT-CAN rpm %s"
          " (real DDE +94)" % (ms(t_crank), ms(t_rpm)))
    end = t82 or trel or td + 3e6
    fb = [(t, p[4]) for t, p in by.get(0x3FB, []) if len(p) > 4 and td <= t <= end]
    f0 = next((t for t, v in fb if v == 0xF0), None)
    msg = "0x3FB p[4] f1->f0 during crank (gateway copies it to FR 0xE9 b5): f0 %s" % ms(f0)
    if prof in ("short", "realistic"):
        res(f0 is not None, msg)
    else:
        print("       " + msg + " [%s profile: reported only]" % (prof or "unknown"))
    dend = trel or td + 3e6
    b1b3 = [p[4] for t, i, p in rx if i == 0x1B3 and len(p) > 4 and td <= t <= dend]
    if b1b3:
        n = sum(1 for b in b1b3 if b & 0x10)
        print("       0x1B3 p[4] bit4 (Ian: KL50 / starter) during D: set in %d of %d frames (values %s) [reported only]"
              % (n, len(b1b3), " ".join(sorted({"%02x" % b for b in b1b3}))))
    else:
        print("       0x1B3 not heard during D")


GEAR_3FD = {0x20: "P", 0x40: "R", 0x60: "N", 0x80: "D"}
GEAR_NIB = {3: "P", 2: "R", 1: "N", 5: "D"}   # 0x39A p[3], 0x3F9 p[6] low nibble, 0xE6 b7 low nibble
LEVER = {0: "centre", 1: "forward 1", 2: "forward 2", 3: "back 1", 4: "back 2"}


def gear_report(rx, by, cap, T, con, running):
    """v1h.1 --gear: lever / brake / what we sent, change by change."""
    print("\n5. gear (v1h.1)")
    lv = edges([(t, (p[2] >> 4, p[3] == 0xD5)) for t, i, p in rx if i == 0x197 and len(p) > 3])
    print("  lever 0x197: %d frames, %d changes" % (sum(1 for _, i, _ in rx if i == 0x197), len(lv)))
    brk = edges([(t, p[7] & 3) for t, i, p in rx if i == 0x173 and len(p) > 7])
    brake_at = lambda t: next((v for tt, v in reversed(brk) if tt <= t), None)  # noqa: E731
    f3fd = edges([(t, p[2]) for t, p in by.get(0x3FD, []) if len(p) > 2])
    f39a = edges([(t, (p[3], p[4])) for t, p in by.get(0x39A, []) if len(p) > 4])
    f3f9 = edges([(t, ((p[3] >> 4) & 3, p[6] & 15)) for t, p in by.get(0x3F9, []) if len(p) > 6])
    fdc = edges([(t, (p[2], p[5])) for t, p in by.get(0x0DC, []) if len(p) > 5])
    c12f = edges([(t, p[5]) for t, i, p in rx if i == 0x12F and len(p) > 5])
    fr = load_fr(cap, {0xE6, 0x28, 0x74}) if os.path.exists(cap) and not cap.endswith(".can.csv") else []
    e6 = edges([(t, ((p[3] >> 4) & 3, p[6] & 15)) for t, s, c, p in fr if s == 0xE6 and len(p) > 6])
    f28 = edges([(t, (p[10], p[13])) for t, s, c, p in fr if s == 0x28 and c % 4 == 3 and len(p) > 13])
    f74 = edges([(t, p[5]) for t, s, c, p in fr if s == 0x74 and c % 2 == 0 and len(p) > 5])
    fmt = lambda seq, f: " ".join("%s@%.2f" % (f(v), T(t)) for t, v in seq[:40]) + (" ..." if len(seq) > 40 else "")  # noqa: E731
    print("  our 0x3FD p[2]:        " + (fmt(f3fd, lambda v: "%02X(%s)" % (v, GEAR_3FD.get(v, "-"))) or "not sent"))
    print("  our 0x39A p[3] p[4]:   " + (fmt(f39a, lambda v: "%02X/%02X" % v) or "not sent"))
    print("  our 0x3F9 bits28-29/gear: " + (fmt(f3f9, lambda v: "%d/%X" % v) or "not sent"))
    print("  our 0x0DC p[2] p[5]:   " + (fmt(fdc, lambda v: "%02X/%02X" % v) or "not sent"))
    print("  FR 0xE6 b4 park/b7 gear: " + (fmt(e6, lambda v: "%d/%X" % v) or "not captured"))
    print("  FR 0x28 c3 b11/b14:    " + (fmt(f28, lambda v: "%02X/%02X" % v) or "not captured"))
    print("  CAS 0x12F p[5]:        " + (fmt(c12f, lambda v: "%02X" % v) or "not seen"))
    print("  FR 0x74 p[5]:          " + (fmt(f74, lambda v: "%02X" % v) or "not captured"))
    glines = []
    if os.path.exists(con):
        glines = [ln.strip() for ln in open(con, errors="replace") if "GEAR:" in ln]
    print("  console GEAR lines: %d" % len(glines))
    for ln in glines[:40]:
        print("    " + ln)
    # per lever push (a non-centre position or the P button): what we did
    print("  lever pushes:")
    for t, (pos, pb) in lv:
        if pos == 0 and not pb:
            continue
        what = ("P button" if pb else LEVER.get(pos, "pos %X" % pos))
        win = 1.5e6 if pb else 0.25e6
        ch = next(((tt, v) for tt, v in f3fd if t < tt <= t + win), None)
        a39 = next(((tt, v) for tt, v in f39a if t < tt <= t + win), None)
        b = brake_at(t)
        print("    %.2f s %-9s brake %s -> %s" % (T(t), what, "-" if b is None else ("on" if b == 3 else "off"),
              ("0x3FD %s %+.0f ms" % (GEAR_3FD.get(ch[1], "%02X" % ch[1]), (ch[0] - t) / 1e3)) if ch else
              ("0x39A %02X/%02X %+.0f ms" % (a39[1] + ((a39[0] - t) / 1e3,))) if a39 else "no change (refused? see GEAR lines)"))
    if not (f3fd or f39a):
        print("  (no own 0x3FD / 0x39A: gear checks skipped)")
        return
    # agreement outside transitions (1.2 s after any change of any of the sources)
    chg = sorted([t for t, _ in f3fd + f39a + f3f9 + e6])
    settle = lambda t: any(c <= t <= c + 1.2e6 for c in chg)  # noqa: E731
    last = lambda seq, t: next((v for tt, v in reversed(seq) if tt <= t), None)  # noqa: E731
    bad, n, notrun, pawl_bad = 0, 0, 0, 0
    for t, p in by.get(0x3F9, []):
        if len(p) < 7 or settle(t):
            continue
        g = [GEAR_3FD.get(last(f3fd, t)), GEAR_NIB.get((last(f39a, t) or (0, 0))[0]), GEAR_NIB.get(p[6] & 15)]
        ge6 = last(e6, t)
        if ge6 is not None:
            g.append(GEAR_NIB.get(ge6[1]))
        g = [x for x in g if x]
        n += 1
        if len(set(g)) > 1:
            bad += 1
        if g and g[0] != "P" and not running(t):
            notrun += 1
        a = last(f39a, t)
        if a and (a[1] == 0xE2) != (((p[3] >> 4) & 3) == 2):
            pawl_bad += 1
    res(bad == 0, "0x3FD / 0x39A / 0x3F9 / 0xE6 agree on the gear outside transitions (%d of %d samples differ)" % (bad, n))
    res(notrun == 0, "R/N/D only while our engine runs (%d samples not)" % notrun)
    res(pawl_bad == 0, "P pawl: 0x39A E2 always with 0x3F9 bits 28-29 = 2 (%d samples not)" % pawl_bad)


FLICKER_IDS = (0x39A, 0x3FD, 0x3F9, 0x0DC, 0x397)
FLICKER_E2E = (0x3F9, 0x3FD, 0x0DC)   # bytes 0-1 = E2E CRC + alive counter: change every frame


def flicker_report(by, T):
    """v1h.2 --flicker: any single-frame A -> B -> A on our gear / engine-status frames = FAIL.

    No settle window. The one exception is a real, short gear change: 0x39A, 0x3F9 and
    0x0DC are sent slowly (on change + a long cycle), so a gear the driver held for 400 ms
    (e.g. R -> N -> R) can legitimately be a single frame. Such a B frame is accepted only
    when the master gear frame 0x3FD (100 ms cycle) confirms it: 0x3FD shows a gear at the
    B frame that differs from its gear at both A frames, held for >= 2 0x3FD frames. A stale
    frame (run 9: P / stock content while 0x3FD stayed on D) never passes that test.
    0x3FD and 0x397 themselves have no exception (apart from the stock 0x397 e3 blip)."""
    print("\n6. single-frame flicker of our gear / engine-status frames (v1h.2)")
    g3 = [(t, p[2] & 0xF0) for t, p in by.get(0x3FD, []) if len(p) > 2]
    g3t = [t for t, _ in g3]

    def seg_at(t):   # index range of the 0x3FD run (same gear) covering time t
        k = bisect.bisect_right(g3t, t) - 1
        if k < 0:
            return None, 0
        v = g3[k][1]
        a = k
        while a > 0 and g3[a - 1][1] == v:
            a -= 1
        b = k
        while b + 1 < len(g3) and g3[b + 1][1] == v:
            b += 1
        return v, b - a + 1

    LAG = 25000   # us: 0x39A/0x3F9/0x0DC follow 0x3FD by up to 21 ms (or lead it by 19 ms)
    for i in FLICKER_IDS:
        fr = [(t, p) for t, p in by.get(i, [])]
        sig = [bytes(p[2:]) if i in FLICKER_E2E else bytes(p) for _, p in fr]
        flick, blips, gears = [], 0, 0
        for k in range(1, len(sig) - 1):
            if sig[k] != sig[k - 1] and sig[k + 1] == sig[k - 1]:
                a, b = sig[k - 1], sig[k]
                if (i == 0x397 and len(a) > 2 and {a[1], b[1]} == {0xE2, 0xE3} and a[0] == b[0] and a[2:] == b[2:]):
                    blips += 1   # the real DDE's own e3 blip (stock 0x397 model)
                    continue
                if i in (0x39A, 0x3F9, 0x0DC) and g3:
                    gb, nb = seg_at(fr[k][0] + LAG)
                    ga, _ = seg_at(fr[k - 1][0] + LAG)
                    gc, _ = seg_at(fr[k + 1][0] + LAG)
                    if gb is not None and gb != ga and gb != gc and nb >= 2:
                        gears += 1   # a real short gear (0x3FD agrees), not a stale frame
                        continue
                flick.append((fr[k][0], fr[k - 1][1], fr[k][1]))
        ex = "; ".join("%.3f s %s -> %s" % (T(t), a.hex(), b.hex()) for t, a, b in flick[:4])
        res(not flick, "0x%03X: %d single-frame flickers in %d frames%s%s%s" % (
            i, len(flick), len(fr), (" (e.g. " + ex + ")") if flick else "",
            (", %d stock e3 blips (not counted)" % blips) if blips else "",
            (", %d short gears confirmed by 0x3FD (not counted)" % gears) if gears else ""))


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    cap = argv[0]
    engine = "--engine" in argv
    gear = "--gear" in argv
    canp = cap + ".can.csv" if os.path.exists(cap + ".can.csv") else (cap if cap.endswith(".can.csv") else None)
    if not canp:
        print("no CAN CSV (%s.can.csv) - was the logger run with --can?" % cap)
        return 2
    con = argv[argv.index("--console") + 1] if "--console" in argv[:-1] else cap + ".console.log"
    check_console(con)
    rx, tx = load_can(canp)
    t0 = min([x[0] for x in rx[:1] + tx[:1]] or [0])
    T = lambda t: (t - t0) / 1e6  # noqa: E731
    print("CAN: %d frames heard, %d own TX frames logged (dir=Tx), %.1f s" % (len(rx), len(tx), T(max(rx[-1][0], tx[-1][0] if tx else 0))))

    # KL15 / ST_KL
    stkl = edges([(t, p[2] & 15) for t, i, p in rx if i == 0x12F and len(p) > 2])
    print("ST_KL (0x12F d2): " + " ".join("%X@%.2f" % (v, T(t)) for t, v in stkl[:30]) + (" ..." if len(stkl) > 30 else ""))
    off_t = [t for (t, v), (tp, vp) in zip(stkl[1:], stkl) if vp >= 0xA and v < 0xA and v != 0xF]

    # 1. own TX
    print("\n1. own PT-CAN TX (needs v1f firmware + logger)")
    if not tx:
        res(False, "0 dir=Tx rows: logger older than v1f, or firmware older than v1f, or CAN TX was off")
    by = {}
    for t, i, p in tx:
        by.setdefault(i, []).append((t, p))
    for i in EMU_IDS:
        v = by.get(i, [])
        if not v:
            continue
        ts = [t for t, _ in v]
        per = PERIOD_MS.get(i)
        gaps = [b - a for a, b in zip(ts, ts[1:]) if b - a <= 1e6]
        line = "0x%03X %6d frames, longest gap %.0f ms" % (i, len(ts), max(gaps) / 1000 if gaps else 0)
        if per:
            slots = sum(round((r[-1] - r[0]) / (per * 1000)) + 1 for r in runs_of(ts))
            res(len(ts) >= 0.995 * slots and (max(gaps) if gaps else 0) <= 3 * per * 1000,
                "%s; %d of %d slots (%.2f %%) [need >= 99.5 %%, gap <= %d ms]" % (line, len(ts), slots, 100.0 * len(ts) / slots, 3 * per))
        else:
            print("       " + line)
    # v1h: engine-running windows (our own 0x0A5 rpm > 0, with 0.5 s margin) - the real
    # DDE sends 0x397 every 60 ms while running, ~380 ms on average otherwise.
    rpm_tx = [(t, (p[5] | p[6] << 8) / 4.0) for t, p in by.get(0x0A5, []) if len(p) > 6]
    run_win = [(r[0], r[-1] + 0.5e6) for r in runs_of([t for t, v in rpm_tx if v > 0], 0.3e6)]
    running = lambda t: any(a <= t <= b for a, b in run_win)  # noqa: E731
    v397 = by.get(0x397, [])
    if v397:
        on = [(t, p[1]) for t, p in v397]
        blips = sum(1 for (t, b), (tp, bp) in zip(on[1:], on) if b == 0xE3 and bp != 0xE3)
        g_stop = [b - a for a, b in zip([t for t, _ in v397], [t for t, _ in v397][1:]) if not running(a) and not running(b)]
        g_run = sorted(b - a for a, b in zip([t for t, _ in v397], [t for t, _ in v397][1:]) if running(a) and running(b))
        mean = sum(g_stop) / len(g_stop) / 1000 if g_stop else 0
        if g_stop:
            res(250 <= mean <= 600 and blips >= 3, "0x397 engine not running: mean spacing %.0f ms (real DDE ~380), %d e3 blips" % (mean, blips))
        if g_run:
            med = g_run[len(g_run) // 2] / 1000
            res(50 <= med <= 70, "0x397 engine running: median spacing %.0f ms over %d gaps (real DDE 60 ms)" % (med, len(g_run)))
        for o in off_t[:2]:
            walk = edges([(t, b) for t, b in on if o <= t <= o + 5e6])
            s = " ".join("%02x@+%.2f" % (b, (t - o) / 1e6) for t, b in walk)
            res(any(b == 0xDD for _, b in walk) and any(b == 0xDF for _, b in walk), "0x397 after KL15 off: " + s)
    for i, lim in ((0x512, 46.0), (0x518, 30.0)):
        if by.get(i) and off_t:
            last = max(t for t, _ in by[i] if t >= off_t[0]) if any(t >= off_t[0] for t, _ in by[i]) else None
            if last:
                print("       0x%03X last frame OFF+%.2f s (real DDE/EGS: %.2f)" % (i, (last - off_t[0]) / 1e6, lim - 0.07))

    # 2. foreign copies
    print("\n2. emulated IDs heard from another node (dir=Rx)")
    foreign = sorted({i for _, i, _ in rx if i in EMU_IDS})
    res(not foreign, "none" if not foreign else "heard: " + " ".join("0x%03X" % i for i in foreign))

    # 3. brake
    print("\n3. brake signals")
    # p[7] = 8th byte (0-based index). v1f/v1g read p[6] / p[5] here: one byte too early.
    b173 = edges([(t, p[7] & 3) for t, i, p in rx if i == 0x173 and len(p) > 7])   # DSC copy
    b12f = edges([(t, p[6] & 3) for t, i, p in rx if i == 0x12F and len(p) > 6])   # body (CAS) copy
    print("  PT-CAN 0x173 p[7]&3: " + (" ".join("%d@%.2f" % (v, T(t)) for t, v in b173[:20]) or "not seen"))
    print("  PT-CAN 0x12F p[6]&3: " + (" ".join("%d@%.2f" % (v, T(t)) for t, v in b12f[:20]) or "not seen"))
    fr = load_fr(cap, {0x74, 0x28, 0xE6}) if os.path.exists(cap) and not cap.endswith(".can.csv") else []
    if fr:
        f74 = edges([(t, p[6] & 3) for t, s, c, p in fr if s == 0x74 and len(p) > 6 and c % 2 == 0])
        btn = edges([(t, p[5]) for t, s, c, p in fr if s == 0x74 and len(p) > 5 and c % 2 == 0])
        print("  FlexRay 0x74 p[6]&3 (even cycles): " + (" ".join("%d@%.2f" % (v, T(t)) for t, v in f74[:20]) or "not seen"))
        print("  FlexRay 0x74 p[5] (0x85 = START button): " + " ".join("%02X@%.2f" % (v, T(t)) for t, v in btn[:20]))
    res(bool(b173 or b12f), "brake signals captured on PT-CAN")

    # 4. engine (v1g)
    if engine:
        prof = console_profile(con)
        print("\n4. fake engine start/stop (v1g; start profile %s)" % (prof or "unknown - console log has no FAKE START PROFILE line"))
        if fr:   # reload with every frame the engine changes
            fr = load_fr(cap, {0x74, 0x26, 0x28, 0x29, 0x3D, 0xE6})
        rpm = [(t, (p[5] | p[6] << 8) / 4.0) for t, p in by.get(0x0A5, []) if len(p) > 6]
        if rpm:
            runs_ = [r for r in runs_of([t for t, v in rpm if v > 0], 0.3e6)]
            for r in runs_:
                vals = [v for t, v in rpm if r[0] <= t <= r[-1]]
                print("  PT-CAN 0x0A5 rpm > 0 from %.2f to %.2f s, max %.0f, last %.0f" % (T(r[0]), T(r[-1]), max(vals), vals[-1]))
            res(bool(runs_), "rpm sent on PT-CAN")
            d9 = edges([(t, p[1] >> 4) for t, p in by.get(0x0D9, []) if len(p) > 1])
            print("  PT-CAN 0x0D9 CounterHi: " + " ".join("%X@%.2f" % (v, T(t)) for t, v in d9[:20]))
        m592 = {bytes(p[2:]) for t, p in by.get(0x592, [])}
        if m592:
            res(len(m592) == 1, "0x592 (MIL / check-control) content never changed (%d variants)" % len(m592))
        if fr:
            st74 = edges([(t, p[2] & 15) for t, s, c, p in fr if s == 0x74 and len(p) > 6 and c % 2 == 0])
            print("  FlexRay 0x74 ST_KL: " + " ".join("%X@%.2f" % (v, T(t)) for t, v in st74[:30]))
            e6 = edges([(t, p[2]) for t, s, c, p in fr if s == 0xE6 and len(p) > 2])
            print("  FlexRay 0xE6 b3: " + " ".join("%02X@%.2f" % (v, T(t)) for t, v in e6[:30]))
            r28 = [(t, (p[5] | p[6] << 8) / 4.0) for t, s, c, p in fr if s == 0x28 and c % 4 == 1 and len(p) > 6]
            res(any(v == 0x82 for _, v in e6), "0xE6 b3 reached 82 (running)")
            # per start: ST_KL D edge -> first rpm -> 81 -> 82 -> D released
            for td, _ in [(t, v) for t, v in st74 if v == 0xD]:
                f_rpm = next((t for t, v in r28 if t >= td and v > 0), None)
                t81 = next((t for t, v in e6 if t >= td and v == 0x81), None)
                t82 = next((t for t, v in e6 if t >= td and v == 0x82), None)
                trel = next((t for t, v in st74 if t > td and v != 0xD), None)
                ms = lambda x: "%+.0f ms" % ((x - td) / 1e3) if x else "never"  # noqa: E731
                print("  start at %.2f s (ST_KL D): first rpm %s, E6 81 %s, E6 82 %s, D released %s"
                      % (T(td), ms(f_rpm), ms(t81), ms(t82), ms(trel)))
                res(t82 is not None and (trel is None or trel >= t82),
                    "0x82 sent before CAS released the starter (D -> %s)"
                    % ("%X" % next(v for t, v in st74 if t == trel) if trel else "-"))
                if trel is not None and (t82 is None or trel < t82):
                    print("  *** CAS REFUSED / GAVE UP on this start: ST_KL D ended %s, our running report (E6 82) %s ***"
                          % (ms(trel), ms(t82)))
                elif t82 is not None and (trel is None or trel - t82 > 300e3):
                    print("  *** CAS did not accept the running report: ST_KL D still held %s after E6 82 ***"
                          % ("%.0f ms" % ((trel - t82) / 1e3) if trel else "until the end"))
                v1h_start_checks(td, t82, trel, by, rx, T, prof)
            # v1h: 0x92 (stopping) within 20 ms of ST_KL 8 (real DDE +19 ms; run8 profile does not)
            # ST_KL 8 = the first of the PT-CAN 0x12F and FlexRay 0x74 copies (0x74 can lag)
            for t8f in [t for t, v in st74 if v == 0x8]:
                t8 = min([t for t, v in stkl if v == 0x8 and abs(t - t8f) < 0.5e6] + [t8f])
                t92 = next((t for t, v in e6 if t >= t8 - 50e3 and v == 0x92), None)
                dt = (t92 - t8) / 1e3 if t92 else None
                msg = "0xE6 0x92 (stopping) %s after ST_KL 8 at %.2f s (real DDE +19 ms)" % (
                    "%+.0f ms" % dt if dt is not None else "never", T(t8))
                if prof in ("run8", "default") or prof is None:
                    print("       " + msg + " [run8/unknown profile: reported only]")
                else:
                    res(dt is not None and dt <= 25, msg + " [need <= 20 ms, 25 ms with the 5 ms slot grid]")
            for r in runs_of([t for t, v in r28 if v > 0], 0.3e6):
                idle = [v for t, v in r28 if r[-1] - 2e6 <= t <= r[-1] - 1.3e6]
                if r[-1] - r[0] > 8e6 and idle:
                    med = sorted(idle)[len(idle) // 2]
                    res(760 <= med <= 820, "0x28 rpm %.2f..%.2f s: idle %.0f rpm (780 = 30 0C)" % (T(r[0]), T(r[-1]), med))
            if model:
                ours = [(t, s, c, p) for t, s, c, p in fr if s in (0x26, 0x28, 0x29, 0x3D)]
                bad = sum(1 for t, s, c, p in ours if not model.e2e_check(p, s, c)[0])
                e6bad = sum(1 for t, s, c, p in fr if s == 0xE6 and len(p) >= 8
                            and model.crc8_e2e(p[1:8]) ^ 0x04 != p[0])
                res(bad == 0 and e6bad == 0, "E2E CRCs: %d static frames, %d bad; 0xE6 %d bad" % (len(ours), bad, e6bad))
        st_fr = [v for _, v in st74] if fr else []
        res(any(v in (0xC, 0xD) for _, v in stkl) or any(v in (0xC, 0xD) for v in st_fr),
            "ST_KL reached C/D (CAS start request; PT-CAN 0x12F or FlexRay 0x74)")
    if gear:
        gear_report(rx, by, cap, T, con, running)
    if "--flicker" in argv:
        flicker_report(by, T)
    print("\n%s" % ("ALL PASS" if not fails else "%d FAIL(s)" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
