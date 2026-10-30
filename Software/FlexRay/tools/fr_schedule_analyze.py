#!/usr/bin/env python3
"""Derive the FlexRay schedule from a TS-CAPTURE CSV (fr_ts_logger.py output).

Reports (device-clock microseconds; the device crystal is +-~30 ppm):
  * gdCycle (cycle length) - least squares over all stable slots
  * gdStaticSlot (static slot length), the static slot line and the slots on it
  * static payload length (gPayloadLengthStatic)
  * minislot length (gdMinislot) from dynamic-frame spacings
  * gdMacrotick candidates (1..6 us, multiples of 25 ns) that make cycle,
    static slot and minislot whole numbers of macroticks
  * number of static slots (gNumberOfStaticSlots) candidates, and the
    difference gdActionPointOffset - gdMinislotActionPointOffset
  * bounds on gdActionPointOffset, on the minislot count and on NIT
  * per-slot table: offset of each frame start from static slot 1's frame
    start (= cycle start + gdActionPointOffset), its spread, segment, cycle
    multiplexing (base/repetition), payload length, null %

  python3 fr_schedule_analyze.py capture.csv
  python3 fr_schedule_analyze.py capture.csv --json schedule.json --slots-csv slots.csv
"""
import argparse
import csv
import json
import math
import statistics
import sys
from collections import defaultdict

DDE_IDS = (0x26, 0x28, 0x29, 0x3D, 0x5C, 0x68, 0xE6, 0xE8, 0xFB, 0x142)
BIT_US = 0.1


def frame_dur_us(payload_bytes, tss_bits=9):
    return (tss_bits + 1 + (5 + payload_bytes + 3) * 10 + 2) * BIT_US


def load(path, channel=None, hw_only=True):
    rows = []
    stats = dict(total=0, no_time=0, no_hw=0, seq_gaps=0, seq_missing=0)
    last_seq = None
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            stats["total"] += 1
            if r.get("seq"):
                s = int(r["seq"])
                if last_seq is not None:
                    d = (s - last_seq - 1) & 0xFFFFFFFF
                    if 0 < d < 0x80000000:
                        stats["seq_gaps"] += 1
                        stats["seq_missing"] += d
                last_seq = s
            if not r["t_us"]:
                stats["no_time"] += 1
                continue
            if hw_only and r.get("ts_hw", "1") == "0":
                stats["no_hw"] += 1
                continue
            if channel and r["channel"] != channel:
                continue
            rows.append((float(r["t_us"]), r["channel"], int(r["slot_id"]), int(r["cycle"]),
                         int(r["payload_len"]), int(r.get("null_frame") or 0)))
    rows.sort()
    return rows, stats


def median(v):
    return statistics.median(v) if v else float("nan")


def estimate_cycle(rows):
    by = defaultdict(list)
    for t, ch, sid, cyc, _pl, _nf in rows:
        by[(ch, sid)].append((t, cyc))
    cands = []
    for lst in by.values():
        for (t1, c1), (t2, c2) in zip(lst, lst[1:]):
            dc = (c2 - c1) % 64
            if dc:
                cands.append((t2 - t1) / dc)
    T0 = median(cands)
    refined = []
    for lst in by.values():
        for (t1, c1), (t2, c2) in zip(lst, lst[1:]):
            k = round((t2 - t1) / T0)
            if k > 0 and k % 64 == (c2 - c1) % 64:
                refined.append((t2 - t1) / k)
    return median(refined) if refined else T0


def number_cycles(rows, T):
    """global cycle number n for each row (consistent with the 6-bit counter)."""
    t_ref, _, _, c_ref, _, _ = rows[0]
    out = []
    for t, ch, sid, cyc, pl, nf in rows:
        x = (t - t_ref) / T + c_ref  # approx. global cycle (fractional)
        n = cyc + 64 * round((x - cyc) / 64)
        out.append((t, ch, sid, cyc, pl, nf, n))
    return out


def refine_cycle(nrows, T, max_std=1.0):
    """LS slope with one intercept per (channel, slot), over slots with a stable phase."""
    by = defaultdict(list)
    for t, ch, sid, _c, _pl, _nf, n in nrows:
        by[(ch, sid)].append((n, t))
    # Coarse-to-fine: the first estimate can be off by ~10 ns/cycle (quantised
    # median), which over a long capture smears every phase by hundreds of us.
    # So the first passes accept all slots and judge stability on the
    # *detrended* residual (std about each slot's own LS line).
    # Only slots seen in a good share of the cycles vote: a slot with 3-4 frames
    # in neighbouring cycles always looks "stable" and, on a long capture where
    # warm-up drift pushes every real slot over the tight threshold, would
    # otherwise set T on its own (run 4: T = 5009 us from slot 0xB7).
    nmax = max((len(v) for v in by.values()), default=0)
    min_len = max(3, nmax // 20)
    for thr in (float("inf"), 50.0, 5.0, max_std, max_std):
        num = den = 0.0
        used = 0
        for lst in by.values():
            if len(lst) < min_len:
                continue
            nb0 = statistics.fmean(n for n, _ in lst)
            tb0 = statistics.fmean(t for _, t in lst)
            sxx = sum((n - nb0) ** 2 for n, _ in lst)
            if sxx <= 0:
                continue
            k = sum((n - nb0) * (t - tb0) for n, t in lst) / sxx
            res = [t - tb0 - k * (n - nb0) for n, t in lst]
            m = 0.0
            if statistics.pstdev(res) > thr:
                continue
            nb = statistics.fmean(n for n, _ in lst)
            tb = statistics.fmean(t for _, t in lst)
            num += sum((n - nb) * (t - tb) for n, t in lst)
            den += sum((n - nb) ** 2 for n, _ in lst)
            used += 1
            _ = m
        if den > 0 and used >= 3:   # too few slots at this threshold: keep the previous T
            T = num / den
    return T


def detrend_clock(nrows, T, min_presence=0.5, win=2000):
    """Remove slow device-vs-cluster clock drift (crystal warm-up etc.).

    A single straight line t = n*T + phase is not enough over minutes: a few
    ppm of drift smears every phase by tens of us. Here each global cycle gets
    its own reference = median over 'anchor' slots (present in >= min_presence
    of the cycles) of (t - n*T - anchor offset), iterated. Frame times are then
    re-expressed as n*T + (offset inside that cycle). Returns (nrows', info).
    """
    if not nrows:
        return nrows, {}
    n_lo = min(r[6] for r in nrows)
    n_hi = max(r[6] for r in nrows)
    ncyc = n_hi - n_lo + 1
    by = defaultdict(list)
    for t, ch, sid, _c, _pl, _nf, n in nrows:
        by[(ch, sid)].append((n, t - n * T))
    anchors = [k for k, v in by.items() if len(v) >= min_presence * ncyc]
    if len(anchors) < 3 or ncyc < 50:
        return nrows, {"applied": False}
    drift = {}
    for _ in range(3):
        off = {}
        for k in anchors:
            off[k] = median([r - drift.get(n, 0.0) for n, r in by[k]])
        per = defaultdict(list)
        for k in anchors:
            for n, r in by[k]:
                per[n].append(r - off[k])
        drift = {n: median(v) for n, v in per.items() if len(v) >= max(2, len(anchors) // 4)}
    if not drift:
        return nrows, {"applied": False}
    ns = sorted(drift)
    # fill cycles without anchors by linear interpolation
    import bisect

    def d_of(n):
        if n in drift:
            return drift[n]
        i = bisect.bisect_left(ns, n)
        if i == 0:
            return drift[ns[0]]
        if i >= len(ns):
            return drift[ns[-1]]
        a, b = ns[i - 1], ns[i]
        return drift[a] + (drift[b] - drift[a]) * (n - a) / (b - a)

    out = [(n * T + (t - n * T - d_of(n)), ch, sid, c, pl, nf, n) for t, ch, sid, c, pl, nf, n in nrows]
    # local cycle length per window (device us), and cycle-to-cycle reference jitter
    loc = []
    for i in range(0, len(ns) - win, win // 2):
        a, b = ns[i], ns[i + win - 1]
        loc.append(T + (drift[b] - drift[a]) / (b - a))
    steps = [drift[b] - drift[a] for a, b in zip(ns, ns[1:]) if b == a + 1]
    st_mean = statistics.fmean(steps) if steps else 0.0
    info = {
        "applied": True,
        "anchors": len(anchors),
        "cycles": ncyc,
        "cycles_with_ref": len(ns),
        "drift_span_us": max(drift.values()) - min(drift.values()),
        "local_cycle_us_min": min(loc) if loc else T,
        "local_cycle_us_max": max(loc) if loc else T,
        "local_cycle_us_first": loc[0] if loc else T,
        "local_cycle_us_last": loc[-1] if loc else T,
        "cycle_to_cycle_jitter_ns": 1000 * statistics.pstdev(steps) if len(steps) > 1 else 0.0,
        "_mean_step": st_mean,
    }
    return out, info


def slot_phases(nrows, T):
    """per (channel, slot): list of phase residuals t - n*T, and metadata."""
    by = defaultdict(lambda: dict(res=[], cycles=set(), data_cycles=set(), pl=defaultdict(int), nulls=0, n=0,
                                  cyc_res=[]))
    for t, ch, sid, cyc, pl, nf, n in nrows:
        d = by[(ch, sid)]
        d["res"].append(t - n * T)
        d["cyc_res"].append((n, t - n * T))
        d["cycles"].add(cyc)
        if not nf:
            d["data_cycles"].add(cyc)
        d["pl"][pl] += 1
        d["nulls"] += nf
        d["n"] += 1
    return by


def circ_mean(vals, T):
    """mean of phases mod T (handles wrap)."""
    a = [2 * math.pi * (v % T) / T for v in vals]
    s = sum(math.sin(x) for x in a)
    c = sum(math.cos(x) for x in a)
    return (math.atan2(s, c) / (2 * math.pi) * T) % T


def minislot_from_firsts(firsts, min_ids=3, min_n=3, tol_us=1.0):
    """Robust line u = b + M*id through per-id medians of first-in-cycle dynamic frames."""
    by = defaultdict(list)
    for u, sid, _, _ in firsts:
        by[sid].append(u)
    pts = []
    for sid, us in by.items():
        if len(us) < min_n:
            continue
        m = median(us)
        if median([abs(x - m) for x in us]) > 0.2:
            continue  # this id is not reliably first at a fixed position
        pts.append((sid, m))
    if len(pts) < min_ids:
        return None
    keep = pts
    for _ in range(5):
        xs = [x for x, _ in keep]
        ys = [y for _, y in keep]
        xb, yb = statistics.fmean(xs), statistics.fmean(ys)
        den = sum((x - xb) ** 2 for x in xs)
        if not den:
            return None
        M = sum((x - xb) * (y - yb) for x, y in zip(xs, ys)) / den
        b = yb - M * xb
        res = [(x, y, y - b - M * x) for x, y in pts]
        new = [(x, y) for x, y, r in res if abs(r) <= tol_us]
        if len(new) < min_ids or new == keep:
            break
        keep = new
    rms = statistics.pstdev([y - b - M * x for x, y in keep]) if len(keep) > 1 else 0.0
    return dict(minislot_us=M, ids_used=len(keep), ids_total=len(pts),
                fit_fraction=len(keep) / len(pts), resid_rms_us=rms)


def fit_static_line(slots, T, tol):
    """RANSAC: rel = a + S*id over stable slots. Returns (a, S, inlier keys)."""
    stable = [(k[1], v["phase"], k) for k, v in slots.items() if v["std"] < tol and v["n"] >= 3]
    best = (0, None)
    for i in range(len(stable)):
        for j in range(i + 1, len(stable)):
            id1, p1, _ = stable[i]
            id2, p2, _ = stable[j]
            if id1 == id2:
                continue
            S = ((p2 - p1) % T if id2 > id1 else (p1 - p2) % T) / abs(id2 - id1)
            if not (5.0 <= S <= 700.0):
                continue
            lo = (id1, p1) if id1 < id2 else (id2, p2)
            inl = []
            for idk, pk, kk in stable:
                pred = (lo[1] + (idk - lo[0]) * S) % T
                e = ((pk - pred + T / 2) % T) - T / 2
                if abs(e) < tol:
                    inl.append(kk)
            # prefer more inliers, then the line that starts at the lowest id
            score = (len(inl), -min(k[1] for k in inl) if inl else 0)
            if best[1] is None or score > best[0]:
                best = (score, (lo, S, inl))
    if best[1] is None:
        return None
    lo, S, inl = best[1]
    # least-squares refit: unwrap each inlier phase relative to the line
    xs, ys = [], []
    for k in inl:
        idk = k[1]
        pred = lo[1] + (idk - lo[0]) * S
        e = ((slots[k]["phase"] - pred + T / 2) % T) - T / 2
        xs.append(idk)
        ys.append(pred + e)
    if len(set(xs)) >= 2:
        xb, yb = statistics.fmean(xs), statistics.fmean(ys)
        S = sum((x - xb) * (y - yb) for x, y in zip(xs, ys)) / sum((x - xb) ** 2 for x in xs)
        a = yb - S * xb
    else:
        a = ys[0] - S * xs[0]
    return a, S, inl


def minislot_from_deltas(deltas, lo=1.0, hi=60.0, tol=0.08):
    """largest M such that (almost) all deltas are whole multiples of M."""
    deltas = [d for d in deltas if d > lo * 0.9]
    if len(deltas) < 3:
        return None, 0.0
    dmin = min(deltas)
    best = None
    for k in range(1, 40):
        M = dmin / k
        if M < lo:
            break
        if M > hi:
            continue
        # refine M by LS on integer multiples, short spacings first
        M2 = M
        for lim in (6, 25, 100, 1e9):
            sel = [d for d in deltas if d / M2 < lim]
            q = [round(d / M2) for d in sel]
            if sum(q) == 0:
                break
            M2 = sum(d * qq for d, qq in zip(sel, q)) / sum(qq * qq for qq in q)
        t = min(tol, M2 / 5)
        ok = sum(1 for d in deltas if abs(d / M2 - round(d / M2)) * M2 < t) / len(deltas)
        if ok >= 0.95:
            best = (M2, ok)
            break
    return best if best else (None, 0.0)


def cycle_mux(cycles):
    """(repetition, base): largest power-of-two r with all cycles == base (mod r).
    Only meaningful when the capture covers >= 64 cycles."""
    if not cycles:
        return None, None
    for r in (64, 32, 16, 8, 4, 2, 1):
        if len({c % r for c in cycles}) == 1:
            return r, cycles[0] % r
    return 1, 0


def macrotick_candidates(T, S, M, max_ppm, err_mt=0.02, err_ms_mt=0.02):
    """gdMacrotick values (1..6 us, n*25 ns) making cycle, static slot and minislot integer."""
    out = []
    for m in range(40, 241):
        MT = m * 0.025
        n0 = round(T / MT)
        for nc in (n0 - 1, n0, n0 + 1):
            ppm = (T / (nc * MT) - 1.0) * 1e6
            if abs(ppm) > max_ppm:
                continue
            k = 1 + ppm * 1e-6
            ss, mm = S / k / MT, (M / k / MT if M else None)
            es = abs(ss - round(ss))
            em = abs(mm - round(mm)) if M else 0.0
            if es <= err_mt and em <= err_ms_mt and round(ss) >= 1 and (not M or round(mm) >= 2):
                out.append(dict(macrotick_us=MT, cycle_mt=nc, static_slot_mt=round(ss),
                                minislot_mt=round(mm) if M else None, ppm=ppm,
                                static_err_mt=es, minislot_err_mt=em))
    # plausible ones first: real device + ECU clock errors are < ~100 ppm
    out.sort(key=lambda c: (abs(c["ppm"]) > 100, c["macrotick_us"] if abs(c["ppm"]) <= 100 else abs(c["ppm"])))
    return out


def analyze(rows, args):
    rep = {}
    T = estimate_cycle(rows)
    nrows = number_cycles(rows, T)
    T = refine_cycle(nrows, T)
    nrows = number_cycles(rows, T)
    T = refine_cycle(nrows, T)
    rep["cycle_us"] = T
    if not getattr(args, "no_detrend", False):
        nrows, dinfo = detrend_clock(nrows, T)
        rep["clock_detrend"] = dinfo
    slots = slot_phases(nrows, T)
    for k, v in slots.items():
        v["phase"] = circ_mean(v["res"], T)
        devs = [((r - v["phase"] + T / 2) % T) - T / 2 for r in v["res"]]
        v["std"] = statistics.pstdev(devs) if len(devs) > 1 else 0.0
        v["min"] = min(devs)
        v["max"] = max(devs)
    line = fit_static_line(slots, T, args.static_tol)
    if line is None:
        raise SystemExit("could not find a static slot line (too few stable slots)")
    a, S, inl = line
    rep["static_slot_us"] = S
    # all static frames share gPayloadLengthStatic: a slot on the line with another
    # length is a dynamic slot that happens to sit there (e.g. the first dynamic slot)
    pls0 = defaultdict(int)
    for k in inl:
        for pl, c in slots[k]["pl"].items():
            pls0[pl] += c
    stat_pl = max(pls0, key=pls0.get)
    inl = [k for k in inl if max(slots[k]["pl"], key=slots[k]["pl"].get) == stat_pl]
    inl_set = set(inl)
    static_ids = sorted({k[1] for k in inl})
    rep["static_ids_seen"] = static_ids
    max_static = max(static_ids)
    origin = a + S  # rel phase of static slot 1's frame start (cycle start + gdActionPointOffset)

    def u_of(phase):
        # offset from static slot 1's frame start, in [-S, T - S)
        return ((phase - origin + S) % T) - S

    pls = defaultdict(int)
    for k in inl:
        for pl, c in slots[k]["pl"].items():
            pls[pl] += c
    rep["static_payload_bytes"] = max(pls, key=pls.get)
    rep["static_payload_bytes_hist"] = dict(pls)

    # dynamic frames: per cycle, ordered positions after the static segment
    static_end_min = max_static * S
    per_cycle = defaultdict(list)
    for t, ch, sid, cyc, pl, nf, n in nrows:
        if (ch, sid) in inl_set:
            continue
        u = (((t - n * T) - origin + S) % T) - S
        per_cycle[n].append((u, sid, pl, ch))
    deltas = []
    firsts = []
    for n, lst in per_cycle.items():
        lst.sort()
        lst = [x for x in lst if x[0] > static_end_min - S]
        if not lst:
            continue
        firsts.append(lst[0])
        for (u1, s1, p1, _), (u2, s2, p2, _) in zip(lst, lst[1:]):
            if s2 > s1:
                deltas.append(u2 - u1)
    st_std = median([slots[k]["std"] for k in inl])
    rep["static_jitter_std_us"] = st_std
    M, okfrac = minislot_from_deltas(deltas, tol=max(0.08, 6.0 * st_std))
    Mf = minislot_from_firsts(firsts)
    rep["minislot_firsts_fit"] = Mf
    if Mf is not None and (M is None or (okfrac or 0) < 0.5):
        # first frame of a cycle sits at a fixed minislot (no earlier frame consumed any):
        # u = b + M*id. Used when consecutive-frame spacing does not fit well.
        M, okfrac = Mf["minislot_us"], Mf["fit_fraction"]
    rep["minislot_us"] = M
    rep["minislot_fit_fraction"] = okfrac

    # number of static slots: u_first(j) = N*S + (j-N-1)*M + (dAP - AP)
    ncands = []
    if M and firsts:
        for N in range(max_static, max_static + 400):
            r = [u - N * S - (j - N - 1) * M for u, j, _, _ in firsts]
            rm = median(r)
            spread = max(abs(x - rm) for x in r)
            if N * S > T:
                break
            if min(j for _, j, _, _ in firsts) <= N:
                break
            ncands.append(dict(n_static=N, apdiff_us=rm, spread_us=spread))
        ncands.sort(key=lambda d: abs(d["apdiff_us"]))
    rep["n_static_candidates"] = ncands[:5]

    # minislot precision is worse than the static slot's (fewer, noisier spacings)
    mts = macrotick_candidates(T, S, M, args.max_ppm, err_ms_mt=max(0.02, 3.0 * st_std / 1.0))
    rep["macrotick_candidates"] = mts

    # static frame -> action point bound: 2*AP + frame + CID(11 bits) <= slot
    fd = frame_dur_us(rep["static_payload_bytes"])
    rep["static_frame_dur_us"] = fd
    rep["ap_max_us"] = (S - fd - 1.1) / 2

    # dynamic segment end / NIT bounds
    if ncands and M:
        N = ncands[0]["n_static"]
        dyn_start_u = N * S - 0.0  # relative to static slot 1 frame start: minus AP (unknown, small)
        last_end = 0.0
        max_ms_used = 0
        for n, lst in per_cycle.items():
            for u, sid, pl, ch in lst:
                if sid > N:
                    end = u + frame_dur_us(pl)
                    last_end = max(last_end, end)
                    max_ms_used = max(max_ms_used, math.ceil((end - dyn_start_u) / M))
        rep["dynamic"] = dict(n_static_used=N, dyn_start_u=dyn_start_u, last_frame_end_u=last_end,
                              min_minislots=max_ms_used,
                              max_minislots=int((T - dyn_start_u) // M),
                              nit_plus_symbol_window_max_us=T - last_end)

    # per-slot table
    table = []
    for k, v in sorted(slots.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        ch, sid = k
        cyc = sorted(v["cycles"])
        rep_c, base = cycle_mux(cyc)
        drep, dbase = cycle_mux(sorted(v["data_cycles"]))
        seg = "static" if k in inl_set else ("dynamic" if sid > max_static else "static?")
        u = u_of(v["phase"])
        e = dict(slot_id=sid, slot_hex="0x%X" % sid, channel=ch, frames=v["n"], segment=seg,
                 offset_us=u, std_us=v["std"], min_us=v["min"], max_us=v["max"],
                 payload_bytes=max(v["pl"], key=v["pl"].get), null_pct=100.0 * v["nulls"] / v["n"],
                 cycles_seen=len(cyc), repetition=rep_c, base_cycle=base,
                 data_repetition=drep, data_base_cycle=dbase,
                 dde=sid in DDE_IDS)
        if seg == "static":
            e["static_pos_us"] = (sid - 1) * S
            e["residual_ns"] = ((u - (sid - 1) * S + T / 2) % T - T / 2) * 1000
        elif M and ncands:
            N = ncands[0]["n_static"]
            e["minislot_index_mean"] = (u - N * S - ncands[0]["apdiff_us"]) / M
        table.append(e)
    rep["slots"] = table
    return rep


def print_report(rep, stats, args, out=sys.stdout):
    p = lambda *a: print(*a, file=out)  # noqa: E731
    T, S, M = rep["cycle_us"], rep["static_slot_us"], rep["minislot_us"]
    p("== FlexRay schedule from %s ==" % args.csv)
    p("rows: %d used, %d total, %d without time, %d without hw timestamp (excluded), seq gaps %d (%d frames lost in USB FIFO)"
      % (stats["used"], stats["total"], stats["no_time"], stats["no_hw"], stats["seq_gaps"], stats["seq_missing"]))
    p("cycle length (gdCycle)        : %.4f us (device clock, mean over the capture)" % T)
    di = rep.get("clock_detrend") or {}
    if di.get("applied"):
        p("clock drift removed           : %d anchor slots, phase drift span %.1f us; local cycle %.4f .. %.4f us"
          " (first %.4f, last %.4f; %.1f ppm range); cycle-to-cycle reference jitter %.1f ns rms"
          % (di["anchors"], di["drift_span_us"], di["local_cycle_us_min"], di["local_cycle_us_max"],
             di["local_cycle_us_first"], di["local_cycle_us_last"],
             1e6 * (di["local_cycle_us_max"] - di["local_cycle_us_min"]) / T, di["cycle_to_cycle_jitter_ns"]))
    p("static slot (gdStaticSlot)    : %.4f us   static slots seen: %d, highest id %d (0x%X)"
      % (S, len(rep["static_ids_seen"]), max(rep["static_ids_seen"]), max(rep["static_ids_seen"])))
    p("static payload                : %d bytes (%d words)  hist %s"
      % (rep["static_payload_bytes"], rep["static_payload_bytes"] // 2, rep["static_payload_bytes_hist"]))
    p("minislot (gdMinislot)         : %s" % ("%.4f us (%.0f%% of dynamic spacings fit)" % (M, 100 * rep["minislot_fit_fraction"]) if M else "not determined (too few dynamic frames)"))
    p("")
    p("gdMacrotick candidates (cycle, static slot and minislot all whole macroticks; 1..6 us in 25 ns steps;")
    p("  ppm = device clock vs cluster; real clocks are within ~100 ppm, so the ones listed first are plausible):")
    if not rep["macrotick_candidates"]:
        p("  none within %.0f ppm - check the data (or relax --max-ppm)" % args.max_ppm)
    for c in rep["macrotick_candidates"][:12]:
        p("  %s MT %.3f us: gdCycle %d MT, gdStaticSlot %d MT, gdMinislot %s MT, device clock %+.1f ppm vs cluster"
          % ("  " if abs(c["ppm"]) <= 100 else "? ", c["macrotick_us"], c["cycle_mt"], c["static_slot_mt"], c["minislot_mt"], c["ppm"]))
    if len(rep["macrotick_candidates"]) > 1:
        p("  (timing alone cannot tell a macrotick from its divisors/multiples; pick the one matching the")
        p("   ECU config / pdMicrotick 25 ns * pMicroPerMacroNom; smaller lists = the right one is among them)")
    p("")
    p("gNumberOfStaticSlots candidates (from first dynamic frame per cycle):")
    for c in rep["n_static_candidates"]:
        p("  N=%d (0x%X): gdActionPointOffset - gdMinislotActionPointOffset = %+.3f us (spread %.3f us)"
          % (c["n_static"], c["n_static"], -c["apdiff_us"], c["spread_us"]))
    if not rep["n_static_candidates"]:
        p("  not determined (needs dynamic frames)")
    p("gdActionPointOffset: not directly observable (all frames shift with it); upper bound from the static")
    p("  slot: (gdStaticSlot - frame %.1f us - CID 1.1 us)/2 = %.2f us" % (rep["static_frame_dur_us"], rep["ap_max_us"]))
    d = rep.get("dynamic")
    if d:
        p("dynamic segment (N=%d): starts %.2f us after slot-1 frame start (minus AP); last frame ends at %.2f us;"
          % (d["n_static_used"], d["dyn_start_u"], d["last_frame_end_u"]))
        p("  gNumberOfMinislots >= %d (and <= %d); NIT + symbol window <= %.2f us"
          % (d["min_minislots"], d["max_minislots"], d["nit_plus_symbol_window_max_us"]))
    p("")
    p("Offsets are frame start (TSS falling edge at the capture pin) minus static slot 1's frame start")
    p("(= cycle start + gdActionPointOffset), averaged over all cycles; std/min/max = spread around it.")
    p("base/rep = cycle multiplexing of all frames; data b/r = of non-null frames only")
    p("%-6s %-4s %-8s %7s %11s %8s %8s %8s %4s %5s %-8s %-8s %s" % (
        "slot", "ch", "segment", "frames", "offset_us", "std_ns", "min_ns", "max_ns", "len", "null%", "base/rep",
        "data b/r", "note"))
    for e in rep["slots"]:
        if args.only_dde and not e["dde"]:
            continue
        note = []
        if "residual_ns" in e:
            note.append("static pos %.3f, resid %+.0f ns" % (e["static_pos_us"], e["residual_ns"]))
        if "minislot_index_mean" in e:
            note.append("minislot idx ~%.2f" % e["minislot_index_mean"])
        if e["dde"]:
            note.append("DDE")
        dbr = "%d/%d" % (e["data_base_cycle"], e["data_repetition"]) if e["data_repetition"] else "-"
        p("%-6s %-4s %-8s %7d %11.3f %8.0f %8.0f %8.0f %4d %5.1f %-8s %-8s %s" % (
            e["slot_hex"], e["channel"], e["segment"], e["frames"], e["offset_us"], e["std_us"] * 1000,
            e["min_us"] * 1000, e["max_us"] * 1000, e["payload_bytes"], e["null_pct"],
            "%d/%d" % (e["base_cycle"], e["repetition"]), dbr, "; ".join(note)))


def check_truth(rep, truth_path):
    with open(truth_path) as f:
        tr = json.load(f)
    res = []

    def chk(name, ok, detail):
        res.append((name, ok, detail))
    chk("cycle", abs(rep["cycle_us"] - tr["cycle_us_device"]) < 0.01,
        "%.5f vs %.5f" % (rep["cycle_us"], tr["cycle_us_device"]))
    chk("static_slot", abs(rep["static_slot_us"] - tr["static_slot_us_device"]) < 0.002,
        "%.5f vs %.5f" % (rep["static_slot_us"], tr["static_slot_us_device"]))
    chk("minislot", rep["minislot_us"] is not None and abs(rep["minislot_us"] - tr["minislot_us_device"]) < 0.01,
        "%s vs %.5f" % (rep["minislot_us"], tr["minislot_us_device"]))
    chk("payload", rep["static_payload_bytes"] == 2 * tr["payload_words"], str(rep["static_payload_bytes"]))
    mts = [c["macrotick_us"] for c in rep["macrotick_candidates"]]
    chk("macrotick_in_candidates", any(abs(m - tr["mt_us"]) < 1e-9 for m in mts), str(mts))
    nst = rep["n_static_candidates"][0]["n_static"] if rep["n_static_candidates"] else None
    chk("n_static", nst == tr["n_static"], "%s vs %d" % (nst, tr["n_static"]))
    apd = (tr["ap_mt"] - tr["mini_ap_mt"]) * tr["mt_us"]
    got = -rep["n_static_candidates"][0]["apdiff_us"] if rep["n_static_candidates"] else float("nan")
    chk("ap_minus_mini_ap", abs(got - apd) < 0.05, "%.4f vs %.4f" % (got, apd))
    k = 1 + tr["ppm"] * 1e-6
    worst = 0.0
    for e in rep["slots"]:
        if e["segment"] == "static":
            want = tr["static_offsets_us"].get(str(e["slot_id"]))
            if want is not None:
                worst = max(worst, abs(e["offset_us"] - want * k))
    chk("static_offsets", worst < 0.02, "worst %.4f us" % worst)
    bad_mux = 0
    for e in rep["slots"]:
        cfg = tr.get("static_cfg", {}).get(str(e["slot_id"]))
        if cfg and e["segment"] == "static" and e["data_repetition"] is not None:
            if (e["data_repetition"], e["data_base_cycle"]) != (cfg["rep"], cfg["base"]):
                bad_mux += 1
    chk("static_data_mux", bad_mux == 0, "%d mismatches" % bad_mux)
    statics = sorted(e["slot_id"] for e in rep["slots"] if e["segment"] == "static")
    chk("static_set", set(statics) == set(tr["static_ids"]), "%d vs %d" % (len(set(statics)), len(tr["static_ids"])))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--channel", help="only this channel (FR1 or FR2); default both")
    ap.add_argument("--include-cpu-ts", action="store_true", help="also use rows without a hardware timestamp")
    ap.add_argument("--static-tol", type=float, default=0.5, help="us; max phase spread of a static slot")
    ap.add_argument("--max-ppm", type=float, default=300.0, help="device vs cluster clock limit for MT search")
    ap.add_argument("--only-dde", action="store_true", help="slot table: DDE slots only")
    ap.add_argument("--no-detrend", action="store_true",
                    help="do not remove slow clock drift (single straight-line cycle model)")
    ap.add_argument("--json", help="write the full result as JSON")
    ap.add_argument("--slots-csv", help="write the per-slot table as CSV")
    ap.add_argument("--truth", help="(tests) compare with a fr_ts_synth.py truth JSON; exit 1 on mismatch")
    args = ap.parse_args(argv)

    rows, stats = load(args.csv, args.channel, not args.include_cpu_ts)
    stats["used"] = len(rows)
    if len(rows) < 100:
        print("too few rows with timestamps (%d)" % len(rows), file=sys.stderr)
        return 1
    rep = analyze(rows, args)
    print_report(rep, stats, args)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(dict(rep, input_stats=stats), f, indent=1)
    if args.slots_csv:
        keys = ["slot_id", "slot_hex", "channel", "segment", "frames", "offset_us", "std_us", "min_us", "max_us",
                "payload_bytes", "null_pct", "cycles_seen", "base_cycle", "repetition", "data_base_cycle",
                "data_repetition", "dde",
                "static_pos_us", "residual_ns", "minislot_index_mean"]
        with open(args.slots_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for e in rep["slots"]:
                w.writerow(e)
    if args.truth:
        res = check_truth(rep, args.truth)
        print("")
        print("== truth check ==")
        for name, ok, det in res:
            print("  %-24s %s  %s" % (name, "OK  " if ok else "FAIL", det))
        return 0 if all(ok for _, ok, _ in res) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
