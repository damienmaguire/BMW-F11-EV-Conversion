#!/usr/bin/env python3
"""Offline tests of the DDE emulator (no hardware).

 1. CRC / E2E: header CRC, frame CRC and the E2E counter+CRC8 scheme against the
    committed captured DDE frames (tests/host/dde_vectors_run5_2s.txt) and, when
    the run 5 capture is present, against every frame of the run. Must be 100 %.
 2. Table rebuild: the frames the firmware will send (tables + live counters/CRCs)
    are bit-identical to the captured ign-on/engine-off frames (captured sync bits
    kept for the comparison); with EMU_SYNC_BITS=0 only the sync/startup bits and
    the header CRC change.
 3. PIO encode: the real dde_emu_tx / dde_emu_fire programs (assembled with
    pioasm) run in a cycle-accurate mini PIO simulator; the TXEN/TXD trace is
    decoded and must reproduce the captured raw frames: TSS length, FSS, BSS,
    every bit exactly 10 clk_sys cycles, FES, and DTS for the dynamic frame.
    TXEN must fall exactly (delay + constant) cycles after the fire word.
 4. Timing: the schedule maths, and on run 5 the prediction "mark(target) =
    mark(ref) + table delta": ref-to-ref errors and DDE (FR2) frames predicted
    from car (FR1) references.

Usage: python3 test_dde_emu.py [--bin f11_run5.csv.bin] [--pioasm PATH]
"""
import argparse, os, re, statistics, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "tools", "ts_capture"))
import dde_emu_model as m  # noqa: E402

VECTORS = os.path.join(ROOT, "tests", "host", "dde_vectors_run5_2s.txt")
TABLES = os.path.join(ROOT, "src", "dde_emu_tables.h")
PIO_SRC = os.path.join(ROOT, "src", "dde_emu_tx.pio")
DEF_BIN = "/workspace/f11-ts-run5/f11_run5.csv.bin"
TSS = 7

fails = 0


def check(cond, msg):
    global fails
    if not cond:
        fails += 1
        print("FAIL:", msg)
    return cond


# ------------------------------------------------------------------ tables (C header)
def parse_tables(path=TABLES):
    src = open(path).read()
    src_nc = re.sub(r"//[^\n]*", "", src)

    def arr(name):
        mm = re.search(name + r"\s*(\[[^=]*\])?\s*=\s*\{(.*?)\};", src_nc, re.S)
        return [int(x, 0) for x in re.findall(r"-?0x[0-9A-Fa-f]+|-?\d+", mm.group(2))]

    def define(name):
        return int(re.search(r"#define " + name + r"\s+(-?\w+)", src).group(1), 0)

    t = {}
    t["ids"] = arr("dde_emu_static_id")
    nb = define("DDE_EMU_STATIC_PAYLOAD_BYTES")
    ind = arr("dde_emu_static_ind")
    t["ind"] = [ind[i * 64:(i + 1) * 64] for i in range(len(t["ids"]))]
    pl = arr("dde_emu_static_payload")
    t["payload"] = [[bytes(pl[(i * 64 + c) * nb:(i * 64 + c + 1) * nb]) for c in range(64)]
                    for i in range(len(t["ids"]))]
    e2e = arr("dde_emu_e2e")
    t["e2e"] = [tuple(e2e[i:i + 5]) for i in range(0, len(e2e), 5)]
    t["ref_id"] = arr("dde_emu_ref_id")
    t["ref_off_q8"] = arr("dde_emu_ref_off_q8")
    t["tgt_off_q8"] = arr("dde_emu_static_off_q8")
    t["dyn_id"] = define("DDE_EMU_DYN_ID")
    t["dyn_ind"] = define("DDE_EMU_DYN_IND")
    t["dyn_payload"] = bytes(arr("dde_emu_dyn_payload"))
    t["dyn_off_q8"] = int(re.search(r"dde_emu_dyn_off_q8\s*=\s*(-?\d+)", src).group(1))
    t["dyn_dts"] = define("DDE_EMU_DYN_DTS_BITS")
    t["counter_mod"] = define("DDE_EMU_COUNTER_MOD")
    return t


def load_vectors(path):
    out = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        a, b, h = line.split()
        out.append((int(a), int(b), bytes.fromhex(h)))
    return out


def fields(raw):
    ind = raw[0] >> 3
    fid = ((raw[0] & 7) << 8) | raw[1]
    lw = raw[2] >> 1
    hc = ((raw[2] & 1) << 10) | (raw[3] << 2) | (raw[4] >> 6)
    cyc = raw[4] & 63
    return ind, fid, lw, hc, cyc, raw[5:-3], (raw[-3] << 16) | (raw[-2] << 8) | raw[-1]


def counter_of(t, ti, cycle, payload):
    for (tg, cm, cb, se, k) in t["e2e"]:
        if tg == ti and cm == (cycle & 3):
            return payload[cb + 1] & 15
    return 0


def build_from_tables(t, ti, cycle, counter, keep_sync):
    cap = t["ind"][ti][cycle]
    data = bool(cap & 4)
    ind = m.emu_indicators(cap, data, keep_sync)
    p = bytearray(t["payload"][ti][cycle])
    if data:
        for (tg, cm, cb, se, k) in t["e2e"]:
            if tg == ti and cm == (cycle & 3):
                p[cb + 1] = (p[cb + 1] & 0xF0) | (counter & 15)
                p[cb] = m.crc8_e2e(bytes(p[cb + 1:se + 1])) ^ k
    return m.build_frame(ind, t["ids"][ti], cycle, bytes(p))


# ------------------------------------------------------------------ 1 + 2
def test_vectors(t, vecs, label, strict):
    n = nh = nf = ne = nb = ndiff = 0
    for fid, cyc, raw in vecs:
        ind, fid2, lw, hc, cyc2, payload, fc = fields(raw)
        n += 1
        nh += check(m.header_crc((ind >> 1) & 1, ind & 1, fid, lw) == hc, "%s hcrc %x c%d" % (label, fid, cyc))
        nf += check(m.frame_crc(raw[:-3]) == fc, "%s fcrc %x c%d" % (label, fid, cyc))
        if fid in t["ids"] and ind & 4:
            ok, ctrs = m.e2e_check(payload, fid, cyc)
            ne += check(ok and len(ctrs) > 0 and len(set(ctrs)) == 1 and max(ctrs) < m.COUNTER_MOD,
                        "%s e2e %x c%d" % (label, fid, cyc))
        if fid in t["ids"]:
            ti = t["ids"].index(fid)
            ctr = counter_of(t, ti, cyc, payload)
            b = build_from_tables(t, ti, cyc, ctr, True)
            nb += 1
            if b != raw:
                ndiff += 1
                if strict:
                    check(False, "%s rebuild %x c%d\n  got %s\n  cap %s" % (label, fid, cyc, b.hex(), raw.hex()))
            else:
                e = build_from_tables(t, ti, cyc, ctr, False)
                ei, _, _, ehc, ec, ep, efc = fields(e)
                check(ei == (ind & 0x1C) and ehc == m.header_crc(0, 0, fid, lw) and ec == cyc and ep == payload
                      and efc == m.frame_crc(e[:-3]), "%s emu header %x c%d" % (label, fid, cyc))
    print("%s: %d frames, header CRC %d/%d, frame CRC %d/%d, E2E %d/%d data frames, "
          "table rebuild %d/%d identical%s" % (label, n, nh, n, nf, n, ne, ne, nb - ndiff, nb,
                                               "" if strict else " (other vehicle states differ: informational)"))
    return n


# ------------------------------------------------------------------ 3: PIO
def assemble(pioasm):
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "x.h")
        subprocess.run([pioasm, "-o", "c-sdk", PIO_SRC, out], check=True)
        src = open(out).read()
    progs = {}
    for name in ("dde_emu_tx", "dde_emu_fire"):
        body = re.search(r"static const uint16_t %s_program_instructions\[\] = \{(.*?)\};" % name, src, re.S).group(1)
        progs[name] = [int(x, 16) for x in re.findall(r"0x([0-9a-f]{4})", body)]
        wt = int(re.search(r"#define %s_wrap_target (\d+)" % name, src).group(1))
        wr = int(re.search(r"#define %s_wrap (\d+)" % name, src).group(1))
        progs[name + "_wrap"] = (wt, wr)
    return progs


TXEN, TXD = 27, 28


def sim_frame(progs, words, delay, push_at=50, max_ticks=6000):
    sim = m.PioSim()
    tx = sim.add_sm(progs["dde_emu_tx"], 0, TXEN, 2, TXD, 1)
    tx["wrap_bottom"], tx["wrap_top"] = progs["dde_emu_tx_wrap"]
    fire = sim.add_sm(progs["dde_emu_fire"], 0, 0, 0, 0, 0)
    fire["wrap_bottom"], fire["wrap_top"] = progs["dde_emu_fire_wrap"]
    tx["fifo"] = list(words)
    trace = []
    for tick in range(max_ticks):
        if tick == push_at:
            fire["fifo"].append(delay)
        sim.step()
        trace.append((sim.pins[TXEN], sim.pins[TXD]))
        if tick > push_at + delay + 50 and tx["pc"] == 0 and not tx["fifo"] and trace[-1] == (1, 1) \
                and any(s[0] == 0 for s in trace):
            break
    return trace, tx, push_at


def test_pio(t, vecs, pioasm):
    progs = assemble(pioasm)
    print("pioasm: dde_emu_tx %d instructions, dde_emu_fire %d (pio0 has 32)" %
          (len(progs["dde_emu_tx"]), len(progs["dde_emu_fire"])))
    check(len(progs["dde_emu_tx"]) + len(progs["dde_emu_fire"]) <= 32, "fits pio0")
    # a sample of captured frames: every target in cycles 0..7 (data + null) + the 0x5C frames
    picked, seen = [], set()
    for fid, cyc, raw in vecs:
        key = (fid, cyc)
        if fid in t["ids"] and cyc < 8 and key not in seen:
            seen.add(key)
            picked.append((fid, cyc, raw, 0))
        if fid == t["dyn_id"] and (cyc & 15) == 1 and ("dyn", cyc) not in seen:
            seen.add(("dyn", cyc))
            picked.append((fid, cyc, raw, t["dyn_dts"]))
    nok = 0
    consts = set()
    for k, (fid, cyc, raw, dts) in enumerate(picked):
        words = m.pack_words(raw, TSS, dts)
        check(len(words) <= 8 or dts, "static frame fits the 8-word FIFO")
        delay = [20, 137, 600, 1999][k % 4]
        trace, tx, push_at = sim_frame(progs, words, delay)
        try:
            tss, data, trailer, start = m.decode_wire(trace, TXEN, TXD)
        except AssertionError as e:
            check(False, "decode %x c%d: %s" % (fid, cyc, e))
            continue
        exp_trailer = [0, 1] + ([0] * dts + [1] if dts else [])
        ok = check(tss == TSS, "TSS %d bits (%x c%d)" % (tss, fid, cyc))
        ok &= check(data == raw, "PIO bytes %x c%d" % (fid, cyc))
        ok &= check(trailer == exp_trailer, "trailer %s (%x c%d)" % (trailer, fid, cyc))
        nbits = TSS + 1 + 10 * len(raw) + 2 + (dts + 1 if dts else 0)
        lo = sum(1 for s in trace if s[0] == 0)
        ok &= check(lo == nbits * 10, "TXEN low %d ticks, expected %d" % (lo, nbits * 10))
        ok &= check(tx["pc"] == 0 and trace[-1] == (1, 1), "TX SM back to idle, pins high")
        consts.add(start - push_at - delay)
        nok += ok
    check(len(consts) == 1, "fire latency constant over delays: %s" % sorted(consts))
    print("PIO encode: %d/%d frames (static + %d dynamic 0x5C) decoded bit-exact from the simulated "
          "TXEN/TXD trace; TXEN falls exactly delay + %s cycles after the fire word" %
          (nok, len(picked), sum(1 for p in picked if p[3]), sorted(consts)))
    # an already-set stale irq must not be possible: the TX SM waits only after loading
    return nok == len(picked)


# ------------------------------------------------------------------ 4: timing
def test_timing_math():
    # mirror of the C test_schedule(), plus wrap edge cases
    bus, txen, alarm = m.target_from_ref(0xFF00, 1000000, 3300 * 256, 25, 6)
    check(bus == (0xFF00 + 3300) & 0xFFFF and txen == (bus - 25) & 0xFFFF and alarm == 1000000 + 32 - 6, "schedule")
    check(m.fire_delay(10, 65530, 30) == -14, "fire delay wrap")
    for ref in (0, 1, 0x7FFF, 0xFFFF):
        for d in (2000, 3300, 92400, 65536 + 17):
            bus, _, _ = m.target_from_ref(ref, 0, d * 256, 0, 6)
            check(m.wrap16(bus - (ref + d)) == 0, "mod 2^16 %d %d" % (ref, d))


def test_timing_run5(t, path):
    import gen_dde_tables as g
    recs = g.load_records(path)
    t0 = recs[0][0]
    lo, hi = t0 + g.IGN_ON_FROM_S * 1e6, t0 + g.IGN_ON_TO_S * 1e6
    q = 25600.0  # q8 units per us
    refs = dict(zip(t["ref_id"], [o / q for o in t["ref_off_q8"]]))
    tg = dict(zip(t["ids"], [o / q for o in t["tgt_off_q8"]]))
    # group FR1 reference marks per bus cycle: cycle-start estimate (mark - offset)
    # clustered in time (a cycle is 5 ms; estimates of one cycle agree to < 1 us)
    import bisect
    fr1 = sorted((tt - refs[sid], cyc, sid, tt) for (tt, src, sid, cyc, ind, payload, hc, fc) in recs
                 if lo <= tt < hi and src == 1 and sid in refs)
    clusters = []   # [cs, cycle, {sid: mark}]
    for cs, cyc, sid, tt in fr1:
        if clusters and cs - clusters[-1][0] < 100.0 and clusters[-1][1] == cyc:
            clusters[-1][2][sid] = tt
        else:
            clusters.append([cs, cyc, {sid: tt}])
    starts = [c[0] for c in clusters]

    def cluster_for(cs, cyc):
        i = bisect.bisect_left(starts, cs - 100.0)
        while i < len(clusters) and clusters[i][0] <= cs + 100.0:
            if clusters[i][1] == cyc:
                return clusters[i][2]
            i += 1
        return None
    # ref -> next ref of the same cycle (what the firmware does for its targets)
    errs_rr = []
    for cs, cyc, d in clusters:
        ids = sorted(d, key=lambda s: refs[s])
        for x, y in zip(ids, ids[1:]):
            errs_rr.append((d[y] - (d[x] + refs[y] - refs[x])) * 1e3)
    # DDE frames (FR2) predicted from the nearest earlier FR1 ref (>= 20 us before) of the same cycle
    errs_d = []
    for (tt, src, sid, cyc, ind, payload, hc, fc) in recs:
        if not (lo <= tt < hi and src == 2 and sid in tg):
            continue
        d = cluster_for(tt - tg[sid], cyc)
        if not d:
            continue
        cands = [r for r in d if refs[r] <= tg[sid] - 20]
        if not cands:
            continue
        r = max(cands, key=lambda x: refs[x])
        errs_d.append((tt - (d[r] + tg[sid] - refs[r])) * 1e3)

    def st(e):
        return statistics.mean(e), statistics.pstdev(e), max(abs(x) for x in e)
    m1, s1, x1 = st(errs_rr)
    m2, s2, x2 = st(errs_d)
    print("timing ref->ref (FR1, adjacent refs): n=%d mean %+.1f ns std %.1f ns max|e| %.0f ns" % (len(errs_rr), m1, s1, x1))
    print("timing DDE frames predicted from car refs: n=%d mean %+.1f ns std %.1f ns max|e| %.0f ns "
          "(the real DDE's own jitter is ~53 ns std in the slot table; mean = its -42.5 ns residual "
          "vs our -23.8 ns aim)" %
          (len(errs_d), m2, s2, x2))
    check(abs(m1) < 10 and s1 < 30, "ref->ref prediction")
    # our prediction error (ref->ref) is ~9 ns std; the real DDE wanders ~+-270 ns around the
    # same prediction, i.e. receivers already accept far more than our error.
    check(abs(m2) < 60 and s2 < 80 and x2 < 400, "DDE prediction")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=DEF_BIN)
    ap.add_argument("--pioasm", default=os.environ.get("PIOASM", "/workspace/host-pioasm-install/pioasm/pioasm"))
    a = ap.parse_args()
    t = parse_tables()
    check(t["counter_mod"] == m.COUNTER_MOD and
          sorted((t["ids"][g[0]],) + tuple(g[1:]) for g in t["e2e"]) == sorted(tuple(g) for g in m.E2E_GROUPS),
          "E2E groups: C tables == Python model")
    vecs = load_vectors(VECTORS)
    test_vectors(t, vecs, "vectors (2 s, committed)", strict=True)
    if os.path.exists(a.bin):
        import gen_dde_tables as g
        recs = g.load_records(a.bin)
        allv = [(r[2], r[3], g.raw_frame(r)) for r in recs]
        n = test_vectors(t, allv, "run 5 all frames (FR1+FR2)", strict=False)
        print("  (%d frames)" % n)
    else:
        print("run 5 capture not found (%s): full-run checks skipped" % a.bin)
    if os.path.exists(a.pioasm):
        test_pio(t, vecs, a.pioasm)
    else:
        check(False, "pioasm not found: %s (set --pioasm or PIOASM)" % a.pioasm)
    test_timing_math()
    if os.path.exists(a.bin):
        test_timing_run5(t, a.bin)
    print("test_dde_emu.py: %s" % ("all OK" if not fails else "%d FAILURES" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
