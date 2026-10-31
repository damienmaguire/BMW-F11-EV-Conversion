#!/usr/bin/env python3
"""Generate src/dde_emu_tables.h (and host-test vectors) from capture run 5.

Inputs (defaults are the paths on the analysis box):
  --bin    f11_run5.csv.bin  raw bulk-IN record stream written by fr_ts_logger.py
  --slots  f11_schedule_run5_slots.csv  fr_schedule_analyze.py slot table (offsets, residuals)

What it extracts:
  * DDE static frames 0x26/0x28/0x29/0x3D as received on FR2 (DDE side) in one
    64-cycle window of the ign-on / engine-off phase (rel 16.4..60 s; ign on at
    6.4 s, engine start at 62.44 s). The window is the first cycle-0 aligned block
    after --window-start where every (slot, cycle) frame is present and every data
    frame equals the phase's modal content (alive counters and CRC bytes masked).
    The table keeps the captured bytes; the firmware overwrites the counter
    nibbles and CRC bytes live.
  * Header templates (indicators, length) of those frames.
  * DDE dynamic frame 0x5C (minislot 0, cycles 1 mod 16) for the optional
    EMU_DYN_5C build.
  * Timing: expected TXEN-mark offset of every reference slot and target slot
    within the cycle (device ticks, 1/256 resolution), from the run 5 slot fit.
Every E2E CRC in the run (not just the window) is re-checked with the model.
"""
import argparse, collections, csv, os, statistics, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ts_capture"))
import dde_emu_model as m  # noqa: E402
import frts  # noqa: E402

TARGETS = [0x26, 0x28, 0x29, 0x3D]
DYN_ID = 0x5C
IGN_ON_FROM_S, IGN_ON_TO_S = 16.4, 60.0      # rel to the first frame of run 5
REF_STD_MAX_NS = 15.0
REF_RES_MAX_NS = 60.0
MINISLOT_US_DEFAULT = 6.875744
DEF_BIN = "/workspace/f11-ts-run5/f11_run5.csv.bin"
DEF_SLOTS = "/workspace/f11-ts-run5/f11_schedule_run5_slots.csv"


def load_records(path):
    p = frts.RecordParser()
    clk = frts.DeviceClock()
    out = []
    with open(path, "rb") as f:
        while True:
            c = f.read(1 << 20)
            if not c:
                break
            for r in p.feed(c):
                if r["kind"] != "fr":
                    continue
                t = clk.time(r["t_us_raw"], r["t_fine"], True)
                out.append((t, r["source"], r["slot_id"], r["cycle"], r["indicators"],
                            r["payload"], r["header_crc"], r["frame_crc"]))
    return out


def masked(fid, cyc, payload):
    p = bytearray(payload)
    for (_, _, cb, _, _) in m.e2e_groups_for(fid, cyc):
        p[cb] = 0
        p[cb + 1] &= 0xF0
    return bytes(p)


def raw_frame(r):
    t, src, sid, cyc, ind, payload, hc, fc = r
    lw = len(payload) // 2
    h = bytes([(ind << 3) | (sid >> 8), sid & 0xFF, (lw << 1) | (hc >> 10), (hc >> 2) & 0xFF,
               ((hc & 3) << 6) | cyc])
    return h + payload + bytes([fc >> 16, (fc >> 8) & 0xFF, fc & 0xFF])


def c_bytes(b):
    return ", ".join("0x%02X" % x for x in b)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", default=DEF_BIN)
    ap.add_argument("--slots", default=DEF_SLOTS)
    ap.add_argument("--window-start", type=float, default=30.0, help="s rel, search start")
    ap.add_argument("--out", default=os.path.join(HERE, "..", "..", "src", "dde_emu_tables.h"))
    ap.add_argument("--vectors", help="write ALL DDE frames of the run as text vectors here")
    ap.add_argument("--vectors-small", help="write the window's frames (+ a few hundred more) here")
    a = ap.parse_args()

    recs = load_records(a.bin)
    t0 = recs[0][0]
    dde = [r for r in recs if r[1] == 2]
    print("records %d, DDE (FR2) %d" % (len(recs), len(dde)))

    # ---- 1. E2E + CRC check over the whole run (model)
    bad = collections.Counter()
    n_e2e = 0
    for r in dde:
        t, src, sid, cyc, ind, payload, hc, fc = r
        f = m.build_frame(ind, sid, cyc, payload)
        if f != raw_frame(r):
            bad["frame/header crc"] += 1
        if sid in TARGETS and ind & 4:
            ok, _ = m.e2e_check(payload, sid, cyc)
            n_e2e += 1
            if not ok:
                bad["e2e"] += 1
    print("E2E checked %d data frames, CRC/E2E mismatches: %s" % (n_e2e, dict(bad) or "none"))
    if bad:
        sys.exit("model does not reproduce the capture")

    # ---- 2. modal content in the ign-on / engine-off phase
    phase = [r for r in dde if IGN_ON_FROM_S <= (r[0] - t0) / 1e6 <= IGN_ON_TO_S]
    modal = {}
    for sid in TARGETS:
        for cm in range(4):
            c = collections.Counter((r[4], masked(sid, r[3], r[5])) for r in phase
                                    if r[2] == sid and r[3] % 4 == cm)
            modal[(sid, cm)] = c.most_common(1)[0][0]
    # ---- 3. window: first cycle-0 block after window_start, complete and modal
    by_cycle_start = []
    idx = {}
    for i, r in enumerate(phase):
        if r[2] in TARGETS:
            idx.setdefault(r[2], []).append(i)
    starts = [r for r in phase if r[2] == 0x26 and r[3] == 0 and (r[0] - t0) / 1e6 >= a.window_start]
    window = None
    for s in starts:
        tw0 = s[0] - 100.0
        blk = [r for r in phase if tw0 <= r[0] < tw0 + 64 * 4999.93 and r[2] in TARGETS]
        got = {(r[2], r[3]): r for r in blk}
        if len(got) != 64 * len(TARGETS):
            continue
        if all((got[(sid, c)][4], masked(sid, c, got[(sid, c)][5])) == modal[(sid, c % 4)]
               for sid in TARGETS for c in range(64)):
            window = got
            wt = (tw0 - t0) / 1e6
            break
    if window is None:
        sys.exit("no clean 64-cycle window found")
    print("window: rel %.3f s .. %.3f s" % (wt, wt + 0.32))

    # ---- 4. dynamic 0x5C (DDE): template + cycles
    d5c = [r for r in phase if r[2] == DYN_ID]
    d5c_cyc = sorted(set(r[3] for r in d5c))
    d5c_tpl = collections.Counter((r[4], r[5]) for r in d5c).most_common(1)[0][0]
    print("DDE 0x5C cycles", d5c_cyc, "payload", d5c_tpl[1].hex(), "ind", d5c_tpl[0])
    assert all(c % 16 == 1 for c in d5c_cyc)

    # ---- 5. timing from the slot table
    slots = {}
    for row in csv.DictReader(open(a.slots)):
        slots[(row["channel"], int(row["slot_id"]), row["segment"])] = row
    refs = []
    for (ch, sid, seg), row in sorted(slots.items(), key=lambda kv: kv[0][1]):
        if ch != "FR1" or seg != "static":
            continue
        std = float(row["std_us"]) * 1e3
        res = float(row["residual_ns"])
        if int(row["repetition"]) == 1 and int(row["cycles_seen"]) == 64 and std <= REF_STD_MAX_NS \
                and abs(res) <= REF_RES_MAX_NS:
            refs.append((sid, float(row["offset_us"]), std, res))
    res_target = statistics.median(r[3] for r in refs)
    tgt_off = {}
    for sid in TARGETS:
        row = slots[("FR2", sid, "static")]
        tgt_off[sid] = float(row["static_pos_us"]) + res_target / 1e3
    dde_res = statistics.median(float(slots[("FR2", s, "static")]["residual_ns"]) for s in TARGETS)
    drow = slots.get(("FR2", DYN_ID, "dynamic"))
    off5c = float(drow["offset_us"]) - dde_res / 1e3 + res_target / 1e3
    slot_us = float(slots[("FR1", 39, "static")]["static_pos_us"]) - float(slots[("FR1", 38 - 5, "static")]["static_pos_us"])
    slot_us /= 6.0
    print("refs:", ", ".join("0x%02X(%.0f ns)" % (r[0], r[3]) for r in refs))
    print("target residual %.1f ns, DDE FR2 residual %.1f ns, 0x5C offset %.3f us, slot %.5f us"
          % (res_target, dde_res, off5c, slot_us))

    def q8(us):
        return int(round(us * 100.0 * 256.0))

    # ---- 6. 0x5C DTS
    tss_default = 7
    n5c = 5 + len(d5c_tpl[1]) + 3
    frame_bits = tss_default + 1 + n5c * 10 + 2
    k = 1
    while k * MINISLOT_US_DEFAULT * 10 < frame_bits + 1:
        k += 1
    dts_bits = int(round(k * MINISLOT_US_DEFAULT * 10 - frame_bits))

    # ---- 7. write the header
    L = []
    w = L.append
    w("// GENERATED by tools/dde_emu/gen_dde_tables.py -- do not edit.")
    w("// Source: %s" % os.path.basename(a.bin))
    w("//         %s" % os.path.basename(a.slots))
    w("// Window: run 5 rel %.3f s .. %.3f s (ign on, engine off), 64 cycles from cycle 0," % (wt, wt + 0.32))
    w("// DDE side (FR2). Payload bytes are as captured; alive counters and E2E CRC")
    w("// bytes are recomputed by the firmware for every frame it sends.")
    w("#ifndef DDE_EMU_TABLES_H")
    w("#define DDE_EMU_TABLES_H")
    w("#include <stdint.h>")
    w("")
    w("#define DDE_EMU_N_STATIC %d" % len(TARGETS))
    w("#define DDE_EMU_STATIC_PAYLOAD_BYTES 16")
    w("static const uint16_t dde_emu_static_id[DDE_EMU_N_STATIC] = {%s};" % ", ".join("0x%02X" % s for s in TARGETS))
    w("")
    w("// Captured 5-bit header indicators per (target, cycle): bit4 reserved, bit3 payload")
    w("// preamble, bit2 null-frame indicator (1 = data), bit1 sync, bit0 startup.")
    w("static const uint8_t dde_emu_static_ind[DDE_EMU_N_STATIC][64] = {")
    for sid in TARGETS:
        w("    {%s}," % ", ".join("%d" % window[(sid, c)][4] for c in range(64)))
    w("};")
    w("")
    w("// Captured payload per (target, cycle). Null frames are all zero.")
    w("static const uint8_t dde_emu_static_payload[DDE_EMU_N_STATIC][64][DDE_EMU_STATIC_PAYLOAD_BYTES] = {")
    for sid in TARGETS:
        w("    { // 0x%02X" % sid)
        for c in range(64):
            w("        {%s}, // c%d" % (c_bytes(window[(sid, c)][5]), c))
        w("    },")
    w("};")
    w("")
    w("// Captured header CRC per target (with the captured sync/startup bits);")
    w("// used only by the self-test.")
    w("static const uint16_t dde_emu_static_captured_hcrc[DDE_EMU_N_STATIC] = {%s};"
      % ", ".join("0x%03X" % window[(sid, 0)][6] for sid in TARGETS))
    w("")
    w("// E2E groups: {target index, cycle mod 4, crc byte, span end (incl), K}")
    w("// CRC byte = CRC8_0x1D(payload[crc+1 .. end]) ^ K; payload[crc+1] low nibble = alive counter.")
    w("#define DDE_EMU_N_E2E %d" % len(m.E2E_GROUPS))
    w("static const uint8_t dde_emu_e2e[DDE_EMU_N_E2E][5] = {")
    for (sid, cm, cb, se, kk) in m.E2E_GROUPS:
        w("    {%d, %d, %d, %d, 0x%02X}, // 0x%02X c%d" % (TARGETS.index(sid), cm, cb, se, kk, sid, cm))
    w("};")
    w("#define DDE_EMU_COUNTER_MOD %d" % m.COUNTER_MOD)
    w("")
    w("// Timing: expected TXEN-mark offset from cycle start in 1/256 device ticks (10 ns).")
    w("// Refs: FR1 static slots sent every cycle with jitter <= %.0f ns and |residual| <= %.0f ns." % (REF_STD_MAX_NS, REF_RES_MAX_NS))
    w("// Targets: the slot grid plus the refs' median residual (%.1f ns), i.e. our echo is" % res_target)
    w("// aimed to look like an on-grid car node at the pico.")
    w("#define DDE_EMU_N_REFS %d" % len(refs))
    w("static const uint16_t dde_emu_ref_id[DDE_EMU_N_REFS] = {%s};" % ", ".join("0x%02X" % r[0] for r in refs))
    w("static const int32_t dde_emu_ref_off_q8[DDE_EMU_N_REFS] = {%s};" % ", ".join("%d" % q8(r[1]) for r in refs))
    w("static const int32_t dde_emu_static_off_q8[DDE_EMU_N_STATIC] = {%s};" % ", ".join("%d" % q8(tgt_off[s]) for s in TARGETS))
    w("#define DDE_EMU_SLOT_Q8 %d  // static slot, %.5f us" % (q8(slot_us), slot_us))
    w("")
    w("// Dynamic frame 0x5C (EMU_DYN_5C builds only): minislot 0, cycles 1 mod 16.")
    w("#define DDE_EMU_DYN_ID 0x%02X" % DYN_ID)
    w("#define DDE_EMU_DYN_IND %d" % d5c_tpl[0])
    w("#define DDE_EMU_DYN_CYCLE_MASK 15")
    w("#define DDE_EMU_DYN_CYCLE_BASE 1")
    w("#define DDE_EMU_DYN_PAYLOAD_BYTES %d" % len(d5c_tpl[1]))
    w("static const uint8_t dde_emu_dyn_payload[DDE_EMU_DYN_PAYLOAD_BYTES] = {%s};" % c_bytes(d5c_tpl[1]))
    w("static const int32_t dde_emu_dyn_off_q8 = %d; // %.3f us" % (q8(off5c), off5c))
    w("#define DDE_EMU_DYN_DTS_BITS %d // DTS low bits for TSS=%d: frame ends at minislot %d AP" % (dts_bits, tss_default, k))
    w("")
    w("#endif")
    open(a.out, "w").write("\n".join(L) + "\n")
    print("wrote", os.path.normpath(a.out))

    # ---- 8. vectors
    def write_vec(path, rows):
        with open(path, "w") as f:
            f.write("# id cycle raw_frame_hex (header+payload+crc), DDE side of run 5\n")
            for r in rows:
                f.write("%d %d %s\n" % (r[2], r[3], raw_frame(r).hex()))
        print("wrote %d vectors to %s" % (len(rows), path))
    if a.vectors:
        write_vec(a.vectors, dde)
    if a.vectors_small:
        t_lo = wt * 1e6 + t0
        small = [r for r in dde if t_lo - 1e6 <= r[0] < t_lo + 1e6]  # 2 s around the window
        write_vec(a.vectors_small, small)


if __name__ == "__main__":
    main()
