#!/usr/bin/env python3
"""Build f11_flexray_dde.dbc from Fred's f11_flexray_dde_signals.csv.

Encoding (SavvyCAN FlexRay convention): message ID = slot ID, byte 0 = cycle
count, bytes 1.. = FlexRay payload. So CSV "payload byte n" = DBC byte n+1 and
the Intel start bit is 8*(n+1) + start_bit. Cycle-dependent layouts are
multiplexed on low bits of byte 0 (see MUX below).
"""
import csv, collections, os

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "f11_flexray_dde_signals.csv")
OUT = os.path.join(HERE, "f11_flexray_dde.dbc")

# slot -> (mux signal name, bit length, {csv selector -> mux value}, DLC)
# selector is matched against the CSV cycle_mux / payload_len columns.
MUX = {
    0x26: ("CycleMod4", 2, {"c1": 1}),
    0x28: ("CycleMod4", 2, {"c1": 1, "c3": 3}),
    0x29: ("CycleMod4", 2, {"c3": 3}),
    0x3D: ("CycleMod4", 2, {"c3": 3}),
    0x5C: ("CycleMod16", 4, {"base1": 1}),
    # 0xE8: 8-byte frames in odd cycles, 4-byte frames in even cycles.
    0xE8: ("CycleParity", 1, {"L9": 1, "L5": 0}),
    # 0xFB: L7 c2/c6 mod 8 -> 2 mod 4, L9 c3/c7 -> 3 mod 4, L5 c5 (run 4) /
    # c1 (run 5) -> 1 mod 4 in both runs.
    0xFB: ("CycleMod4", 2, {"L7": 2, "L9": 3, "L5": 1}),
}
NAMES = {
    0x26: "DDE_FR_026", 0x28: "DDE_FR_028", 0x29: "DDE_FR_029",
    0x3D: "DDE_FR_03D_Sync", 0x5C: "DDE_FR_05C_NM", 0x68: "DDE_FR_068",
    0xBE: "DDE_FR_0BE_DiagResp", 0xE6: "DDE_FR_0E6", 0xE8: "DDE_FR_0E8",
    0xFB: "DDE_FR_0FB", 0x10A: "DDE_FR_10A", 0x142: "DDE_FR_142",
}
VALS = {  # from the notes column of the CSV / ptcan1 analysis
    (0x28, "GearEngagedCode_c3"): {0x22: "P", 0x25: "R", 0x24: "D"},
    (0xE6, "GearCode"): {3: "P", 2: "R", 5: "D", 1: "Park requested while in D"},
}

def q(s):
    return s.replace('"', "'").replace("\n", " ").strip()

def selector(slot, row):
    if slot not in MUX:
        return None
    keys = MUX[slot][2]
    text = row["cycle_mux"]
    for k, v in keys.items():
        if text.startswith(k) or text.startswith(k + " ") or text.split(":")[0].strip() == k:
            return v
    raise SystemExit(f"no mux value for {hex(slot)} {text!r}")

rows = list(csv.DictReader(open(SRC, newline="")))
msgs = collections.OrderedDict()
for r in rows:
    slot = int(r["frame_id"], 16)
    msgs.setdefault(slot, []).append(r)

out = ['VERSION "F11 DDE FlexRay (generated from f11_flexray_dde_signals.csv)"',
       "", "", "NS_ :", "\tCM_", "\tBA_DEF_", "\tBA_", "\tVAL_", "", "BS_:", "",
       "BU_: DDE VEHICLE", "", ""]
comments, vals, skipped = [], [], []
for slot, rs in msgs.items():
    lens = []
    for r in rs:
        for p in r["payload_len"].split("/"):
            lens.append(int(p))
    dlc = max(lens) + 1
    name = NAMES[slot]
    out.append(f"BO_ {slot} {name}: {dlc} DDE")
    if slot in MUX:
        m, bits, _ = MUX[slot]
        out.append(f' SG_ {m} M : 0|{bits}@1+ (1,0) [0|{2**bits-1}] "" VEHICLE')
    msg_notes = collections.OrderedDict()
    for r in rs:
        if not r["start_byte"].strip():
            skipped.append(f'{hex(slot)} {r["name"]}: {r["notes"]}')
            continue
        mv = selector(slot, r)
        sname = r["name"]
        if slot in MUX and len(MUX[slot][2]) > 1:  # several layouts: names need a suffix
            tag = [k for k, v in MUX[slot][2].items() if v == mv][0]
            if not sname.endswith(tag):
                sname = f"{sname}_{tag}"
        start = 8 * (int(r["start_byte"]) + 1) + int(r["start_bit"])
        length = int(r["length_bits"])
        scale, offset = r["scale"] or "1", r["offset"] or "0"
        unit = "" if r["unit"] in ("raw", "") else r["unit"]
        signed = r["signed"].strip().lower() == "yes"
        mux = f" m{mv}" if mv is not None else ""
        # DBC signals are limited to 64 bits: split longer raw/unused fields.
        parts = [(start, length, sname)] if length <= 64 else [
            (start + o, min(64, length - o), f"{sname}_{i}") for i, o in enumerate(range(0, length, 64))]
        for st, ln, nm in parts:
            sc, of = float(scale), float(offset)
            lo = of if not signed else -(2 ** (ln - 1)) * sc + of
            hi = (2 ** ln - 1) * sc + of if not signed else (2 ** (ln - 1) - 1) * sc + of
            out.append(f' SG_ {nm}{mux} : {st}|{ln}@1{"-" if signed else "+"} ({scale},{offset}) [{lo:g}|{hi:g}] "{unit}" VEHICLE')
            comments.append(f'CM_ SG_ {slot} {nm} "Confidence: {q(r["confidence"])}. Cycles: {q(r["cycle_mux"])}. {q(r["notes"])} Evidence: {q(r["evidence"])}";')
            if (slot, nm) in VALS:
                vals.append(f"VAL_ {slot} {nm} " + " ".join(f'{k} "{v}"' for k, v in VALS[(slot, nm)].items()) + " ;")
        msg_notes[r["cycle_mux"]] = r["payload_len"]
    out.append("")
    desc = "; ".join(f"payload {l} B in {c}" for c, l in msg_notes.items())
    comments.insert(0, f'CM_ BO_ {slot} "DDE FlexRay slot {hex(slot)}. {q(desc)}. DLC = largest payload + 1 (byte 0 = cycle)."')
    comments[0] += ";" if not comments[0].endswith(";") else ""

out.append("")
out.append('CM_ "BMW F11 520d DDE FlexRay frames, reverse engineered (Fred\'s analysis). Message ID = FlexRay slot ID. Byte 0 = cycle count, bytes 1.. = payload, as SavvyCAN decodes FlexRay frames. E2E: CRC8 poly 0x1D init 0 xorout 0 XOR per-group K, see f11_dde_e2e.md. Generated by make_flexray_dbc.py; edit the CSV, not this file.";')
out.append('CM_ BU_ DDE "Engine ECU, FlexRay node 0x12, coldstart sync node (sync frame 0x3D).";')
out.append('CM_ BU_ VEHICLE "Rest of the FlexRay cluster.";')
out += comments
out += vals
open(OUT, "w").write("\n".join(out) + "\n")
print(f"wrote {OUT}: {len(msgs)} messages")
for s in skipped:
    print("not encoded (no position):", s)
