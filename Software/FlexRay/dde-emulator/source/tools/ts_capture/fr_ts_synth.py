#!/usr/bin/env python3
"""Synthetic FlexRay capture in the exact TS-CAPTURE wire format (record format 3).

Builds a schedule from known cluster parameters, places static frames at
(id-1)*gdStaticSlot + gdActionPointOffset and dynamic frames on minislot
action points (minislot counting with real frame lengths), converts the
times to what the firmware would latch (device crystal ppm error, timestamp
jitter, 1 MHz + 100 MHz counters, 32-bit us wrap), encodes format-3 records
and writes the raw stream plus a JSON file with the truth.

  python3 fr_ts_synth.py --out /tmp/synth.bin
  python3 fr_ts_logger.py --replay /tmp/synth.bin -o /tmp/synth.csv
  python3 fr_schedule_analyze.py /tmp/synth.csv --truth /tmp/synth.bin.truth.json
"""
import argparse
import json
import random
import struct

BIT_US = 0.1  # 10 Mbit/s


def frame_bits(payload_bytes, tss_bits=9):
    # TSS + FSS + (5 header + payload + 3 CRC) bytes * 10 bits (BSS + 8) + FES
    return tss_bits + 1 + (5 + payload_bytes + 3) * 10 + 2


SCENARIOS = {
    # MT = 1 us, 5000 MT cycle, 75 static slots
    "a": dict(mt_us=1.0, cycle_mt=5000, n_static=75, static_slot_mt=32, ap_mt=3, payload_words=8,
              minislot_mt=7, mini_ap_mt=2, n_minislots=290),
    # BMW-like guess with a non-integer macrotick
    "b": dict(mt_us=1.375, cycle_mt=3636, n_static=70, static_slot_mt=24, ap_mt=4, payload_words=8,
              minislot_mt=6, mini_ap_mt=2, n_minislots=200),
}


def build(args, p):
    rnd = random.Random(args.seed)
    MT = p["mt_us"]
    T = p["cycle_mt"] * MT
    S = p["static_slot_mt"] * MT
    M = p["minislot_mt"] * MT
    N = p["n_static"]
    dyn_start = N * S
    plw = p["payload_words"]

    # static slots in use: ids 1..N with gaps, each on FR1 (vehicle) or FR2 (DDE)
    dde_static = {0x26, 0x28, 0x29, 0x3D}
    static_ids = sorted(set(rnd.sample(range(1, N + 1), min(N, 45))) | (dde_static & set(range(1, N + 1))))
    static_cfg = {}
    for sid in static_ids:
        rep = rnd.choice([1, 1, 1, 2, 4])
        static_cfg[sid] = dict(ch=2 if sid in dde_static else rnd.choice([1, 1, 2]), rep=rep,
                               base=rnd.randrange(rep), null_other=rnd.random() < 0.6)
    # dynamic: ids above N, mixed lengths and send probabilities
    dyn_ids = sorted(set(rnd.sample(range(N + 1, N + 1 + p["n_minislots"] // 2), 25)))
    for extra in (0x68, 0xE6, 0xE8, 0xFB, 0x142):
        if N < extra < N + p["n_minislots"] // 2:
            dyn_ids.append(extra)
    dyn_ids = sorted(set(dyn_ids))
    dyn_cfg = {d: dict(ch=rnd.choice([1, 2]), words=rnd.choice([1, 4, 8, 15, 16]),
                       prob=rnd.choice([1.0, 1.0, 0.5, 0.25, 0.1])) for d in dyn_ids}

    frames = []  # (flexray_time_us, ch, id, cycle, payload bytes, indicators)
    for n in range(args.cycles):
        c0 = n * T
        cyc = (n + args.first_cycle) & 63
        for sid in static_ids:
            cfg = static_cfg[sid]
            t = c0 + (sid - 1) * S + p["ap_mt"] * MT
            data_cycle = (cyc % cfg["rep"]) == cfg["base"]
            if not data_cycle and not cfg["null_other"]:
                continue  # some slots are just not sent on other cycles (not FlexRay-correct, tests robustness)
            payload = bytes(rnd.randrange(256) for _ in range(2 * plw)) if data_cycle else bytes(2 * plw)
            ind = 0x04 if data_cycle else 0x00  # NFI=1 data, 0 null
            if sid in (1, 2):
                ind |= 0x03  # sync + startup
            frames.append((t, cfg["ch"], sid, cyc, payload, ind))
        # dynamic segment: minislot counter
        ms = 0
        for slot in range(N + 1, N + 1 + p["n_minislots"]):
            if ms >= p["n_minislots"]:
                break
            cfg = dyn_cfg.get(slot)
            if cfg and rnd.random() < cfg["prob"]:
                nbytes = 2 * cfg["words"]
                dur = frame_bits(nbytes) * BIT_US + 1.1  # + channel idle delimiter
                need = int(-(-(p["mini_ap_mt"] * MT + dur) // M)) + 1  # minislots spanned (+DTS)
                if ms + need > p["n_minislots"]:
                    ms += 1
                    continue  # pLatestTx: would not fit
                t = c0 + dyn_start + ms * M + p["mini_ap_mt"] * MT
                frames.append((t, cfg["ch"], slot, cyc, bytes(rnd.randrange(256) for _ in range(nbytes)), 0x04))
                ms += need
            else:
                ms += 1
    frames.sort(key=lambda f: f[0])
    return frames, dict(static_ids=static_ids, dyn_ids=dyn_ids, static_cfg=static_cfg)


def encode_v3(source, flags, t_us, t_fine, seq, slot, cyc, ind, payload):
    words = len(payload) // 2
    h0 = ((ind & 0x1F) << 3) | ((slot >> 8) & 7)
    hdr = bytes([h0, slot & 0xFF, (words << 1) & 0xFE, 0x55, 0x40 | (cyc & 0x3F)])
    body = bytes([3, flags, source]) + struct.pack("<IHI", t_us & 0xFFFFFFFF, t_fine & 0xFFFF, seq) \
        + hdr + payload + b"\x12\x34\x56"
    return struct.pack("<H", 0x8000 | len(body)) + body


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="raw stream output (.bin); truth goes to <out>.truth.json")
    ap.add_argument("--scenario", default="a", choices=sorted(SCENARIOS))
    ap.add_argument("--cycles", type=int, default=400)
    ap.add_argument("--first-cycle", type=int, default=13)
    ap.add_argument("--ppm", type=float, default=37.0, help="device crystal error vs. the FlexRay cluster")
    ap.add_argument("--jitter-ns", type=float, default=30.0, help="timestamp jitter (uniform +-)")
    ap.add_argument("--start-us", type=int, default=0xFFFFFFFF - 700_000, help="device us at start (tests wrap)")
    ap.add_argument("--drop-every", type=int, default=997, help="drop every Nth record (USB FIFO drop, seq gap)")
    ap.add_argument("--no-hw-every", type=int, default=0, help="every Nth record without hw timestamp")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)
    p = SCENARIOS[args.scenario]
    frames, meta = build(args, p)
    rnd = random.Random(args.seed + 1000)

    out = bytearray()
    seq = 0
    dropped = 0
    base = args.start_us + 0.123  # arbitrary sub-us phase
    k = 1.0 + args.ppm * 1e-6
    for i, (t, ch, slot, cyc, payload, ind) in enumerate(frames):
        dev = base + t * k + rnd.uniform(-1, 1) * args.jitter_ns * 1e-3
        fine = int(dev * 100) & 0xFFFF          # 100 MHz clk_sys counter latched first
        us = int(dev + rnd.uniform(0, 0.05))    # 1 MHz timer latched ~0-50 ns later
        hw = not (args.no_hw_every and i % args.no_hw_every == args.no_hw_every - 1)
        flags = 0x03 | (0x04 if hw else 0)
        if not hw:
            fine = 0
            us = int(dev + 3.0)  # CPU parse time fallback
        s = seq
        seq += 1
        if args.drop_every and i % args.drop_every == args.drop_every - 1:
            dropped += 1
            continue
        out += encode_v3(ch, flags, us, fine, s, slot, cyc, ind, payload)

    with open(args.out, "wb") as f:
        f.write(out)
    MT = p["mt_us"]
    truth = dict(p, scenario=args.scenario, ppm=args.ppm, cycle_us=p["cycle_mt"] * MT,
                 static_slot_us=p["static_slot_mt"] * MT, minislot_us=p["minislot_mt"] * MT,
                 cycle_us_device=p["cycle_mt"] * MT * k, static_slot_us_device=p["static_slot_mt"] * MT * k,
                 minislot_us_device=p["minislot_mt"] * MT * k,
                 records=len(frames) - dropped, dropped=dropped, static_ids=meta["static_ids"],
                 dyn_ids=meta["dyn_ids"], static_cfg={str(k): v for k, v in meta["static_cfg"].items()},
                 # frame start relative to static slot 1's frame start (cycle start + AP), FlexRay time
                 static_offsets_us={str(s): (s - 1) * p["static_slot_mt"] * MT for s in meta["static_ids"]})
    with open(args.out + ".truth.json", "w") as f:
        json.dump(truth, f, indent=1)
    print("wrote %s (%d records, %d dropped, %d bytes) and %s.truth.json"
          % (args.out, len(frames) - dropped, dropped, len(out), args.out))


if __name__ == "__main__":
    main()
