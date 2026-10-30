# F11 DDE end-to-end protection (CRC + alive counter)

**Author and date:** Fred, 2026-10-30 (Europe/Dublin).

**Scope:** the BMW F11 520d DDE (EDC17C50), on FlexRay and on PT-CAN / PT-CAN2. Two EGS IDs are included for reference.

**Status:** the scheme is solved for every group listed below. It fits 100 % (or near 100 % on CAN, see section 5) on every log tested.

Byte numbering is defined as follows:
- **FlexRay "payload byte n":** byte n of the frame payload, where 0 is the first payload byte. It equals SavvyCAN `data[n+1]`, because SavvyCAN `data[0]` is the cycle counter.
- **CAN "dN":** CAN data byte N, 0-based.

## 1. Algorithm

```
crc  = CRC8(span, poly 0x1D, init 0x00, xorout 0x00, not reflected)   # SAE J1850 polynomial
byte = crc XOR K          # K is a constant per signal group / CAN ID
```

- **Layout:** each protected group is laid out as `[CRC][counter byte][data …]`.
- **Span:** the span starts at the counter byte, which comes right after the CRC byte, and runs through the last protected byte (see the tables).
- **Unprotected bytes:** some groups end before the end of the 8-byte area. Bytes after the span are not protected, and including them breaks the fit.
- **Data-ID / init / xorout:**
  - A CRC is affine, so any init value, any final XOR, and any prepended data-ID bytes (8- or 16-bit) all collapse into the single constant K.
  - The fit was brute-forced that way, so whether BMW uses a data-ID does not matter for generating valid frames.
  - The real data-ID values are therefore **not known**, and they are not needed.
- **Implementation check:**
  - `CRC8("123456789")` with these parameters = **0x37**.
  - `CRC8([0x31])` = 0x57.

```python
def crc8_1d(data):
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x1D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c
```

## 2. Alive counter

- **Position:** the low nibble of the byte right after the CRC.
- **Range:** 0..14 (0x0..0xE). 15 is never used. It wraps 14 → 0 and steps +1 per transmission of that group.
- **Start value:** every wake starts at **1**.
  - Shown on the FlexRay static groups in ts run 4: 1 at first frame, with no resets during the run.
  - Shown on the MITM logs.
- **High nibble:** fixed per group and **inside the CRC span**, so it must be reproduced exactly.
  - It is `0xF` for 0x26 c1 group 1 and 0x28 c1 group 1.
  - It is `0x1` for all other DDE groups.
  - Exception: in running logs only, bit 0x20 is set (high nibble 1 → 3) for about 60–80 ms around crank. Its meaning is unknown.
- **Event-driven frames:** 0xE6 / CAN 0x3F9 count per transmission. Extra frames sent on change also advance the counter.
- **CAN copies:** the CAN copies of FlexRay groups (0x0A5, 0x0D9, 0x08F, 0x145, 0x0DC, 0x3F9) use the same K and layout. Their counters run **independently** of the FlexRay counters.
- **Other counters, without a CRC:**
  - FR 0xFB L7: payload byte 0 low nibble 0..14, high nibble F.
  - FR 0xFB L5: payload byte 0 low nibble 0..14, high nibble 4 (first frame per wake `01`).
  - FR 0x142: payload byte 0 is a full-byte counter 0..254.
  - No CRC was found in these frames; no byte in them fits any CRC8, XOR or sum.

## 3. K tables

### 3.1 FlexRay (DDE transmit, static segment, 16-byte payload = two 8-byte groups; plus 0xE6)

| Frame | Cycle mux | Group | CRC byte | Counter byte (hi nibble) | CRC span | K | Notes |
|---|---|---|---|---|---|---|---|
| 0x26 | c1 (mod 4) | 1 | 0 | 1 (F) | 1..3 | 0xA8 | Payload bytes 4..15 are always FF (unused) |
| 0x28 | c1 | 1 | 0 | 1 (F) | 1..7 | 0x6A | Same PDU as CAN 0x0A5 |
| 0x28 | c1 | 2 | 8 | 9 (1) | 9..15 | 0xD7 | Same PDU as CAN 0x0D9 |
| 0x28 | c3 | 1 | 0 | 1 (1) | 1..6 | 0xE4 | Byte 7 (FF) is not covered; no CAN copy found |
| 0x28 | c3 | 2 | 8 | 9 (1) | 9..13 | 0x0F | Same as CAN 0x0DC; bytes 14..15 (FF) are not covered |
| 0x29 | c3 | 1 | 0 | 1 (1) | 1..7 | 0x3C | Same as CAN 0x08F |
| 0x29 | c3 | 2 | 8 | 9 (1) | 9..15 | 0x69 | No CAN copy found |
| 0x3D | c3 | 1 | 0 | 1 (1) | 1..7 | 0x0C | Same as CAN 0x145 (PT-CAN2) |
| 0x3D | c3 | 2 | 8 | 9 (1) | 9..11 | 0x5C | Bytes 12..15 (FF) are not covered |
| 0xE6 | dynamic, mostly c2 mod 4 | 1 | 0 | 1 (1) | 1..7 | 0x04 | 8-byte payload; same as CAN 0x3F9 |

### 3.2 CAN (CRC in d0, counter in d1 low nibble)

| CAN ID | Bus(es) in F11 | Sender | Span | K | FlexRay twin |
|---|---|---|---|---|---|
| 0x0A5 | PT-CAN (20 ms) + PT-CAN2 (10 ms) | DDE (bench + car) | d1..d7 | 0x6A | 0x28 c1 g1 |
| 0x0D9 | PT-CAN (20 ms) + PT-CAN2 (10 ms) | DDE (bench + car) | d1..d7 | 0xD7 | 0x28 c1 g2 |
| 0x08F | PT-CAN (~30 ms) + PT-CAN2 (10 ms) | DDE (bench + car) | d1..d7 | 0x3C | 0x29 c3 g1 |
| 0x0DC | PT-CAN only (20 ms) | DDE (bench + car) | d1..d5 | 0x0F | 0x28 c3 g2 |
| 0x3F9 | both (~1 s + on change) | DDE (bench + car) | d1..d7 | 0x04 | 0xE6 |
| 0x145 | PT-CAN2 only (10 ms) | DDE by layout/K; **not in the F10 bench log** | d1..d7 | 0x0C | 0x3D c3 g1 |
| 0x0F3 | PT-CAN only (20 ms) | DDE (F10 bench) | d1..d7 | 0x8F | none found |
| 0x1AF | PT-CAN2 only (20 ms) | **EGS** | d1..d7 | 0x43 | turbine re-packed in FR 0x26 |
| 0x3FD | both (100 ms) | **EGS** | d1..d4 | 0x70 | — |

These PT-CAN2 IDs have K values fitted, but their **sender is unproven**: 0x0A0 (0xCC), 0x0A6 (0xC5), 0x0A7 (0x23), 0x0B0 (0x65). They are listed for completeness only and left out of the DBCs.

## 4. Worked examples (real frames)

**FlexRay 0x28, cycle 1**
- Source: ptcan1 3krev log, bus 1, rpm ≈ 1005.
- SavvyCAN row bytes: `09 | fd f2 9a 38 87 b4 0f f1 | 17 12 00 10 00 e0 7f c0`.
  - `09` = data[0], the cycle counter, which is not payload.
  - Payload bytes 0..15 follow.
- **Group 1:**
  - CRC8(payload 1..7 = `f2 9a 38 87 b4 0f f1`) = 0x97.
  - 0x97 XOR K 0x6A = **0xFD** = payload byte 0. ✔
  - Counter: payload byte 1 = 0xF2 → high nibble F, counter 2.
  - EngineSpeed: payload bytes 5..6 LE = 0x0FB4 = 4020 → × 0.25 = 1005 rpm.
- **Group 2:**
  - CRC8(payload 9..15 = `12 00 10 00 e0 7f c0`) = 0xC0.
  - 0xC0 XOR K 0xD7 = **0x17** = payload byte 8. ✔
  - Counter: 0x12 → high nibble 1, counter 2.

**CAN 0x0A5**
- Source: PT-CAN, engine off.
- Frame: `f9 f1 c6 67 7c 00 00 f1`.
- CRC8(d1..d7 = `f1 c6 67 7c 00 00 f1`) = 0x93.
- 0x93 XOR 0x6A = **0xF9** = d0. ✔
- Counter: d1 = 0xF1 → counter 1.
- Torque1: bits 16..27 = 0x7C6 = 1990 → 1990 × 0.5 − 1023.5 = −28.5 Nm. EngineSpeed: 0 rpm.

All three results were recomputed with the function above (2026-10-30).

## 5. Validation summary

| Data set | What was checked | Result |
|---|---|---|
| 5 MITM logs (DDE side, old + new pico firmware) | All 10 FlexRay groups in 3.1, unique payloads per log 241 / 1003 / 1032 / 241 / 1383 (~15k unique payloads in total) | 100 % |
| Non-MITM F11_ignon / engstart (DDE IDs only) | 0x26 c1, 0x28 c1 | 100 % (240/240, 623/623) |
| ts run 4 (Sam's fixed timestamp build) | 9 static groups, ≈134k checks; counters start at 1, no resets | 100 % |
| ptcan1, 3 logs (ignonoff, 3krev, drive), FlexRay bus 1 | 0x26 c1 13412/13412; 0x28 c1 g1, g2 13261 each; 0x28 c3 g1, g2 13521 each; 0x29 c3 g1, g2 13659 each; 0x3D c3 g1, g2 13662 each; 0xE6 336/336 | 100 % |
| ptcan1, CAN bus 2 (PT-CAN2) | 0x0A5 27257/27260; 0x0D9 27246/27251; 0x08F 27256/27256; 0x145 27237/27239; 0x3F9 347/347; 0x1AF 13627/13628; 0x3FD 2738/2738 | ≥ 99.98 % |
| ptcan1, CAN bus 3 (PT-CAN) | 0x0A5 13628/13629; 0x0D9 13610/13610; 0x08F 9083/9085; 0x0DC 13612/13612; 0x3F9 350/350; 0x0F3 13608/13608; 0x3FD 2736/2736 | ≥ 99.98 % |
| F10 DDE-alone bench log | 0x08F, 0x0A5, 0x0D9, 0x0DC, 0x3F9, 0x0F3 | 100 % |

**Open point:** a few CAN frames fail the check, at most 5 per ID out of about 27k. These were **not investigated**; possible causes include logger corruption and frames merged by the ms-batched timestamps. The FlexRay side has no failures.

## 6. Not established
- Real BMW data-ID values (not needed, see section 1).
- Why the counter high nibble gains 0x20 at crank.
- Whether receivers check the counter sequence strictly, for example how they handle a gap or a restart at 1. This needs a bench test.
