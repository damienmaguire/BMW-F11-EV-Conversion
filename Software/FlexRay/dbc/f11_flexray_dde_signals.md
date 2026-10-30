# F11 DDE FlexRay signals (Fred, 2026-10-30, Europe/Dublin)

Machine-readable twin: `f11_flexray_dde_signals.csv` (same rows).

**Byte numbering:**
- `start_byte` counts from **0 = first payload byte**, which is SavvyCAN `data[1]`.
- In our SavvyCAN FlexRay CSVs, `data[0]` is the cycle counter. So payload byte *n* = SavvyCAN `data[n+1]`, and SavvyCAN `LEN` = payload_len + 1.
- Each row's notes also give the SavvyCAN `data[]` index.
- Older Fred notes write `bN`, meaning SavvyCAN `data[N]` = payload byte N-1.

**Bit numbering:**
- `start_bit` is the bit inside `start_byte` where the field's least significant bit sits (0 = LSB).
- Multi-byte fields are little-endian: Intel order, the next byte holds the more significant bits.
- Example: a 12-bit field at byte 2, bit 0 = byte2 + (byte3 & 0x0F) * 256.

**Physical value** = raw * scale + offset.

**Cycle mux:**
- `c1` = cycles where cycle mod 4 = 1. "base/rep" follows the FlexRay base-cycle / repetition convention.
- Static DDE slots are 0x26, 0x28, 0x29 and 0x3D. There are 91 static slots, and all static payloads are 16 B. 0x26 carries data only in c1 (nulls otherwise). 0x28 carries different content in c1 and c3. 0x29 and 0x3D carry data in c3.
- Dynamic DDE frames are 0xE6, 0xE8, 0xFB, 0x142, 0x5C, 0x10A, 0x68 and 0xBE. 0xE8 and 0xFB use several lengths, each with its own content.

**Confidence:**
- confirmed: matched to an independent label (OBD, or a CAN label of known meaning), or shown constant / fitted in every log.
- high: equal to a CAN field whose meaning is evidenced by gear or lever behaviour.
- medium / low / candidate: as stated in the evidence column.
- undecoded: raw bytes kept for completeness.

Pure speculation is left out.

E2E details are in `f11_dde_e2e.md`.


| frame_id | segment | payload_len | cycle_mux | start_byte | start_bit | length_bits | byte_order | signed | scale | offset | unit | name | confidence | evidence | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0x26 | static | 16 | c1 (base1/rep4); nulls in other cycles | 0 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 1..3, XOR K=0xA8. SavvyCAN data[1]. |
| 0x26 | static | 16 | c1 (base1/rep4); nulls in other cycles | 1 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[2]. |
| 0x26 | static | 16 | c1 (base1/rep4); nulls in other cycles | 1 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0xF; inside the CRC span. |
| 0x26 | static | 16 | c1 (base1/rep4); nulls in other cycles | 2 | 0 | 16 | little_endian | no | 1 | -2000 | rpm | TurbineSpeed | medium-high | ptcan1: equal to EGS PT-CAN2 0x1AF d3..4 (r 1.000, 0x1AF leads ~8 ms); obd1/obd2: rpm-related, NOT equal to PID 0x0C (idle bias -56 rpm); R/D test | SavvyCAN data[3..4]. raw-2000; 2000 (=0) when stopped. Turbine meaning inferred from behaviour (~rpm in P, ~150 R / ~350 D at creep with brake). EGS data re-packed by the DDE. |
| 0x26 | static | 16 | c1 (base1/rep4); nulls in other cycles | 4 | 0 | 96 | little_endian | no | 1 | 0 |  | Unused | confirmed | all logs | SavvyCAN data[5..16]. Always FF; not in CRC span. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 0 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 1..7, XOR K=0x6A. SavvyCAN data[1]. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 1 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[2]. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 1 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0xF; inside the CRC span. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 2 | 0 | 12 | little_endian | no | 0.5 | -1023.5 | Nm | EngineTorque1 | high (field) / hint (unit) | ptcan1: equal to CAN 0x0A5 bits16..27 (r 0.999 slope 1.0, same PDU) | SavvyCAN data[3] + low nibble data[4]. Scale/offset from a public hint; ~232 at crank, 25..50 idle, about -35 engine off. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 3 | 4 | 12 | little_endian | no | 0.5 | -1023.5 | Nm | EngineTorque2 | medium | ptcan1: equal to CAN 0x0A5 bits28..39 | SavvyCAN high nibble data[4] + data[5]. Same format as torque 1; role unknown. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 5 | 0 | 16 | little_endian | no | 0.25 | 0 | rpm | EngineSpeed | confirmed | obd1 + obd2: OBD PID 0x0C r 0.9998 (88..92 % exact in request window); ptcan1: = CAN 0x0A5 d5..6 | SavvyCAN data[6..7]. FR 25..145 ms behind OBD reply. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 7 | 0 | 8 | n/a | no | 1 | 0 |  | Byte7_const | confirmed | all logs | SavvyCAN data[8]. Constant 0xF1; inside CRC span. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 8 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 9..15, XOR K=0xD7. SavvyCAN data[9]. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 9 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[10]. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 9 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0x1 (0x3 for ~60..80 ms at crank); inside the CRC span. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 10 | 0 | 12 | little_endian | no | 0.025 | 0 | % | AccelPedal | high (field) / hint (scale) | ptcan1 3krev: = CAN 0x0D9 bits16..27 (r 0.9998 slope 1.001) | SavvyCAN data[11] + low nibble data[12]. 0 throughout the drive (refutes earlier brake idea). No OBD pedal PID logged. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 12 | 0 | 12 | little_endian | no | 0.025 | 0 | % | AccelPedal2 | medium | ptcan1 3krev vs 0x0D9 (r 0.989) | SavvyCAN data[13] + low nibble data[14]. Second pedal/throttle-like value. |
| 0x28 | static | 16 | c1 (cycle mod 4 = 1) | 14 | 0 | 16 | little_endian | no | 1 | 0 |  | Raw_14_15 | undecoded | ign-on constant 7F C0 | SavvyCAN data[15..16]. Inside CRC span. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 0 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 1..6, XOR K=0xE4. SavvyCAN data[1]. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 1 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[2]. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 1 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0x1 (0x3 briefly at crank); inside the CRC span. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 2 | 0 | 8 | n/a | no | 1 | 0 |  | Raw_2_drift | undecoded | dde_e2e.md | SavvyCAN data[3]. Slow drift at ign-on (0x97 -> 0x88), full range when running. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 3 | 0 | 24 | little_endian | no | 1 | 0 |  | Raw_3_5 | undecoded | dde_e2e.md | SavvyCAN data[4..6]. Ign-on 7C E0 E0, vary when running. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 6 | 0 | 8 | n/a | no | 1 | 0 |  | Byte6_const | confirmed | dde_e2e.md | SavvyCAN data[7]. FF, inside CRC span. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 7 | 0 | 8 | n/a | no | 1 | 0 |  | Byte7_unprotected | confirmed | dde_e2e.md | SavvyCAN data[8]. FF, NOT in CRC span (including it breaks the fit). |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 8 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 9..13, XOR K=0x0F. SavvyCAN data[9]. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 9 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[10]. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 9 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0x1; inside the CRC span. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 10 | 0 | 8 | n/a | no | 1 | 0 |  | GearEngagedCode | high | ptcan1 drive vs EGS 0x3FD; = CAN 0x0DC d2 | SavvyCAN data[11]. 0x22 P, 0x25 R, 0x24 D; wake 0x80 -> 0x20 -> 0x22. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 11 | 0 | 16 | little_endian | no | 1 | -2000 | raw | SpeedLike | medium-high | ptcan1 drive vs 0x1A1 (r 0.998, FR ~80 ms behind); = CAN 0x0DC d3..4 | SavvyCAN data[12..13]. raw-2000, ~7.67 raw per km/h (fit), positive in R; physical unit unknown. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 13 | 0 | 8 | n/a | no | 1 | 0 |  | GearRelated_13 | medium | ptcan1 drive; = CAN 0x0DC d5 | SavvyCAN data[14]. 0xFE P, 0xF4 R/D, 0xFC during park request. |
| 0x28 | static | 16 | c3 (cycle mod 4 = 3) | 14 | 0 | 16 | little_endian | no | 1 | 0 |  | Unprotected_FF | confirmed | dde_e2e.md | SavvyCAN data[15..16]. FF, NOT in CRC span. |
| 0x29 | static | 16 | c3 (base3/rep4) | 0 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 1..7, XOR K=0x3C. SavvyCAN data[1]. |
| 0x29 | static | 16 | c3 (base3/rep4) | 1 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[2]. |
| 0x29 | static | 16 | c3 (base3/rep4) | 1 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0x1 (0x3 briefly at crank); inside the CRC span. |
| 0x29 | static | 16 | c3 (base3/rep4) | 2 | 0 | 40 | little_endian | no | 1 | 0 |  | Raw_2_6 | undecoded / low | ptcan1: = CAN 0x08F d2..6; changes only with R/D engaged | SavvyCAN data[3..7]. LE pairs ~0x7D00 / 0x7CF9 in P, ~0x7F00..0x8000 in gear; /r/<0.7 vs rpm, torque, turbine, brake, speed. |
| 0x29 | static | 16 | c3 (base3/rep4) | 7 | 0 | 8 | n/a | no | 1 | 0 |  | StateEnum_7 | low | dde_e2e.md; ts run 4 | SavvyCAN data[8]. Quasi-static enum 0x80 / 0x10 / 0xC0; meaning unknown. |
| 0x29 | static | 16 | c3 (base3/rep4) | 8 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 9..15, XOR K=0x69. SavvyCAN data[9]. |
| 0x29 | static | 16 | c3 (base3/rep4) | 9 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[10]. |
| 0x29 | static | 16 | c3 (base3/rep4) | 9 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0x1; inside the CRC span. |
| 0x29 | static | 16 | c3 (base3/rep4) | 10 | 0 | 48 | little_endian | no | 1 | 0 |  | Raw_10_15 | undecoded / low | ptcan1 | SavvyCAN data[11..16]. LE pairs 00 7D <-> F9 7C; payload byte 11 changes only with R/D engaged. |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 0 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 1..7, XOR K=0x0C. SavvyCAN data[1]. |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 1 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[2]. |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 1 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0x1 (0x3 briefly at crank); inside the CRC span. |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 2 | 0 | 48 | little_endian | no | 1 | 0 |  | Raw_2_7 | undecoded | ptcan1: = CAN 0x145 d2..7 | SavvyCAN data[3..8]. FF FF then pairs 00 7D (off) / F9 7C (running). |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 8 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 9..11, XOR K=0x5C. SavvyCAN data[9]. |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 9 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[10]. |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 9 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group2 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0x1; inside the CRC span. |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 10 | 0 | 16 | little_endian | no | 1 | 0 |  | Raw_10_11 | undecoded | dde_e2e.md | SavvyCAN data[11..12]. 00 7D. |
| 0x3D | static | 16 | c3 (base3/rep4); 0x3D is a sync frame | 12 | 0 | 32 | little_endian | no | 1 | 0 |  | Unprotected_FF | confirmed | dde_e2e.md | SavvyCAN data[13..16]. FF, NOT in CRC span. |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 0 | 0 | 8 | n/a | no | 1 | 0 |  | CRC_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | E2E CRC8 poly 0x1D init 0 xorout 0 over payload 1..7, XOR K=0x04. SavvyCAN data[1]. |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 1 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Low nibble 0..14 (15 never used), +1 per transmission, starts at 1 each wake. SavvyCAN data[2]. |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 1 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_group1 | confirmed | dde_e2e.md: 5 MITM logs + non-MITM ignon, 100 %; ptcan1 3 logs 100 %; ts run 4 100 % | Fixed high nibble 0x1 (0x3 briefly at crank); counter steps per transmission; inside the CRC span. |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 2 | 0 | 8 | n/a | no | 1 | 0 |  | Raw_2 | undecoded | ptcan1 (= CAN 0x3F9 d2) | SavvyCAN data[3]. 0x80 / 0x82 seen. |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 3 | 0 | 8 | n/a | no | 1 | 0 |  | Raw_3 | undecoded | dde_e2e.md | SavvyCAN data[4]. Init 34 -> 74 -> 64, varies when running. |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 4 | 0 | 8 | n/a | no | 1 | -48 | degC | CoolantTemp | confirmed | obd1: PID 0x05 r 0.9968, 89.5 % exact (run 1); = CAN 0x3F9 d4 | SavvyCAN data[5]. obd2 plausible 21..41 C (PID 0x05 not polled in run 2). |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 5 | 0 | 8 | n/a | no | 1 | -48 | degC | Temp2_Candidate | candidate | obd1: NOT coolant (lags, x1.09-53 fit r 0.992); public hint says oil temp (CAN 0x3F9 d5) | SavvyCAN data[6]. No independent label. |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 6 | 0 | 4 | n/a | no | 1 | 0 |  | GearCode | high | ptcan1 drive vs EGS 0x3FD; = CAN 0x3F9 d6 and EGS 0x39A d3 | SavvyCAN data[7] low nibble. 3 P, 2 R, 5 D, 1 park requested while in D. N not captured. |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 6 | 4 | 4 | n/a | no | 1 | 0 |  | EngineStateNibble | medium | ts run 4 (0x23 -> 0x13 crank -> 0x03 running -> 0x13 after stop); ptcan1 | SavvyCAN data[7] high nibble. 2 ign-on pre-crank, 1 crank/after stop, 0 running (inference from timing). |
| 0xE6 | dynamic | 8 | mostly c2 mod 4; ~1 s + on change | 7 | 0 | 8 | n/a | no | 1 | 0 |  | Byte7 | confirmed constant | all logs | SavvyCAN data[8]. 0x47 steady (00 -> 10 -> 20 -> 23 -> 47 during init); inside CRC span. |
| 0xE8 | dynamic | 8 | L9: odd cycles; ~1 s, ~15/s while running | 0 | 0 | 64 | little_endian | no | 1 | 0 |  | Raw_L9 | undecoded | dde_e2e.md; ptcan1: payload identical to CAN 0x1BA (PT-CAN2) | SavvyCAN data[1..8]. No CRC/counter. Payload bytes 3..4 LE16 vary continuously after ~5 s; 0..2 vary when running; 5 F3, 7 F0. |
| 0xE8 | dynamic | 4 | L5: c0 mod 8; ~5 s | 0 | 0 | 32 | little_endian | no | 1 | 0 |  | Const_L5 | confirmed constant | dde_e2e.md; = CAN 0x33D | SavvyCAN data[1..4]. 86 78 FE FE (first frame differs). Only ign-on to engine stop in ts run 4. |
| 0xFB | dynamic | 6 | L7: c2/c6 mod 8; 200 ms | 0 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_L7 | confirmed | dde_e2e.md; ts run 4 | SavvyCAN data[1] low nibble, 0..14, starts at 1; no CRC found. |
| 0xFB | dynamic | 6 | L7: c2/c6 mod 8; 200 ms | 0 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_L7 | confirmed | dde_e2e.md | Fixed 0xF. |
| 0xFB | dynamic | 6 | L7: c2/c6 mod 8; 200 ms | 1 | 0 | 8 | n/a | no | 1 | 0 |  | Byte1_const | confirmed constant | dde_e2e.md | SavvyCAN data[2]. FF. |
| 0xFB | dynamic | 6 | L7: c2/c6 mod 8; 200 ms | 2 | 0 | 16 | little_endian | no | 0.001 | 0 | V | SupplyVoltage | confirmed | obd2: PID 0x42 r 0.9961 (fixed x0.001, OBD 48 mV lower, FR lags ~0.5 s); ptcan1: tracks CAN 0x281 | SavvyCAN data[3..4]. Crank dip ~11.3 V, ~14.7 V charging. |
| 0xFB | dynamic | 6 | L7: c2/c6 mod 8; 200 ms | 4 | 0 | 16 | little_endian | no | 1 | 0 |  | Bytes4_5_const | confirmed constant | dde_e2e.md | SavvyCAN data[5..6]. 00 F0. |
| 0xFB | dynamic | 4 | L5: c5 mod 8; ~5 s | 0 | 0 | 4 | n/a | no | 1 | 0 |  | Counter_L5 | confirmed | dde_e2e.md; ts run 4 | SavvyCAN data[1] low nibble 0..14; first frame per wake 01 00 F1 FF, then 0x4x. |
| 0xFB | dynamic | 4 | L5: c5 mod 8; ~5 s | 0 | 4 | 4 | n/a | no | 1 | 0 |  | CounterHi_L5 | confirmed | dde_e2e.md | Fixed 0x4 after the first frame (0x0 in the first). |
| 0xFB | dynamic | 4 | L5: c5 mod 8; ~5 s | 1 | 0 | 24 | little_endian | no | 1 | 0 |  | Const_L5 | confirmed constant | dde_e2e.md | SavvyCAN data[2..4]. 60 F1 FF. |
| 0xFB | dynamic | 8 | L9: c3/c7 mod 8; ~5 s + event | 0 | 0 | 64 | little_endian | no | 1 | 0 |  | Raw_L9 | undecoded | dde_e2e.md; ts run 4 | SavvyCAN data[1..8]. No CRC/counter. Byte 0 1F (first 0F); byte 1 0x00..0xC8 while running; bytes 2..3 quasi-static FF 7F / 04 00; rest constant. |
| 0x142 | dynamic | 30 | c0/c1/c2 mod 4; ~47/s | 0 | 0 | 8 | n/a | no | 1 | 0 |  | Counter | confirmed | dde_e2e.md; ts run 4 | SavvyCAN data[1]. Full byte 0..254 (255 never used), +1 per transmission, starts at 1. No CRC found. |
| 0x142 | dynamic | 30 | c0/c1/c2 mod 4; ~47/s | 1 | 0 | 8 | n/a | no | 1 | 0 |  | MonotonicCount_1 | undecoded | dde_e2e.md | SavvyCAN data[2]. Monotonic, irregular steps. |
| 0x142 | dynamic | 30 | c0/c1/c2 mod 4; ~47/s | 3 | 0 | 8 | n/a | no | 1 | 0 |  | GearLike_3 | low | ptcan1 drive | SavvyCAN data[4]. 2 / 3 / 4 in P / R / D. |
| 0x142 | dynamic | 30 | c0/c1/c2 mod 4; ~47/s | 5 | 0 | 8 | n/a | no | 1 | 0 |  | MonotonicCount_5 | undecoded | dde_e2e.md | SavvyCAN data[6]. Monotonic count (with byte 4 a BE16 count in door wake). |
| 0x142 | dynamic | 30 | c0/c1/c2 mod 4; ~47/s | 9 | 0 | 12 | little_endian | no | 1 | 0 |  | PedalLike | low-medium | ptcan1 3krev vs CAN 0x0D9 (r 0.937) | SavvyCAN data[10] + low nibble data[11]. Scale not established. |
| 0x142 | dynamic | 30 | c0/c1/c2 mod 4; ~47/s | 11 | 0 | 8 | n/a | no | 1 | 0 |  | GearAmbiguous_11 | low | ptcan1 drive | SavvyCAN data[12]. 0x80 P, 0x84 R, 0x88/0x98 D, but back to 0x80 at brake release in D. |
| 0x142 | dynamic | 30 | c0/c1/c2 mod 4; ~47/s |  |  |  | n/a | no | 1 | 0 |  | Rest_undecoded | undecoded | dde_e2e.md | Remaining bytes (2, 4, 6..8, 10, 12..29) not decoded; several step once at ~5 s after ign-on. |
| 0x5C | dynamic | 8 | base1/rep16 (DDE share) | 0 | 0 | 56 | little_endian | no | 1 | 0 |  | NM_Raw | confirmed constant | dde_e2e.md; ptcan1: = CAN 0x512 | SavvyCAN data[1..7]. 00 00 00 00 FF FF 00. |
| 0x5C | dynamic | 8 | base1/rep16 (DDE share) | 7 | 0 | 8 | n/a | no | 1 | 0 |  | NM_NodeAddress | confirmed | all logs | SavvyCAN data[8]. 0x12 = DDE. Other cycles of 0x5C belong to other nodes (NM). |
| 0x10A | dynamic | 4 | base2/rep8; ~1 s | 0 | 0 | 32 | little_endian | no | 1 | 0 |  | Const | confirmed constant | dde_e2e.md; ts run 4 | SavvyCAN data[1..4]. 00 FF FF FF. Sent only from ~5 s after ign-on until engine stop request. |
| 0x68 | dynamic | 2 | c0/c3/c7 mod 8; ~0.2/s | 0 | 0 | 16 | little_endian | no | 1 | 0 |  | Const | confirmed constant | dde_e2e.md; ts run 4 | SavvyCAN data[1..2]. C0 FF. |
| 0xBE | dynamic | 4/10/16/32 | sporadic | 0 | 0 | 8 | n/a | no | 1 | 0 |  | DiagTransport_Raw | medium (role) / undecoded (content) | ptcan1: DDE answers UDS 22 F101 here after request on vehicle FR 0x9A; ts run 4 | Variable length. Frames starting 62 17 04 carry codes 290D00 and 241800 (meaning unknown). Not periodic. |
