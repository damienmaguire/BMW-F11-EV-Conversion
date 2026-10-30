# DBC files for the F11 DDE

Decoding by **Fred**. Signal table, E2E solution, both CAN DBCs and the caveats
below are his work. The FlexRay DBC is generated from his signal table.

| File | Bus | What it is |
|---|---|---|
| `f11_flexray_dde.dbc` | FlexRay, DDE side | 12 DDE slots, 90 signals. Generated, do not edit by hand |
| `f11_ptcan_dde.dbc` | PT-CAN (SavvyCAN bus 3 in our logs) | 16 DDE messages (Fred, unchanged) |
| `f11_ptcan2_egs_dde.dbc` | PT-CAN2 (bus 2) | 10 DDE messages + EGS 0x1AF / 0x3FD (Fred, unchanged) |
| `f11_flexray_dde_signals.csv` / `.md` | FlexRay | Fred's signal table: 83 rows with confidence and evidence |
| `f11_dde_e2e.md` | all | CRC8 / alive-counter scheme, K tables, worked examples |
| `make_flexray_dbc.py` | – | Rebuilds `f11_flexray_dde.dbc` from the CSV |

## FlexRay DBC layout (SavvyCAN)

| Item | Rule |
|---|---|
| Message ID | FlexRay slot ID (e.g. `BO_ 40` = slot 0x28) |
| Byte 0 | cycle count (0–63), as SavvyCAN puts it in front of the payload |
| Bytes 1.. | FlexRay payload. Fred's "payload byte n" = DBC byte n+1 |
| DLC | largest payload + 1 (16-byte static slot = 17) |
| Cycle-dependent layouts | multiplexed on low bits of byte 0 (table below) |

| Slot | Multiplexer | Values used |
|---|---|---|
| 0x26 | `CycleMod4` (2 bits) | 1 |
| 0x28 | `CycleMod4` | 1 (torque, rpm, pedal), 3 (gear, speed-like) |
| 0x29, 0x3D | `CycleMod4` | 3 |
| 0x5C | `CycleMod16` (4 bits) | 1 = DDE share of the NM slot |
| 0xE8 | `CycleParity` (1 bit) | 1 = 8-byte frame, 0 = 4-byte frame |
| 0xFB | `CycleMod4` | 2 = 6-byte (voltage), 3 = 8-byte, 1 = 4-byte |
| 0xE6, 0x68, 0x10A, 0x142, 0xBE | none | the cycle is not fixed |

The 0xE8 / 0xFB short variants used different exact cycles in ts run 4 and
run 5; the multiplexers above fit both runs. Signals that exist in several
layouts carry a suffix (`_c1`, `_c3`, `_L5`, `_L7`, `_L9`).

To rebuild after editing the CSV:

```
python3 make_flexray_dbc.py
```

## Check with cantools

All three files load with `cantools.database.load_file(..., strict=True)`
(cantools 45.1.2). Real frames decode as expected, for example:

| Frame | Decoded |
|---|---|
| FR 0x28 cycle 37 `25 94 fb 2f f8 82 48 0c f1 98 1b 00 10 00 e0 7f c0` | 786 rpm, torque 24 Nm, pedal 0 % |
| FR 0x28 cycle 31 `1f 71 1a 39 7d 20 20 ff ff b7 1a 22 d0 07 fe ff ff` | gear code P, speed-like 0 |
| FR 0xE6 `3a d4 13 82 26 4f 4c 03 47` | coolant 31 °C, gear P |
| FR 0xFB cycle 58 `3a f3 ff ca 38 00 f0` | supply 14.538 V |
| CAN 0x0A5 `F9 F1 C6 67 7C 00 00 F1` | 0 rpm, −28.5 Nm |
| CAN 0x3F9 `14 14 82 06 4f 4d 03 47` | coolant 31 °C, gear P |
| CAN 0x281 `65 f3` | 13.035 V |

The E2E CRC bytes of the FlexRay frames above also recompute correctly with the
scheme in `f11_dde_e2e.md`.

## Confidence

Every signal has a `CM_` comment with its confidence and evidence. In short:

| Word | Meaning |
|---|---|
| confirmed | matched to an independent label (OBD, DDE alone on the bench, or 100 % fit) |
| high | equal to a CAN field whose meaning follows from lever / gear behaviour |
| medium / low / candidate | as stated per signal |
| undecoded | raw bytes only (`Raw_*`) |
| hint | public source, not verified by us; used only for scale or unit |

## Caveats (read before using)

1. **Torque and pedal scales come from a public hint.** Only the field
   positions are verified (FlexRay ↔ CAN equality). There is no OBD torque or
   pedal reference.
2. **0x281 voltage is medium confidence.** The bench shows the DDE sends 0x281
   (earlier it was taken as a vehicle message). The formula
   (d0 + (d1 & 0x0F) · 256) · 15 mV tracks FlexRay 0xFB voltage with r 0.92–0.97.
3. **0x3FB is raw only.** The F10 bench DDE sends it, but in the F11 its content
   equals vehicle FlexRay 0xE9 L7 about 7 ms later, so it is likely a gateway
   copy there. d0 is 0xCD in the car and 0xBE on the bench.
4. **0x592 carries a per-wake immobiliser-style handshake.** DDE ↔ CAS/FEM 0x5C0
   on PT-CAN, EGS 0x598 → DDE on PT-CAN2. The payload is random every wake; the
   algorithm and key are unknown. No signals are defined.
5. **Cycle times are approximate.** They come from frame counts and bench
   periods. The SavvyCAN logs have ms timestamps that wrap every second.
6. **The bench unit is a different F10 DDE on a single bus.** Its variant may
   differ from the F11 DDE (8513250), and which bus it was wired to is unclear:
   it showed 0x137 and 0x1BA (PT-CAN2-only in the F11) and 0x0DC (PT-CAN-only in
   the F11). 0x137 and 0x1BA are in the PT-CAN file for that reason.
7. **0x1B9 DLC differs:** 2 in the F11 (`00 FF`), 4 on the bench.
8. **0x145 and 0x33D are not in the bench log.** They are attributed to the DDE
   by FlexRay layout and K (0x145 = FR 0x3D c3 group 1) and by content
   (0x33D = FR 0xE8 4-byte frame).
9. **Left out because the sender is unproven:** PT-CAN2 0x0A0 (K 0xCC), 0x0A6
   (0xC5), 0x0A7 (0x23), 0x0B0 (0x65), 0x0F8 (d0..1 ≈ rpm × 1.6), 0x2B1 (all
   zero), 0x3BE. 0x335 is the fuel pump (EKPS) by a DTC hint, not the DDE.
10. **Small CAN CRC-failure counts:** at most 5 per ID out of about 27k, not
    investigated. FlexRay shows 0 failures.
11. **0x142 cycle:** Fred's table lists cycles 0/1/2 mod 4 (run 4). In ts run 5
    it was 1/2/3 mod 4, one frame every 4 cycles. The phase is not fixed, so the
    DBC does not multiplex it.
12. **0x142 `Rest_undecoded`** (payload bytes 2, 4, 6..8, 10, 12..29) has no
    position in the table and is not in the DBC.
