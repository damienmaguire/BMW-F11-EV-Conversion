# Car test notes (runs 6 to 10)

Plain-English write-ups of the in-car tests of the DDE/EGS emulator, copied from
the analysis notes (`flexray-re-notes/analysis/`). Times are seconds from the
start of each capture unless a note says otherwise.

| Note | Run | Firmware | In one line |
|---|---|---|---|
| [`f11_emu_run6.md`](f11_emu_run6.md) | 6 | v1e | First full FlexRay + PT-CAN emulation. Clean dash, P shown, MIL off, normal sleep. |
| [`f11_emu_run7_v1f.md`](f11_emu_run7_v1f.md) | 7 | v1f | 20 ms PT-CAN frames now go out at the full 50 per second. Old logger used, so own TX not logged; USB drops found. |
| [`f11_emu_run8_v1g.md`](f11_emu_run8_v1g.md) | 8 | v1g.2 | First fake engine start through CAS (ST_KL D). Start works; USB drops 11 %, echo misses up. |
| [`f11_emu_run9_v1h1.md`](f11_emu_run9_v1h1.md) | 9 | v1h.1 | First gear shifts (P/R/D). Gears work, but stale PT-CAN frames and 12.5 % FlexRay echo misses. |
| [`f11_emu_run10_v1h2.md`](f11_emu_run10_v1h2.md) | 10a/10b | v1h.2 | OEM-like. No fault codes, 0 flickers, 0.03 % echo misses, brake gating works. |
| [`f11_emu_run10_fr.md`](f11_emu_run10_fr.md) | 10a/10b | v1h.2 | Fred's FlexRay + PT-CAN cross-check of run 10 against runs 8 and 9. |
| [`f11_emu_run10_gears_can.md`](f11_emu_run10_gears_can.md) | 10a/10b | v1h.2 | Ian's gear and PT-CAN check of run 10. |
| [`egs_gear_report_tx.md`](egs_gear_report_tx.md) | (reference) | – | Ian's report on what the real EGS sends when the gear changes. The v1h.1 gear logic is built from it. |

Notes for readers:

- **Immobiliser (EWS) traffic is described by CAN ID, frame count and timing
  only.** Payload bytes of immobiliser frames have been removed from these
  copies, and no VIN appears anywhere.
- The notes refer to captures, CSV files and scripts under `/workspace/...` or
  `analysis/...`. Those files are not in this repository.
