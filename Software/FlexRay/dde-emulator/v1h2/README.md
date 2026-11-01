# DDE + EGS emulator v1h.2 (pico-flexray)

> **Status: tested in the car (run 10, two sessions: 10a and 10b) and it behaved
> like the original car.** Damien: "From what I could tell the car behaved as
> OEM. Shifts without brake and/or without the side button on the stick were
> refused. Car thought it was running." No fault codes were raised by any
> control unit, and only 0.03 % of the emulator's FlexRay frames were missed.

The full release notes from the build are in [`README.txt`](README.txt). This
page is the short version. What changed since v1e is in
[`../CHANGELOG.md`](../CHANGELOG.md), and the car test write-ups are in
[`../docs/notes/`](../docs/notes/).

## What it does

The engine ECU (DDE) and the 8HP gearbox ECU (EGS) are being removed from the
F11. This firmware runs on the Pico 2 FlexRay board and stands in for both:

- **FlexRay (V1 and V2 boards):** the DDE's static and dynamic frames on FR1,
  following the ignition (KL15) like the real DDE.
- **PT-CAN (V2 board only):** the DDE's and EGS's PT-CAN frames, with correct
  checksums and counters. CAN transmit is still opt-in from the logger (or
  always on in the standalone build).
- **Fake engine start:** press the brake and START. CAS asks for a start
  (ST_KL D), the emulator reports cranking and then "running" at idle (780 rpm)
  after about 0.18 s, and CAS releases the start. Nothing cranks: the starter
  fuse is out.
- **Gears P / R / N / D (V2 board):** the emulator reads the gear lever (0x197)
  and the brake (0x173) from PT-CAN and changes gear like the real EGS.
  Leaving P, and P/N to R/D, need the brake and a running (emulated) engine.
  The P button gives P after about 0.9 s. `--no-gear` keeps it in P.
- **Standalone build:** starts emulating by itself at power-up, with no
  laptop (same settings as the logger command below).

It never sends on PT-CAN2, never sends 0x137, diagnostics (0x6xx) or a lever
frame (0x197). It sends no immobiliser frame of any kind (no challenge, no
response); the immobiliser IDs are unchanged from v1e.

## Which file for which board

| File | Board | SHA-256 |
|---|---|---|
| `firmware/pico_flexray_dde_emu_v1h2_ignon.uf2` | **V1** (Pi_Flex_V1). FlexRay only, laptop controlled. | `11588c2fabb3ee3c074a5fa8c17f4a6a1e434b65c9fb5af6256ac2df8011be60` |
| `firmware/pico_flexray_dde_emu_v1h2_ignon_v2.uf2` | **V2** (Pi_Flex_V2, MCP251863 CAN). FlexRay + PT-CAN + gears, laptop controlled. **This is what run 10 used.** | `03224b98a3cf9e769bc3a7ab552fec6673257237c0abe0e396190ddce97d5c2a` |
| `firmware/pico_flexray_dde_emu_v1h2_standalone.uf2` | **V1**, standalone. Needs a USB power supply on the Pico. | `e2e913a20869184ea111ea35a7960b61c09da986899291483676a51c04c33ed6` |
| `firmware/pico_flexray_dde_emu_v1h2_standalone_v2.uf2` | **V2**, standalone. Emulates as soon as it has power. | `bc1b32580abbfd15bff192bc83bdcd4df69552cb473e401cc1d863141a6270aa` |

Check them with `cd firmware && sha256sum -c SHA256SUMS`.

Built from pico-flexray commit `bc62e95` (local branch `dde-emu-v1h`) with
pico-sdk 2.2.0 and xPack arm-none-eabi-gcc 14.2.1-1.1, for `pico2` /
`rp2350-arm-s`, Release, `-DPICO_FR_USB_FAST=ON -DEMU_GEAR=1`. A rebuild of
`../source/` gives all four UF2s byte for byte.

The two bench A/B builds mentioned in `README.txt` (`bench-ab/`) were only for
a bench comparison and are not included.

## Folder contents

| Path | What |
|---|---|
| `firmware/` | The four UF2s and `SHA256SUMS` |
| `tools/fr_ts_logger.py`, `tools/frts.py` | Logger v1h.2. Copy **both** and replace older copies (the logger warns "VERSION MISMATCH" if they do not match the firmware). |
| `tools/check_emu_capture.py`, `tools/dde_emu_model.py` | Capture checker (`--engine --gear --flicker`) and its model |
| `README.txt` | Full v1h.2 release notes (with the v1h.1 and v1h text kept below) |
| `patches/0020..0027` | v1f to v1h.2 commits, on top of `../patches/0001..0011` and `../v1e/patches/0012..0019` |
| `../source/` | Full emulator source tree at v1h.2 |
| `../tools/reset_to_bootloader.py` | Puts the Pico into its USB bootloader (unchanged) |

The tools are the same files as in `../source/tools/` at `bc62e95`.

## How to run it

1. **Flash.** `python3 ../tools/reset_to_bootloader.py` (or hold BOOTSEL while
   plugging in), then copy the UF2 for your board to the `RP2350` drive.
2. **Wiring** as for v1e: DDE unplugged, EGS fuse pulled, starter fuse pulled.
   Pico FR1 to the DDE harness (car side), FR2 open. On V2, CAN-H/CAN-L to
   PT-CAN, no terminator.
3. **Start the logger before waking the car** (this is what run 10 used):
   ```
   python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu10_v1h2.csv --emulate --can --can-tx --fake-start cas --start-profile short
   ```
   Add `--no-gear` to stay in P. With a standalone UF2 the logger is optional.
4. **Check the capture afterwards:**
   ```
   python3 check_emu_capture.py ~/Downloads/FlexRay/fr-ts/f11_emu10_v1h2.csv --engine --gear --flicker
   ```
5. **Building from source:** see `../source/tools/dde_emu/build-v1h2.sh` for
   the four build configurations (it uses local paths; change them to yours).
   Host tests: `cd ../source/tests/host && sh run.sh`.

## Car run 10 results (v1h.2, V2 ignon build; runs 10a and 10b)

- **What the driver saw:** the car behaved as OEM. It started, "ran", and
  shifted. Shifts without the brake, or without the side button on the lever,
  were refused.
- **No fault codes:** no DTC event from any control unit on FlexRay (run 9 had
  14). The bonnet warning no longer flaps on and off.
- **FlexRay:** 0.03 % echo misses (55 of 169 336 in 10a, 37 of 129 315 in
  10b; run 9 was 12.5 %). Slot 0x3D is on the bus ~100 % of the time (run 9:
  41 %). The 0xE6 frame's longest gap is 1.04 s (run 9: 3.75 s).
- **PT-CAN:** 0 stale or flickering frames (run 9 had hundreds). CAN TX fail,
  FIFO-full and error counters all 0. E2E: 0 bad frames.
- **Start:** CAS accepted both starts. The start request lasted ~0.2 s and
  CAS released it 23–24 ms after our "running" report (real cars: 8–39 ms).
- **Gears:** 10a: 8 changes, 0 refusals, N held for 7.3 s. 10b: 5 changes and
  13 refusals, all of them real pushes without the brake.
- **USB:** 0 frames dropped.

Full notes: [`../docs/notes/f11_emu_run10_v1h2.md`](../docs/notes/f11_emu_run10_v1h2.md),
plus Fred's [`f11_emu_run10_fr.md`](../docs/notes/f11_emu_run10_fr.md) and
Ian's [`f11_emu_run10_gears_can.md`](../docs/notes/f11_emu_run10_gears_can.md).

## Known small issues (to be fixed in v1h.3)

None of these was visible in the car or caused a fault code.

- **0xE6 sometimes goes out in odd cycles.** The real DDE sends 0xE6 only in
  even cycles. ICM and DSC wait for the even copy, so they become ready about
  0.7 s later after a start.
- **The 0xE6 0x92 "engine stopping" state is sent late** (about 0.7 s after
  the stop, instead of within 20 ms as an event frame). The checker flags it.
- **False "lever 0x197 lost" alarms** on the console, although the bus shows
  no gap over 60 ms.
- Cosmetic: the run used the v1h.1 logger, so the checker reports a version
  mismatch, and two new diagnostics were not recorded. Some 16-bit counters
  saturate at 65535.

**Fallback:** unplug the Pico, refit the EGS fuse and reconnect the DDE.
