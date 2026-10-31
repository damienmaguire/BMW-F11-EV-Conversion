# DDE + EGS emulator v1e (pico-flexray)

> **Status: tested in the car (run 6, 31 Oct 2026) and it worked.** With the DDE
> powered off and the EGS fuse pulled, the dash was clean, P was shown, the
> engine light (MIL) went out, and the car went to sleep normally after ignition off.

## What it does

The engine ECU (DDE) and the 8HP gearbox ECU (EGS) are being removed from the
F11. The other modules in the car still expect to hear from them. This firmware
runs on a Raspberry Pi Pico 2 board and stands in for them:

- **FlexRay (both boards):** it sends the DDE's FlexRay frames on FR1. These are
  the static frames 0x26/0x28/0x29/0x3D, the dynamic frames 0xE6/0xFB/0xE8, the
  network-management frame 0x5C, and 0x10A/0x142/0x68. The content follows the
  ignition (KL15) the way the real DDE does, including the changes after
  ignition off.
- **PT-CAN (V2 board only):** it sends the DDE's and the EGS's PT-CAN frames,
  with the correct checksums (E2E CRC) and alive counters. These include P in
  the gear display, "engine off" and "no fault".
  **CAN transmit is off unless the logger turns it on.**

No engine runs in v1e. Everything is "ignition on, engine off, Park".

## Which file for which board

| File | Board | SHA-256 |
|---|---|---|
| `firmware/pico_flexray_dde_emu_v1e_ignon.uf2` | **V1** (Pi_Flex_V1: relays GP17/18, LED GP20). FlexRay only, no CAN. | `c2e780abff83a3ae71d25b11152cd100114dfcda271c5d09941888693b928caf` |
| `firmware/pico_flexray_dde_emu_v1e_ignon_v2.uf2` | **V2** (Pi_Flex_V2, MCP251863 CAN). FlexRay + PT-CAN capture + PT-CAN TX (opt-in). | `c4871ce7313ec796ad30cc3af5c311134589f26f41cb8f629b6a390506744037` |

Check them with `cd firmware && sha256sum -c SHA256SUMS`.

Built from pico-flexray commit `a2f3321` (source tree here at `d7e490a`, comments only on top: the V2 UF2 is byte-identical) (branch `dde-emulator`, local) with
pico-sdk 2.2.0 and xPack arm-none-eabi-gcc 14.2.1-1.1, for `pico2` /
`rp2350-arm-s`, Release.

## Folder contents

| Path | What |
|---|---|
| `firmware/` | The two UF2s and `SHA256SUMS` |
| `tools/fr_ts_logger.py`, `tools/frts.py` | Logger. Copy **both** files and replace older copies. Works with v1 to v1e firmware. |
| `tools/reset_to_bootloader.py` | Puts the Pico into its USB bootloader |
| `tools/check_ptcan_replay.py` | Offline check of PT-CAN frames against a capture |
| `patches/0012..0019` | The v1d and v1e commits (0019 = comments only), on top of patches 0001..0011 in `../patches` |
| `../source/` | Full emulator source tree at v1e (`d7e490a`) |

## How to run it

1. **Flash.** With the Pico plugged into the laptop, run
   `python3 tools/reset_to_bootloader.py`, or hold BOOTSEL while plugging it in.
   An `RP2350` drive appears. Then copy the UF2:
   ```
   V1:  cp firmware/pico_flexray_dde_emu_v1e_ignon.uf2    /media/$USER/RP2350/
   V2:  cp firmware/pico_flexray_dde_emu_v1e_ignon_v2.uf2 /media/$USER/RP2350/
   ```
2. **Wiring.** DDE unplugged, EGS fuse pulled. Connect Pico FR1 to the DDE
   harness (car side) and leave FR2 open. On V2, connect CAN-H/CAN-L to PT-CAN.
   Do not add a terminator.
3. **Start the logger before waking the car.** Use one of these commands:
   ```
   # FlexRay emulation + PT-CAN capture, no CAN TX
   python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu6_fronly.csv --emulate --can

   # Full emulation: FlexRay + all DDE/EGS PT-CAN frames (this is what run 6 used)
   python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu6_fullemu.csv --emulate --can --can-tx

   # Only selected PT-CAN IDs (step-by-step bring-up)
   python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu6_step1.csv --emulate --can --can-tx-ids step1
   ```
   The `--can-tx-ids` sets are: `step1` = 3F9, `step2` = +3FD, `step3` = +39A,
   `step4` = +396,518, `step5`/`all` = everything. `dde` and `egs` are also
   accepted, as is a comma-separated list of hex IDs.
4. Wake the car and switch the ignition on. Keep it on for at least 60 s. Then
   switch it off and keep logging for at least 90 s. Then press Ctrl-C.
5. **Building from source:**
   ```
   cmake -S ../source -B build -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pico2 \
         -DPICO_PLATFORM=rp2350-arm-s -DCMAKE_BUILD_TYPE=Release -DPICO_FLEXRAY_DDE_EMU=ON
   # V2 board: add -DPICO_FLEXRAY_BOARD_V2=ON -DEMU_CAN_TX=ON
   cmake --build build -j
   ```
   Host tests: `bash ../source/tests/host/run.sh`.

## Car run 6 results (v1e, DDE 12 V off, EGS fuse pulled, full PT-CAN TX)

- **What the driver saw:** a clean dash, P shown, and the MIL went off.
- **Ignition status 0x12F:** ST_KL went 6 → 9 → A at ignition on, then A → 7 → 6
  at ignition off. It ended at **6**, the same as with the real DDE. Without the
  emulated 0x3F9 it used to stick at 8 (KL_R).
- **Bus sleep:** the last FlexRay frame came at **OFF + 52.77 s**. The real
  DDE+EGS figure is +52.75 s. 0x501/0x560 were released, and the emulator's NM
  frames stopped on time: 0x518 at +29.7 s and 0x512 at +45.7 s.
- **PT-CAN health:** no new IDs and no diagnostic (0x6xx) requests. There was no
  immobiliser traffic or escalation, no CAN TX fault, and no foreign-ID
  collision. FIFO-full and failed-TX counts were 0. TEC touched 4 once and then
  went back to 0 (one transient error frame).
- **Content:** all 19 emulated IDs carry the real DDE/EGS payloads. The 7 E2E IDs
  passed CRC 100 % with 0 counter breaks.
- **FlexRay:** echo miss 0, foreign 0. The E2E of the statics was 100 % valid. The
  after-off content (0x29 b8 = c0, FB L9 = `1f 00 ff 7f`) matches the real DDE.

Full analysis: `flexray-re-notes/analysis/f11_emu_run6.md` (not in this repo).

## Safety design

- **The V2 CAN controller boots in listen-only mode.** In that mode it sends no
  ACKs, no error frames and nothing else. It switches to normal mode only while
  the logger keeps CAN TX enabled, with a keepalive every 2 s.
- **It drops back to listen-only immediately** on any of these: Ctrl-C or logger
  exit, no keepalive for 20 s, bus-off, TX error passive (TEC ≥ 128), a mode
  change error, or one of *our* IDs arriving from another node (meaning a real
  DDE or EGS is still on the bus). A fault stays latched until the next start.
- **It never wakes the car or keeps it awake.** It sends only while the car
  itself has been talking on PT-CAN within the last 200 ms. Each frame stops at
  the point the real ECU stopped it. The NM frames have a hard 60 s ceiling
  after the wake or the last KL15-on.
- **Blocked IDs are never sent.** They are checked twice: in the scheduler and
  again just before the TX FIFO. The list is 0x80, 0x85, 0x351, 0x380, 0x580,
  0x5C0 (immobiliser/EWS), 0x137, 0x27A, 0x598, and every 0x6xx (diagnostics).
  Nothing is ever sent on PT-CAN2, which is not wired. There is no
  challenge/response of any kind.
- **FlexRay:** the emulator sends only in the DDE's own slots, and only after it
  has locked onto the cluster. It stops on any echo mismatch or foreign frame
  in our slots, and it has the same NM ceiling.

## Known caveats

- About 2.5 % of the fast PT-CAN frames (20/30/100 ms IDs) were not sent. The TX
  loop drops a slot rather than sending it late. Nothing in the car reacted.
  This is planned for v1f.
- The capture does not contain our own PT-CAN TX frames, because the controller
  does not echo them. Run 6 TX was checked by replaying the run through the
  emulator core. This is planned for v1f.
- 0x397 is sent at a constant 1 s period. The real one runs at about 380 ms,
  with short blips and changes after ignition off.
- 0x592: only the MIL-off check-control frame is sent. The real DDE's service
  frames after ignition off (`26/30/32 …`) are not decoded, so they are not
  sent.
- 0x512/0x518 stop about 0.25 s earlier than the real ones. The 0x3FD P-hold ends
  80 ms late.
- No engine start is emulated: the tacho stays at 0.
- A door wake without KL15 is not captured; it uses an assumed 30 s for 0x518.
- KL15 is read from 0x12F without checking 0x12F's own E2E.
- Read the fault memory (all ECUs, especially FEM/CAS) after each test.
  Whether CAS/FEM stores a "no DDE challenge" fault cannot be seen on the bus.

**Fallback:** unplug the Pico, refit the EGS fuse and reconnect the DDE.
