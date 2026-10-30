# DDE emulator for the F11 EV conversion (pico-flexray V1)

> **Status: UNTESTED ON THE CAR.** It builds, and every offline test against the
> run 5 capture passes. It has never been flashed or put on a bus. Do the first
> test exactly as described under [Test procedure](#test-procedure-in-the-car).

Firmware: `firmware/pico_flexray_dde_emu_v1_ignon.uf2` (V1 board, Pico 2 / RP2350).
SHA-256 is in `firmware/SHA256SUMS`.

## What it does

The diesel ECU (DDE) is gone from the car, and the rest of the car expects to see
its FlexRay frames. This firmware turns the pico-flexray board into a stand-in for
the DDE:

- On **FR1** (the car side), it sends the DDE's static frames **0x26, 0x28, 0x29
  and 0x3D** in the DDE's own slots, every cycle.
- The content is a replay of the DDE's frames recorded in run 5 with **ignition on
  and the engine off**. There is one recorded frame per slot and per cycle
  (cycle 0..63).
- Every frame gets the current cycle number, a live alive-counter, a recomputed
  E2E checksum and recomputed header and frame CRCs, so it looks freshly made.
- It also keeps the timestamped capture, so `fr_ts_logger.py` records the whole
  bus (including our own frames) while it runs.

What it does **not** do:

- **No dynamic frames.** The DDE also sends 0x5C, 0xE6, 0xE8, 0xFB, 0x142, 0x68,
  0x10A and 0xBE in the dynamic segment. Their position depends on counting
  minislots, and the pico misses about 2 % of frames, which makes that count
  unreliable. A wrong count means a collision with another ECU.
  - Only 0x5C (minislot 0, the first dynamic slot) can be placed without
    counting. It is behind a build option, `EMU_DYN_5C=1`, which is **off** in
    the shipped UF2.
- It does not start the bus or keep it awake by itself. It only transmits while
  it hears the car's own frames.
- It never transmits on FR2, and it does not forward FR1 to FR2 (this is not a
  repeater build).
- It does not use relays or any 12 V logic. GP17/GP18 are set exactly as in the
  stock firmware.
- It does not follow vehicle state: it always sends "ign on, engine off".

## How it works

The FR1 receiver timestamps every frame from the car with the board's 100 MHz
counter (10 ns steps).

**Lock.** Fourteen car frames that come every cycle with very little jitter
(0x07, 0x21, 0x27, 0x34 … 0x5A) are the timing references. Each reference
tells the firmware where the current cycle started.

**Scheduling.** For each DDE slot, it computes the exact 10 ns tick at which the
frame must start (slot positions measured in run 5). It prepares the frame
bytes, then a timer interrupt hands them to a PIO state machine about 6 µs
before the slot. A second PIO state machine waits for the exact tick and then
starts transmission (TXEN low + bits on TXD).

**Echo calibration.** The transceiver echoes our own frame back on RXD. The
firmware times that echo and corrects its own fire delay and loop delay, so
after the first few frames the start error is about ±20 ns. That is better than
the real DDE, which wanders by about ±270 ns.

**Header.** The header is copied from the recording, with these changes:

- the sync and startup bits are **cleared**, so the pico is never a sync or
  coldstart node (`EMU_SYNC_BITS=1` would keep them; **unsafe and untested**);
- the cycle field is the current car cycle;
- the header CRC is recomputed.

Frame layout: TSS 7 bits (`EMU_TSS_BITS`), FSS, then BSS + byte for each byte,
then FES.

**Control.** TX is **off at boot**. USB vendor request `0xa3` with wValue 1
enables it and wValue 0 disables it. Request `0xa4` reads the counters. The
logger handles both with `--emulate`.

## Safety

The firmware only transmits when **all** of the following are true:

- the laptop enabled it, and keeps re-enabling it at least every 20 s (the
  logger does this every 2 s). If the laptop crashes, sleeps or the cable comes
  out, TX stops by itself within 20 s;
- a reference frame was seen in **this** cycle. It never free-runs: no car
  frames means no frames from us;
- the lock is good. Lock is lost when more than 2 cycles go by without a
  reference, and it re-arms only after 8 good cycles in a row;
- 8 quiet cycles have passed since TX was enabled. During that time it checks
  that nobody else is using the DDE slots;
- there is no latched fault. A fault stops TX until the pico is power-cycled or
  re-enabled. The faults are:
  - **FAULT 1** – the TX state machine got stuck. The pins are forced to "off".
  - **FAULT 2** – a frame in a DDE slot that we did not send. The real DDE is
    probably still connected.
  - **FAULT 3** – 8 of our own frames in a row were not seen coming back. Check
    the wiring or the transceiver.

Counters on the logger's `emu:` line:

| Field | Meaning |
|---|---|
| `state` | disabled / no-lock / arming / ACTIVE / FAULT |
| `sent[...]` | frames sent per slot |
| `lock_losses` | times the lock was lost |
| `rearms` | times the lock came back |
| `late` | slots skipped because we would have been late (nothing sent) |
| `echo ok`, `echo miss` | our frames seen / not seen back on RX |
| `foreign` | frames in our slots that were not ours |
| `echo_err` | timing error of our frames, ns |
| `fire_err`, `loop`, `K` | calibration values |
| `tss` | TSS length in bits |

Unplugging the pico's USB cable removes power, and the transceiver then goes
silent.

## Files

```
firmware/                 the UF2 + SHA256SUMS
tools/                    laptop tools: fr_ts_logger.py (with --emulate), frts.py, reset_to_bootloader.py
source/                   complete firmware source tree (dde-emulator branch) incl. tests and generator
  src/dde_emu*.{c,h,pio}  emulator firmware (dde_emu_tables.h is generated)
  tools/dde_emu/          table generator, Python model, test_dde_emu.py, test_logger_emulate.py
  tests/host/             C host tests (run.sh), incl. test_dde_emu_core.c + captured vectors
patches/                  git format-patch series: ts-capture-fixed -> dde-emulator
BASE.md                   which upstream commit / local branch this is based on
```

## Laptop setup (Ubuntu, once)

The commands assume the tools live in `~/Downloads/FlexRay/fr-ts`. Copy
`tools/fr_ts_logger.py`, `tools/frts.py`, `tools/reset_to_bootloader.py` and
`firmware/pico_flexray_dde_emu_v1_ignon.uf2` there. The new `fr_ts_logger.py`
and `frts.py` replace the old ones; they still work with the capture firmware.

```
sudo apt install python3-usb
```

If the logger later says "Access denied" or "insufficient permissions", add this
udev rule (or run the logger with `sudo`):

```
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="3801", ATTR{idProduct}=="ddcc", MODE="0666"' | sudo tee /etc/udev/rules.d/99-pico-flexray.rules
```

```
sudo udevadm control --reload-rules
```

```
sudo udevadm trigger
```

Check the UF2 is intact:

```
cd ~/Downloads/FlexRay/fr-ts
```

```
sha256sum pico_flexray_dde_emu_v1_ignon.uf2
```

It must match the line in `firmware/SHA256SUMS`.

## Test procedure (in the car)

Do **not** start the engine during this test.

1. **Car asleep.** Locked or left alone until the bus is quiet. Ignition off.

2. **Unplug the DDE** connector.

3. **Wire the pico's FR1** to the DDE harness (the car side, where the DDE used
   to be). Leave FR2 unconnected.

4. **Flash the UF2.** With the pico plugged into the laptop:

   ```
   python3 ~/Downloads/FlexRay/fr-ts/reset_to_bootloader.py
   ```

   An RP2350 drive appears. (You can also unplug, hold BOOTSEL, plug in and
   release.)

   ```
   cp ~/Downloads/FlexRay/fr-ts/pico_flexray_dde_emu_v1_ignon.uf2 /media/$USER/RP2350/
   ```

   The pico reboots by itself, and the GP20 LED blinks once per second. TX is
   off at this point.

5. **Start the logger with `--emulate`:**

   ```
   python3 ~/Downloads/FlexRay/fr-ts/fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu1.csv --emulate
   ```

   You should see `EMULATOR TX ENABLED`, then every 5 s a status line plus an
   `emu:` line. While the bus is asleep it shows `state=no-lock` and
   `0 frames/s`. That is correct.

6. **Ignition on (do NOT start the engine).** Within a fraction of a second the
   `emu:` line should show:
   - `state=ACTIVE locked`;
   - the `sent[...]` counters rising by about 200 per second per slot;
   - `echo ok` rising, with `echo miss`, `foreign`, `late` and `lock_losses`
     staying at (or near) 0;
   - `echo_err avg` within about ±50 ns. The `max` value can show the
     error of the very first frame (up to a few hundred ns), but it should
     not keep growing.

   If it shows `FAULT`, the firmware has already stopped sending. Read the fault
   text and go to step 9. In the status line, frames from FR1 now include ours
   (0x26/0x28/0x29/0x3D).

7. **Watch the dash and the console** for a minute or two. Note any warnings or
   messages on the dash, for comparison with "DDE unplugged, no pico".

8. **Ignition off, wait 70 s.** The bus should go to sleep as it did in run 4
   (about 61 s after ign off). The status line drops to `0 frames/s` and
   `emu:` goes to `no-lock`. If the bus stays awake well past 70 s, the
   emulator is keeping it awake: stop the logger (Ctrl-C) and note it.

9. **Stop the logger** with Ctrl-C. It prints `EMULATOR TX DISABLED`, then
   `emu at stop: ...`. Keep these three files, from `~/Downloads/FlexRay/fr-ts/`:
   - `f11_emu1.csv`
   - `f11_emu1.csv.bin`
   - `f11_emu1.csv.console.log`

If anything looks wrong (dash goes mad, the car will not sleep, a FAULT
appears), press Ctrl-C in the logger first. TX is switched off at once.

### Fallback

Unplug the pico and reconnect the DDE. The pico has no relays and nothing in it
holds its state. Once unplugged, it is gone from the bus.

To put the normal capture firmware back on the pico, use
`pico_flexray_ts_capture_v1_FIXED.uf2` or `pico_flexray_passthrough_KNOWN_GOOD.uf2`
from the fr-ts folder, the same way as in step 4.

## Known risks / open points

- **Never tested on the car or on a bench bus.**
- **TSS length.** We send a 7-bit TSS. If the car's receivers expect a longer
  one, our frames are dropped: `echo ok` would rise but the dash would act as
  if the DDE were missing. If so, rebuild with `-DEMU_TSS_BITS=<n>`.
- **No dynamic frames** (see above). Anything the car reads from 0xE6, 0xE8,
  0xFB, 0x142, 0x68, 0x10A, 0xBE or 0x5C will be missing.
- **Fixed state.** The replay is one 64-cycle window (run 5, ign on, engine off).
  Nothing changes with the pedal, temperatures and so on.
- **Network management / sleep.** The replayed frames may contain "stay awake"
  information. Step 8 checks this.
- **The DDE was a sync/coldstart node.** We clear those bits, so the car
  must start and sync the bus without it. Run 5 suggests it does, but this is
  not confirmed.
- **First frames.** The very first frame after enable uses default calibration
  values and may be a few hundred ns off. The echo calibration fixes this
  within a few frames.

## Building (developers)

Same toolchain as the FIXED capture build: pico-sdk 2.2.0, arm-none-eabi-gcc
14.2.1, board `pico2`.

```
cd source
```

```
mkdir build-emu && cd build-emu
```

```
cmake -G Ninja -DPICO_SDK_PATH=$HOME/pico-sdk -DPICO_BOARD=pico2 -DPICO_FLEXRAY_DDE_EMU=ON ..
```

```
ninja
```

The output is `pico_flexray.uf2`. Options (CMake `-D`):

| Option | Default | Meaning |
|---|---|---|
| `PICO_FLEXRAY_DDE_EMU` | `OFF` | `OFF` builds the normal capture firmware (unchanged code) |
| `EMU_TSS_BITS` | `7` | TSS length |
| `EMU_DYN_5C` | `0` | `1` = also send dynamic 0x5C (minislot 0, every 16th cycle); untested |
| `EMU_SYNC_BITS` | `0` | `1` = keep the captured sync/startup bits; **unsafe, untested** |

Tests (no hardware). `pioasm` comes from the pico-sdk build; pass it with
`--pioasm` or `PIOASM=`. The full-run checks also need the run 5 capture
(`--bin f11_run5.csv.bin`, or `DDE_RUN5_BIN=` for `run.sh`). Without the
capture they are skipped.

```
cd source/tests/host && sh run.sh
```

```
python3 source/tools/dde_emu/test_dde_emu.py --pioasm /path/to/pioasm
```

```
python3 source/tools/dde_emu/test_logger_emulate.py
```

```
python3 source/tools/ts_capture/test_ts_tools.py
```

To regenerate the tables from a capture (`dde_emu_tables.h` is committed):

```
python3 source/tools/dde_emu/gen_dde_tables.py --bin f11_run5.csv.bin --slots f11_schedule_run5_slots.csv --out source/src/dde_emu_tables.h
```
