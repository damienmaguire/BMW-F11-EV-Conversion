# DDE emulator for the F11 EV conversion (pico-flexray V1)

> **Status: v1b, not yet tested on the car.** v1 was tested once (car test 1):
> it locked and sent in the right slots, but a false "TX state machine stuck"
> fault stopped it in cycle 53 and 0x3D never reached the bus (see
> [Changelog](#changelog)). v1b fixes that. It builds, and every offline test,
> including a replay of the car test 1 capture, passes. Do the next test exactly as
> described under [Test procedure](#test-procedure-in-the-car).

Firmware: `firmware/pico_flexray_dde_emu_v1b_ignon.uf2` (V1 board, Pico 2 / RP2350).
SHA-256 is in `firmware/SHA256SUMS`. `pico_flexray_dde_emu_v1_ignon.uf2` is the
old v1 and is kept only for reference: do not use it.

## Changelog

### v1b (after car test 1)

**What went wrong in car test 1.** The console showed
`FAULT 1 "TX state machine stuck"`, `sent[0x26=2 0x28=2 0x29=2 0x3D=1]`, `late=1`,
`echo ok=6`, and 0x3D was never seen on the bus. The fault was false: the
state machine was fine.

- v1 started a "busy since" timer when it loaded 0x26 (~1215 µs into the cycle).
- It only cleared that timer when it found the state machine idle. It only looked
  right after loading 0x28 and 0x29, when the state machine was busy each time.
- At the 0x3D alarm (~1974 µs) the timer read ~759 µs, more than the 200 µs
  limit. So it raised FAULT 1, switched the pins off and stopped the state
  machines just before 0x3D went out. 0x3D was counted as sent but never left
  the pin.
- In cycle 52, 0x3D had been skipped as "late" (an alarm a few µs late), so the
  first 0x3D load, and the fault, came in cycle 53. v1 did not record why a slot
  was late, so the cause of that one late alarm is not known. v1b records it.

**Evidence.** `tests/host/test_dde_emu_replay.c` runs the real `dde_emu.c` on
the PC against the FR1 frame timing captured in car test 1:

- The v1 source faults at the first 0x3D.
- v1 with one alarm delayed by 4 µs reproduces the car console exactly: sent
  2/2/2/1, late 1, echo ok 6, miss 0, foreign 0, FAULT 1, K=34, max echo error
  0.181 µs (console: 180 ns).
- v1b sends all four frames in every cycle of the replay and of the full 60 s
  capture (48 000 frames, all within 0.181 µs of their slot), with no fault.

v1 also matched echoes only within 200 µs. A late-reported echo of our own frame
would have counted as "foreign" (FAULT 2).

**Changes in v1b:**

- **Stuck check.** Each frame gets a deadline: its start, plus its length, plus
  20 µs. FAULT 1 is raised only if the state machine is still busy after that.
  Busy with nothing sent needs 200 µs of continuous busy.
- **Own echo is never foreign.** Echoes are matched per slot and cycle within
  1 ms. A second report of the same frame is counted as `echo_dup`, not foreign.
  FAULT 2 is otherwise as strict as before.
- **FAULT 3 (no echo)** only after at least 8 misses in a row, spread over at
  least 2 cycles, while locked (references present).
- **Faults re-arm by themselves** after 2 s with no new sign of the fault. After
  that, TX needs 8 quiet cycles again. On the 3rd fault it stays latched until
  the laptop clears it (logger restart, or `0xa3` wValue 0).
- **Late check.** The pre-check before a slot was tightened from 2.5 µs to
  1.2 µs, because the margin is checked again after loading. A short ISR delay
  like the one in car test 1 no longer skips a slot.
- **First-fault record.** It holds the fault code, slot, cycle, time and the
  counters at the moment of the fault. It also counts late slots by reason
  (busy / fire / short / window / missed). It is in the `0xa4` stats (v2, 128
  bytes; the first 64 bytes keep the v1 layout), on the logger's `emu:` line, and
  printed once as `EMU FAULT ...`. The pico's own USB console also prints it
  once.
- The time-critical core code now runs from RAM.
- Frame content is unchanged from v1 (it matches run 5).


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
- there is no active fault. A fault stops TX at once. If there is no new sign of
  it for 2 s, TX re-arms (after 8 quiet cycles). The 3rd fault stays latched
  until the logger is restarted (it sends `0xa3` wValue 0) or the pico is
  power-cycled. The faults are:
  - **FAULT 1** – the TX state machine is still busy 20 µs after the frame
    should have ended. The pins are forced to "off" until re-arm.
  - **FAULT 2** – a frame in a DDE slot that we did not send. The real DDE is
    probably still connected. While it keeps sending, the fault keeps
    refreshing and never re-arms.
  - **FAULT 3** – 8 or more of our own frames in a row, over at least 2 cycles,
    were not seen coming back while locked. Check the wiring or the transceiver.

Counters on the logger's `emu:` line:

| Field | Meaning |
|---|---|
| `state` | disabled / no-lock / arming / ACTIVE / FAULT |
| `sent[...]` | frames sent per slot |
| `lock_losses` | times the lock was lost |
| `rearms` | times the lock came back |
| `late` | slots skipped because we would have been late (nothing sent) |
| `late[busy fire short win missed]` | v1b: late slots by reason, plus the last one (`last=0x3D/short r=...`, r = margin in 10 ns ticks) |
| `faults`, `auto_clears` | v1b: faults since the last clear, and how many re-armed by themselves |
| `first FAULT ...` | v1b: the first fault since the last clear: code, slot, cycle, how long ago, detail, counters at the fault, `locked` / `tx-in-flight` |
| `echo ok`, `echo miss` | our frames seen / not seen back on RX |
| `foreign` | frames in our slots that were not ours |
| `echo_err` | timing error of our frames, ns |
| `fire_err`, `loop`, `K` | calibration values |
| `tss` | TSS length in bits |

When a first fault appears, the logger prints it once, for example:

```
EMU FAULT 1 "TX state machine stuck" slot=0x3D cycle=53 0.412 s ago detail=21 | at the fault: sent=7 late=1 echo ok=6 miss=0 foreign=0 locked tx-in-flight (logger t=+14.2s)
```

`detail` means:
- FAULT 1: µs past the frame's deadline. A negative value is the µs the state
  machine was busy with nothing sent.
- FAULT 2: µs between our frame and the other one. −1 means we sent nothing in
  that slot in that cycle.
- FAULT 3: the number of misses in a row.

Unplugging the pico's USB cable removes power, and the transceiver then goes
silent.

## Files

```
firmware/                 v1b UF2 (use this), old v1 UF2, SHA256SUMS
tools/                    laptop tools: fr_ts_logger.py (with --emulate), frts.py, reset_to_bootloader.py
source/                   complete firmware source tree (dde-emulator branch) incl. tests and generator
  src/dde_emu*.{c,h,pio}  emulator firmware (dde_emu_tables.h is generated)
  tools/dde_emu/          table generator, Python model, test_dde_emu.py, test_logger_emulate.py
  tests/host/             C host tests (run.sh), incl. test_dde_emu_core.c + captured vectors
                          test_dde_emu_replay.c: dde_emu.c replayed against car test 1 timing (emu_stubs/, emu_run1_fr1_1600ms.txt)
  tools/dde_emu/extract_replay.py  makes the replay vectors from a logger CSV
patches/                  git format-patch series: ts-capture-fixed -> dde-emulator
BASE.md                   which upstream commit / local branch this is based on
```

## Laptop setup (Ubuntu, once)

The commands assume the tools live in `~/Downloads/FlexRay/fr-ts`. Copy
`tools/fr_ts_logger.py`, `tools/frts.py`, `tools/reset_to_bootloader.py` and
`firmware/pico_flexray_dde_emu_v1b_ignon.uf2` there. The new `fr_ts_logger.py`
and `frts.py` replace the old ones. They still work with the capture firmware
and with v1. Copy them again for v1b: the old ones do not show the first fault.

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
sha256sum pico_flexray_dde_emu_v1b_ignon.uf2
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
   cp ~/Downloads/FlexRay/fr-ts/pico_flexray_dde_emu_v1b_ignon.uf2 /media/$USER/RP2350/
   ```

   The pico reboots by itself, and the GP20 LED blinks once per second. TX is
   off at this point.

5. **Start the logger with `--emulate`:**

   ```
   python3 ~/Downloads/FlexRay/fr-ts/fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu2.csv --emulate
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

   If it shows `FAULT` or the console prints `EMU FAULT ...`, the firmware has
   stopped sending. It re-arms by itself after 2 s if the cause is gone, and
   stays off after the 3rd fault. Note the `EMU FAULT` line and go to step 9. In the status line, frames from FR1 now include ours
   (0x26/0x28/0x29/0x3D).

   **Pass check (in the capture, afterwards):** the car's 0x2F byte 4 goes
   0e→02 and byte 5 goes 30→20 about 1.05 s after ign on. Without a working
   DDE (or with v1) byte 4 goes to 06 about 5.76 s after wake, and byte 5 stays
   30.

7. **Watch the dash and the console** for a minute or two. Note any warnings or
   messages on the dash, for comparison with "DDE unplugged, no pico".

8. **Ignition off, wait 70 s.** The bus should go to sleep as it did in run 4
   (about 61 s after ign off). The status line drops to `0 frames/s` and
   `emu:` goes to `no-lock`. If the bus stays awake well past 70 s, the
   emulator is keeping it awake: stop the logger (Ctrl-C) and note it.

9. **Stop the logger** with Ctrl-C. It prints `EMULATOR TX DISABLED`, then
   `emu at stop: ...`. Keep these three files, from `~/Downloads/FlexRay/fr-ts/`:
   - `f11_emu2.csv`
   - `f11_emu2.csv.bin`
   - `f11_emu2.csv.console.log`

If anything looks wrong (dash goes mad, the car will not sleep, a FAULT
appears), press Ctrl-C in the logger first. TX is switched off at once.

### Fallback

Unplug the pico and reconnect the DDE. The pico has no relays and nothing in it
holds its state. Once unplugged, it is gone from the bus.

To put the normal capture firmware back on the pico, use
`pico_flexray_ts_capture_v1_FIXED.uf2` or `pico_flexray_passthrough_KNOWN_GOOD.uf2`
from the fr-ts folder, the same way as in step 4.

## Known risks / open points

- **v1b is not yet tested on the car.** v1 ran on the car once (car test 1,
  stopped by the false fault). The traction control failure seen then
  (0x3D, which carries WMOM_DRV/TORQ_CRSH, never reached the bus) should be
  checked again with v1b.
- **TSS length.** We send a 7-bit TSS. If the car's receivers expect a longer
  one, our frames are dropped: `echo ok` would rise but the dash would act as
  if the DDE were missing. If so, rebuild with `-DEMU_TSS_BITS=<n>`.
- **No dynamic frames** (see above). Anything the car reads from 0xE6, 0xE8,
  0xFB, 0x142, 0x68, 0x10A, 0xBE or 0x5C will be missing.
  - Planned next, **not in v1b: 0xE6.** Track dynamic-segment activity per
    cycle, as worked out from car test 1 (`f11_emu_run1.md`), and only send in
    cycles where the minislot count is known to be safe. Content from Fred's
    template: K=04, b2 = 0x10 | counter, 1 Hz, in cycles ≡ 2 (mod 8).
- **Late alarms.** Car test 1 had one slot skipped as late in cycle 52, cause
  unknown. v1b tolerates a delay like that one. If it happens again, the
  `late[...]` reasons and `last=` show which slot and why.
- **Pre-ign values.** Before ign on, the real DDE sends 0x28 c1 = `c1 17 7c` and
  0x29 b8 = 80. The emulator always sends its ign-on content. This was not
  changed in v1b (small, optional follow-up).
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
