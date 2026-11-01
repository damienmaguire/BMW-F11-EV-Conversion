# DDE emulator for the F11 EV conversion (pico-flexray V1)

> **Latest: v1h.2** (FlexRay + PT-CAN DDE/EGS emulation with fake engine start
> and P/R/N/D gears; car run 10 behaved like the original car, no fault codes).
> See [`v1h2/README.md`](v1h2/README.md), the [`CHANGELOG.md`](CHANGELOG.md) and
> the car test notes in [`docs/notes/`](docs/notes/). The `source/` tree is now
> at v1h.2. The previous release is in [`v1e/`](v1e/README.md); the v1c notes
> below are kept as history.

> **Status: v1c, not yet tested on the car.** Car test 2 (v1b) showed the
> static frames are now right: their DTCs are gone and there was no emulator
> fault. But the DSC still degraded at ign + 1.15 s, and the remaining DTCs
> name the DDE's **dynamic** frames 0xE6, 0xFB and 0xE8. v1c adds those three
> frames and the pre-ign static values. It builds, and every offline test passes,
> including replays of car tests 1 and 2 and a byte/timing comparison with the
> real DDE in run 5. Do the next test exactly as described under
> [Test procedure](#test-procedure-in-the-car).

Firmware: `firmware/pico_flexray_dde_emu_v1c_ignon.uf2` (V1 board, Pico 2 / RP2350).
SHA-256 is in `firmware/SHA256SUMS`. The v1b and v1 UF2s are kept only for
reference: do not use them.

## Changelog

### v1c (after car test 2)

**What car test 2 (v1b) showed** (full report: `flexray-re-notes/analysis/f11_emu_run2.md`):

- **The statics are fixed.** 0x26/0x28/0x29/0x3D went out every cycle from
  0.18 s after wake, with correct E2E and counters. The pico reported
  `echo ok=40028 miss=0 foreign=0 late=0`, no fault. All of car test 1's
  static-frame DTCs (TORQ_CRSH_1, WMOM_DRV_*) are gone, and 0x107 b3 goes to 03 as
  with the real DDE.
- **DSC still degraded.** 0x2F b4 went 0e→06 at ign + 1.15 s, and b5 stayed 30.
  With the real DDE, b4 = 02 and b5 = 20 at ign + 1.05 s.
- **The remaining DTCs** (DSC, ICM, EPS) all name dynamic DDE frames:
  - DT_PT_2 (0xE6), first at ign + 1.53 s;
  - DT_PT_3 and voltage (0xFB);
  - ST_ENERG_GEN and SPEC_PWR_EL (0xE8).
- **Two limits of the run:**
  - It was not a fully cold wake.
  - The log stopped 15 s after ign off, so bus sleep could not be checked.

**Changes in v1c:**

- **Dynamic frames on FR1.** They use the same transmitter, pins and safety
  rules as the statics. Content is the real DDE's (Fred's v1c spec, from runs 4
  and 5); periods and cycle filters are listed below.

  | Frame | Payload sent | Every | Cycle |
  |---|---|---|---|
  | 0xE6 DT_PT_2 | `CRC 1c 80 64 45 44 23 47` (E2E, K=04) | 1 s (200 cycles) | %8 = 2 |
  | 0xFB L5 DT_PT_3 | `4c 60 f1 ff` | 5 s | %8 = 5 |
  | 0xFB L7 voltage | `Fc ff 47 32 00 f0` | 200 ms | %8 = 6 |
  | 0xFB L9 | `1f 00 04 00 00 f3 ff ff` (pre-ign `1f 00 ff 7f 00 f3 ff ff`) | 5 s | %8 = 3 |
  | 0xE8 L9 ST_ENERG_GEN | `00 00 00 9a 29 f3 56 f0` | 1 s | %8 = 3 |
  | 0xE8 L5 SPEC_PWR_EL | `86 78 fe fe`, only from 225 ms after ign on | 5 s | %8 = 0 |

  `c` is a counter that runs 0..14 (never 15). It starts at 1 after each wake.

  Not sent: 0x5C, 0x142, 0x10A, 0x68, 0x88, and the second 0xE6 buffer (base 1).
- **Dynamic-segment tracking, and skip when unsure.** In the dynamic segment, a
  frame's position depends on which other ECUs sent before it in that cycle.
  - Every FR1 header is placed on the minislot grid (6.876 µs per minislot,
    fitted on run 5). The firmware then predicts the minislot for 0xE6/0xE8/0xFB.
  - It sends **only** if every frame before ours in that cycle is explained,
    with a known length and on the grid, and the minislot is ≤ 260.
  - This is checked again about 4 µs before TX. If anything is unexplained (an
    undecoded frame, a missed header, an unknown length), the frame is skipped
    for that cycle, the reason is counted, and it is retried 8 cycles later.
- **Pre-ign statics.** 0x28 c1 b3–b5 = `c1 17 7c` and 0x29 c3 b8 = `80` until
  60 ms after the first 0x103 (ign on), then the ign-on values, as the real DDE
  does (40–90 ms).
- **Stats v3** (`0xa4`, 192 bytes; the first 128 bytes are the v2 layout) add sent
  and skip counts per dynamic frame, skip reasons, the last minislots and the
  ign state. They are shown on the logger's `emu:` line (see the table below).
  The old logger still works with v1c but does not show them.
- **Build switches** `EMU_DYN_E6`, `EMU_DYN_FB`, `EMU_DYN_E8` and `EMU_PREIGN`
  (all on).
- **Unchanged from v1b:**
  - the fault behaviour;
  - static timing and content after ign on.

**Cycle filters: capture versus DDE flash.** Three sources were compared, and
they agree:

- **The DDE flash** (Ian's decode, `dde_flash_eray_config.md`) gives base/rep
  filters:
  - 0xE6 0/2;
  - 0xE8 L9 1/2, L5 0/2;
  - 0xFB L5 1/4, L7 2/4, L9 3/4.
- **Fred's template** gives cycle%8 values. Each one lies inside the flash's
  base/rep class.
- **The run 5 real-DDE capture** shows these cycle%8 values:
  - 0xE6 0, 2, 4, 6 (even);
  - 0xE8 L9 3, 5, 7 (odd) and L5 0, 2, 6 (even);
  - 0xFB L5 1, 5, L7 2, L9 3, 7.

  All of them fit the flash filters.

There was nothing to correct, so v1c keeps the template values. The %8 phase
is only where the DDE's period happens to fall; the flash allows any cycle of
the class. The emulator retries a skipped frame 8 cycles later, so it never leaves
its class.

**Cross-check with the DDE flash** (Ian's decode; host test `test_dyn_flash`):

- **E2E.** 0xE6 is the only dynamic frame with a CRC. It uses data ID 0x37,
  CRC8 poly 0x1D, init 0 and xorout 0. The K=04 the emulator uses equals the
  CRC of (ID 0x37 + zeros). Every 0xE6 we can send carries exactly the DDE's
  CRC. The same check against the flash data IDs and CRC lengths passes for all
  nine static E2E groups.
- **No CRC on 0xE8 or 0xFB.**
- **Unused bits are 1** in every frame we send (0xFF fill, LSB-first).
- **Substitute values.** The 0xFB L7 12-bit field stays at its substitute
  0xFFF. The high nibble of 0xFB L5 b1 is part of a 12-bit value (0x604), not
  part of the counter.
- **Periods** are 1 s, 5 s and 200 ms, as in the flash.
- **No init images in flash.** Replaying capture payloads is the right model.

**Channel.** The DDE is configured for FlexRay channel A only. The "FR1/FR2" in
the captures are the pico's two ports, not FlexRay channels A/B. In run 5 the
real DDE sat on the pico's FR2 port and the car on FR1. The emulator sends the
dynamic frames through the same state machine and the same FR1 pins (TXD/TXEN
FR_1) as the statics, which car test 2 proved reach the car.

**Evidence (offline, `tests/host/run.sh`):**

- **Bytes against run 5's real-DDE frames.**
  - E2E: 273/273 frames.
  - Counters: 1506/1506.
  - FB L5, FB L9 and E8 L5 match exactly.
  - Cycle classes match.
  - Fields that vary between runs (0xE6 temperatures, the 0xFB voltage, 0xE8
    current) are frozen at the spec values.
- **Tracker against the whole run 5 bus.** 63 400 minislot predictions:
  - 98.6 % exact;
  - the rest later than predicted, all capture gaps;
  - **none earlier** (earlier would mean a collision).
- **Replays.** v1c runs on the PC against the car-side timing of car tests 1
  and 2 (460 and 381 dynamic frames):
  - grid error ≤ 0.03 µs;
  - no overlap;
  - bytes, counters, periods and pre-ign statics correct.
- **Unsure cases.** Car test 2 was replayed with injected unexplained frames,
  unknown lengths and missing anchors. 0xE6 is never sent in those cycles.
- **Timing against the real DDE (run 5).** 92 % of 2746 frames land at the same
  minislot and time. The rest are FR2 or capture artefacts. None is off-grid.

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
- **v1c:** it also sends the DDE's dynamic frames **0xE6, 0xE8 and 0xFB**
  (see the v1c changelog for content and timing). It sends them only in cycles
  where it is sure of their position. Otherwise it skips that cycle.
- The static content is a replay of the DDE's frames recorded in run 5 with
  **ignition on and the engine off**. There is one recorded frame per slot and
  per cycle (cycle 0..63). Until 60 ms after ign on, 0x28 and 0x29 carry the
  real DDE's pre-ign values.
- Every frame gets the current cycle number, a live alive-counter, a recomputed
  E2E checksum and recomputed header and frame CRCs, so it looks freshly made.
- It also keeps the timestamped capture, so `fr_ts_logger.py` records the whole
  bus (including our own frames) while it runs.

What it does **not** do:

- **Some dynamic frames are not sent:** 0x5C, 0x142, 0x10A, 0x68 and 0xBE.
  - 0x5C (minislot 0) is behind a build option, `EMU_DYN_5C=1`, which is **off**
    in the shipped UF2.
  - The flash also configures 0x88 and a second 0xE6 buffer. Neither has ever been
    seen on the bus, so neither is emulated.
- It does not start the bus or keep it awake by itself. It only transmits while
  it hears the car's own frames.
- It never transmits on FR2, and it does not forward FR1 to FR2 (this is not a
  repeater build).
- It does not use relays or any 12 V logic. GP17/GP18 are set exactly as in the
  stock firmware.
- It does not follow vehicle state. It sends "ign on, engine off", apart from
  the pre-ign values before the first 0x103.

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
| `dyn[E6= FB5= FB7= FB9= E8L9= E8L5=]` | v1c: dynamic frames sent. Per 5 s line with ign on, expect about +5 / +1 / +25 / +1 / +5 / +1 |
| `ms[E6= E8= FB=]` | v1c: minislot of the last dynamic frame sent (run 2: 138 / 148 / 163) |
| `ign=` | v1c: `off` (no 0x103 yet), `seen`, or `on` (ign-on content) |
| `skip[...] why[anchor mark passed len grid limit late] defer=` | v1c: dynamic frames held back because the position was not certain, by reason. A few are normal. Many in a row for one frame mean it is not reaching the car |
| `parks` | v1c: only shown if it differs from the FB L7 count (each 6-byte FB L7 restarts the TX state machine, which is expected) |
| `build=0x..` | v1c: only shown if a dynamic frame was switched off at build time |

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
v1h2/                     current release: 4 UF2s, SHA256SUMS, tools, patches 0020-0027, README
v1e/                      previous release (car run 6)
docs/notes/               car test write-ups, runs 6 to 10
CHANGELOG.md              what changed in each version
firmware/                 v1c UF2, old v1b and v1 UF2s, SHA256SUMS (history only)
tools/                    laptop tools: fr_ts_logger.py (with --emulate), frts.py (v1c: reads stats v3), reset_to_bootloader.py
source/                   complete firmware source tree (now at v1h.2) incl. tests and generator
  src/dde_emu*.{c,h,pio}  emulator firmware (dde_emu_tables.h is generated)
  tools/dde_emu/          table generator, Python model, test_dde_emu.py, test_logger_emulate.py
                          extract_replay.py (replay vectors from a logger CSV), extract_dyn.py (dynamic frames)
  tests/host/             C host tests (run.sh), incl. test_dde_emu_core.c + captured vectors
                          test_dde_emu_replay.c: dde_emu.c replayed against car tests 1/2 timing
                          dde_dyn_run5.txt: the real DDE's dynamic frames from run 5
patches/                  git format-patch series: ts-capture-fixed -> dde-emulator
BASE.md                   which upstream commit / local branch this is based on
```

## Laptop setup (Ubuntu, once)

The commands assume the tools live in `~/Downloads/FlexRay/fr-ts`. Copy
`tools/fr_ts_logger.py`, `tools/frts.py`, `tools/reset_to_bootloader.py` and
`firmware/pico_flexray_dde_emu_v1c_ignon.uf2` there. The new `fr_ts_logger.py`
and `frts.py` replace the old ones. **Copy both again for v1c.** `frts.py` is new;
`fr_ts_logger.py` is the same as v1b's, but keep the pair together. The old tools
still run v1c, but do not show the dynamic-frame counters. The new ones still
work with the capture firmware, v1 and v1b.

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
sha256sum pico_flexray_dde_emu_v1c_ignon.uf2
```

It must match the line in `firmware/SHA256SUMS`.

## Test procedure (in the car)

Do **not** start the engine during this test.

1. **Cold wake.** The car must be fully asleep: locked or left alone, ignition
   off, for **well over 60 s** after the bus went quiet (car test 2 was not a
   cold wake, which makes the start-up hard to compare). Prepare steps 2–5
   before that wait, or do them and then wait again.

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
   cp ~/Downloads/FlexRay/fr-ts/pico_flexray_dde_emu_v1c_ignon.uf2 /media/$USER/RP2350/
   ```

   The pico reboots by itself, and the GP20 LED blinks once per second. TX is
   off at this point.

5. **Start the logger with `--emulate`:**

   ```
   python3 ~/Downloads/FlexRay/fr-ts/fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu3.csv --emulate
   ```

   You should see `EMULATOR TX ENABLED`, then every 5 s a status line plus an
   `emu:` line. While the bus is asleep it shows `state=no-lock` and
   `0 frames/s`. That is correct.

6. **Ignition on (do NOT start the engine). Keep it on for at least 60 s.**
   Within a fraction of a second the `emu:` line should show:
   - `state=ACTIVE locked`;
   - the `sent[...]` counters rising by about 200 per second per slot;
   - `echo ok` rising, with `echo miss`, `foreign`, `late` and `lock_losses`
     staying at (or near) 0;
   - `echo_err avg` within about ±50 ns. The `max` value can show the
     error of the very first frame (up to a few hundred ns), but it should
     not keep growing;
   - v1c: `ign=on`, and `dyn[...]` rising by about E6 +5, FB5 +1, FB7 +25,
     FB9 +1, E8L9 +5, E8L5 +1 per 5 s line. A few `skip` counts are fine. If
     `dyn[E6=...]` does not rise at all, note the `why[...]` reasons.

   If it shows `FAULT` or the console prints `EMU FAULT ...`, the firmware has
   stopped sending. It re-arms by itself after 2 s if the cause is gone, and
   stays off after the 3rd fault. Note the `EMU FAULT` line and go to step 9.
   In the status line, frames from FR1 now include ours
   (0x26/0x28/0x29/0x3D, and 0xE6/0xE8/0xFB).

   **Pass check (in the capture, afterwards). All of these must hold:**
   - the car's **0x2F b4 = 02 and b5 = 20** at about ign + 1.05 s. Without a
     working DDE, b4 goes to 06 about 5.7 s after wake and b5 stays 30
     (car test 2: ign + 1.15 s);
   - **no DSC/ICM/EPS fault events.** These are frames
     `01 f0 <src> … 62 17 04 <DTC>`, found with Fred's `events.py`. Car test 2
     had DSC D35744/D35452, EPS D51414/D51744/D51C1B and ICM D01744/D01C1F;
   - **no 0xB4/0xCF/0xB7 events within 60 s** of ign on;
   - **0x3F c1 b4 = f3** (car test 2: f6);
   - **0x2B c3 b11/b12 = e0/f8** (car test 2: 60/d8);
   - **0x70 L9 = c8/20** (car test 2: d8/60).

7. **Watch the dash and the console** for a minute or two. Note any warnings or
   messages on the dash, for comparison with "DDE unplugged, no pico".

8. **Ignition off, and keep logging for at least 90 s.** The bus should go to sleep as it did in run 4
   (about 61 s after ign off). The status line drops to `0 frames/s` and
   `emu:` goes to `no-lock`. Do **not** stop the logger before 90 s after
   ign off (car test 2 stopped at 15 s, so sleep could not be checked). If the
   bus stays awake well past 90 s, the emulator is keeping it awake: stop the
   logger (Ctrl-C) and note it.

9. **Stop the logger** with Ctrl-C. It prints `EMULATOR TX DISABLED`, then
   `emu at stop: ...`. Keep these three files, from `~/Downloads/FlexRay/fr-ts/`:
   - `f11_emu3.csv`
   - `f11_emu3.csv.bin`
   - `f11_emu3.csv.console.log`

If anything looks wrong (dash goes mad, the car will not sleep, a FAULT
appears), press Ctrl-C in the logger first. TX is switched off at once.

### Fallback

Unplug the pico and reconnect the DDE. The pico has no relays and nothing in it
holds its state. Once unplugged, it is gone from the bus.

To put the normal capture firmware back on the pico, use
`pico_flexray_ts_capture_v1_FIXED.uf2` or `pico_flexray_passthrough_KNOWN_GOOD.uf2`
from the fr-ts folder, the same way as in step 4.

## Known risks / open points

- **v1c is not yet tested on the car.** v1b's statics worked in car test 2.
  The new parts are the dynamic frames and the pre-ign values.
- **Dynamic frames can collide in theory.** The position of our dynamic frames
  depends on the other ECUs' frames before them in the same cycle. v1c sends
  only when every one of them is explained. On the run 5 bus its prediction was
  never earlier than the real frame. A FlexRay node the pico cannot decode at
  all, or a bus-side length rule we have not seen, would still be a risk.
  Watch for `foreign`, for `echo miss`, and for any new DTCs on other ECUs.
- **Fixed dynamic values.** The 0xE6 temperatures, the 0xFB voltage (12.871 V)
  and the 0xE8 current/temperature are frozen at the spec values (run 4). The
  real DDE updates them.
- **0xE8 L5 start.** The real DDE starts it at ign + 75–125 ms, first with
  `86 78 00 f8`. v1c follows the spec: `86 78 fe fe` from ign + 225 ms.
- **If the DSC still degrades with 0xE6 present**, the next candidate is
  0x142 (not in v1c).
- **TSS length.** We send a 7-bit TSS. If the car's receivers expect a longer
  one, our frames are dropped: `echo ok` would rise but the dash would act as
  if the DDE were missing. If so, rebuild with `-DEMU_TSS_BITS=<n>`. Car test 2
  shows the statics are received.
- **Late alarms.** If one happens, the `late[...]` reasons and `last=` show
  which slot and why. A dynamic frame whose time has already passed is skipped
  (`why[late]`) instead.
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
| `EMU_DYN_E6` | `1` | v1c: send dynamic 0xE6 (DT_PT_2) |
| `EMU_DYN_FB` | `1` | v1c: send dynamic 0xFB L5/L7/L9 |
| `EMU_DYN_E8` | `1` | v1c: send dynamic 0xE8 L9/L5 |
| `EMU_PREIGN` | `1` | v1c: pre-ign values in 0x28/0x29 (and 0xFB L9) until 60 ms after the first 0x103 |
| `EMU_DYN_5C` | `0` | `1` = also send dynamic 0x5C (minislot 0, every 16th cycle); untested |
| `EMU_SYNC_BITS` | `0` | `1` = keep the captured sync/startup bits; **unsafe, untested** |

Tests (no hardware). `pioasm` comes from the pico-sdk build; pass it with
`--pioasm` or `PIOASM=`. The full-run checks also need the run 5 capture
(`--bin f11_run5.csv.bin`, or `DDE_RUN5_BIN=` for `run.sh`). The run 2 and
run 5 CSVs are needed for the v1c replay checks. Without a capture, the checks
that need it are skipped.

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
