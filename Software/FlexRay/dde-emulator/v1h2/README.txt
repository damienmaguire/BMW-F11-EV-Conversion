DDE emulator v1h.2  (pico-flexray, local branch dde-emu-v1h, on top of v1h.1 = d3ebc30)
=====================================================================================

v1h.2 = v1h.1 + fixes for the two run 9 problems (stale/flickering PT-CAN gear and
engine frames; 12.5 % FlexRay echo misses) + Ian's and Fred's run 9 timing notes.
The sections "v1h.2: ..." are new. Everything from "v1h.1: gears" on is the v1h.1 text,
kept for reference; where it disagrees with a v1h.2 section, v1h.2 wins.

Firmware version: dde-emu-v1h.2 (standalone: dde-emu-standalone-v1h.2)   Logger version: v1h.2
Copy fr_ts_logger.py AND frts.py from this zip (the logger checks that they match the
firmware and shows a big "VERSION MISMATCH" box otherwise).

NOTHING in this release has run on hardware yet. Checked in host simulation, on the
run 9 recording (replayed) and by static analysis of the built firmware only.
Nothing changed on the immobiliser IDs (0x592, 0x5C0, 0x598).
We still never transmit on PT-CAN2, never on 0x137 or 0x6xx, never any lever (0x197) frame.

How to run (car), v1h.2
-----------------------
  python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu10_v1h2.csv --emulate --can --can-tx --fake-start cas --start-profile short
(add --no-gear for fixed P). Check afterwards:
  python3 check_emu_capture.py ~/Downloads/FlexRay/fr-ts/f11_emu10_v1h2.csv --engine --gear --flicker

v1h.2: 1. Stale / flickering PT-CAN frames (run 9: 0x39A/0x3FD/0x3F9/0x0DC/0x397)
---------------------------------------------------------------------------------
Cause: core 0 (PT-CAN) reads the engine/gear state that core 1 writes. When a read
landed in core 1's write window, v1h.1 fell back to the stock template (= P / engine
stopped) for that one frame.
Fix:
  - Core 1 now computes the whole gear and engine status BEFORE the shared-data write
    window opens; the window itself is only a ~64-byte copy (about 60 instructions,
    under 1 us, interrupts off).
  - Core 0 retries with a short bounded wait; if the read still fails or is torn it
    REUSES THE LAST GOOD SNAPSHOT. Once emulation has produced data we never fall back
    to stock/park frames again.
  - 0x39A change detection compares against the frame we INTEND to send, not the last
    one sent.
  - New counters on the "emu diag" line: "snapshot read fails N" and
    "background step max N us".
New checker test (check_emu_capture.py --flicker): FAIL on ANY single-frame A -> B -> A
on 0x39A / 0x3FD / 0x3F9 / 0x0DC / 0x397 - no settle window. Only two exceptions: the
stock 0x397 e2<->e3 blip, and on 0x39A/0x3F9/0x0DC a short REAL gear that 0x3FD also
shows (>= 2 frames).
  run 9 capture (v1h.1, real):        277 / 2 / 2148 / 8 / 993 flickers -> FAIL
  run 9 inputs replayed, v1h.1 model: 1627 / 7 / 10558 / 44 / 2405 flickers
  run 9 inputs replayed, v1h.2:       0 / 0 / 0 / 0 / 0, with 3 % AND with 30 % of
                                      snapshot reads forced to fail
  (order: 0x39A / 0x3FD / 0x3F9 / 0x0DC / 0x397)

v1h.2: 2. FlexRay echo misses (run 9: 12.5 %, slot 0x3D only 52 %)
-------------------------------------------------------------------
Root cause found: in v1h.1 the whole FR1 header interrupt ran with interrupts OFF,
including the engine and gear step (reading their tables from flash). While it ran
(up to 189 us in run 9), the TX timer interrupt could not fire, so frames went out
late or not at all; 91 % of the misses were slots whose timer had been re-armed
less than 40 us before the slot.
Fix:
  - All gear and engine logic moved out of the interrupt into core 1's background
    loop (runs right after every interrupt). For engine/gear the header interrupt
    now only signals "new cycle" (two instructions). Its remaining work is a strict subset of what
    v1h (7514b68, before gears) did in it, so its worst case is back to v1h level
    or below. For scale, the engine+gear work removed from it measured ~780
    instructions (~8 us) under emulation of the real firmware, plus flash stalls.
  - Everything an interrupt can reach is now in RAM (tables, payloads, E2E tables,
    the frame counter, small copies that the compiler had turned into flash reads).
    A static check of the built firmware finds flash accesses only on rare paths
    (first init, fault/reinit, wake-up clear, the MITM injector).
  - Re-arm guard ON: DDE_EMU_RESCHED_MIN_US = 70. An already armed slot is not
    re-armed by a reference that arrives less than 70 us before it (run 9: 0x26,
    armed ~165 us early, had 0 misses; 0x28/0x3D were re-armed 14-15 us before the
    slot, 0x29 47 us). -DDDE_EMU_RESCHED_MIN_US=0 gives v1h.1 behaviour.
  - 0xE6 event copies no longer shift its 1 s grid (Fred: EPS D51744 came from
    0xE6 gaps > 2 s). Host model: longest 0xE6 gap with KL15 on = 1.000 s.
Host model (cannot model interrupt latency!): 0x26/0x28/0x29/0x3D and 0xE6 100 %
sent, 0 echo misses; 0x3D carries the CounterHi 3 start window like 0x29 (run 9's
missing 0x3D c3 frames were TX misses, not a wrong counter).
USB (Fred's suspicion of the 1 KB transfers): USB runs only on core 0 and shares no
lock with the FlexRay interrupts on core 1. The only coupling found was the flash
cache (core 0 busy with USB/printf evicting code the core 1 interrupts read from
flash) - removed by the RAM move above. USB work never runs in the TX-critical path.
The bench A/B below checks this on hardware.

v1h.2: 3. Gear timings (Ian's run 9 notes, real EGS values in brackets)
------------------------------------------------------------------------
  Leaving P: 0x39A E1 and 0x3F9 P-lock 1 20 ms after the FIRST notch (real +20).
  P -> R: straight to R 40 ms after the second notch (real +39; v1h.1 showed N at
    +45 and R at +108 - the extra ~70 ms is gone). N only if forward 1 is held
    >= 40 ms.
  Gear code on 0x39A / 0x3F9 / 0x0DC follows 0x3FD by 10 ms (real +2 .. +21 ms;
    e.g. P -> D: 0x39A 05 ~10 ms after 0x3FD D). FlexRay 0xE6 moves with the gear code.
  P button: 0x39A 01 +21 ms, 0x3F9 +42 ms, 0x39A 03 +881 ms (19 ms before P),
    0x3FD P +900 ms.
  Pawl (0x39A E2, 0x3F9 P-lock 2): 340 ms after brake release, 3.0 s after the
    P button at the latest (host test: 331 ms).
  0xE6 b4 never shows 16 in transit (stays "P locked" until the gear code moves).
  Replay of the real r10md10m lever/brake frames: 4 of 4 shifts match the real EGS.

v1h.2: PT-CAN2 logging
----------------------
Not possible with this hardware: the V2 board has exactly one CAN controller and
transceiver (MCP251863), and it is on PT-CAN (where we transmit). Logging PT-CAN2
would need a second CAN interface (e.g. a separate receive-only USB-CAN adapter on the
laptop). We never transmit on PT-CAN2.

v1h.2: Fred's other points
--------------------------
  0x142 byte 12: NOT driven (not clearly understood yet; unchanged).
  0x21A: sent by the car, not by us; the inputs we provide to it are now flicker-free.
         Check that it is steady in run 10.

Run 10 test list (V2 board, engine "running" via --fake-start cas)
-----------------------------------------------------------------
  1. Start the logger (command above), fake start (brake + START).
  2. Brake on: forward 2 = R (no N on the way). Back 1 = N, HOLD N for 5 s. Back 1 = D.
  3. R -> D without passing P: from R, back 2 = D.
  4. P button = P (after ~0.9 s); keep the brake 2 s, then release it: note when the
     pawl clicks/dash changes (expected ~0.34 s after release).
  5. WITHOUT the brake: from P try forward 2 and back 1. Expected: refused, stays P,
     console "GEAR: refused, brake not pressed".
  6. Do everything with the bonnet CLOSED.
  7. Note dash/selector/gong/warnings with times. Afterwards run the checker with
     --engine --gear --flicker; in the "emu diag" line look at miss/late per slot,
     "hdr ISR max", "snapshot read fails", "background step max", "guard".

Bench A/B for the echo misses (no car, receive-only bench or replay rig, 10 min each)
-------------------------------------------------------------------------------------
  A  v1h.1 (dde-emu-v1h.zip, ignon_v2)
  B  v1h.2 pico_flexray_dde_emu_v1h2_ignon_v2.uf2
  C  bench-ab/pico_flexray_dde_emu_v1h2_ignon_v2_guard0.uf2   (v1h.2, guard OFF)
  D  bench-ab/pico_flexray_dde_emu_v1h2_ignon_v2_usb64.uf2    (v1h.2, 64-byte USB)
Compare from the "emu diag" line: miss/late per slot, no-cand, hdr ISR max, alarm ISR
max, snapshot read fails, background step max, guard skips; and usb_fifo_dropped.
B vs A = the fix; C vs B = what the guard adds; D vs B = whether 1 KB USB matters.
Target: misses < 0.5 % overall, 0x3D 100 %.

Files
-----
  pico_flexray_dde_emu_v1h2_ignon.uf2          board V1
  pico_flexray_dde_emu_v1h2_ignon_v2.uf2       board V2 (CAN chip on board)
  pico_flexray_dde_emu_v1h2_standalone.uf2     board V1, standalone (USB power supply)
  pico_flexray_dde_emu_v1h2_standalone_v2.uf2  board V2, standalone
  bench-ab/*.uf2                               bench A/B only (see above), not for the car
  fr_ts_logger.py, frts.py                     logger v1h.2 (copy BOTH)
  check_emu_capture.py, dde_emu_model.py       checker (new --flicker)
  SHA256SUMS

Risks (v1h.2)
-------------
- Not run on hardware. The host model cannot model interrupt latency: the echo-miss
  fix is argued from the code (interrupts-off finding, ISR work now a subset of v1h,
  no flash in interrupts) - the bench A/B / run 10 must confirm it.
- Engine/gear now run in core 1's background loop. If core 1 is starved by interrupts
  the step is late (not lost); watch "background step max" (expect < 50 us).
- Guard 70 us: keeps the earlier timer arm when a re-arm comes late; if the earlier
  arm was wrong, that frame is a bit off rather than missing. Untested on hardware.
- Held N, the brake rule, and pawl timing are partly hypothesis; held N never seen.
- PT-CAN2: we still send nothing there (GWS may complain); we cannot log it either.
- 0x142 byte 12 not driven.
- Standalone emulates as soon as it has power.

================ Earlier text (v1h.1 / v1h), kept for reference ================

v1h.1: gears (P / R / N / D)
----------------------------
What it does: the pico now reads the driver's gear lever (GWS frame 0x197) and the
brake (0x173), both RECEIVED on PT-CAN only, decides the gear like the real EGS, and
puts that gear into the frames we ALREADY send (0x3FD, 0x39A, 0x3F9, 0x0DC on PT-CAN;
FlexRay 0xE6 and 0x28). We never send 0x197 (lever) frames, nothing on PT-CAN2,
nothing on 0x137 or 0x6xx, and the immobiliser IDs (0x592/0x5C0/0x598) are unchanged.
Source of truth: Ian's flexray-re-notes/analysis/egs_gear_report_tx.md.

Rules (src/dde_gear.h has the full plain-English description):
  From P: forward 1 = N, forward 2 = R, back = D.   From R: back 1 = N, back 2 = D.
  From N: forward = R, back = D.   From D: forward 1 = N, forward 2 = R.
  P button = P.  Leaving P and P/N -> R/D need the engine running (our fake engine)
  AND the brake pressed. Engine stops in R/N/D -> back to P automatically.
  We act 40 ms after a lever notch appears, on the deepest notch of that push (a notch
  passed through in ~21 ms on the way to the 2nd notch is skipped). No valid 0x197 for
  90 ms = lever lost: we hold the gear and log it.
  P -> R shows N on 0x3FD for 60 ms first (like the real EGS); leaving P sends 0x39A E1
  and 0x3F9 bits 28-29 = 1 for 60 ms (~100 ms after the lever).
  P button: 0x39A gear 01 at +40 ms, old gear still on 0x3FD, P shown at ~+900 ms, then
  the pawl (0x39A E2, 0x3F9 bits 28-29 = 2) when the brake is released or 3 s after the
  button at the latest.
The pico console and the logger print lines such as
  GEAR: P -> R (brake on) lever forward 2
  GEAR: refused, brake not pressed (P -> D, lever back 1)
  GEAR: refused, engine not running (P -> R, lever forward 2)
  GEAR: P pawl confirmed (0x39A E2, 0x3F9 bits 28-29 = 2)
  GEAR: lever 0x197 lost (no valid frame for 90 ms), holding D
The logger status line has "gear: ..." and at stop "gear at stop: ...".

Switching gears off (fixed P exactly as v1h, byte for byte - host-tested):
  at run time:   logger option  --no-gear
  at build time: cmake -DEMU_GEAR=0

What is CAPTURED (copied from the real EGS, F11_4log_r10md10m) and what is a HYPOTHESIS:
  CAPTURED  P, R, D bytes on 0x3FD / 0x39A / 0x3F9 / 0x0DC / 0xE6 / 0x28; 0x3FD N = 60;
            the 40 ms delay; the skipped pass-through notch; the 90 ms lever timeout;
            N on 0x3FD for 60 ms during P -> R; the P-button sequence (39A 01, P on
            0x3FD at +900 ms).
            Replay test: the real lever/brake frames of the capture fed into our gear
            logic give the same 0x3FD changes as the real EGS: 4 of 4, within +0, +5,
            +2 and +21 ms.
  HYPOTHESIS
    - N bytes other than 0x3FD 60 (0x39A gear 01, gear nibble 1, 0x0DC/0x28 22 FE):
      N was only ever seen for 60 ms in transit.
    - The brake rule: no shift without the brake was ever captured. We refuse it.
    - No road speed is read: the emulator assumes the car is standing still.
    - Pawl timing: Ian's rule "brake release or 3 s" is implemented. In the capture the
      pawl (39A E2) came +3.0 s after the P button, i.e. 340 ms AFTER the brake was
      released (+2.7 s) - so in the real car it looks like "3 s", not "brake release".
    - Leaving P: we send 0x39A E1 at +40 ms after the lever; the real EGS sent it at
      +20 ms (0x3F9 bits 1 at +22 ms). Small, noted.
    - Torque/load bytes of the EGS (PT-CAN2 0x0A0 / 0x0B0 / 0x1AF) are not emulated.
  RISK: PT-CAN2. The real EGS sends 0x3FD on PT-CAN AND PT-CAN2, and Ian thinks the GWS
    may read both (GWS DTC E09400/E09410). We do not transmit on PT-CAN2 at all, so if
    PT-CAN2 is still wired to the GWS, expect a GWS fault or a selector that does not
    follow us. Watch for it. [HYPOTHESIS]
  Board V1 has no CAN chip: it cannot read the lever or the brake, so it stays in P.

Note on timings: our capture loader (gear-analysis/load.py) adds a false +1 s to rows
that arrive out of order around a 1-second wrap. That caused the earlier "+1880 ms" /
pawl confusion. The test vectors (tests/host/gear_vectors_r10md10m.txt) were made with
a fixed unwrap; the corrected timings match Ian's report.

Test in the car (V2 board, engine "running" via --fake-start cas):
  1. Start the logger (command below) and do the fake start as before (brake + START).
  2. Hold the brake. Lever: forward 2 = R (dash and selector show R; on the way N for a
     moment). Then back 1 = N, back 1 again = D. Then press the P button = P after about
     1 s. Release the brake.
  3. WITHOUT the brake: try forward 2 / back 1 from P. Expected: stays in P, console
     "GEAR: refused, brake not pressed".
  4. Watch the dash gear display and the selector LEDs at every step, and note anything
     odd (gong, warning, selector blinking, GWS fault). Note the times.
  5. Stop the logger, then run the checker with --gear (below).

v1h.1: standalone (no laptop needed)
------------------------------------
Separate firmware: pico_flexray_dde_emu_v1h_standalone.uf2 (V1) and
pico_flexray_dde_emu_v1h_standalone_v2.uf2 (V2). On power-up it starts emulating by
itself with the same settings as the logger command below:
  --emulate, PT-CAN TX on (V2 only), --fake-start cas, --start-profile short, gears on.
It does not wait for USB and never blocks on USB. The firmware safety timeouts (the
host must refresh emulation within 20 s, CAN TX within 20 s, the engine mode within
3 s) are still there - the board itself refreshes the settings every 1 s.
LED: slow blink (0.5 s) = emulating; fast blink (0.1 s) = fault (a latched fault such
as a TX error stops emulation; only a power cycle clears it in standalone); a short
flash every 2 s = waiting (not emulating yet).
A laptop can still be plugged in and log as usual. If the laptop sends a command
(e.g. the logger starts with other options), the laptop's settings win, and the board
takes over again 25 s after the laptop's last command.
Board V1 is powered from USB only: standalone on V1 needs a USB power supply (phone
charger) on its USB port. V1 has no CAN chip, so no PT-CAN TX and no gears on V1.
Host test: the standalone build starts with no host at all and is emulating 0.2 s after
power-up; the fake start then runs normally.

How to run (car), v1h.1
-----------------------
  python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu9_v1h1.csv --emulate --can --can-tx --fake-start cas --start-profile short
(add --no-gear for fixed P as v1h). Check afterwards:
  python3 check_emu_capture.py ~/Downloads/FlexRay/fr-ts/f11_emu9_v1h1.csv --engine --gear
The --gear section lists every lever push with the brake state and what we did (and
after how many ms), every change of our gear bytes, the console "GEAR:" lines, and
checks that 0x3FD / 0x39A / 0x3F9 / 0xE6 agree, that R/N/D only appear while our engine
runs, and that the P pawl bytes agree.

What is new in v1h
------------------
1. USB: bigger USB transfers (PICO_FR_USB_FAST, now ON in the emulator build)
   Run 8 lost 90,960 of 829,409 FlexRay frames (11 %) INSIDE the pico
   ("usb_fifo_dropped"): with 64-byte USB transfers the link tops out at about
   217 kB/s, and run 8 needed about 250 kB/s at its peaks.
   v1h sends 1 KB per USB transfer (16 KB send buffer). A timing model of the USB path
   (tests/host/usb_fast_model.py, calibrated on the 217 kB/s we measured) gives:
       64-byte transfers, 250 kB/s (run 8 peak)   -> 12.4 % lost (run 8 really: 11 %)
       1 KB transfers, 300 kB/s                   ->  0 % lost
       1 KB transfers, 600 kB/s (2x the target)   ->  0 % lost
       1 KB, 300 kB/s with the pico 4x slower to service USB, or only every 8 frames,
       or a slow laptop (10 packets/ms) at 300 and 450 kB/s -> 0 % lost
   Laptop side: the logger already reads 64 KB at a time and keeps partial reads
   (pyusb >= 1.3 returns the data even when a read times out). A host test replays
   4 MB of the run 8 raw dump through the logger with 64 KB, 1 KB and 777-byte read
   sizes: identical CSV files, and the parser runs at about 5,600 kB/s (19x 300 kB/s).
   CAVEAT: the 1 KB transfer mode has never run on the pico. The model is a model.
   Bench test it (below) before relying on it. Capture-only builds keep the old 64-byte
   mode unless built with -DPICO_FR_USB_FAST=ON; -DPICO_FR_USB_FAST=OFF turns it off.

2. Shorter fake start, selectable ("start profile")
   Damien has pulled the starter fuse, so the start no longer needs to look like a
   real crank. New logger option  --start-profile  (default: short):
     short      crank marker and first rpm in the first FlexRay cycle after ST_KL D,
                quick run-up; "running" (0xE6/0x3F9 = 82) together with start-ready 0
                at about D+170 ms (host test: 150..200 ms). Faster than any real start
                we have seen (fastest real: D+851 ms), so the CAS might refuse it.
     realistic  crank marker +45 ms, first rpm ~+90 ms (real +51..59 / +94 ms),
                82 at about D+840 ms (just inside the fastest real start).
     run8       exactly the start that passed run 8 (82 at D+941 ms), for A/B tests.
   We are never "running" before START: every profile waits for the start trigger
   (ST_KL D with brake in --fake-start cas mode).
   Fred's points, in short and realistic (run8 keeps its old behaviour on purpose):
     - 0xE6 b4 = 26 from the very first 82 frame (no 250 ms of 66);
     - 0xE8 L9 at about 5.7/s while cranking (real 4-7/s; v1g.2 sent ~1/s);
     - 0x3FB d4 f1 -> f0 during the crank ("injecting"; the gateway copies it to
       FlexRay 0xE9 b5);
     - 0xE6 0x92 (stopping) within 20 ms of ST_KL 8 (real +19 ms).
   If the CAS refuses: in short/realistic, if ST_KL D ends before our 82 the start is
   abandoned (we go back to "not running"). Both the pico console and the logger print
     ENGINE: *** CAS ended the start request (ST_KL D -> C) N ms after START, BEFORE
             our running report - CAS refused / gave up on our start ***
   and, if D is still held 300 ms after our 82,
     ENGINE: *** CAS still requesting start 300 ms after our running report ... ***
   The engine status line also shows "profile ..., START->rpm, START->82, START held,
   CAS ENDED-EARLY / HELD-LONG, aborts".

3. Echo misses (0x28 misses went 1 -> 343, late frames 0 -> 15 from v1f to v1g.2)
   Root cause NOT found. What we ruled out: no flash access in the interrupt code, not
   the engine content, not start-up effects. The missing 0x28 frames were fired by the
   pico but are absent both in our own receive echo and in the capture, i.e. they most
   likely did not go out correctly. Main suspect: 0x28 is re-scheduled by 0x27 only
   ~7 us before its timer must be armed (only a 7 us gap after 0x27).
   v1h adds counters so the next run can tell (logger line "emu diag ...", every
   status interval and at stop; USB request 0xaa):
     miss/late per frame (0x26/28/29/3D); details of the LAST miss: slot, cycle,
     "re-armed<40us", "header CRC fail", "no-candidate IRQ", "alarm inside the header
     interrupt", "previous frame still on the wire", timer values before/after re-arm,
     interrupt lateness, who armed it and how early; max header-interrupt and
     timer-interrupt durations; how many misses had each cause.
   A possible fix exists but is OFF and untested: -DDDE_EMU_RESCHED_MIN_US=40 skips a
   re-arm that comes less than 40 us before the slot (keeps the earlier timing).

4. check_emu_capture.py
   - 0x397: the ~380 ms spacing is checked only while the engine is NOT running; while
     running the real DDE sends it every 60 ms, which is now checked separately
     (run 8: 482 ms stopped, 60 ms running, both PASS - run 8 was wrongly marked FAIL).
   - per start: crank nibble and first-rpm latency vs the real DDE (+51..59 / +94 ms),
     0x3FB f0 during the crank, 0x1B3 p[4] bit4 (Ian: KL50) during D (reported only;
     run 8: never set), 0x92 within 20 ms of ST_KL 8, "CAS REFUSED" / "CAS did not
     accept" lines. The profile is read from the console log.
   - run 8 now gives exactly one FAIL: the USB loss (fixed by item 1).

How to run (car)
----------------
  python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu9_v1h.csv --emulate --can --can-tx --fake-start cas --start-profile short
then, if the CAS refuses the short start, the same with  --start-profile realistic
(new -o name), and if needed  --start-profile run8 .
Check afterwards:
  python3 check_emu_capture.py ~/Downloads/FlexRay/fr-ts/f11_emu9_v1h.csv --engine
Needs pyusb 1.3 or newer on the laptop ("python3 -c 'import usb; print(usb.__version__)'").

Bench test (before the car)
---------------------------
Flash, connect the logger with --emulate --can on a bench FlexRay/PT-CAN setup (or a
recording replay rig) for 10 minutes and check in the console log: usb_fifo_dropped=0,
notify_dropped=0, ring_overflow=0, and the logger's kB/s figure. If anything is lost,
rebuild with -DPICO_FR_USB_FAST=OFF (v1g.2 USB behaviour) and report.

TODO
----
- 0xBE / UDS 22 F101 identification burst after the start (Fred): not sent. It is a
  diagnostic answer we do not understand yet; we only add it once we know who asks and
  what the answer must contain.
- Ian's 0x1B3 KL50 bit: watch what the CAS sends during D (checker reports it).
