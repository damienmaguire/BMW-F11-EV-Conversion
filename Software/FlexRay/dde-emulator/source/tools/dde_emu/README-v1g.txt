DDE emulator v1g.2  (pico-flexray, branch dde-emu-v1g)
======================================================

Firmware version: dde-emu-v1g.2      Logger version: v1g.2
The logger prints both at start. If they do not belong together it prints a big
"WARNING: VERSION MISMATCH" box - stop and copy the files from THIS zip.

What changed in v1g.2 (fixes the lost FlexRay frames of car run 7)
-----------------------------------------------------------------
Run 7 lost 187,933 FlexRay frames between the pico and the laptop ("usb_fifo_dropped").
The losses started when CAN TX was switched on. Cause: after every pass of its main loop
the pico asked the CAN chip "which of my frames have gone out?" (the TX event FIFO,
used for the dir=Tx log rows). That question costs time on the chip's SPI wire, and it
was asked about 30,000 times a second, even when nothing was waiting. In simulation it
took about 68 % of the processor's time, so the USB link to the laptop was serviced too
rarely and its buffer overflowed.
  1. The pico now asks the CAN chip only when (a) the logger asked for own-TX logging,
     (b) at least one sent frame is still unconfirmed, and (c) at most once per
     millisecond. Simulated cost: 2 % of the time (0 % with logging off), main loop
     about 3x faster (96,000 passes/s instead of 30,000). No dir=Tx rows lost in
     simulation (5,576 of 5,576 in 20 s). The timestamps of the dir=Tx rows are still the
     chip's exact send times; the rows can just arrive up to ~1 ms later.
     If the CAN chip never confirms frames (for example nobody on the bus answers),
     the pico stops waiting for them after 50 ms instead of asking forever.
  2. While the pico is busy pushing a burst of FlexRay frames, it now also keeps the
     USB link serviced (every 8 frames), so the laptop is fed during bursts.
  3. Bigger buffers: the FlexRay frame buffer holds 1024 frames instead of 256 (about
     0.2 s of traffic instead of 0.05 s) and the CAN record buffer 512 instead of 256.
     RAM used: V2 about 407 KiB of 512 KiB, V1 about 361 KiB.
  4. CAN chip SPI clock stays at 4 MHz (the speed used on every car run). The requested
     10 MHz is NOT allowed: the MCP251863 on the V2 board runs from a 16 MHz crystal,
     and the datasheet limit is 0.85 x 16 MHz / 2 = 6.8 MHz. With the fix above the
     faster clock is not needed. (A build switch V2_CAN_SPI_HZ exists for bench tests
     only; the build refuses anything above 6.8 MHz.)
  5. Version check: the firmware reports "dde-emu-v1g.2" (USB request 0xd6, printed in
     the console at boot); the logger prints "fr_ts_logger.py version v1g.2" at start,
     reads the firmware version and warns on a mismatch. "python3 fr_ts_logger.py
     --version" prints the logger version.
  6. check_emu_capture.py:
     - brake byte positions fixed: PT-CAN 0x173 brake = p[7] bits 0-1, 0x12F = p[6]
       (counting bytes from p[0]). The firmware already used these positions.
     - new section "0. capture health" reads the .console.log and FAILS when FlexRay
       frames were lost (usb_fifo_dropped, notify_dropped or ring_overflow above 0), when
       the logger is too old (no "own TX frames logged as dir=Tx" banner), on a version
       mismatch, or when the console log is missing. Option --console PATH if the log
       is not next to the capture.
     - still FAILS when the capture has 0 dir=Tx rows.
     Run 7 checked with the new checker: 3 FAILs (old logger, 187,933 frames lost,
     0 dir=Tx rows) and the brake now shows 3 press/release pairs on 0x173 and 0x12F.
  7. Plain-English comments in the changed code.
Everything else (what is sent on FlexRay / PT-CAN, the fake engine) is unchanged from
v1g (58cf388); all v1a..v1g host replays give the same bytes.

v1f plus an OPT-IN "fake engine start". With no diesel engine the car never sees "engine
running". v1g plays back a recorded crank -> run -> idle -> stop sequence when the driver
starts the car. The data comes from Ian's dde_start_sequence and Fred's
f11_fr_engine_running_diff (3krev / engstart captures). It is OFF unless the logger is
started with --fake-start. Without that flag v1g sends byte-for-byte what v1f sends
(host-tested).
Set-up is the same as runs 6/7: DDE unplugged and powered off, EGS fuse pulled, pico on FR1
(DDE harness) and, on the V2 board, on PT-CAN.

Files
-----
  pico_flexray_dde_emu_v1g_ignon.uf2     V1 board: FlexRay only (fake start on FlexRay)
  pico_flexray_dde_emu_v1g_ignon_v2.uf2  V2 board: FlexRay + PT-CAN (fake start on both)
  fr_ts_logger.py, frts.py               logger v1g.2 with --fake-start. Copy BOTH (replace
                                         v1f/v1g - the old logger does not log dir=Tx).
  check_emu_capture.py, dde_emu_model.py checker (--engine, capture health). Keep both in
                                         the same folder.
  v1g_engine_timeline_sim.csv            what the firmware will send (host simulation,
                                         20 ms steps: start at 3.6 s, stop at 22.5 s,
                                         restart, ...) - for comparison with the capture
  SHA256SUMS                             sha256sum -c SHA256SUMS

Built from the v1g.2 commit (branch dde-emu-v1g; v1g was 58cf388), pico-sdk 2.2.0, xPack
arm-none-eabi-gcc 14.2.1-1.1, pico2 / rp2350-arm-s, Release, same switches as v1f.

What the fake engine does
-------------------------
Only bytes of frames the emulator already sends change. No new IDs are sent. Every alive
counter, E2E CRC and FlexRay frame CRC is recomputed after the change. 0x592 (MIL /
check-control) never changes.
Times are from the start trigger (T0) and the stop trigger (TS):
  T0+50..155 ms   CounterHi 1 -> 3 on every DDE E2E group (FR 0x26/28/29/3D/E6, PT-CAN
                  0x0D9/0x3F9) = Ian's crank marker
  T0+95 ms        first rpm (~115 rpm, crank ripple) and crank torque
  T0+100 ms       0xE6 b3 80 -> 81 (cranking), sent at once (not on the 40 ms grid)
  crank           0x28 c3, 0x29, 0x3D and 0x142 take the cranking values from Ian/Fred
  T0+0.7..0.95 s  run-up; at 950 rpm 0xE6 = 82 (running). This is sent at once
                  (within 1 FlexRay cycle, usually 1-3 ms) because CAS probably waits for
                  0x82 before it lets go of the starter (D -> C). Real start capture:
                  D -> C came 8 ms after 82.
                  Then b4 66 -> 26, start-ready 0 (+20 ms), 0x142 running value.
  flare           1000-1060 rpm until +1.5 s, then settles to 780 rpm (+-1) =
                  0x28 c1 / 0x0A5 rpm = 30 0C (OBD 0x0C = 780), idle torque.
  first rpm +2 s  0xFB L9 b2 ramps 2D -> C8 (+4 per 100 ms, sent on every change)
  first rpm +2.8 s 0xE8 L9 running content, sent ~16/s
  first rpm +5 s  0xFB L9 b3..4 04 00 -> 00 00
  battery         0xFB L7 / 0x281: charging voltage ramps to 14.70 V (6C 39)
  PT-CAN (V2)     0x0A5 torque + rpm, 0x0F3 rpm/10 (lagged), 0x2C4 fuel counter, 0x281,
                  0x381, 0x3B3 = FB L9, 0x397 running walk, 0x3F9 = 0xE6 state,
                  0x08F = copy of FR 0x29
Stop. ST_KL 9 (engine-off request) sets 0x142 b8 A4 at +50 ms. The engine then stops at
ST_KL <= 8, or when 9 has been held for 1 s:
  TS+20 ms 0xE6 92 (stopping); TS+35 ms 0x29 C0; TS+240 ms FB L9 FF 7F;
  rpm reaches 0 at TS+1.2 s. Then 0xE6 80 / b4 64, and start-ready 1 20 ms later.
  After that the "after stop" values stay (0x28 c3, 0x29 7C/C0, F9 7C, 0x142 A4/24,
  FB L9 00 FF 7F, E8 L9 stopped content).
  A restart from "stopped" plays the whole sequence again.
The engine also stops when:
  - the PC goes away: no keepalive for 3 s (Ctrl-C does it at once);
  - ST_KL is not heard for 600 ms;
  - you type "x";
  - there is a new FlexRay wake (that resets everything to off).
A start that is aborted in the first 95 ms (before first rpm) leaves no trace.

Two start modes (choose one on the logger command line)
-------------------------------------------------------
  --fake-start cas       (try this first) the car's own start. Ignition on, foot on the
                         brake, press START. CAS sends ST_KL = D (starter request) and the
                         emulator answers within one FlexRay cycle. Needs ST_KL = D
                         (FlexRay 0x74 or PT-CAN 0x12F) and the brake (0x74 b7 / 0x12F
                         b6 / 0x173 b7 bits 0-1) seen within 2 s.
  --fake-start fallback  if CAS never sends D (for example because it waits for an EWS/DDE
                         release we do not send). Ignition on (ST_KL = A, NOT the START
                         button), hold the brake, type   s  + Enter   in the logger window.
                         The brake must be seen within 300 ms, and the command is valid for
                         2 s. A refused command is used up (no surprise start later); the
                         logger prints the reason ("start refused - no brake").
In both modes:   x + Enter = stop the fake engine,   ? + Enter = engine status.
The logger prints every engine change as "ENGINE: mode .. phase .. rpm .. E6 ..",
and adds an "engine:" line to every status line.

Flashing
--------
  python3 ~/Downloads/FlexRay/fr-ts/reset_to_bootloader.py   (or BOOTSEL + plug in)
  sha256sum -c SHA256SUMS
  V1:  cp pico_flexray_dde_emu_v1g_ignon.uf2    /media/$USER/RP2350/
  V2:  cp pico_flexray_dde_emu_v1g_ignon_v2.uf2 /media/$USER/RP2350/

Test (car) - fake start
-----------------------
0. Read and save the fault memory (all ECUs). Car in P, park brake on.
1. Car fully asleep (bus quiet well over 60 s). DDE unplugged, EGS fuse pulled.
2. Flash the V2 UF2. Start the logger before the wake:
     cd ~/Downloads/FlexRay/fr-ts
     python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu8_v1g.csv --emulate --can --can-tx --fake-start cas
   (V1 board: drop --can --can-tx.)
3. Wake, ign ON (no brake) for 20 s: nothing must change (engine: phase off).
4. Foot on the brake, press START once. Watch for "ENGINE: ... phase crank" then
   "phase run rpm ~1000 E6 82/66" and, within a few seconds, "rpm 780 E6 82/26".
   Note what the dash shows (rev counter, warnings, the READY/start text, gear selector).
5. Leave it "running" for 60 s. Press START again (engine off). Expect ST_KL 9 then 8,
   "phase stop", then "phase stopped".
6. If CAS never sent D in step 4 ("ENGINE:" line still shows ST_KL A / C), Ctrl-C, start
   the logger again with  --fake-start fallback  (same -o name with _fb, e.g.
   f11_emu8_v1g_fb.csv), ign on, hold the brake, type s + Enter. Then do step 5 with
   x + Enter, then ign off.
7. Ign OFF, keep logging for at least 90 s. Ctrl-C.
8. Check:
     python3 check_emu_capture.py ~/Downloads/FlexRay/fr-ts/f11_emu8_v1g.csv --engine
9. Read the fault memory again and compare.
If anything looks wrong: type x + Enter (engine off), Ctrl-C (all TX off at once), or
unplug the pico.

Pass criteria
-------------
- At start the logger window shows "fr_ts_logger.py version v1g.2", "firmware version:
  dde-emu-v1g.2" and NO "VERSION MISMATCH" box.
- check_emu_capture.py --engine: ALL PASS. In detail:
  * "0. capture health": usb_fifo_dropped=0 notify_dropped=0 ring_overflow=0, logger
    v1g.2 banner present (this is new in v1g.2 and was the run 7 failure);
  * at least one dir=Tx row (own PT-CAN frames logged);
  * all v1f criteria (own TX slots >= 99.5 %, 0x397, no foreign IDs, brake signals);
  * "rpm sent on PT-CAN" and "0xE6 b3 reached 82";
  * per start: "start at .. (ST_KL D): first rpm ~+95 ms, E6 81 ~+100 ms, E6 82 ~+0.95 s,
    D released ..." and PASS "0x82 sent before CAS released the starter";
  * idle 760-820 rpm (runs > 8 s); E2E CRCs 0 bad; 0x592 never changed.
- Logger at stop: no EMU FAULT, no CANTX FAULT, cantx late about 0, engine rejects 0
  (CAS mode) and starts = stops.
- No MIL, no new fault memory entries in DME/CAS/EGS/DSC/KOMBI caused by the fake start
  (record any new ones, they are the most useful result).
Keep .csv, .csv.bin, .csv.console.log, .can.csv and .savvy.csv.

Offline test results (host, all pass)
-------------------------------------
- Mode off: every FlexRay static/dynamic frame and every PT-CAN ID byte-identical to v1f.
  All v1a..v1f replays unchanged.
- Engine timeline (CAS) after ST_KL D: CounterHi=3 +55..160 ms, first rpm +100 ms,
  E6 81 +100, E6 82 +940 ms, b4 26 +1190, ready 0 +960, FB9 ramp +2.1 s (C8 at +6.0 s),
  FB9 00 00 +5.1 s, E8 fast +2.9 s, idle 779-781 rpm (e.g. 2D 0C). Stop
  after ST_KL 8: 92 +20 ms, C0 +35 ms, FF 7F +240 ms, rpm 0 +1.2 s, 80 +1.2 s, ready 1
  +1.22 s. Restart: the same timeline again. (Times include up to 5 ms of input timing.)
- 13,050 FlexRay E2E groups checked during start/run/stop: 0 bad. PT-CAN: all CRCs and
  alive counters valid, 0x0D9 CounterHi 3, 0x592 unchanged, no forbidden ID.
- Fallback: no brake -> refused (and the command used up), plain D ignored, A + brake +
  command -> runs. Stop command, PC lost (3 s), ST_KL lost (600 ms), abort while
  cranking: all stop correctly.
- Firmware replay of car run 2 (50 s, real bus timing) with injected 0x74 (D at 8 s,
  9/8 at 22 s, D at 26 s, 8 at KL15 off): grid/minislot/counters/order all clean, no
  fault/foreign; E6 82 on the bus 2.7 ms after rpm 950; 14 E6 event frames; E8 L9
  every 60.8 ms while running; 90 FB L9 frames; 2 starts, 2 stops, 0 rejects.
- check_emu_capture.py --engine on the real DDE start capture (F11_engstartthenstop):
  real ST_KL A -> C -> D, first rpm +124 ms, 81 +121 ms, 82 +2.30 s, CAS let go of D
  8 ms after 82 (this confirms the "send 82" assumption), real E2E 0 bad.
- Logger: --fake-start keepalive every 1 s, s / x / ? commands, engine off before
  emulator TX off on exit, refused by older firmware with a clear error.

Caveats / open points
---------------------
- v1g.2 is NOT tested on the pico or the car: the TEF change is checked only by a host
  simulation (fake CAN chip with SPI timing) and a clean compile with no warnings. On
  the bench, before the car, run the logger for a few minutes with --can --can-tx and
  check that usb_fifo_dropped stays 0.
- NOT tested on the car. The fake start only changes what the DDE says. It does not do
  the EWS/immobiliser release; the forbidden IDs are still blocked.
  If CAS/DME/KOMBI need the EWS handshake, the start will not be accepted. This is
  what the fallback mode and the capture are for.
- Dash warnings, a transmission or DSC fault, or a "drive train" message are possible. The
  car may also ignore it all. Record what happens; it is not a failure of the test.
- The values are a replay of the captured warm-ish start, not a model: no load, the
  temperatures are not warmed up (0xE6 b5/b6 unchanged), and no A/C or alternator load.
- The engine logic runs inside the FlexRay cycle interrupt (from RAM, ~4 KB). It was not
  measured on the chip; watch the logger's emu line (late counters, EMU FAULT).
- PT-CAN copies only change while the FlexRay side is stepping (stale > 0.5 s -> stock
  PT-CAN bytes).

Open questions for Ian (gaps the capture should answer)
-------------------------------------------------------
 1. EWS / C-D gating: does CAS go A -> C -> D without the DDE EWS reply? (Real:
    C came about 100 ms before D.) If not, the fallback mode is the only start.
 2. KL50 release: real D -> C came 8 ms after 82; with us at +0.95 s, does CAS do the same?
 3. 0xE6 start-ready 1 vs 2 (b7 high nibble) before the start / after the stop.
 4. Check-control 14/18 (0x592 / KOMBI) when "running" without a real DME handshake.
 5. K-CAN side (KOMBI rev counter): not emulated; only what KOMBI takes from FR/PT-CAN.
 6. Stop with KL15 kept on (ST_KL 9 only): we stop after 1 s of 9. Is that right?
 7. Warm idle: real 780-880 rpm; we hold 780.
 8. FB L9 ramp: Fred +2.0 s after first rpm; 3krev 0x3B3 differs a little. Which one?
 9. 0x2C4 fuel counter rate: approximate.
10. 0x08F CounterHi during the crank marker (we do not change it).
11. Does the car need the 15/s 0xE8 L9 rate at all?
12. 0x5C0 / 0x1B3 KL50 flags: not sent (0x5C0 is forbidden); PT-CAN2: nothing sent.
13. E6 event frames go out in the next free 0xE6 minislot in ANY cycle (not only
    cycle%8 = 2). The real DDE seems to do the same (81/82 off the grid); please confirm.
14. 0x397 d2 is random 5F..79 every 60 ms while running (3krev looked random).
15. PT-CAN bytes fall back to stock if the FlexRay side stops stepping for > 0.5 s.
16. 0xE6 b4 0x66 during 92 (stopping): taken from Ian; Fred's capture shows the same?
17. The 0xE4 change at +32 s is not modelled.
18. Temperatures (0xE6 b5/b6, PT-CAN 0x2C4/0x3F9) stay cold/stock.
19. Real cold start 82 came at +2.3 s (F11_engstartthenstop) vs 1.0 s (3krev); we use
    0.95 s. Is CAS sensitive to a fast 82?
