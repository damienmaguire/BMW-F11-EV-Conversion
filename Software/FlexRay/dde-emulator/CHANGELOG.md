# DDE emulator changelog

Newest first. Each release folder has its own README with the files, hashes
and how to run it. The car test write-ups are in [`docs/notes/`](docs/notes/).

## v1h.2 (current) – [`v1h2/`](v1h2/README.md)

Tested in the car in run 10 (10a and 10b): **the car behaved like the
original.** No fault codes from any control unit, no flickering frames, and
0.03 % FlexRay echo misses (run 9: 12.5 %). Brake gating of the gears works.

What v1f to v1h.2 added on top of v1e, in plain English:

- **Fake engine start through CAS (v1g).** Brake + START: CAS asks for a start
  (ST_KL D), the emulator reports cranking and then a running engine at idle on
  FlexRay and PT-CAN, and CAS ends the start request. Opt-in with the logger's
  `--fake-start cas`. First tried in run 8, and it worked.
- **Short start profile (v1h).** "Running" is reported about 0.17 s after the
  start request instead of ~0.9 s, as the starter fuse is out. `realistic` and
  `run8` profiles are still selectable with `--start-profile`. The logger and
  console warn loudly if CAS gives up on a start.
- **Gear shifting P / R / N / D (v1h.1, V2 board).** The emulator reads the
  lever (0x197) and the brake (0x173) and changes the gear in the frames it
  already sends, using the real EGS's timings. Leaving P, and P/N to R/D, need
  the brake and the (emulated) engine running; the P button gives P. A push
  without the brake is refused and logged. `--no-gear` keeps it in P.
- **Standalone build (v1h.1).** Separate UF2s that start emulating on power-up,
  with no laptop.
- **USB 1 KB transfers (v1h).** The logger link was dropping up to 13 % of the
  captured frames (runs 7 and 8). With 1 KB USB transfers, runs 9 and 10
  dropped none.
- **Own CAN transmissions are logged (v1f)**, so a capture shows what we sent.
  The 20 ms PT-CAN frames now go out at the full rate (v1e missed ~2.5 %).
- **Stale-frame fix (v1h.2).** In run 9 some PT-CAN frames briefly went out
  with "engine off / P" content, which made CAS flip to P and the bonnet
  warning flap. The PT-CAN side now always reuses the last good engine/gear
  state. 0 flickers in run 10.
- **Interrupt fixes (v1h.2).** The gear and engine logic moved out of the
  FlexRay header interrupt, everything an interrupt touches now runs from RAM,
  and a 70 µs re-arm guard stops last-moment timer changes. FlexRay echo misses
  went from 12.5 % (run 9) to 0.03 % (run 10).
- **Checker** `check_emu_capture.py`: `--engine`, `--gear` and `--flicker`
  checks of a capture.

Known small issues, to be fixed in v1h.3 (none was visible in the car):

- 0xE6 is sometimes sent in odd cycles; the real DDE uses only even cycles.
  This delays ICM/DSC readiness by about 0.7 s after a start.
- The 0xE6 0x92 "engine stopping" state goes out about 0.7 s late.
- False "lever 0x197 lost" alarms on the console.

## v1h.1 – gears, standalone (run 9)

Gears worked and the car accepted them, but two regressions showed: stale
PT-CAN frames and 12.5 % FlexRay echo misses. Both fixed in v1h.2. Not kept
as a separate folder; see git history and `v1h2/README.txt`.

## v1h – USB 1 KB transfers, start profiles (not run in the car)

## v1g / v1g.2 – fake engine start (run 8)

The start worked. USB still dropped 11 % of the capture, and echo misses rose.

## v1f – PT-CAN timing, own-TX logging (run 7)

The car accepted it; 20 ms frames at full rate. Run 7 used the old logger,
so own TX was not logged.

## v1e – [`v1e/`](v1e/README.md) (run 6)

First full FlexRay + PT-CAN DDE/EGS emulation. Clean dash, P shown, MIL off,
normal bus sleep.

## v1c and older

FlexRay-only versions, described in [`README.md`](README.md). Their UF2s are in
`firmware/` for reference only.
