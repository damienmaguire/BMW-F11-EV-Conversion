# Base of the DDE emulator

| What | Where | Commit |
|---|---|---|
| Upstream | https://github.com/dynm/pico-flexray, branch `feature/frame-blocker` | `5c77833ea0b1f9337de2973e54bc3378c0533d34` |
| Local, not on any remote | pass-through build (the known-good firmware) | `9a9ddcc` |
| Local, not on any remote | `ts-capture-fixed` (FIXED timestamped capture build) | `acf675e5693eadddcd28ed0d1cda9be54f53aaa5` |
| Local, not on any remote | `dde-emulator` (this work, v1c) | `60e749b` (firmware commit `a256526`; v1b was `f120e10`, v1 was `954110e`) |

History, oldest first:

```
5c77833 Enhance FlexRay functionality by introducing frame blocking and forwarding features. Added flexray_blo
9a9ddcc Pass-through build: disable injection + blocker by default, target Pico 2 (pico2/rp2350-arm-s)
ef2f378 ts-capture FIXED: hardware frame timestamps on the known-good 9a9ddcc pass-through repeater
37fcfba sim: model of flexray_transparent.pio mid-frame TXEN release on real sync frames
7cc598b fr_schedule_analyze: refine_cycle ignores sparse slots (min_len=max(3,nmax//20)), require >=3 slots in
acf675e ts-capture: crc_fail counter in TS info (was uncounted), PICO_FR_USB_FAST option (1 KB vendor IN trans
854bbe3 dde-emu: table generator and bit-level model from run 5 capture
4530346 dde-emu: host-compilable core (frame build, CRC/E2E, schedule, lock FSM) + host test
5698464 dde-emu: firmware (PICO_FLEXRAY_DDE_EMU=ON): transmit DDE static frames on FR1
954110e dde-emu: logger --emulate, emulator stats, Python validation tests   (= v1)
ab10e9c dde-emu v1b: fix false 'TX SM stuck' fault that dropped 0x3D; first-fault record, stats v2
9a7c938 tests: replay car run 1 (f11_emu1.csv) through dde_emu.c on the host
f120e10 logger: read emu stats v2 (128 bytes), show first fault, print EMU FAULT once   (= v1b)
a256526 dde-emu v1c: send dynamic 0xE6/0xFB/0xE8 and the pre-ign statics; stats v3
cd3f70d tests: v1c dynamic frames - run 5 bytes/minislots, run 1/2 replays, unsure cases
64375c7 logger: read emu stats v3 (192 bytes), show the v1c dynamic frames on the emu line
60e749b tests: cross-check v1c against the DDE flash decode (E2E ID 0x37, base/rep, periods, 0xFF fill, FB L7 0xFFF)
```

`patches/` is the series ts-capture-fixed..dde-emulator. These are 11 patches:
0001–0004 = v1 (`git format-patch ts-capture-fixed..954110e`, unchanged),
0005–0007 = v1b (`git format-patch --start-number 5 954110e..f120e10`, unchanged), and
0008–0011 = v1c (`git format-patch --start-number 8 f120e10..dde-emulator`).
Apply them with `git am patches/*.patch` on a checkout of `ts-capture-fixed`.

`ts-capture-fixed` is not on any remote, so `source/` holds the full source
tree of `dde-emulator` (firmware, tests, tools; no images, web UI or build
output). That tree builds the same code as the shipped v1c UF2: the `.text`
section is byte-identical to the build that made it (checked again for v1c).

With `PICO_FLEXRAY_DDE_EMU=OFF` (the default), the tree builds the FIXED
capture firmware. Every symbol has the same size as in a ts-capture-fixed
build. The UF2 hash differs only because the build path is embedded.
