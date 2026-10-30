# Base of the DDE emulator

| What | Where | Commit |
|---|---|---|
| Upstream | https://github.com/dynm/pico-flexray, branch `feature/frame-blocker` | `5c77833ea0b1f9337de2973e54bc3378c0533d34` |
| Local, not on any remote | pass-through build (the known-good firmware) | `9a9ddcc` |
| Local, not on any remote | `ts-capture-fixed` (FIXED timestamped capture build) | `acf675e5693eadddcd28ed0d1cda9be54f53aaa5` |
| Local, not on any remote | `dde-emulator` (this work) | `954110e28797604ba3129f63c42597ce1e855b84` |

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
954110e dde-emu: logger --emulate, emulator stats, Python validation tests
```

`patches/` is `git format-patch ts-capture-fixed..dde-emulator`. These are 4 patches.
Apply them with `git am patches/*.patch` on a checkout of `ts-capture-fixed`.

`ts-capture-fixed` is not on any remote, so `source/` holds the full source
tree of `dde-emulator` (firmware, tests, tools; no images, web UI or build
output). That tree builds the same code as the shipped UF2: the `.text`
section is byte-identical to the build that made it.

With `PICO_FLEXRAY_DDE_EMU=OFF` (the default), the tree builds the FIXED
capture firmware. Every symbol has the same size as in a ts-capture-fixed
build. The UF2 hash differs only because the build path is embedded.
