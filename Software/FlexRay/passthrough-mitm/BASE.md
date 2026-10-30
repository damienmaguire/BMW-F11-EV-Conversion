# Base of the pass-through MITM firmware

| What | Where | Commit |
|---|---|---|
| Upstream | https://github.com/dynm/pico-flexray, branch `feature/frame-blocker` | `5c77833ea0b1f9337de2973e54bc3378c0533d34` |
| Local, not on any remote | pass-through build (KNOWN_GOOD) | `9a9ddcc71fb5c6e410c38a6ef33a7772346323c9` |

History, oldest first:

```
5c77833 Enhance FlexRay functionality by introducing frame blocking and forwarding features. (upstream)
9a9ddcc Pass-through build: disable injection + blocker by default, target Pico 2 (pico2/rp2350-arm-s)
```

## Changes against upstream (`9a9ddcc`)

| File | Change |
|---|---|
| `src/flexray_injector_rules.h` | `INJECT_TRIGGERS[]` is empty: no injection rules |
| `src/flexray_injector.c` | `injector_enabled` defaults to `false`; a `_Static_assert` checks that there are no trigger rules |
| `src/flexray_blocker.c` | `flexray_filter_enabled` defaults to `false` |
| `CMakeLists.txt` | `PICO_BOARD` is `pico2` (was `pico2_w`) |

Nothing else differs from upstream `5c77833`.

`patches/0001-*.patch` is `git format-patch 5c77833..9a9ddcc`. To apply it to
upstream:

```
git clone -b feature/frame-blocker https://github.com/dynm/pico-flexray.git
```

```
cd pico-flexray && git checkout 5c77833
```

```
git am /path/to/patches/0001-Pass-through-build-disable-injection-blocker-by-defa.patch
```

## What `source/` is

`git archive 9a9ddcc` of the tree above, with these left out:

- `imgs/` (4 images used by the upstream README: `pico-flexray.png`,
  `pico-flexray-bmw-g30.webp`, `openpilot-lateral-bmw-g30.webp`, `wiring.png`).
  They are not needed to build. See them upstream.
- No SDK, submodules or build output (the repository has no submodules).

## pico-sdk used

pico-sdk **2.2.0**, tag `2.2.0`, commit `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`.
The only submodule the build needs is `lib/tinyusb` at `86ad6e56c1700e85f1c5678607a762cfe3aa2f47`
(TinyUSB 0.18.0), which is the one pinned by that tag.

```
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git
```

```
git -C pico-sdk submodule update --init lib/tinyusb
```

Other build settings of the KNOWN_GOOD UF2: board `pico2`, platform
`rp2350-arm-s`, `Release`, xPack `arm-none-eabi-gcc` 14.2.1-1.1, `pioasm` and
`picotool` 2.2.0. Build date `Oct 29 2026`.
