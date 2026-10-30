# Pass-through MITM firmware (source of the KNOWN_GOOD build)

This is the source of
[`../firmware/pico_flexray_passthrough_KNOWN_GOOD.uf2`](../firmware/pico_flexray_passthrough_KNOWN_GOOD.uf2).

It is a plain FlexRay man-in-the-middle bridge. The board sits between the car
(FR1) and the DDE (FR2) and copies every frame from one side to the other.
Frame injection and the frame blocker are both **off**. Nothing is added,
changed or dropped. Frames are also streamed over USB (Panda-compatible
interface), so SavvyCAN can show them.

It is upstream [dynm/pico-flexray](https://github.com/dynm/pico-flexray)
with one small commit on top (see [`BASE.md`](BASE.md)).

| Folder | What is in it |
|---|---|
| [`source/`](source/) | Full source tree at that commit (no images, no SDK, no build output) |
| [`patches/`](patches/) | The change against upstream, as a `git format-patch` file |
| [`BASE.md`](BASE.md) | Upstream repo, branch and commit, and the list of changes |

## Hardware

- Pi_Flex **V1** board (see `Design/FlexRay`).
- Raspberry Pi **Pico 2** (RP2350A). It will not run on an original Pico (RP2040).

| Signal | GPIO |
|---|---|
| FR1 (car side) TXD / TXEN / RXD | GP28 / GP27 / GP26 |
| FR2 (DDE side) TXD / TXEN / RXD | GP4 / GP5 / GP6 |
| BGE (transceiver bus guardian enable) | GP2 |
| STBN (transceiver standby, active low) | GP3 |
| Bypass relays FR1/FR2, FR3/FR4 | GP17, GP18 |
| LED | GP20 |

TXEN is active low (low = transmitting). The pins are set in
`source/src/flexray_pins.h`. FR3/FR4 are not used on the V1 board.

## Building

You need:

- pico-sdk **2.2.0** with the `lib/tinyusb` submodule (see `BASE.md`);
- Arm GNU toolchain **14.2.1** (`arm-none-eabi-gcc`). The KNOWN_GOOD build used
  xPack `arm-none-eabi-gcc` 14.2.1-1.1;
- CMake 3.13 or newer, Ninja, and a host C/C++ compiler (the SDK uses it to
  build `pioasm` and `picotool`).

Get the SDK:

```
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk
```

```
git -C $HOME/pico-sdk submodule update --init lib/tinyusb
```

Build:

```
cd source
```

```
mkdir build && cd build
```

```
export SOURCE_DATE_EPOCH=1793275200
```

```
cmake -G Ninja -DPICO_SDK_PATH=$HOME/pico-sdk -DPICO_BOARD=pico2 -DCMAKE_BUILD_TYPE=Release ..
```

```
ninja
```

The output is `pico_flexray.uf2` in `source/build`.

`SOURCE_DATE_EPOCH` is only there to get the same file, byte for byte. The SDK
puts the build date into the firmware, and the KNOWN_GOOD file says
`Oct 29 2026`. Without it, the only difference is that date (2 bytes).

If you have the Raspberry Pi Pico VS Code extension installed
(`~/.pico-sdk/cmake/pico-vscode.cmake` exists), `CMakeLists.txt` loads it, and it can
switch the SDK or toolchain. Pass the paths explicitly as above.

## Checking the build

Compare your build with the shipped file (from `source/build`):

```
sha256sum pico_flexray.uf2 ../../../firmware/pico_flexray_passthrough_KNOWN_GOOD.uf2
```

Both lines should show:

```
c5fa46a859c5404021d5a56075e7c5f16bac880d69146db77a74f2f411d7f0e0
```

Or check the shipped files against their list:

```
cd ../../../firmware && sha256sum -c SHA256SUMS
```

This was checked on 30 Oct 2026: a build from `source/`, with a fresh clone of
pico-sdk 2.2.0, matched the KNOWN_GOOD UF2 byte for byte.

Flashing is described in [`../firmware/README.md`](../firmware/README.md).

## Credit and licence

All of the firmware is the work of **dynm** and the contributors to
[dynm/pico-flexray](https://github.com/dynm/pico-flexray). The frame blocker
there was inspired by [ergoadams/pico-flexray](https://github.com/ergoadams/pico-flexray).
The only change here is the small pass-through commit in `patches/`.

**Upstream has no licence file.** There is no `LICENSE` in the repository
(checked 30 Oct 2026, all branches), so none is included here. Copyright
stays with the upstream authors. This copy is kept so the KNOWN_GOOD build
can be checked and rebuilt. For any other use, follow the upstream repository.

`source/pico_sdk_import.cmake` comes from the Raspberry Pi pico-sdk and keeps
its own BSD-3-Clause notice.
