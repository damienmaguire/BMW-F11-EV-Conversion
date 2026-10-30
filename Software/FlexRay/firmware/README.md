# Pico FlexRay firmware for the Pi_Flex V1 board

Two builds, both known to work in the car with the DDE connected through the board.

| File | Use it for | SHA-256 |
|---|---|---|
| `pico_flexray_ts_capture_v1_FIXED.uf2` | MITM bridge **plus** timestamped capture (10 ns resolution) with `tools/fr_ts_logger.py`. This is the normal choice. | `3c6f9933…065d` |
| `pico_flexray_passthrough_KNOWN_GOOD.uf2` | Plain pass-through bridge, no timestamps. Fallback if anything looks wrong with the FIXED build. Works with SavvyCAN as before. | `c5fa46a8…f0e0` |

Full hashes are in [`SHA256SUMS`](SHA256SUMS). Check them with:

```
sha256sum -c SHA256SUMS
```

Both builds are for the **V1 board** and a Pico 2 (RP2350). There is no tested V2
board build yet.

Older "transparent repeater" builds are deliberately **not** included: they cut
frames mid-frame and stop the DDE from joining the bus (see section 6 of the
[findings README](../README.md)).

## Flashing

Hold BOOTSEL while plugging the Pico in (or run `tools/reset_to_bootloader.py`).
An `RP2350` drive appears. Copy the UF2 onto it:

```
cp pico_flexray_ts_capture_v1_FIXED.uf2 /media/$USER/RP2350/
```

The Pico reboots by itself.

## V1 board pinout

| Signal | GPIO |
|---|---|
| FR1 (car side) TXD / TXEN / RXD | GP28 / GP27 / GP26 |
| FR2 (DDE side) TXD / TXEN / RXD | GP4 / GP5 / GP6 |
| BGE (transceiver bus guardian enable) | GP2 |
| STBN (transceiver standby, active low) | GP3 |
| Bypass relays FR1/FR2, FR3/FR4 | GP17, GP18 |
| LED | GP20 |

TXEN is active low (low = transmitting).

## Source

Both builds are based on [dynm/pico-flexray](https://github.com/dynm/pico-flexray)
(branch `feature/frame-blocker`, commit `5c77833`). The changes are on a local
branch `ts-capture-fixed` and are not published yet:

| Commit | Change |
|---|---|
| `9a9ddcc` | Pass-through build: injection and frame blocker disabled, target Pico 2. **KNOWN_GOOD is built from this.** |
| `ef2f378` | Hardware frame timestamps added on top of `9a9ddcc` (read-only PIO state machines watching TXEN, DMA latch of a 100 MHz counter). **FIXED is built from this.** The bridge PIO programs, pins and clocks are byte-identical to `9a9ddcc`. |
| `37fcfba` | Simulation model showing why the old transparent repeater cut frames |
| `7cc598b` | Schedule analyser fix for long captures |
| `acf675e` | `crc_fail` counter and an optional faster USB mode (default off). Not in either UF2; untested on hardware |

## How the FIXED build timestamps frames

- The timestamp is the moment the bridge switches TXEN on for a frame, about
  60–80 ns after the start of the frame at the board's RX pin. That fixed offset
  cancels in all time differences.
- Resolution is 10 ns. Static slots show 10–55 ns spread, depending on the sender.
- Timing uses the Pico's crystal (≈ +85 ppm against the car), so absolute durations
  read slightly long.
