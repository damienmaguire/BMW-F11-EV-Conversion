# F11 FlexRay findings (DDE branch)

What we have learned so far about the FlexRay bus on the F11 520d, and about the
frames the DDE (engine ECU) sends on it. Everything here comes from captures taken
in the car with a Pico 2 man-in-the-middle board on the DDE branch (Pi_Flex V1,
see `Design/FlexRay`).

| Folder | What is in it |
|---|---|
| [`firmware/`](firmware/) | The two UF2 builds that are known to work in the car |
| [`tools/`](tools/) | Python logger and schedule analyser for timestamped captures |
| [`dbc/`](dbc/) | DBC files for the DDE FlexRay frames (SavvyCAN layout) and the DDE's PT-CAN / PT-CAN2 messages, plus Fred's signal table and E2E notes |

**Confidence key** used below: **high** = reproduced in several captures;
**medium** = consistent but measured once or indirectly; **bound only** = we only
know a limit, not the value.

## Captures behind these numbers

| Capture | Firmware | What it showed |
|---|---|---|
| ts run 1 | transparent repeater (known-bad) | Schedule measured, but the DDE never integrated |
| ts run 4 | `pico_flexray_ts_capture_v1_FIXED` | DDE integrated for the whole session; first full schedule and bus-sleep capture; ~4.5 % USB loss (host side) |
| ts run 5 | `pico_flexray_ts_capture_v1_FIXED` + threaded logger | Lossless (0 seq gaps); wake-up captured from the first frame; repeat of run 4 |
| SavvyCAN MITM logs in `Logs/FlexRay` | pass-through | DDE IDs, payloads, E2E, OBD correlation |
| PT-CAN + PT-CAN2 + FlexRay 4-bus logs | pass-through | Which CAN IDs the DDE sends itself |

The timestamped captures (run 4 / run 5) are 150–170 MB CSVs each and are not
committed here. Ask if you want them.

## 1. Bus parameters

Times are measured with the Pico's crystal, which runs about +85 ppm against the
car's cluster clock. So measured values are a little longer than the nominal
values in macroticks (MT).

| Parameter | Value | Confidence |
|---|---|---|
| Bit rate | 10 Mbit/s | high |
| Macrotick | 1.375 µs | high |
| Cycle | 3636 MT = 4999.5 µs nominal (measured 4999.92–4999.93 µs on the Pico clock) | high |
| Static slots | 91 slots of 24 MT (measured 33.0025 µs each) | high |
| Static payload | 16 bytes in every static frame | high |
| Dynamic segment start | measured ≈ 3003.2 µs after cycle start (91 × 24 MT = 2184 MT) | high |
| Minislot | 5 MT (measured 6.8758 µs) | high |
| Number of minislots | 282–290 | medium (bounds) |
| NIT + symbol window | ≤ 59.9 µs (≤ ~43 MT) | medium (bound) |
| Action point offset | ≤ 3.35 µs | bound only |
| Static AP vs minislot AP | equal within ~0.02 µs | medium |
| Sync / coldstart nodes | 0x40, 0x46, 0x5A (vehicle) and 0x3D (DDE); all four set the sync and startup bits | high |

## 2. DDE static slots

Offsets are from the start of the cycle to the frame start (TXEN edge on the
Pico), measured in run 4. Run 5 matches within ±13 ns after removing a common
fit offset. Static frames are sent every cycle; outside the data cycles they are
null frames.

| Slot | Offset (µs) | Data in cycles | Other cycles | Jitter (std) | Notes |
|---|---|---|---|---|---|
| 0x26 | 1221.04 | every 4th, starting at cycle 1 | null | ~53 ns | turbine-speed-like value, one E2E group |
| 0x28 | 1287.04 | every 2nd, starting at cycle 1 | null | ~54 ns | cycle ≡1: rpm, torque, pedal; cycle ≡3: gear code, speed-like value |
| 0x29 | 1320.06 | every 4th, starting at cycle 3 | null | ~53 ns | content not understood yet |
| 0x3D | 1980.09 | every 4th, starting at cycle 3 | null | ~54 ns | **DDE sync frame** (sync + startup bits) |

The DDE's sync frame sits inside the spread of the vehicle sync nodes: relative to
0x40 in the same cycle, 0x3D is +14 to +16 ns, 0x5A +37 to +54 ns and 0x46
+170 to +180 ns (runs 5 and 4).

## 3. DDE dynamic frames

Dynamic frame positions move with how many minislots lower IDs used in that
cycle, so the offsets are typical values from run 4, not fixed times.

| ID | Payload | When it is sent | Typical offset (µs) | Notes |
|---|---|---|---|---|
| 0x5C | 8 B | every 16th cycle (cycle ≡1 mod 16), minislot 0 | 3003.1 | The DDE's share of a shared NM slot; the vehicle sends 0x5C in other cycles. Last byte 0x12 = DDE node address. Stops ~47 s after engine stop |
| 0x68 | 2 B | ~0.2/s, event-driven | ~3102 | always `c0 ff` so far |
| 0xBE | 4–32 B (varies) | rare | ~3700 | Diagnostic response transport. Always paired with vehicle frame 0x9A (request to ECU 0x12) |
| 0xE6 | 8 B | ~1/s, extra frames on change | ~3958 | coolant, gear / engine state; same PDU as CAN 0x3F9 |
| 0xE8 | 8 B (odd cycles), 4 B (even cycles) | ~1/s; **~15/s with the engine running** | ~3994 | 8 B = same content as CAN 0x1BA; 4 B = `86 78 fe fe` |
| 0xFB | 4 / 6 / 8 B | ~5/s (6 B), the others slower | ~4106 | 6 B frame carries battery voltage |
| 0x10A | 4 B | ~1/s, **only from ignition on to engine stop** | ~4203 | always `00 ff ff ff` so far |
| 0x142 | 30 B | ~49/s (one frame every 4 cycles, 20 ms; its cycle mod 4 is not fixed: 1, 2 and 3 seen in run 5) | ~4626 (±57) | byte counter, several state bytes |

The exact cycle used for some short variants differed between run 4 and run 5
(0xFB 4 B: cycle 5 mod 8 vs 1 mod 8; 0xE8 4 B: different even cycles). The
FlexRay DBC therefore multiplexes 0xFB on cycle mod 4 and 0xE8 on cycle parity,
which fits both runs.

## 4. Startup and sync behaviour

- The DDE is a **coldstart-capable sync node**: its sync frame 0x3D has the startup
  bit set on every frame.
- In run 5 (wake captured from the first frame):

  | Cycle | Time after wake | What joins |
  |---|---|---|
  | 1 | 0 ms | 0x46 alone (leading coldstart node) |
  | 10 | 44.7 ms | DDE 0x3D |
  | 12 | 54.8 ms | 0x40 |
  | 13 | ~60 ms | the rest of the DDE statics and dynamics |
  | 14 | ~65 ms | 0x5A |
  | 17 | ~78 ms | most other vehicle nodes |

- In run 4 the order was slower (0x40 at ~200 ms, DDE at ~450 ms, 0x5A at
  ~500 ms) but 0x46 still led. The join order is not fixed.
- **The cluster forms without the DDE.** In run 1 (and in the "no DDE" log) the
  vehicle nodes start and run normally while the DDE is missing; the car then logs
  DDE-timeout faults.
- The DDE is on air about 60 ms after wake, several seconds before ignition on.
- We cannot see who sends the wake-up pattern (WUP); symbols are not recorded.

## 5. Shutdown

| Item | Run 5 | Run 4 |
|---|---|---|
| Ignition off → bus silent | 60.48 s | 61.49 s |
| DDE behaviour after ign off | stays a full sync node to the last cycle | same |
| Last cycle | DDE, 0x5A and most nodes; last frame is vehicle dynamic 0xDA | same |
| One cycle earlier | the 0x40 node and 0x46 stop | same |

- The ~60 s after-run looks like a fixed network-management timer.
- The DDE has no shutdown-specific frame. After ign off its static data is frozen
  apart from the alive counters; it simply stops with the cluster.
- 0xFB (6 B) byte 2 steps about 27 s after ign off in both runs (meaning unknown).

## 6. Firmware lesson: TXEN must be held for the whole frame

- The earlier "transparent repeater" builds turned the transmitter (TXEN) off after
  about **3.9 recessive bit times**.
- Inside a frame, a byte with 3 or more trailing 1-bits plus the following BSS high
  bit gives a recessive run that long. So these builds **cut frames mid-frame**.
  Modelled on the car's own sync frames: 0x40 cut in 85 % of cycles, 0x46 in 52 %,
  0x5A in 100 %.
- The DDE never saw a clean set of sync frames, so it could not integrate. The
  vehicle side was not affected.
- The working builds (plain pass-through and the FIXED timestamped build) switch
  TXEN on once per frame and **hold it until the frame has ended**. With those the
  DDE integrates every time.
- Rule for any future bridge or emulator firmware: gate TXEN per frame, never on a
  short recessive timeout.

## 7. Open questions

| Question | Why it matters | What would settle it |
|---|---|---|
| Exact number of minislots (282–290) | Needed for a correct emulator config | Real config (FIBEX / ECU dump), or a frame in the last minislots |
| How the ≤ 59.9 µs is split between NIT and symbol window | Same | Same |
| Absolute action point offset (only ≤ 3.35 µs known) | Frame timing inside the slot | Config data; cannot be seen from frame start times |
| Clock-correction parameters (offset / rate limits, microtick) | Emulator must stay in sync | Config data |
| What triggers the event frames (0x68, 0xBE/0x9A, 0x10A, 0xE8 / 0xFB variants) | Emulator must send them at the right time | More targeted captures |
| ~2 % of frames on the wire are never decoded (even-cycle biased, no seq gap) | Capture completeness, possibly signal integrity | `crc_fail` counter (built, not yet run) and a scope look at the vehicle-side RX |

## 8. Byte numbering used in this folder

SavvyCAN shows a FlexRay frame with the **slot ID as the frame ID** and puts the
**cycle count in front of the payload** for decoding. So in these notes and in the
DBC, `bN` means byte N of `[cycle][payload…]`: b0 is the cycle count and b1 is the
first payload byte. A 16-byte static frame therefore shows as LEN 17 in SavvyCAN.
Fred's signal table (`dbc/f11_flexray_dde_signals.csv`) counts **payload** bytes
from 0 instead, so his "payload byte n" is b(n+1) here (= SavvyCAN `data[n+1]`).

## Credits

- **Fred:** FlexRay and PT-CAN payload decoding, the E2E (CRC / counter) solution,
  the signal table, both CAN DBCs and the caveats in [`dbc/README.md`](dbc/README.md).
- **Sam:** timestamp-capture firmware fix, logger / analyser tools, and the
  schedule, startup and shutdown measurements in sections 1 to 6.
- **Damien:** the car, the Pi_Flex hardware and all the captures.
