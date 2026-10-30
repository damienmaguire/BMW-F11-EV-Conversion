# Capture and analysis tools

Python tools for timestamped FlexRay captures with the
`pico_flexray_ts_capture_v1_FIXED.uf2` firmware (see [`../firmware`](../firmware/)).

| File | What it does |
|---|---|
| `fr_ts_logger.py` | Reads the board over USB and writes one CSV row per frame (plus a raw `.bin` dump and a `.console.log`) |
| `fr_schedule_analyze.py` | Works out the bus schedule from a capture CSV: cycle, macrotick, slot sizes, per-slot offsets and cycle multiplexing |
| `frts.py` | Shared parser used by the two scripts above. Keep it in the same folder |
| `reset_to_bootloader.py` | Puts the Pico into BOOTSEL mode over USB, so no button press is needed |
| `fr_ts_synth.py`, `test_ts_tools.py` | Synthetic capture generator and self-test; no hardware needed |

## Install

Ubuntu's system Python is "externally managed", so install PyUSB from apt rather
than with pip:

```
sudo apt install python3-usb
```

Let your user open the board without sudo (VID 0x3801, PID 0xDDCC):

```
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="3801", ATTR{idProduct}=="ddcc", MODE="0666"' | sudo tee /etc/udev/rules.d/99-pico-flexray.rules
```

```
sudo udevadm control --reload-rules
```

```
sudo udevadm trigger
```

## Self-test (no hardware)

```
python3 test_ts_tools.py
```

Expected: `Ran 7 tests ... OK`. One test may be skipped; it needs a file that only
exists on the firmware build machine.

## Capture in the car

Close SavvyCAN first; only one program can read the board at a time.

Quick bench check that the right firmware is on the board (car can be off):

```
python3 fr_ts_logger.py -o bench_check.csv --duration 5
```

The first line should say `TS-CAPTURE firmware: record format 3`.

Full run, 10 minutes. Start it, then wake the car. This covers about 1 min ign on,
1 min engine running, 1 min ign on, ign off and the ~60 s until the bus sleeps:

```
python3 fr_ts_logger.py -o f11_run.csv --duration 600
```

Every 5 s it prints a status line. Healthy values: `hw_ts` 100 %, `seq_gaps=0`,
`usb_fifo_dropped=0`, `notify_dropped=0`, `ring_overflow=0`, `ts_missing=0`.
After the bus sleeps it shows 0 frames/s; that is expected.

Output files:

| File | Contents |
|---|---|
| `f11_run.csv` | One row per frame (columns below) |
| `f11_run.csv.bin` | Raw USB bytes, so the CSV can be rebuilt |
| `f11_run.csv.console.log` | Everything the logger printed, including device counters. Keep it with the CSV |

On exit the logger puts the board back into its normal record format, so SavvyCAN
works as before.

Rebuild the CSV from the raw dump if needed:

```
python3 fr_ts_logger.py --replay f11_run.csv.bin -o f11_run_replay.csv
```

## Analyse

Full report, with JSON and a per-slot table:

```
python3 fr_schedule_analyze.py f11_run.csv --json f11_run_schedule.json --slots-csv f11_run_slots.csv
```

Only the DDE slots in the per-slot table:

```
python3 fr_schedule_analyze.py f11_run.csv --only-dde
```

It takes about 10 s per 700k frames.

## CSV columns

| Column | Meaning |
|---|---|
| `t_us` | Device time of the frame start in µs (10 ns steps, never wraps) |
| `channel` | `FR1` = car side, `FR2` = DDE side (the side the frame came from) |
| `slot_id` | Frame ID (decimal) |
| `cycle` | Cycle counter 0–63 |
| `payload_len` | Payload bytes |
| `header_flags` | The 5 header indicator bits (hex) |
| `payload_hex` | Payload bytes, **without** a cycle byte in front |
| `null_frame`, `sync`, `startup` | Header bits as 0/1 |
| `seq` | Device sequence number; a gap means USB loss |
| `ts_hw` | 1 = hardware timestamp |
| others | `t_us_raw`, `t_fine`, `rec_flags`, `rec_format`, `host_time_s` (raw fields) |

Note: SavvyCAN exports put the cycle in front of the payload (LEN = payload + 1);
this CSV does not.
