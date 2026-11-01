#!/usr/bin/env python3
"""v1h: USB throughput model for PICO_FR_USB_FAST (1 KB vendor IN transfers).

What limits the capture rate (TinyUSB 0.18 vendor stream, src/tusb_config.h):
  * a bulk-IN transfer moves at most CFG_TUD_VENDOR_EPSIZE bytes (64 B normally,
    1024 B with PICO_FR_USB_FAST); the DCD splits it into 64 B packets in the ISR;
  * the next transfer is only started from tud_task() (vendord_xfer_cb) or from the
    next tud_vendor_write() - i.e. at most once per "re-arm opportunity";
  * the USB full-speed wire: at most 19 x 64 B bulk packets per 1 ms frame; we assume
    a conservative 13 packets/ms (832 kB/s) for a busy laptop host controller;
  * device FIFO (8 KB normal, 16 KB fast): records that do not fit are dropped and
    counted (usb_fifo_dropped).

Calibration (measured, not assumed): with 64 B transfers the IN rate topped out at
~217 kB/s (run 4) -> ~3,390 re-arms/s. Run 8 (v1g.2, 64 B) needed ~250 kB/s at its peak
and dropped 11 % of FlexRay records (usb_fifo_dropped 90,960 of 829,409); this model
with the same numbers gives a similar loss, so it is used to predict the 1 KB build.

The model is discrete-time (10 us steps), FlexRay frames arrive in bursts (2/3 of a
5 ms cycle's frames in its first half), CAN records spread evenly.
Exit code 0 when the fast build carries >= 300 kB/s with <= 0.1 % loss and >= 2x
headroom in every case, and the 64 B build reproduces a run-8-like loss.

This is a MODEL. The real proof is a bench test on the board (see README-v1h.txt).
"""
import sys

REC_FR = 37.7         # bytes per FlexRay record (run 8 raw dump: 31.17 MB / 825,977 records)
REC_CAN = 18.0
STEP_US = 10


def sim(rate_bps, epsize, fifo, rearm_per_s, every8=True, pkts_per_ms=13, seconds=4.0, can_per_s=470):
    fr_per_s = max(0.0, (rate_bps - can_per_s * REC_CAN) / REC_FR)
    steps = int(seconds * 1e6 / STEP_US)
    q = 0.0                      # bytes in the device FIFO
    xfer_left = 0.0              # bytes of the running transfer still to go on the wire
    armed = False                # a transfer is running
    sent = dropped = offered = 0.0
    wire_per_step = pkts_per_ms * 64 * STEP_US / 1000.0
    rearm_acc = 0.0
    frames_acc = 0.0
    frame_count = 0
    for k in range(steps):
        t_in_cycle = (k * STEP_US) % 5000
        burst = 4.0 / 3.0 if t_in_cycle < 2500 else 2.0 / 3.0
        frames_acc += fr_per_s * burst * STEP_US / 1e6
        new_frames = int(frames_acc)
        frames_acc -= new_frames
        incoming = new_frames * REC_FR + can_per_s * REC_CAN * STEP_US / 1e6
        offered += incoming
        if q + incoming > fifo:
            dropped += q + incoming - fifo
            q = fifo
        else:
            q += incoming
        # wire
        if armed:
            m = min(xfer_left, wire_per_step)
            xfer_left -= m
            sent += m
            if xfer_left <= 0:
                armed = False     # complete: waits for the next re-arm opportunity
        # re-arm opportunities: periodic tud_task + every 8 FlexRay frames (main.c)
        rearm_acc += rearm_per_s * STEP_US / 1e6
        opp = int(rearm_acc)
        rearm_acc -= opp
        if every8:
            before = frame_count
            frame_count += new_frames
            opp += frame_count // 8 - before // 8
        if opp and not armed and q >= 1:
            n = min(q, epsize)
            q -= n
            xfer_left = n
            armed = True
    loss = dropped / offered if offered else 0.0
    return sent / seconds, loss


def main():
    ok = True
    # rearm rate for the 64 B calibration: 217 kB/s / 64 B, the every-8-frames calls are part of it
    cal = 217000 / 64
    print("calibration: 64 B transfers, %.0f re-arms/s -> ceiling %.0f kB/s (run 4 measured ~217)" % (cal, cal * 64 / 1e3))
    print("%-34s %10s %10s %8s" % ("case", "offered", "carried", "loss"))
    _, l8 = sim(250e3, 64, 8192, cal, every8=False)
    print("%-34s %8.0f k %8.0f k %7.2f%%" % ("run 8 peak, 64 B (v1g.2)", 250, sim(250e3, 64, 8192, cal, every8=False)[0] / 1e3, l8 * 100))
    if not (0.05 <= l8 <= 0.20):
        print("  model does not reproduce run 8's ~11 % loss"); ok = False
    cases = [
        ("fast, 300 kB/s, nominal", 300e3, cal, 13),
        ("fast, 600 kB/s (2x), nominal", 600e3, cal, 13),
        ("fast, 300 kB/s, task 4x slower", 300e3, cal / 4, 13),
        ("fast, 300 kB/s, every-8 only", 300e3, 0, 13),
        ("fast, 300 kB/s, slow host 10 pkt/ms", 300e3, cal / 4, 10),
        ("fast, 450 kB/s, slow host 10 pkt/ms", 450e3, cal / 4, 10),
    ]
    for name, rate, rearm, pk in cases:
        carried, loss = sim(rate, 1024, 16384, rearm, pkts_per_ms=pk)
        print("%-34s %8.0f k %8.0f k %7.2f%%" % (name, rate / 1e3, carried / 1e3, loss * 100))
        if loss > 0.001:
            ok = False
    # same 2x case with 64 B transfers for contrast
    c64, l64 = sim(600e3, 64, 8192, cal)
    print("%-34s %8.0f k %8.0f k %7.2f%%" % ("(64 B, 600 kB/s, for contrast)", 600, c64 / 1e3, l64 * 100))
    print("usb fast model: %s" % ("ok" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
