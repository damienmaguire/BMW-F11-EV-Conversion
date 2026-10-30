#!/usr/bin/env python3
"""Tests for the TS-CAPTURE host tools (no hardware needed).

  python3 test_ts_tools.py            (from tools/ts_capture/)
Runs: record parser (formats 1/2/3, CAN, resync), device-clock reconstruction
(us wrap, 10 ns fine field), the C encoder's sample stream (if
tests/host/run.sh has produced it), and synthetic schedules end to end:
fr_ts_synth -> raw stream -> fr_ts_logger --replay -> CSV -> analysis vs truth.
"""
import csv
import os
import random
import struct
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import frts  # noqa: E402
import fr_ts_synth  # noqa: E402
import fr_ts_logger  # noqa: E402
import fr_schedule_analyze  # noqa: E402

C_STREAM = "/tmp/pico_flexray_ts_sample_stream.bin"


def v3(src, flags, t_us, fine, seq, slot, cyc, payload, ind=0x04):
    return fr_ts_synth.encode_v3(src, flags, t_us, fine, seq, slot, cyc, ind, payload)


class ParserTests(unittest.TestCase):
    def test_formats_and_resync(self):
        pl = bytes(range(16))
        f1_body = bytes([1]) + bytes([0x00, 0x26, 16, 0x5A, 0x45]) + pl + b"\xab\xcd\xef"
        f1 = struct.pack("<H", len(f1_body)) + f1_body
        f2_body = bytes([2, 7, 2]) + struct.pack("<I", 123456) + bytes([0x00, 0x26, 16, 0x5A, 0x45]) + pl + b"\x01\x02\x03"
        f2 = struct.pack("<H", 0x8000 | len(f2_body)) + f2_body
        f3 = v3(2, 7, 0xFFFFFFF0, 0xBEEF, 77, 0x3D, 5, pl)
        can = struct.pack("<HBBBBII", 0x8000 | 14, 0x43, 2, 0, 2, 0x123, 999) + b"\x11\x22"
        junk = b"\x05\x00\xff\xff\x13"
        stream = junk + f1 + f2 + junk + f3 + can + f1
        p = frts.RecordParser()
        recs = []
        for i in range(0, len(stream), 7):  # split across USB transfers
            recs += p.feed(stream[i:i + 7])
        kinds = [(r["kind"], r.get("fmt")) for r in recs]
        self.assertEqual(kinds, [("fr", 1), ("fr", 2), ("fr", 3), ("can", None), ("fr", 1)])
        self.assertEqual(recs[0]["slot_id"], 0x26)
        self.assertEqual(recs[0]["cycle"], 5)
        self.assertEqual(recs[0]["frame_crc"], 0xABCDEF)
        self.assertEqual(recs[1]["t_us_raw"], 123456)
        self.assertEqual((recs[2]["t_us_raw"], recs[2]["t_fine"], recs[2]["seq"], recs[2]["slot_id"]),
                         (0xFFFFFFF0, 0xBEEF, 77, 0x3D))
        self.assertEqual((recs[3]["can_id"], recs[3]["t_us_raw"], recs[3]["data"]), (0x123, 999, b"\x11\x22"))
        self.assertEqual(p.bytes_skipped, 2 * len(junk))

    def test_c_encoder_stream(self):
        if not os.path.exists(C_STREAM):
            self.skipTest("run tests/host/run.sh first (writes %s)" % C_STREAM)
        with open(C_STREAM, "rb") as f:
            recs = frts.RecordParser().feed(f.read())
        self.assertEqual([r["fmt"] for r in recs], [1, 1, 1, 2, 2, 2, 3, 3, 3])
        self.assertEqual([r["slot_id"] for r in recs], [0x26, 0x5C, 0x7FF] * 3)
        self.assertEqual([len(r["payload"]) for r in recs], [16, 0, 254] * 3)
        self.assertEqual([r["cycle"] for r in recs], [0, 1, 63] * 3)
        self.assertEqual([r["seq"] for r in recs[6:]], [30, 31, 32])
        self.assertEqual([r["t_us_raw"] for r in recs[3:]], [20000, 21000, 22000, 30000, 31000, 32000])
        self.assertTrue(all(r["frame_crc"] == 0xABCDEF for r in recs))

    def test_ts_info(self):
        raw = struct.pack("<4B13I8x", 1, 3, 16, 1, 100_000_000, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16)
        i = frts.parse_ts_info(raw)
        self.assertEqual((i["record_format"], i["fine_hz"], i["usb_fifo_dropped"], i["valid_frames"]),
                         (3, 100_000_000, 7, 16))


class ClockTests(unittest.TestCase):
    def test_wrap_and_fine(self):
        rnd = random.Random(3)
        clk = frts.DeviceClock(100_000_000)
        start = 0xFFFFFFFF - 5000 + 0.37
        worst = 0.0
        first = None
        t = 0.0
        for i in range(20000):
            t += rnd.uniform(1, 400)
            dev = start + t
            fine = int(dev * 100) & 0xFFFF
            us = int(dev + rnd.uniform(0, 0.06)) & 0xFFFFFFFF
            got = clk.time(us, fine)
            if first is None:
                first = got - dev
            worst = max(worst, abs((got - dev) - first))
        self.assertLess(worst, 0.011)  # 10 ns quantisation
        self.assertEqual(clk.anomalies, 0)
        self.assertGreater(clk.t64, 0xFFFFFFFF)  # unwrapped past 2^32

    def test_seq_tracker(self):
        s = frts.SeqTracker()
        for q in (5, 6, 9, 10, 0xFFFFFFFF, 0, 3):
            s.add(q)
        self.assertEqual((s.gaps, s.missing), (2, 4))  # 7,8 and 1,2 missing; wrap is not a gap


class EndToEnd(unittest.TestCase):
    CASES = [
        ["--scenario", "a", "--seed", "1"],
        ["--scenario", "b", "--ppm", "-80", "--jitter-ns", "100", "--no-hw-every", "50", "--seed", "7",
         "--cycles", "600"],
        ["--scenario", "b", "--seed", "3", "--jitter-ns", "20", "--ppm", "25"],
        ["--scenario", "a", "--seed", "11", "--jitter-ns", "150", "--ppm", "-45", "--cycles", "300"],
    ]

    def test_cases(self):
        with tempfile.TemporaryDirectory() as d:
            for i, case in enumerate(self.CASES):
                with self.subTest(case=" ".join(case)):
                    b = os.path.join(d, "s%d.bin" % i)
                    c = os.path.join(d, "s%d.csv" % i)
                    fr_ts_synth.main(["--out", b] + case)
                    self.assertEqual(fr_ts_logger.main(["--replay", b, "-o", c]), 0)
                    with open(c) as f:
                        hdr = next(csv.reader(f))
                    self.assertEqual(hdr[:7], ["t_us", "channel", "slot_id", "cycle", "payload_len",
                                               "header_flags", "payload_hex"])
                    rc = fr_schedule_analyze.main([c, "--truth", b + ".truth.json",
                                                   "--json", os.path.join(d, "s%d.json" % i)])
                    self.assertEqual(rc, 0, "analysis does not match the synthetic truth")

    def test_cli_runs(self):
        with tempfile.TemporaryDirectory() as d:
            b = os.path.join(d, "x.bin")
            c = os.path.join(d, "x.csv")
            subprocess.check_call([sys.executable, os.path.join(HERE, "fr_ts_synth.py"), "--out", b, "--cycles", "100"])
            subprocess.check_call([sys.executable, os.path.join(HERE, "fr_ts_logger.py"), "--replay", b, "-o", c])
            subprocess.check_call([sys.executable, os.path.join(HERE, "fr_schedule_analyze.py"), c,
                                   "--slots-csv", os.path.join(d, "slots.csv"), "--only-dde"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
