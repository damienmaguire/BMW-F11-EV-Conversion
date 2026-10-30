"""Shared helpers for the pico-flexray TS-CAPTURE tools.

* RecordParser: splits the bulk-IN byte stream (endpoint 0x81) into records.
  Understands FlexRay format 1 (stock), 2 (SavvyCAN device-us), 3 (high
  resolution, this firmware) and CAN records (type 0x43, V2 board).
  Layouts: src/pico_fr_record.h and src/pico_can_record.h in the firmware.
* DeviceClock: rebuilds a 10 ns resolution, never-wrapping device time from
  the 32-bit microsecond field and the 16-bit clk_sys field of format 3.
* CSV helpers shared by the logger, the synthetic generator and the analyser.
"""
import struct

VID, PID = 0x3801, 0xDDCC
EP_IN = 0x81
REQ_SET_RECORD_FORMAT = 0xA0
REQ_SET_CAN_RECORDS = 0xA1
REQ_GET_TS_INFO = 0xA2
REQ_RESET_COMMS = 0xC0

FLAG_FRAME_CRC_OK = 0x01
FLAG_HEADER_CRC_OK = 0x02
FLAG_TS_HW = 0x04

TS_INFO_FIELDS = (
    "info_version", "record_format", "fine_bits", "armed", "fine_hz", "now_us",
    "seq_next", "usb_fifo_dropped", "notify_dropped", "ring_overflow",
    "len_mismatch", "parse_fail", "ts_matched", "ts_missing", "ts_skipped",
    "held_pre_arm", "valid_frames", "crc_fail",
)
TS_INFO_STRUCT = struct.Struct("<4B14I4x")  # 64 bytes
assert TS_INFO_STRUCT.size == 64

CSV_COLUMNS = [
    # requested columns first
    "t_us", "channel", "slot_id", "cycle", "payload_len", "header_flags", "payload_hex",
    # extras
    "null_frame", "sync", "startup", "t_us_raw", "t_fine", "seq", "rec_flags", "ts_hw",
    "rec_format", "host_time_s",
]
CAN_CSV_COLUMNS = ["t_us_raw", "bus", "can_id", "ide", "rtr", "fdf", "brs", "dlc", "data_hex", "host_time_s"]


def parse_ts_info(raw):
    vals = TS_INFO_STRUCT.unpack(bytes(raw[:64]))
    return dict(zip(TS_INFO_FIELDS, vals))


def channel_name(source):
    """Firmware source digits -> name (1 -> FR1, 2 -> FR2, 13 -> FR1>FR3 ...)."""
    s = str(source)
    if len(s) == 1:
        return "FR" + s
    return ">".join("FR" + c for c in s)


class RecordParser:
    """Incremental record splitter. feed(bytes) returns a list of dicts.

    FlexRay dict: kind='fr', fmt, source, flags, t_us_raw, t_fine, seq,
                  indicators, slot_id, cycle, payload(bytes), header_crc, frame_crc
    CAN dict:     kind='can', bus, can_flags, dlc, can_id, t_us_raw, data(bytes)
    Bytes that don't form a plausible record are skipped one at a time and
    counted in .bytes_skipped (resynchronisation, as SavvyCAN does).
    """

    def __init__(self):
        self.buf = bytearray()
        self.bytes_skipped = 0
        self.records = 0

    def feed(self, data):
        self.buf += data
        out = []
        b = self.buf
        pos = 0
        n = len(b)
        while pos + 2 <= n:
            lenf = b[pos] | (b[pos + 1] << 8)
            ext = bool(lenf & 0x8000)
            body_len = lenf & 0x7FFF
            if ext:
                if pos + 3 > n:
                    break
                kind = b[pos + 2]
                if kind == 0x43:
                    if body_len < 12 or body_len > 12 + 64:
                        pos += 1; self.bytes_skipped += 1; continue
                    if pos + 2 + body_len > n:
                        break
                    body = b[pos + 2:pos + 2 + body_len]
                    can_id, t = struct.unpack_from("<II", body, 4)
                    out.append(dict(kind="can", bus=body[1], can_flags=body[2], dlc=body[3],
                                    can_id=can_id, t_us_raw=t, data=bytes(body[12:])))
                    pos += 2 + body_len
                    self.records += 1
                    continue
                if kind == 2:
                    prefix = 7
                elif kind == 3:
                    prefix = 13
                else:
                    pos += 1; self.bytes_skipped += 1; continue
                fmt = kind
            else:
                prefix = 1
                fmt = 1
            if body_len < prefix + 8 or body_len > prefix + 8 + 254:
                pos += 1; self.bytes_skipped += 1; continue
            if pos + 2 + prefix + 5 > n:
                break
            hdr_off = pos + 2 + prefix
            plen = ((b[hdr_off + 2] >> 1) & 0x7F) * 2
            if prefix + 5 + plen + 3 != body_len:
                pos += 1; self.bytes_skipped += 1; continue
            if pos + 2 + body_len > n:
                break
            body = b[pos + 2:pos + 2 + body_len]
            if fmt == 1:
                source, flags, t_us, t_fine, seq = body[0], FLAG_FRAME_CRC_OK | FLAG_HEADER_CRC_OK, None, None, None
            else:
                flags, source = body[1], body[2]
                t_us = struct.unpack_from("<I", body, 3)[0]
                if fmt == 3:
                    t_fine, seq = struct.unpack_from("<HI", body, 7)
                else:
                    t_fine, seq = None, None
            h = body[prefix:prefix + 5]
            payload = bytes(body[prefix + 5:prefix + 5 + plen])
            c = body[prefix + 5 + plen:prefix + 8 + plen]
            out.append(dict(
                kind="fr", fmt=fmt, source=source, flags=flags, t_us_raw=t_us, t_fine=t_fine, seq=seq,
                indicators=h[0] >> 3, slot_id=((h[0] & 7) << 8) | h[1], cycle=h[4] & 0x3F,
                header_crc=((h[2] & 1) << 10) | (h[3] << 2) | (h[4] >> 6),
                frame_crc=(c[0] << 16) | (c[1] << 8) | c[2], payload=payload))
            pos += 2 + body_len
            self.records += 1
        del self.buf[:pos]
        return out


def wrap16(v):
    v &= 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


class DeviceClock:
    """Device time in us (float) from (t_us u32, t_fine u16) pairs.

    t_fine is clk_sys mod 65536 and t_us the 1 MHz timer; both come from the
    same crystal, ratio R = fine_hz / 1e6 (100). The u32 us value only
    resolves which 655 us window the fine counter is in, so a small skew
    between the two DMA reads cancels out:
        d = (t_fine - T64 * R) mod 65536;  t = T64 + wrap16(d - d0) / R
    (d0 from the first record -> t has a constant offset < 1 us vs. time_us,
    which does not matter for intervals). |wrap16(d - d0)| > 2R means the two
    clocks disagree: counted in .anomalies and re-anchored.
    """

    def __init__(self, fine_hz=100_000_000):
        self.R = fine_hz / 1e6
        self.R_int = int(round(self.R))
        self.fine_ok = abs(self.R - self.R_int) < 1e-9 and self.R_int >= 1
        self.last_raw = None
        self.t64 = 0
        self.d0 = None
        self.anomalies = 0

    def unwrap(self, t_us):
        if self.last_raw is None:
            self.t64 = t_us
        else:
            delta = ((t_us - self.last_raw + 0x80000000) & 0xFFFFFFFF) - 0x80000000
            self.t64 += delta
        self.last_raw = t_us
        return self.t64

    def time(self, t_us, t_fine=None, hw=True):
        T = self.unwrap(t_us)
        if t_fine is None or not hw or not self.fine_ok:
            return float(T)
        d = (t_fine - T * self.R_int) & 0xFFFF
        if self.d0 is None:
            self.d0 = d
        e = wrap16(d - self.d0)
        if abs(e) > 2 * self.R_int:
            self.anomalies += 1
            self.d0 = d
            e = 0
        return T + e / self.R_int


def fr_csv_row(rec, t_us, host_time=None):
    ind = rec["indicators"]
    return {
        "t_us": "%.2f" % t_us if t_us is not None else "",
        "channel": channel_name(rec["source"]),
        "slot_id": rec["slot_id"],
        "cycle": rec["cycle"],
        "payload_len": len(rec["payload"]),
        "header_flags": "0x%02X" % ind,
        "payload_hex": rec["payload"].hex(),
        "null_frame": 0 if ind & 0x04 else 1,   # null frame indicator 0 = null frame
        "sync": 1 if ind & 0x02 else 0,
        "startup": 1 if ind & 0x01 else 0,
        "t_us_raw": "" if rec["t_us_raw"] is None else rec["t_us_raw"],
        "t_fine": "" if rec["t_fine"] is None else rec["t_fine"],
        "seq": "" if rec["seq"] is None else rec["seq"],
        "rec_flags": "0x%02X" % rec["flags"],
        "ts_hw": 1 if rec["flags"] & FLAG_TS_HW else 0,
        "rec_format": rec["fmt"],
        "host_time_s": "" if host_time is None else "%.6f" % host_time,
    }


def can_csv_row(rec, host_time=None):
    f = rec["can_flags"]
    return {
        "t_us_raw": rec["t_us_raw"], "bus": rec["bus"], "can_id": "0x%X" % rec["can_id"],
        "ide": f & 1, "rtr": (f >> 1) & 1, "fdf": (f >> 2) & 1, "brs": (f >> 3) & 1,
        "dlc": rec["dlc"], "data_hex": rec["data"].hex(),
        "host_time_s": "" if host_time is None else "%.6f" % host_time,
    }


class SeqTracker:
    """Counts gaps in the format-3 seq field (= frames dropped by the USB FIFO)."""

    def __init__(self):
        self.last = None
        self.gaps = 0
        self.missing = 0

    def add(self, seq):
        if seq is None:
            return 0
        lost = 0
        if self.last is not None:
            d = (seq - self.last - 1) & 0xFFFFFFFF
            if 0 < d < 0x80000000:
                self.gaps += 1
                self.missing += d
                lost = d
        self.last = seq
        return lost
