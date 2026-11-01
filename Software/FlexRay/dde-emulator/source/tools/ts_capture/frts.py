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
REQ_SET_EMU = 0xA3        # DDE emulator build: wValue 1 = enable TX / keepalive, 0 = disable + clear fault
REQ_GET_EMU = 0xA4        # DDE emulator build: stats, 64/128/192/256 bytes by version (src/dde_emu.h dde_emu_stats_t)
REQ_SET_CANTX = 0xA5      # v1e V2 DDE emulator: PT-CAN TX ID mask, wValue = bits 0..15, wIndex = bits 16..31, 0 = off
REQ_GET_CANTX = 0xA6      # v1e: PT-CAN TX stats (src/ptcan_emu.h ptcan_stats_t, 178 bytes; v1f: 214)
REQ_SET_ENGINE = 0xA7     # v1g: fake engine start, wValue = mode | cmd << 8 (repeat <= 3 s)
REQ_GET_ENGINE = 0xA9     # v1g: engine status (src/dde_emu.h dde_engine_status_t, 40 bytes; v1h: 56)
REQ_GET_EMUDIAG = 0xAA    # v1h: echo-miss diagnostics (src/dde_emu.h dde_emu_diag_t, 64 bytes)
REQ_GET_GEAR = 0xAB       # v1h.1: gear status (src/dde_gear.h dde_gear_status_t, 64 bytes)
GEAR_OFF_WINDEX = 0x100   # v1h.1: SET_ENGINE wIndex bit 8 = gear shifting off (fixed P as v1h)
REQ_RESET_COMMS = 0xC0
REQ_GET_FW_VERSION = 0xD6  # panda "git version": v1g.2+ firmware answers e.g. "dde-emu-v1h" (older: "dev")

# v1g.2: version of this logger. The firmware of the same release answers a version that
# ends with the same text (src/fw_version.h); the logger warns at start when it does not.
LOGGER_VERSION = "v1h.2"


def fw_version_matches(fw):
    """True if the firmware version string belongs to this logger's release."""
    return bool(fw) and fw.endswith("-" + LOGGER_VERSION)

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
# v1f: "dir" = Rx (heard on the bus) or Tx (sent by us: the board's own PT-CAN TX, from the
# controller's TX event FIFO, time stamped at the start of frame like the RX frames).
CAN_CSV_COLUMNS = ["t_us", "t_us_raw", "bus", "can_id", "ide", "rtr", "fdf", "brs", "dlc", "data_hex", "host_time_s",
                   "dir"]
CAN_FLAG_TX = 0x20        # CAN record flags bit5 (src/pico_can_record.h PICO_CAN_FLAG_TX)
CAN_RECORDS_RX = 1        # REQ_SET_CAN_RECORDS wValue bit0: CAN records on
CAN_RECORDS_TX = 2        # bit1 (v1f firmware): our own TX frames too (older firmware ignores it)


EMU_STATS_STRUCT = struct.Struct("<4B5I6I3h2H2BI")  # 64 bytes, src/dde_emu.h (v1 layout = first 64 bytes of v2)
assert EMU_STATS_STRUCT.size == 64
EMU_STATS_V2_STRUCT = struct.Struct("<4B6Ii4B5H2BhH3I")  # bytes 64..127 (v2, firmware v1b)
assert EMU_STATS_V2_STRUCT.size == 64
EMU_STATS_V3_STRUCT = struct.Struct("<6I6H7HH3H2BHH")  # bytes 128..191 (v3, firmware v1c: dynamic frames)
assert EMU_STATS_V3_STRUCT.size == 64
EMU_STATS_V4_STRUCT = struct.Struct("<3I3H3H2B3H2I4h2H2BH2I")  # bytes 192..255 (v4, firmware v1d)
assert EMU_STATS_V4_STRUCT.size == 64
EMU_STATS_LEN = 256   # request this many; v1 firmware answers 64, v1b 128, v1c 192
EMU_DYN_PDUS2 = ("10A", "142", "68")
EMU_DYN_PDUS = ("E6", "FB5", "FB7", "FB9", "E8L9", "E8L5")
EMU_DYN_SKIP_REASONS = ("anchor", "mark", "passed", "len", "grid", "limit", "late")
EMU_LATE_REASONS = ("busy", "fire", "short", "win", "missed")
EMU_STATES = {0: "disabled", 1: "no-lock", 2: "arming", 3: "ACTIVE", 4: "FAULT"}
EMU_FAULTS = {0: "none", 1: "TX state machine stuck", 2: "DDE frame seen that we did not send (DDE still connected?)",
              3: "own frames not seen on FR1 RX (wiring / transceiver?)"}
EMU_SLOTS = ("0x26", "0x28", "0x29", "0x3D", "0x5C")


# ---- v1e PT-CAN DDE/EGS emulation TX (V2 board) ---------------------------------
# Index = mask bit; order and IDs must match src/ptcan_emu_core.h (checked by tests/host/run.sh).
CANTX_IDS = (0x3F9, 0x3FD, 0x39A, 0x396, 0x518, 0x0A5, 0x0D9, 0x0DC, 0x0F3, 0x08F,
             0x2C4, 0x281, 0x1B9, 0x381, 0x3B3, 0x397, 0x3FB, 0x512, 0x592)
CANTX_EGS = (0x3FD, 0x39A, 0x396, 0x518)
CANTX_SETS = {
    "step1": (0x3F9,),
    "step2": (0x3F9, 0x3FD),
    "step3": (0x3F9, 0x3FD, 0x39A),
    "step4": (0x3F9, 0x3FD, 0x39A, 0x396, 0x518),
    "step5": CANTX_IDS,
    "all": CANTX_IDS,
    "egs": CANTX_EGS,
    "dde": tuple(i for i in CANTX_IDS if i not in CANTX_EGS),
}
CANTX_STATS_STRUCT = struct.Struct("<BBBBIHBBBBBBIIIiiiiiIIII19H19I")
assert CANTX_STATS_STRUCT.size == 178
CANTX_STATS_V2_STRUCT = struct.Struct("<IIIIIIIIHH")   # bytes 178..213 (v1f firmware)
assert CANTX_STATS_V2_STRUCT.size == 36
CANTX_STATS_LEN = 214     # request this many; v1e firmware answers 178
CANTX_STATES = ("off", "wait-bus", "RUN", "silent", "prep-sleep", "FAULT")
CANTX_FAULTS = ("none", "foreign sender of an emulated ID (real DDE/EGS still on the bus?)", "BUS-OFF",
                "TX error passive", "controller mode change failed", "SPI")


def cantx_mask(spec):
    """'--can-tx-ids' value -> mask. Set names (step1..step5, dde, egs, all) and/or hex IDs,
    comma separated ('3F9', '3F9,3FD,39A', 'step2,396'). Raises ValueError."""
    mask = 0
    for tok in str(spec).replace(" ", "").split(","):
        if not tok:
            continue
        ids = CANTX_SETS.get(tok.lower())
        if ids is None:
            try:
                v = int(tok, 16)
            except ValueError:
                raise ValueError("unknown CAN TX set or ID %r (sets: %s)" % (tok, ", ".join(CANTX_SETS)))
            if v not in CANTX_IDS:
                raise ValueError("0x%X is not an emulated PT-CAN ID (emulated: %s)"
                                 % (v, " ".join("%03X" % i for i in CANTX_IDS)))
            ids = (v,)
        for i in ids:
            mask |= 1 << CANTX_IDS.index(i)
    if not mask:
        raise ValueError("empty CAN TX selection")
    return mask


def cantx_ids_of(mask):
    return [i for b, i in enumerate(CANTX_IDS) if mask >> b & 1]


def parse_cantx_stats(raw):
    raw = bytes(raw)
    if len(raw) < CANTX_STATS_STRUCT.size:
        return None
    v = CANTX_STATS_STRUCT.unpack_from(raw)
    names = ("version", "state", "fault", "flags", "mask", "fault_id", "st_kl", "n_frames", "tec", "rec", "opmod",
             "reserved", "rx_frames", "n12f", "wakes", "ms_wake", "ms_rx", "ms_on", "ms_off", "ms_fnm",
             "tx_total", "fifo_full", "tx_fail", "mode_switches")
    d = dict(zip(names, v[:24]))
    d["ids"] = list(v[24:43])
    d["tx"] = list(v[43:62])
    fl = d["flags"]
    for b, k in enumerate(("enabled", "normal", "kl15", "kl15_ever", "bus_off", "tx_passive", "rx_passive",
                           "keepalive_lost")):
        d[k] = bool(fl >> b & 1)
    d["v2"] = len(raw) >= CANTX_STATS_STRUCT.size + CANTX_STATS_V2_STRUCT.size
    if d["v2"]:
        w = CANTX_STATS_V2_STRUCT.unpack_from(raw, CANTX_STATS_STRUCT.size)
        d.update(zip(("loop_gap_max_us", "loop_gaps_20ms", "late_max_us", "late_sends", "resyncs", "tx_logged",
                      "tx_log_lost", "console_dropped", "burst_max"), w[:9]))
    return d


def _cantx_ago(ms):
    return "-" if ms < 0 else "%.1fs" % (ms / 1000.0)


def fmt_cantx(d, prev_tx=None):
    """One status line. prev_tx: per-ID counts at the previous line (for +deltas)."""
    if d is None:
        return "(short PT-CAN TX stats)"
    st = CANTX_STATES[d["state"]] if d["state"] < len(CANTX_STATES) else str(d["state"])
    flt = CANTX_FAULTS[d["fault"]] if d["fault"] < len(CANTX_FAULTS) else str(d["fault"])
    s = "%s%s ctl=%s mask=%s | KL15=%s(ST_KL %X) wake %s rx %s on %s off %s | tx %d fifo-full %d fail %d TEC %d REC %d%s" % (
        st, (" FAULT=%s%s" % (flt, (" id=0x%03X" % d["fault_id"]) if d["fault"] == 1 else "")) if d["fault"] else "",
        "Normal" if d["normal"] else "ListenOnly", ",".join("%03X" % i for i in cantx_ids_of(d["mask"])) or "-",
        "on" if d["kl15"] else "off", d["st_kl"], _cantx_ago(d["ms_wake"]), _cantx_ago(d["ms_rx"]),
        _cantx_ago(d["ms_on"]), _cantx_ago(d["ms_off"]), d["tx_total"], d["fifo_full"], d["tx_fail"], d["tec"],
        d["rec"], " BUS-OFF" if d["bus_off"] else (" TX-PASSIVE" if d["tx_passive"] else ""))
    if d["keepalive_lost"]:
        s += " KEEPALIVE-LOST"
    if d.get("v2"):
        s += " | loop-gap max %.1fms >20ms:%d late %d (max %.1fms) resync %d burst %d | tx-log %d lost %d" % (
            d["loop_gap_max_us"] / 1000.0, d["loop_gaps_20ms"], d["late_sends"], d["late_max_us"] / 1000.0,
            d["resyncs"], d["burst_max"], d["tx_logged"], d["tx_log_lost"])
        if d["console_dropped"]:
            s += " console-dropped %d" % d["console_dropped"]
    per = []
    for k, (i, n) in enumerate(zip(d["ids"], d["tx"])):
        if n or (d["mask"] >> k & 1):
            per.append("%03X=%d%s" % (i, n, ("(+%d)" % (n - prev_tx[k])) if prev_tx else ""))
    if per:
        s += " | " + " ".join(per)
    return s


def parse_emu_stats(raw):
    v = EMU_STATS_STRUCT.unpack(bytes(raw[:64]))
    d = dict(version=v[0], state=v[1], enabled=v[2], fault=v[3], sent=list(v[4:9]),
             lock_losses=v[9], late=v[10], echo_ok=v[11], echo_miss=v[12], foreign=v[13], cycles_good=v[14],
             echo_err_avg_ns=v[15], echo_err_maxabs_ns=v[16], fire_err_avg_ns=v[17], loop_delay_ns=v[18],
             fire_k_ticks=v[19], tss_bits=v[20], flags=v[21], rearms=v[22])
    d["locked"] = bool(d["flags"] & 4)
    d["v2"] = False
    if d["version"] >= 2 and len(raw) >= 128:
        w = EMU_STATS_V2_STRUCT.unpack(bytes(raw[64:128]))
        d.update(v2=True, ff_code=w[0], ff_slot=w[1], ff_cycle=w[2], ff_flags=w[3], ff_us=w[4],
                 ff_sent_total=w[5], ff_late=w[6], ff_echo_ok=w[7], ff_echo_miss=w[8], ff_foreign=w[9],
                 ff_detail=w[10], fault_count=w[11], auto_clears=w[12], latched=w[13], last_fault=w[14],
                 late_reason=list(w[15:20]), last_late_slot=w[20], last_late_reason=w[21], last_late_r=w[22],
                 echo_dup=w[23], last_fault_us=w[24], now_us=w[25])
        d["ff_age_s"] = ((d["now_us"] - d["ff_us"]) & 0xFFFFFFFF) / 1e6
    d["v3"] = False
    if d["version"] >= 3 and len(raw) >= 192:
        x = EMU_STATS_V3_STRUCT.unpack(bytes(raw[128:192]))
        d.update(v3=True, dyn_sent=list(x[0:6]), dyn_skip=list(x[6:12]), dyn_skip_reason=list(x[12:19]),
                 dyn_defer=x[19], dyn_last_ms=list(x[20:23]), ign=x[23], dyn_flags=x[24], dyn_parks=x[25])
    d["v4"] = False
    if d["version"] >= 4 and len(raw) >= 256:
        y = EMU_STATS_V4_STRUCT.unpack(bytes(raw[192:256]))
        d.update(v4=True, dyn_sent2=list(y[0:3]), dyn_skip2=list(y[3:6]), dyn_last_ms2=list(y[6:9]),
                 kl15=y[9], f1_b3=y[10], kl15_on_edges=y[11], kl15_off_edges=y[12], nm_ceiling_stops=y[13],
                 kl15_off_ago_ms=y[14], ceil_left_ms=y[15], isr_late_min_us=y[16], isr_late_max_us=y[17],
                 r_entry_min=y[18], r_final_min=y[19], topup_timeouts=y[20], hk_deferred=y[21],
                 dyn_flags2=y[22], ph142=y[23], abs_cycle=y[25])
    return d


def emu_slot_name(d, slot):
    """ff_slot / last_late_slot are 8 bits: v1d's 0x10A / 0x142 show up as 0x0A / 0x42."""
    if d.get("v4") and slot in (0x0A, 0x42):
        return "0x1%02X" % slot
    return "0x%02X" % slot


def fmt_emu_kl15(d):
    """v1d KL15 / NM part of the emu line, '' for older firmware."""
    if not d.get("v4"):
        return ""
    k = d["kl15"]
    src = "F1" if k & 4 else ("103" if k & 8 else "none")
    s = "kl15=%s(src=%s f1b3=0x%02X on=%d off=%d" % (
        "ON" if k & 1 else ("off(was on)" if k & 2 else "off"), src, d["f1_b3"],
        d["kl15_on_edges"], d["kl15_off_edges"])
    if d["kl15_off_ago_ms"] != 0xFFFFFFFF:
        s += " off_ago=%.1fs" % (d["kl15_off_ago_ms"] / 1000.0)
    s += ") nm=%s ceil_left=%.1fs" % ("TX" if k & 16 else "quiet", d["ceil_left_ms"] / 1000.0)
    if k & 32 or d["nm_ceiling_stops"]:
        s += " CEILING(stops=%d)" % d["nm_ceiling_stops"]
    s += " dyn2[%s] ms2[%s] ph142=%d abs=%d" % (
        " ".join("%s=%d" % (n, c) for n, c in zip(EMU_DYN_PDUS2, d["dyn_sent2"])),
        " ".join("%s=%d" % (n, c) for n, c in zip(EMU_DYN_PDUS2, d["dyn_last_ms2"])),
        d["ph142"], d["abs_cycle"])
    if any(d["dyn_skip2"]):
        s += " skip2[%s]" % " ".join("%s=%d" % (n, c) for n, c in zip(EMU_DYN_PDUS2, d["dyn_skip2"]) if c)
    s += " isr_late=%d..%dus r_min=%d/%d" % (d["isr_late_min_us"], d["isr_late_max_us"],
                                             d["r_entry_min"], d["r_final_min"])
    if d["topup_timeouts"]:
        s += " TOPUP_TIMEOUTS=%d" % d["topup_timeouts"]
    if d["dyn_flags2"] & 31 != 31:
        s += " build2=0x%X" % d["dyn_flags2"]
    return s


def fmt_emu_dyn(d):
    """v1c dynamic-frame part of the emu line, '' for older firmware."""
    if not d.get("v3"):
        return ""
    s = "dyn[%s] ms[E6=%d E8=%d FB=%d] ign=%s%s%s" % (
        " ".join("%s=%d" % (n, c) for n, c in zip(EMU_DYN_PDUS, d["dyn_sent"])),
        d["dyn_last_ms"][0], d["dyn_last_ms"][1], d["dyn_last_ms"][2],
        "on" if d["ign"] & 2 else ("seen" if d["ign"] & 1 else "off"),
        "" if d["dyn_flags"] & 15 == 15 else " build=0x%X" % d["dyn_flags"],
        " parks=%d" % d["dyn_parks"] if d["dyn_parks"] != d["dyn_sent"][2] else "")
    if any(d["dyn_skip"]) or d["dyn_defer"]:
        s += " skip[%s] why[%s] defer=%d" % (
            " ".join("%s=%d" % (n, c) for n, c in zip(EMU_DYN_PDUS, d["dyn_skip"]) if c),
            " ".join("%s=%d" % (n, c) for n, c in zip(EMU_DYN_SKIP_REASONS, d["dyn_skip_reason"]) if c),
            d["dyn_defer"])
    return s


def fmt_emu_first_fault(d):
    """One-line first-fault record (v2 stats), '' when there is none."""
    if not d.get("v2") or not d["ff_code"]:
        return ""
    return ("FAULT %d \"%s\" slot=%s cycle=%d %.3f s ago detail=%d | at the fault: sent=%d late=%d "
            "echo ok=%d miss=%d foreign=%d%s%s" % (
                d["ff_code"], EMU_FAULTS.get(d["ff_code"], "?"), emu_slot_name(d, d["ff_slot"]), d["ff_cycle"], d["ff_age_s"],
                d["ff_detail"], d["ff_sent_total"], d["ff_late"], d["ff_echo_ok"], d["ff_echo_miss"],
                d["ff_foreign"], " locked" if d["ff_flags"] & 1 else " not-locked",
                " tx-in-flight" if d["ff_flags"] & 2 else ""))


def fmt_emu(d):
    s = "state=%s%s sent[%s] lock_losses=%d rearms=%d late=%d echo ok=%d miss=%d foreign=%d " \
        "echo_err avg=%+dns max=%dns fire_err=%+dns loop=%dns K=%d tss=%d%s%s" % (
            EMU_STATES.get(d["state"], d["state"]), " locked" if d["locked"] else "",
            " ".join("%s=%d" % (n, c) for n, c in zip(EMU_SLOTS, d["sent"])),
            d["lock_losses"], d["rearms"], d["late"], d["echo_ok"], d["echo_miss"], d["foreign"],
            d["echo_err_avg_ns"], d["echo_err_maxabs_ns"], d["fire_err_avg_ns"], d["loop_delay_ns"],
            d["fire_k_ticks"], d["tss_bits"], " SYNC_BITS" if d["flags"] & 1 else "", " DYN_5C" if d["flags"] & 2 else "")
    if d["fault"]:
        s += " | FAULT %d: %s" % (d["fault"], EMU_FAULTS.get(d["fault"], "?"))
        if d.get("v2"):
            s += " (%s)" % ("LATCHED until restart" if d["latched"] else "auto re-arm after 2 s quiet")
    if d.get("v2"):
        if any(d["late_reason"]):
            s += " | late[%s] last=%s/%s r=%d" % (
                " ".join("%s=%d" % (n, c) for n, c in zip(EMU_LATE_REASONS, d["late_reason"])),
                emu_slot_name(d, d["last_late_slot"]), EMU_LATE_REASONS[d["last_late_reason"] - 1] if 1 <= d["last_late_reason"] <= 5
                else d["last_late_reason"], d["last_late_r"])
        if d["fault_count"] or d["auto_clears"]:
            s += " | faults=%d auto_clears=%d" % (d["fault_count"], d["auto_clears"])
        ff = fmt_emu_first_fault(d)
        if ff:
            s += " | first " + ff
    dyn = fmt_emu_dyn(d)
    if dyn:
        s += " | " + dyn
    kl = fmt_emu_kl15(d)
    if kl:
        s += " | " + kl
    return s


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


def can_csv_row(rec, host_time=None, t_us=None):
    f = rec["can_flags"]
    return {
        "t_us": "" if t_us is None else "%.2f" % t_us,
        "t_us_raw": rec["t_us_raw"], "bus": rec["bus"], "can_id": "0x%X" % rec["can_id"],
        "ide": f & 1, "rtr": (f >> 1) & 1, "fdf": (f >> 2) & 1, "brs": (f >> 3) & 1,
        "dlc": rec["dlc"], "data_hex": rec["data"].hex(),
        "host_time_s": "" if host_time is None else "%.6f" % host_time,
        "dir": "Tx" if f & CAN_FLAG_TX else "Rx",
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


# SavvyCAN native CSV (as in the MITM captures): FlexRay rows put the cycle in D1 and the
# payload after it (LEN = payload + 1); FR1 = bus 0, FR2 = bus 1, PT-CAN (V2 board) = bus 2.
SAVVY_HEADER = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8"


def savvy_fr_row(rec, t_us):
    bus = {1: 0, 2: 1}.get(rec["source"], (rec["source"] % 10) - 1 if rec["source"] else 0)
    data = bytes([rec["cycle"]]) + rec["payload"]
    tag = "FR" if rec["indicators"] & 0x04 else "FR+NULL"
    return "%d,%08X,false,Rx,%d,%d,%s,%s," % (int(t_us or 0), rec["slot_id"], bus, len(data),
                                              ",".join("%02X" % x for x in data), tag)


def savvy_can_row(rec, t_us):
    f = rec["can_flags"]
    return "%d,%08X,%s,%s,%d,%d%s" % (int(t_us), rec["can_id"], "true" if f & 1 else "false",
                                      "Tx" if f & CAN_FLAG_TX else "Rx", rec["bus"],
                                      len(rec["data"]), "".join(",%02X" % x for x in rec["data"]))


# --- v1g fake engine start ---------------------------------------------------------
ENGINE_MODES = {"off": 0, "cas": 1, "fallback": 2}
ENGINE_CMD_NONE, ENGINE_CMD_START, ENGINE_CMD_STOP = 0, 1, 2
ENGINE_PHASES = ("off", "crank", "run", "stop", "stopped")
ENGINE_REJECTS = ("-", "no brake", "ST_KL not A/D (ignition off?)", "no start command", "busy", "mode off")
ENGINE_STATUS_FIELDS = (
    "version", "mode", "phase", "stkl", "stkl_src", "brake", "last_reject", "e6_b3", "rpm", "torque_raw",
    "starts", "stops", "rejects", "ms_since_start", "ms_since_stkl", "cnt_hi", "fb9_b2", "e6_b4", "e6_ready", "seq",
)
ENGINE_STATUS_STRUCT = struct.Struct("<8B2H5I4BI")
ENGINE_STATUS_LEN = ENGINE_STATUS_STRUCT.size
assert ENGINE_STATUS_LEN == 40


# v1h status additions (version 2, 16 more bytes)
ENGINE_STATUS_V2_FIELDS = ("profile", "profile_req", "cas_flags", "out_flags", "d_to_82_ms", "d_len_ms",
                           "d_end_after_82_ms", "t_rpm_ms", "aborts")
ENGINE_STATUS_V2_STRUCT = struct.Struct("<4B2Hh H I")
ENGINE_STATUS_V2_LEN = ENGINE_STATUS_LEN + ENGINE_STATUS_V2_STRUCT.size
assert ENGINE_STATUS_V2_LEN == 56
# v1h start profiles (SET_ENGINE wIndex; src/dde_engine.h ENG_PROF_*)
ENGINE_PROFILES = {"default": 0, "short": 1, "realistic": 2, "run8": 3}
ENGINE_PROFILE_NAMES = {v: k for k, v in ENGINE_PROFILES.items()}
ENGINE_PROFILE_TEXT = {
    "short": "running ~0.17 s after START (shorter than any real start; the CAS may refuse it)",
    "realistic": "running ~0.84 s after START (fastest real start: 0.85 s)",
    "run8": "the v1g.2 start that passed run 8 (running ~0.94 s after START)",
    "default": "whatever the firmware build defaults to (v1h: short)",
}
ENG_CAS_D_ENDED_EARLY, ENG_CAS_D_ENDED, ENG_CAS_D_LONG, ENG_CAS_ABANDONED = 1, 2, 4, 8


def engine_cas_news(d, prev):
    """v1h: loud messages for new CAS reactions. prev = (starts, cas_flags) of the last poll."""
    if not d or "cas_flags" not in d:
        return []
    old = prev[1] if prev and prev[0] == d["starts"] else 0
    new = d["cas_flags"] & ~old
    out = []
    if new & ENG_CAS_D_ENDED_EARLY:
        out.append("CAS ended the start request (ST_KL D -> %s) %s ms after START, BEFORE our running report "
                   "- CAS refused / gave up on our start"
                   % ("-" if d["stkl"] == 0xFF else "%X" % d["stkl"],
                      "?" if d["d_len_ms"] == 0xFFFF else d["d_len_ms"]))
    if new & ENG_CAS_ABANDONED:
        out.append("start abandoned: no running report sent (profile %s)"
                   % ENGINE_PROFILE_NAMES.get(d["profile"], d["profile"]))
    if new & ENG_CAS_D_LONG:
        out.append("CAS still requesting start 300 ms after our running report (it does not believe it?)")
    if new & ENG_CAS_D_ENDED:
        out.append("CAS released START %d ms after our running report (real cars: 8-39 ms)" % d["d_end_after_82_ms"])
    return out


def engine_wvalue(mode, cmd=ENGINE_CMD_NONE):
    """SET_ENGINE wValue: low byte mode (0 off, 1 cas, 2 fallback), high byte command."""
    if isinstance(mode, str):
        mode = ENGINE_MODES[mode]
    if mode not in (0, 1, 2) or cmd not in (0, 1, 2):
        raise ValueError("bad engine mode/command")
    return mode | cmd << 8


def parse_engine_status(raw):
    raw = bytes(raw)
    if len(raw) < ENGINE_STATUS_LEN:
        return None
    d = dict(zip(ENGINE_STATUS_FIELDS, ENGINE_STATUS_STRUCT.unpack_from(raw)))
    if d["version"] >= 2 and len(raw) >= ENGINE_STATUS_V2_LEN:
        d.update(zip(ENGINE_STATUS_V2_FIELDS, ENGINE_STATUS_V2_STRUCT.unpack_from(raw, ENGINE_STATUS_LEN)))
    return d


def fmt_engine(d):
    if not d:
        return "(no engine status)"
    mode = {v: k for k, v in ENGINE_MODES.items()}.get(d["mode"], str(d["mode"]))
    phase = ENGINE_PHASES[d["phase"]] if d["phase"] < len(ENGINE_PHASES) else str(d["phase"])
    rej = ENGINE_REJECTS[d["last_reject"]] if d["last_reject"] < len(ENGINE_REJECTS) else str(d["last_reject"])
    stkl = "-" if d["stkl"] == 0xFF else "%X" % d["stkl"]
    e6 = "%02X/%02X" % (d["e6_b3"], d["e6_b4"]) if d["e6_b3"] else "stock"
    ready = "-" if d["e6_ready"] == 0xFF else str(d["e6_ready"])
    return ("mode %s phase %s rpm %d E6 %s ready %s CounterHi %d FB9 %02X | ST_KL %s (%s, %s) brake %d | "
            "starts %d stops %d rejects %d (last: %s)"
            % (mode, phase, d["rpm"], e6, ready, d["cnt_hi"], d["fb9_b2"], stkl,
               "PT-CAN 0x12F" if d["stkl_src"] else "FR 0x74",
               "%d ms ago" % d["ms_since_stkl"] if d["ms_since_stkl"] != 0xFFFFFFFF else "never",
               d["brake"], d["starts"], d["stops"], d["rejects"], rej)
            + fmt_engine_v2(d))


def fmt_engine_v2(d):
    if "cas_flags" not in d:
        return ""
    ms = lambda v: "-" if v == 0xFFFF else "%d ms" % v
    f = d["cas_flags"]
    cas = ",".join(n for b, n in ((1, "ENDED-EARLY"), (2, "released"), (4, "HELD-LONG"), (8, "ABANDONED"))
                   if f & b) or "-"
    e8 = "fast" if d["out_flags"] & 4 else "crank" if d["out_flags"] & 2 else "1/s"
    return (" | profile %s, START->rpm %s, START->82 %s, START held %s, CAS %s, aborts %d, inj %d, E8 %s"
            % (ENGINE_PROFILE_NAMES.get(d["profile"] or d["profile_req"], d["profile"]), ms(d["t_rpm_ms"]),
               ms(d["d_to_82_ms"]), ms(d["d_len_ms"]), cas, d["aborts"], d["out_flags"] & 1, e8))


# --- v1h echo-miss diagnostics (USB 0xaa) ---------------------------------------------
EMU_DIAG_STRUCT = struct.Struct("<4B4H4H4hHHHHHHHHIIIHHHH")   # v1h.2: + snap_fail, bg_max_us
EMU_DIAG_LEN = EMU_DIAG_STRUCT.size
assert EMU_DIAG_LEN == 64
EMU_DIAG_FIELDS = ("version", "lm_slot", "lm_cycle", "lm_flags", "miss26", "miss28", "miss29", "miss3d",
                   "late26", "late28", "late29", "late3d", "lm_r_entry", "lm_r_final", "lm_isr_late_us",
                   "lm_sched_lead_us", "lm_sched_ref", "lm_hdr_us", "hdr_isr_max_us", "alarm_isr_max_us",
                   "miss_resched_close", "miss_crc_near", "miss_preempt", "resched_guard_skips", "crc_fail",
                   "no_cand", "static_fired", "hdr_over_10us", "resched_min_us", "snap_fail", "bg_max_us")


def parse_emu_diag(raw):
    if raw is None:
        return None
    raw = bytes(raw)
    if len(raw) < EMU_DIAG_LEN:
        return None
    return dict(zip(EMU_DIAG_FIELDS, EMU_DIAG_STRUCT.unpack_from(raw)))


def fmt_emu_diag(d):
    if not d:
        return "(no diag)"
    fl = d["lm_flags"]
    why = ",".join(n for b, n in ((1, "alarm-in-header-ISR"), (2, "re-armed<40us"), (4, "hdr-CRC-fail"),
                                  (8, "no-candidate-IRQ"), (16, "prev-frame-on-wire")) if fl & b) or "-"
    return ("miss 26/28/29/3D %d/%d/%d/%d late %d/%d/%d/%d | last miss 0x%02X c%d [%s] r %d->%d isr_late %dus "
            "armed by 0x%02X %dus before | hdr ISR max %dus (>10us %d) alarm ISR max %dus | misses: re-armed<40us %d, "
            "CRC-near %d, in-header-ISR %d | hdr CRC fails %d no-cand %d | guard %d (%dus)%s"
            % (d["miss26"], d["miss28"], d["miss29"], d["miss3d"], d["late26"], d["late28"], d["late29"], d["late3d"],
               d["lm_slot"], d["lm_cycle"], why, d["lm_r_entry"], d["lm_r_final"], d["lm_isr_late_us"],
               d["lm_sched_ref"], d["lm_sched_lead_us"], d["hdr_isr_max_us"], d["hdr_over_10us"],
               d["alarm_isr_max_us"], d["miss_resched_close"], d["miss_crc_near"], d["miss_preempt"],
               d["crc_fail"], d["no_cand"], d["resched_guard_skips"], d["resched_min_us"],
               # v1h.2 (diag version 2): PT-CAN snapshot read failures (last good one reused), core 1 background step
               (" | snapshot read fails %d, background step max %dus" % (d["snap_fail"], d["bg_max_us"]))
               if d["version"] >= 2 else ""))


# --- v1h.1 gear status (USB 0xab, src/dde_gear.h) --------------------------------------
GEAR_STATUS_STRUCT = struct.Struct("<8B4H2I" + "HBBBBH" * 5)
GEAR_STATUS_LEN = GEAR_STATUS_STRUCT.size
assert GEAR_STATUS_LEN == 64
GEAR_NAMES = "PRND"
GEAR_STAGES = ("P (pawl in)", "leaving P", "in gear", "P coming", "P (pawl not yet confirmed)",
               "P released (lever pushed)")   # v1h.2: GST_P_RELEASED
GEAR_WHY = {1: "lever forward 1", 2: "lever forward 2", 3: "lever back 1", 4: "lever back 2", 5: "P button",
            6: "engine not running: auto P", 7: "off", 8: "on"}


def parse_gear_status(raw):
    if raw is None:
        return None
    raw = bytes(raw)
    if len(raw) < GEAR_STATUS_LEN:
        return None
    v = GEAR_STATUS_STRUCT.unpack_from(raw)
    d = dict(zip(("version", "enabled", "gear", "stage", "lever", "pbtn", "brake", "running", "ev_seq",
                  "changes", "refusals", "reserved", "lever_frames", "lever_bad"), v[:14]))
    d["ev"] = [dict(zip(("seq", "kind", "from_to", "why", "flags", "t_ms"), v[14 + 6 * i:20 + 6 * i]))
               for i in range(5)]
    return d


def gear_name(g):
    return GEAR_NAMES[g] if 0 <= g < 4 else "?"


def gear_ev_text(e):
    """Same text as gear_ev_text() in src/dde_gear.c."""
    f, t = gear_name(e["from_to"] & 15), gear_name(e["from_to"] >> 4)
    br = "brake on" if e["flags"] & 1 else "brake off"
    k = e["kind"]
    if k == 1:
        return "GEAR: %s -> %s (%s) %s" % (f, t, br, GEAR_WHY.get(e["why"], ""))
    if k == 2:
        return "GEAR: refused, %s (%s -> %s, %s)" % ("brake not pressed" if e["why"] == 16 else "engine not running",
                                                     f, t, GEAR_WHY.get(e["flags"] >> 4, ""))
    if k == 3:
        return "GEAR: P pawl confirmed (0x39A E2, 0x3F9 bits 28-29 = 2)"
    if k == 4:
        return "GEAR: shifting %s" % ("on" if e["why"] == 8 else "off (fixed P as v1h)")
    if k == 5:
        return "GEAR: lever 0x197 lost (no valid frame for 90 ms), holding %s" % t
    if k == 6:
        return "GEAR: lever 0x197 back"
    return "GEAR: event %d" % k


def gear_news(d, seen):
    """New gear event texts since event number 'seen' (oldest first) and the new 'seen'."""
    if not d:
        return [], seen
    out = []
    for e in d["ev"]:
        if e["seq"] and ((e["seq"] - seen) & 0xFFFF) and ((e["seq"] - seen) & 0xFFFF) < 0x8000:
            out.append(gear_ev_text(e))
    lost = ((d["ev_seq"] - seen) & 0xFFFF) - 5
    if seen and lost > 0:
        out.insert(0, "GEAR: (%d older gear events not shown)" % lost)
    return out, d["ev_seq"]


def fmt_gear(d):
    if not d:
        return "(no gear status)"
    if not d["enabled"] & 2:
        return "gear: not in this build (fixed P)"
    lv = {0: "centre", 1: "fwd1", 2: "fwd2", 3: "back1", 4: "back2", 0xFF: "none"}.get(d["lever"], "0x%X" % d["lever"])
    return ("gear %s (%s)%s | lever %s%s, brake %d, running %d | changes %d refusals %d | 0x197 ok %d bad %d"
            % (gear_name(d["gear"]), GEAR_STAGES[d["stage"]] if d["stage"] < len(GEAR_STAGES) else d["stage"],
               "" if d["enabled"] & 1 else " SHIFTING OFF", lv, " P-button" if d["pbtn"] else "", d["brake"],
               d["running"], d["changes"], d["refusals"], d["lever_frames"], d["lever_bad"]))
