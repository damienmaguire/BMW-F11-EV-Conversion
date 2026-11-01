"""Reference model of the DDE emulator (pure Python, no hardware).

Used by gen_dde_tables.py (self-check while generating the C tables) and by
test_dde_emu.py (offline validation against captured run 5 frames).
The C firmware core (src/dde_emu_core.c) implements the same functions; the
host C test (tests/host/test_dde_emu_core.c) checks it against the same
captured vectors.

Wire format of one FlexRay frame as sent by the emulator (channel A, 10 Mbit/s):
  TSS (tss_bits x 0) | FSS (1) | for each byte: BSS (1, 0) + 8 bits MSB first |
  FES (0, 1) | [dynamic only: DTS low (dts_bits x 0) + 1]
Bytes = 5 header bytes + payload + 3 frame-CRC bytes.
"""

BIT_TICKS = 10            # 100 MHz device clock, 10 Mbit/s
TICK_NS = 10

# ---------------------------------------------------------------- CRCs

def header_crc(sync, startup, frame_id, len_words):
    """FlexRay 11-bit header CRC (poly 0x385, init 0x1A) over sync, startup, id, length."""
    data = (sync << 19) | (startup << 18) | (frame_id << 7) | len_words
    crc = 0x1A
    for i in range(19, -1, -1):
        bit = (data >> i) & 1
        fb = ((crc >> 10) & 1) ^ bit
        crc = (crc << 1) & 0x7FF
        if fb:
            crc ^= 0x385
    return crc


def frame_crc(data, init=0xFEDCBA):
    """FlexRay 24-bit frame CRC (poly 0x5D6DCB), channel A init 0xFEDCBA, over header + payload."""
    crc = init
    for b in data:
        for i in range(7, -1, -1):
            fb = ((crc >> 23) & 1) ^ ((b >> i) & 1)
            crc = (crc << 1) & 0xFFFFFF
            if fb:
                crc ^= 0x5D6DCB
    return crc


def crc8_e2e(data):
    """CRC-8 poly 0x1D, init 0x00, no reflection, no final xor (SAE J1850 core)."""
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x1D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


assert crc8_e2e(b"123456789") == 0x37

# ---------------------------------------------------------------- E2E
# (frame id, data cycle mod 4, crc byte index, last byte of the CRC span, K)
# CRC byte = CRC8(payload[crc_byte+1 .. span_end]) XOR K.
# payload[crc_byte+1]: high nibble fixed, low nibble = alive counter 0..14.
E2E_GROUPS = [
    (0x26, 1, 0, 3, 0xA8),
    (0x28, 1, 0, 7, 0x6A),
    (0x28, 1, 8, 15, 0xD7),
    (0x28, 3, 0, 6, 0xE4),
    (0x28, 3, 8, 13, 0x0F),
    (0x29, 3, 0, 7, 0x3C),
    (0x29, 3, 8, 15, 0x69),
    (0x3D, 3, 0, 7, 0x0C),
    (0x3D, 3, 8, 11, 0x5C),
]
COUNTER_MOD = 15          # counter runs 1, 2, ... 14, 0, 1, ...  (starts at 1 after wake)


def e2e_groups_for(frame_id, cycle):
    return [g for g in E2E_GROUPS if g[0] == frame_id and (cycle & 3) == g[1]]


def e2e_apply(payload, frame_id, cycle, counter):
    """Return payload with every E2E group of (frame_id, cycle) given `counter` + fresh CRC."""
    p = bytearray(payload)
    for (_, _, cb, se, k) in e2e_groups_for(frame_id, cycle):
        p[cb + 1] = (p[cb + 1] & 0xF0) | (counter & 0x0F)
        p[cb] = crc8_e2e(p[cb + 1:se + 1]) ^ k
    return bytes(p)


def e2e_check(payload, frame_id, cycle):
    """True when every group's CRC byte matches. Returns (ok, [counters])."""
    ok = True
    ctrs = []
    for (_, _, cb, se, k) in e2e_groups_for(frame_id, cycle):
        if crc8_e2e(payload[cb + 1:se + 1]) ^ k != payload[cb]:
            ok = False
        ctrs.append(payload[cb + 1] & 0x0F)
    return ok, ctrs

# ---------------------------------------------------------------- frame build

def build_header(indicators, frame_id, len_words, cycle):
    """indicators = 5 bits: reserved, payload preamble, null-frame (1 = data), sync, startup."""
    sync = (indicators >> 1) & 1
    startup = indicators & 1
    hc = header_crc(sync, startup, frame_id, len_words)
    return bytes([
        ((indicators & 0x1F) << 3) | ((frame_id >> 8) & 7),
        frame_id & 0xFF,
        ((len_words & 0x7F) << 1) | ((hc >> 10) & 1),
        (hc >> 2) & 0xFF,
        ((hc & 3) << 6) | (cycle & 0x3F),
    ])


def build_frame(indicators, frame_id, cycle, payload):
    assert len(payload) % 2 == 0
    h = build_header(indicators, frame_id, len(payload) // 2, cycle)
    body = h + bytes(payload)
    c = frame_crc(body)
    return body + bytes([(c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF])


def emu_indicators(captured_indicators, data, keep_sync_bits=False):
    """Emulator header indicators: NFI from the table, sync/startup cleared unless EMU_SYNC_BITS."""
    ind = captured_indicators & 0x18          # reserved + payload preamble as captured (both 0 here)
    if data:
        ind |= 0x04
    if keep_sync_bits:
        ind |= captured_indicators & 0x03
    return ind

# ---------------------------------------------------------------- PIO words

def pack_words(frame_bytes, tss_bits, dts_bits=0):
    """TX FIFO words for the dde_emu_tx PIO program (see src/dde_emu_tx.pio).

    w0 = (tss_bits - 2) << 16 | (nbytes - 1); w1 = dts_low_bits - 2 (0 = static frame);
    then the frame bytes MSB first, 4 per word. nbytes must be a multiple of 4.
    """
    n = len(frame_bytes)
    assert n % 4 == 0, "frame length must be a multiple of 4 bytes"
    assert 3 <= tss_bits <= 15
    assert dts_bits == 0 or dts_bits >= 3
    words = [((tss_bits - 2) << 16) | (n - 1), dts_bits - 2 if dts_bits else 0]
    for i in range(0, n, 4):
        b = frame_bytes[i:i + 4]
        words.append((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3])
    return words


def expected_wire_bits(frame_bytes, tss_bits, dts_bits=0):
    """Ideal bit sequence (TXD level per bit while TXEN is low)."""
    bits = [0] * tss_bits + [1]
    for b in frame_bytes:
        bits += [1, 0] + [(b >> (7 - i)) & 1 for i in range(8)]
    bits += [0, 1]
    if dts_bits:
        bits += [0] * dts_bits + [1]
    return bits

# ---------------------------------------------------------------- timing

def wrap16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def target_from_ref(ref_fine, ref_us, delta_q8, loop_ticks, lead_us):
    """Header-ISR schedule maths (same as dde_emu_core.c dde_core_schedule()).

    delta_q8: expected mark(target) - mark(ref) in 1/256 device ticks.
    Returns (bus_fine, txen_fine, alarm_us): echo mark target, TXEN-mark target
    (both mod 2^16) and the TIMER0 alarm time.
    """
    delta_ticks = (delta_q8 + 128) >> 8
    bus = (ref_fine + delta_ticks) & 0xFFFF
    txen = (bus - loop_ticks) & 0xFFFF
    alarm = (ref_us + (delta_ticks - loop_ticks) // 100 - lead_us) & 0xFFFFFFFF
    return bus, txen, alarm


def fire_delay(txen_fine, fine_now, k_ticks):
    """Alarm-ISR maths: PIO delay-loop count so TXEN falls at txen_fine."""
    return wrap16(txen_fine - fine_now) - k_ticks

# ---------------------------------------------------------------- PIO simulator

class PioSim:
    """Tiny RP2040/RP2350 PIO interpreter: just what dde_emu_tx / dde_emu_fire use.

    Instructions supported: JMP (always, !x, x--, !y, y--), WAIT irq, OUT (pins, x, y,
    null, isr), MOV (x <- isr), SET (pins, x, y), IRQ set. Autopull with shift-left OSR.
    Pins: dict gpio -> level. Each SM: set_base/set_count, out_base/out_count.
    """

    def __init__(self, n_pins=32):
        self.pins = {i: 1 for i in range(n_pins)}
        self.irq = 0
        self.sms = []

    def add_sm(self, prog, origin, set_base, set_count, out_base, out_count, pull_thresh=32):
        sm = dict(prog=prog, pc=0, x=0, y=0, isr=0, osr=0, osr_cnt=32, fifo=[], delay=0,
                  set_base=set_base, set_count=set_count, out_base=out_base, out_count=out_count,
                  thresh=pull_thresh, sideset_bits=0, wrap_top=len(prog) - 1, wrap_bottom=0)
        self.sms.append(sm)
        return sm

    def _write_pins(self, base, count, val):
        for i in range(count):
            self.pins[(base + i) % 32] = (val >> i) & 1

    def step(self):
        set_irq = 0
        for sm in self.sms:
            if sm['delay']:
                sm['delay'] -= 1
                continue
            ins = sm['prog'][sm['pc']]
            op = ins >> 13
            delay = (ins >> 8) & 0x1F
            arg1 = (ins >> 5) & 7
            arg2 = ins & 0x1F
            nxt = sm['pc'] + 1 if sm['pc'] != sm['wrap_top'] else sm['wrap_bottom']
            stalled = False
            if op == 0:  # JMP
                cond = arg1
                take = {0: True,
                        1: sm['x'] == 0,
                        2: sm['x'] != 0,
                        3: sm['y'] == 0,
                        4: sm['y'] != 0}.get(cond)
                if take is None:
                    raise NotImplementedError("jmp cond %d" % cond)
                if cond == 2:
                    sm['x'] = (sm['x'] - 1) & 0xFFFFFFFF
                if cond == 4:
                    sm['y'] = (sm['y'] - 1) & 0xFFFFFFFF
                if take:
                    nxt = arg2
            elif op == 1:  # WAIT
                pol = (ins >> 7) & 1
                src = (ins >> 5) & 3
                assert src == 2, "only wait irq"
                idx = arg2 & 7
                if pol and (self.irq >> idx) & 1:
                    self.irq &= ~(1 << idx)
                elif pol:
                    stalled = True
                else:
                    raise NotImplementedError
            elif op == 3:  # OUT
                n = arg2 or 32
                if sm['osr_cnt'] >= sm['thresh']:
                    if not sm['fifo']:
                        stalled = True
                    else:
                        sm['osr'] = sm['fifo'].pop(0)
                        sm['osr_cnt'] = 0
                if not stalled:
                    val = (sm['osr'] >> (32 - n)) & ((1 << n) - 1) if n < 32 else sm['osr']
                    sm['osr'] = (sm['osr'] << n) & 0xFFFFFFFF if n < 32 else 0
                    sm['osr_cnt'] += n
                    dest = arg1
                    if dest == 0:
                        self._write_pins(sm['out_base'], min(n, sm['out_count']), val)
                    elif dest == 1:
                        sm['x'] = val
                    elif dest == 2:
                        sm['y'] = val
                    elif dest == 3:
                        pass
                    elif dest == 6:
                        sm['isr'] = val
                    else:
                        raise NotImplementedError("out dest %d" % dest)
            elif op == 5:  # MOV
                dest, opn, src = arg1, (ins >> 3) & 3, ins & 7
                srcval = {1: sm['x'], 2: sm['y'], 3: 0, 6: sm['isr'], 7: sm['osr']}[src]
                if opn == 1:
                    srcval = (~srcval) & 0xFFFFFFFF
                elif opn != 0:
                    raise NotImplementedError
                if dest == 1:
                    sm['x'] = srcval
                elif dest == 2:
                    sm['y'] = srcval
                else:
                    raise NotImplementedError("mov dest %d" % dest)
            elif op == 6:  # IRQ
                clr = (ins >> 6) & 1
                wait = (ins >> 5) & 1
                assert not clr and not wait
                set_irq |= 1 << (arg2 & 7)
            elif op == 7:  # SET
                dest = arg1
                if dest == 0:
                    self._write_pins(sm['set_base'], sm['set_count'], arg2)
                elif dest == 1:
                    sm['x'] = arg2
                elif dest == 2:
                    sm['y'] = arg2
                else:
                    raise NotImplementedError
            else:
                raise NotImplementedError("op %d" % op)
            if not stalled:
                sm['pc'] = nxt
                sm['delay'] = delay
        self.irq |= set_irq


def decode_wire(samples, txen_pin, txd_pin, bit_ticks=BIT_TICKS):
    """Decode a TXEN/TXD trace (list of (txen, txd) per 10 ns tick) into
    (tss_bits, frame bytes, trailer bits, start_tick). Samples at bit centres."""
    start = next(i for i, s in enumerate(samples) if s[0] == 0)
    end = max(i for i, s in enumerate(samples) if s[0] == 0) + 1
    seg = [s[1] for s in samples[start:end]]
    assert all(s[0] == 0 for s in samples[start:end]), "TXEN glitch inside frame"
    assert len(seg) % bit_ticks == 0, "frame not a whole number of bits: %d ticks" % len(seg)
    bits = []
    for i in range(0, len(seg), bit_ticks):
        chunk = seg[i:i + bit_ticks]
        assert len(set(chunk)) == 1, "level change inside a bit at tick %d" % (start + i)
        bits.append(chunk[0])
    tss = 0
    while bits[tss] == 0:
        tss += 1
    i = tss
    assert bits[i] == 1  # FSS
    i += 1
    out = []
    while i + 10 <= len(bits) and bits[i] == 1 and bits[i + 1] == 0:
        v = 0
        for b in bits[i + 2:i + 10]:
            v = (v << 1) | b
        out.append(v)
        i += 10
    return tss, bytes(out), bits[i:], start
