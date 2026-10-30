#include "dde_emu_core.h"
#include <string.h>
#include "dde_emu_tables.h"

_Static_assert(5u + DDE_EMU_STATIC_PAYLOAD_BYTES + 3u == DDE_STATIC_FRAME_BYTES, "static frame size");
_Static_assert(DDE_STATIC_FRAME_BYTES % 4u == 0, "PIO packs whole words");
_Static_assert((5u + DDE_EMU_DYN_PAYLOAD_BYTES + 3u) % 4u == 0, "PIO packs whole words");
_Static_assert(EMU_TSS_BITS >= 3 && EMU_TSS_BITS <= 15, "TSS 3..15 bits");

static uint32_t crc24_table[256];
static uint8_t crc8_table[256];
static bool tables_ready;

void dde_core_init(void)
{
    for (unsigned i = 0; i < 256; i++) {
        uint32_t c = (uint32_t)i << 16;
        for (int b = 0; b < 8; b++) c = (c & 0x800000u) ? ((c << 1) ^ 0x5D6DCBu) : (c << 1);
        crc24_table[i] = c & 0xFFFFFFu;
        uint8_t d = (uint8_t)i;
        for (int b = 0; b < 8; b++) d = (d & 0x80u) ? (uint8_t)((d << 1) ^ 0x1Du) : (uint8_t)(d << 1);
        crc8_table[i] = d;
    }
    tables_ready = true;
}

uint16_t dde_core_header_crc(unsigned sync, unsigned startup, unsigned frame_id, unsigned len_words)
{
    uint32_t data = ((uint32_t)(sync & 1u) << 19) | ((uint32_t)(startup & 1u) << 18) |
                    ((uint32_t)(frame_id & 0x7FFu) << 7) | (len_words & 0x7Fu);
    uint32_t crc = 0x1A;
    for (int i = 19; i >= 0; i--) {
        uint32_t fb = ((crc >> 10) ^ (data >> i)) & 1u;
        crc = (crc << 1) & 0x7FFu;
        if (fb) crc ^= 0x385u;
    }
    return (uint16_t)crc;
}

uint32_t dde_core_frame_crc(const uint8_t *p, unsigned n)
{
    if (!tables_ready) dde_core_init();
    uint32_t crc = 0xFEDCBAu;
    while (n--) crc = ((crc << 8) ^ crc24_table[((crc >> 16) ^ *p++) & 0xFFu]) & 0xFFFFFFu;
    return crc;
}

uint8_t dde_core_crc8(const uint8_t *p, unsigned n)
{
    if (!tables_ready) dde_core_init();
    uint8_t c = 0;
    while (n--) c = crc8_table[c ^ *p++];
    return c;
}

void dde_core_header(uint8_t ind, unsigned id, unsigned len_words, unsigned cycle, uint8_t h[5])
{
    uint16_t hc = dde_core_header_crc((ind >> 1) & 1u, ind & 1u, id, len_words);
    h[0] = (uint8_t)(((ind & 0x1Fu) << 3) | ((id >> 8) & 7u));
    h[1] = (uint8_t)(id & 0xFFu);
    h[2] = (uint8_t)(((len_words & 0x7Fu) << 1) | ((hc >> 10) & 1u));
    h[3] = (uint8_t)((hc >> 2) & 0xFFu);
    h[4] = (uint8_t)(((hc & 3u) << 6) | (cycle & 0x3Fu));
}

unsigned dde_core_e2e_apply(uint8_t *payload, unsigned target, unsigned cycle, unsigned counter)
{
    unsigned n = 0;
    for (unsigned g = 0; g < DDE_EMU_N_E2E; g++) {
        const uint8_t *e = dde_emu_e2e[g];
        if (e[0] != target || e[1] != (cycle & 3u)) continue;
        unsigned cb = e[2], se = e[3];
        payload[cb + 1] = (uint8_t)((payload[cb + 1] & 0xF0u) | (counter & 0x0Fu));
        payload[cb] = (uint8_t)(dde_core_crc8(&payload[cb + 1], se - cb) ^ e[4]);
        n++;
    }
    return n;
}

uint8_t dde_core_emu_indicators(unsigned target, unsigned cycle, bool keep_sync)
{
    uint8_t cap = dde_emu_static_ind[target][cycle & 63u];
    uint8_t ind = (uint8_t)(cap & 0x1Cu);       // reserved, preamble, null-frame indicator
    if (keep_sync) ind |= (uint8_t)(cap & 0x03u);
    return ind;
}

unsigned dde_core_build_static(unsigned target, unsigned cycle, unsigned counter, bool keep_sync, uint8_t *out)
{
    const unsigned plen = DDE_EMU_STATIC_PAYLOAD_BYTES;
    uint8_t ind = dde_core_emu_indicators(target, cycle, keep_sync);
    dde_core_header(ind, dde_emu_static_id[target], plen / 2u, cycle, out);
    memcpy(out + 5, dde_emu_static_payload[target][cycle & 63u], plen);
    if (ind & 0x04u) dde_core_e2e_apply(out + 5, target, cycle, counter);
    uint32_t c = dde_core_frame_crc(out, 5u + plen);
    out[5 + plen] = (uint8_t)(c >> 16);
    out[6 + plen] = (uint8_t)(c >> 8);
    out[7 + plen] = (uint8_t)c;
    return 8u + plen;
}

bool dde_core_dyn_cycle(unsigned cycle)
{
    return (cycle & DDE_EMU_DYN_CYCLE_MASK) == DDE_EMU_DYN_CYCLE_BASE;
}

unsigned dde_core_build_dyn(unsigned cycle, uint8_t *out)
{
    const unsigned plen = DDE_EMU_DYN_PAYLOAD_BYTES;
    dde_core_header((uint8_t)(DDE_EMU_DYN_IND & 0x1Cu), DDE_EMU_DYN_ID, plen / 2u, cycle, out);
    memcpy(out + 5, dde_emu_dyn_payload, plen);
    uint32_t c = dde_core_frame_crc(out, 5u + plen);
    out[5 + plen] = (uint8_t)(c >> 16);
    out[6 + plen] = (uint8_t)(c >> 8);
    out[7 + plen] = (uint8_t)c;
    return 8u + plen;
}

unsigned dde_core_pack(const uint8_t *frame, unsigned nbytes, unsigned tss_bits, unsigned dts_bits, uint32_t *words)
{
    if (nbytes == 0 || (nbytes & 3u) || nbytes > DDE_MAX_FRAME_BYTES) return 0;
    if (tss_bits < 3 || tss_bits > 15) return 0;
    if (dts_bits != 0 && dts_bits < 3) return 0;
    words[0] = ((uint32_t)(tss_bits - 2u) << 16) | (nbytes - 1u);
    words[1] = dts_bits ? dts_bits - 2u : 0u;
    unsigned w = 2;
    for (unsigned i = 0; i < nbytes; i += 4) {
        words[w++] = ((uint32_t)frame[i] << 24) | ((uint32_t)frame[i + 1] << 16) |
                     ((uint32_t)frame[i + 2] << 8) | frame[i + 3];
    }
    return w;
}

int dde_core_ref_index(unsigned id)
{
    for (int i = 0; i < DDE_EMU_N_REFS; i++) if (dde_emu_ref_id[i] == id) return i;
    return -1;
}

int dde_core_target_index(unsigned id)
{
    for (int i = 0; i < DDE_EMU_N_STATIC; i++) if (dde_emu_static_id[i] == id) return i;
    return -1;
}

void dde_core_schedule(uint16_t ref_fine, uint32_t ref_us, int32_t delta_q8,
                       int32_t loop_ticks, uint32_t lead_us, dde_sched_t *out)
{
    int32_t delta_ticks = (delta_q8 + 128) >> 8;   // round to 10 ns
    out->bus_fine = (uint16_t)(ref_fine + (uint32_t)delta_ticks);
    out->txen_fine = (uint16_t)(out->bus_fine - (uint32_t)loop_ticks);
    int32_t d = delta_ticks - loop_ticks;
    int32_t d_us = d >= 0 ? d / 100 : -((-d + 99) / 100);   // floor
    out->alarm_us = ref_us + (uint32_t)d_us - lead_us;
}

void dde_lock_reset(dde_lock_t *l)
{
    memset(l, 0, sizeof(*l));
}

int dde_lock_on_ref(dde_lock_t *l, unsigned cycle, uint32_t cs_us)
{
    cycle &= 63u;
    if (!l->have) {
        l->have = true;
        l->last_cycle = (uint8_t)cycle;
        l->last_cs_us = cs_us;
        l->good_run = 1;
        l->good_cycles++;
        l->wakes++;
        l->locked = DDE_LOCK_REARM <= 1u;
        return DDE_LOCK_NEW_WAKE;
    }
    int32_t dt = (int32_t)(cs_us - l->last_cs_us);
    if (dt < 0 && dt > -(int32_t)(DDE_CYCLE_US / 2u)) dt = 0;  // same cycle, earlier-slot estimate
    if (dt < 0) {
        // timestamps going backwards by more than half a cycle: inconsistent, restart
        if (l->locked) l->lock_losses++;
        l->locked = false;
        l->good_run = 1;
        l->last_cycle = (uint8_t)cycle;
        l->last_cs_us = cs_us;
        return DDE_LOCK_BAD;
    }
    if ((uint32_t)dt > DDE_LOCK_WAKE_US) {
        if (l->locked) l->lock_losses++;
        l->locked = false;
        l->good_run = 1;
        l->good_cycles++;
        l->wakes++;
        l->last_cycle = (uint8_t)cycle;
        l->last_cs_us = cs_us;
        return DDE_LOCK_NEW_WAKE;
    }
    uint32_t n = ((uint32_t)dt + DDE_CYCLE_US / 2u) / DDE_CYCLE_US;   // cycles since the last good one
    unsigned expect = (l->last_cycle + n) & 63u;
    if (cycle != expect) {
        l->cycle_jumps++;
        if (l->locked) l->lock_losses++;
        l->locked = false;
        l->good_run = 1;
        l->last_cycle = (uint8_t)cycle;
        l->last_cs_us = cs_us;
        return DDE_LOCK_BAD;
    }
    if (n == 0) return DDE_LOCK_SAME_CYCLE;
    l->good_cycles++;
    if (n - 1u > DDE_LOCK_MAX_GAP) {
        if (l->locked) l->lock_losses++;
        l->locked = false;
        l->good_run = 1;
    } else if (n == 1u) {
        l->good_run++;
    } else if (!l->locked) {
        l->good_run = 1;          // a gap while arming restarts the count
    }
    if (!l->locked && l->good_run >= DDE_LOCK_REARM) l->locked = true;
    l->last_cycle = (uint8_t)cycle;
    l->last_cs_us = cs_us;
    return DDE_LOCK_NEW_CYCLE;
}

bool dde_lock_poll(dde_lock_t *l, uint32_t now_us)
{
    if (!l->have || !l->locked) return false;
    // the last good cycle started at last_cs_us; cycles last_cycle+1 .. +GAP may be missed,
    // the next one must have produced a reference by the end of cycle last+GAP+1.
    if ((int32_t)(now_us - l->last_cs_us) > (int32_t)((DDE_LOCK_MAX_GAP + 2u) * DDE_CYCLE_US)) {
        l->locked = false;
        l->good_run = 0;
        l->lock_losses++;
        return true;
    }
    return false;
}
