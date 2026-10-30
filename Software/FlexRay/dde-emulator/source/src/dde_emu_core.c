#include "dde_emu_core.h"
#include <string.h>
#include "dde_emu_tables.h"

// v1b: everything the header ISR / alarm ISR calls runs from RAM on the target
// (no XIP cache misses with interrupts disabled). Host builds: plain functions.
#if defined(__arm__)
#include "pico.h"
#define DDE_RAM(f) __time_critical_func(f)
#else
#define DDE_RAM(f) f
#endif

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

uint16_t DDE_RAM(dde_core_header_crc)(unsigned sync, unsigned startup, unsigned frame_id, unsigned len_words)
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

uint32_t DDE_RAM(dde_core_frame_crc)(const uint8_t *p, unsigned n)
{
    if (!tables_ready) dde_core_init();
    uint32_t crc = 0xFEDCBAu;
    while (n--) crc = ((crc << 8) ^ crc24_table[((crc >> 16) ^ *p++) & 0xFFu]) & 0xFFFFFFu;
    return crc;
}

uint8_t DDE_RAM(dde_core_crc8)(const uint8_t *p, unsigned n)
{
    if (!tables_ready) dde_core_init();
    uint8_t c = 0;
    while (n--) c = crc8_table[c ^ *p++];
    return c;
}

void DDE_RAM(dde_core_header)(uint8_t ind, unsigned id, unsigned len_words, unsigned cycle, uint8_t h[5])
{
    uint16_t hc = dde_core_header_crc((ind >> 1) & 1u, ind & 1u, id, len_words);
    h[0] = (uint8_t)(((ind & 0x1Fu) << 3) | ((id >> 8) & 7u));
    h[1] = (uint8_t)(id & 0xFFu);
    h[2] = (uint8_t)(((len_words & 0x7Fu) << 1) | ((hc >> 10) & 1u));
    h[3] = (uint8_t)((hc >> 2) & 0xFFu);
    h[4] = (uint8_t)(((hc & 3u) << 6) | (cycle & 0x3Fu));
}

unsigned DDE_RAM(dde_core_e2e_apply)(uint8_t *payload, unsigned target, unsigned cycle, unsigned counter)
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

uint8_t DDE_RAM(dde_core_emu_indicators)(unsigned target, unsigned cycle, bool keep_sync)
{
    uint8_t cap = dde_emu_static_ind[target][cycle & 63u];
    uint8_t ind = (uint8_t)(cap & 0x1Cu);       // reserved, preamble, null-frame indicator
    if (keep_sync) ind |= (uint8_t)(cap & 0x03u);
    return ind;
}

unsigned DDE_RAM(dde_core_build_static)(unsigned target, unsigned cycle, unsigned counter, bool keep_sync, uint8_t *out)
{
    return dde_core_build_static_ex(target, cycle, counter, keep_sync, false, out);
}

// Pre-ign content of the real DDE (run 4, Fred f11_emu2_dsc.md section 2):
// 0x28 c%4==1 b3-b5 c1 17 7c (ign-on a7 79 9a), 0x29 c%4==3 b8 80 (ign-on 10).
unsigned DDE_RAM(dde_core_build_static_ex)(unsigned target, unsigned cycle, unsigned counter, bool keep_sync,
                                           bool preign, uint8_t *out)
{
    const unsigned plen = DDE_EMU_STATIC_PAYLOAD_BYTES;
    uint8_t ind = dde_core_emu_indicators(target, cycle, keep_sync);
    dde_core_header(ind, dde_emu_static_id[target], plen / 2u, cycle, out);
    memcpy(out + 5, dde_emu_static_payload[target][cycle & 63u], plen);
    if (preign && (ind & 0x04u)) {
        if (target == 1 && (cycle & 3u) == 1) { out[5 + 2] = 0xC1; out[5 + 3] = 0x17; out[5 + 4] = 0x7C; }
        if (target == 2 && (cycle & 3u) == 3) out[5 + 7] = 0x80;
    }
    if (ind & 0x04u) dde_core_e2e_apply(out + 5, target, cycle, counter);
    uint32_t c = dde_core_frame_crc(out, 5u + plen);
    out[5 + plen] = (uint8_t)(c >> 16);
    out[6 + plen] = (uint8_t)(c >> 8);
    out[7 + plen] = (uint8_t)c;
    return 8u + plen;
}

bool DDE_RAM(dde_core_dyn_cycle)(unsigned cycle)
{
    return (cycle & DDE_EMU_DYN_CYCLE_MASK) == DDE_EMU_DYN_CYCLE_BASE;
}

unsigned DDE_RAM(dde_core_build_dyn)(unsigned cycle, uint8_t *out)
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

unsigned DDE_RAM(dde_core_pack)(const uint8_t *frame, unsigned nbytes, unsigned tss_bits, unsigned dts_bits, uint32_t *words)
{
    // nbytes % 4 == 2 is allowed (6-byte payload frames): the last word is
    // padded; the TX SM then parks at idle+1 after the frame and the firmware
    // restarts it (see tx_sm_idle_or_parked in dde_emu.c).
    if (nbytes < 4 || (nbytes & 1u) || nbytes > DDE_MAX_FRAME_BYTES) return 0;
    if (tss_bits < 3 || tss_bits > 15) return 0;
    if (dts_bits != 0 && dts_bits < 3) return 0;
    words[0] = ((uint32_t)(tss_bits - 2u) << 16) | (nbytes - 1u);
    words[1] = dts_bits ? dts_bits - 2u : 0u;
    unsigned w = 2;
    for (unsigned i = 0; i < nbytes; i += 4) {
        uint32_t v = ((uint32_t)frame[i] << 24) | ((uint32_t)frame[i + 1] << 16);
        if (i + 2 < nbytes) v |= ((uint32_t)frame[i + 2] << 8) | frame[i + 3];
        words[w++] = v;
    }
    return w;
}

int DDE_RAM(dde_core_ref_index)(unsigned id)
{
    for (int i = 0; i < DDE_EMU_N_REFS; i++) if (dde_emu_ref_id[i] == id) return i;
    return -1;
}

int DDE_RAM(dde_core_target_index)(unsigned id)
{
    for (int i = 0; i < DDE_EMU_N_STATIC; i++) if (dde_emu_static_id[i] == id) return i;
    return -1;
}

void DDE_RAM(dde_core_schedule)(uint16_t ref_fine, uint32_t ref_us, int32_t delta_q8,
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

int DDE_RAM(dde_lock_on_ref)(dde_lock_t *l, unsigned cycle, uint32_t cs_us)
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

bool DDE_RAM(dde_lock_poll)(dde_lock_t *l, uint32_t now_us)
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

// ---------------------------------------------------------------------------
// v1c dynamic PDUs. Content from run 4/5 real DDE ign-on (engine off), Fred's
// dde_dynamic_ignon_template.csv (v1c_rank_after_emu2 1..6).
const uint16_t dde_dyn_slot_id[DDE_DYN_N_SLOT] = {0xE6, 0xE8, 0xFB};

const dde_dyn_pdu_t dde_dyn_pdu[DDE_DYN_N_PDU] = {
    // Content = Fred's v1c spec (f11_emu2_dsc.md). Pre-ign: the real DDE (runs 4 and 5) sends
    // the same layout before ign after a ~0.3 s start-up, except 0xFB L9 = 1f 00 ff 7f 00 f3 ff ff.
    // id   slot           plen %8 kind               k     hi    hi_pre ign  period
    {0xE6, DDE_DSLOT_E6, 8, 2, DDE_DYN_KIND_E2E,   0x04, 0x10, 0x10, 0, 200,
     {0x00, 0x10, 0x80, 0x64, 0x45, 0x44, 0x23, 0x47}, {0x00, 0x10, 0x80, 0x64, 0x45, 0x44, 0x23, 0x47}, "E6"},
    {0xFB, DDE_DSLOT_FB, 4, 5, DDE_DYN_KIND_CTR,   0x00, 0x40, 0x40, 0, 1000,
     {0x40, 0x60, 0xF1, 0xFF}, {0x40, 0x60, 0xF1, 0xFF}, "FB5"},
    {0xFB, DDE_DSLOT_FB, 6, 6, DDE_DYN_KIND_CTR,   0x00, 0xF0, 0xF0, 0, 40,
     {0xF0, 0xFF, 0x47, 0x32, 0x00, 0xF0}, {0xF0, 0xFF, 0x47, 0x32, 0x00, 0xF0}, "FB7"},
    {0xFB, DDE_DSLOT_FB, 8, 3, DDE_DYN_KIND_CONST, 0x00, 0x00, 0x00, 0, 1000,
     {0x1F, 0x00, 0x04, 0x00, 0x00, 0xF3, 0xFF, 0xFF}, {0x1F, 0x00, 0xFF, 0x7F, 0x00, 0xF3, 0xFF, 0xFF}, "FB9"},
    {0xE8, DDE_DSLOT_E8, 8, 3, DDE_DYN_KIND_CONST, 0x00, 0x00, 0x00, 0, 200,
     {0x00, 0x00, 0x00, 0x9A, 0x29, 0xF3, 0x56, 0xF0}, {0x00, 0x00, 0x00, 0x9A, 0x29, 0xF3, 0x56, 0xF0}, "E89"},
    {0xE8, DDE_DSLOT_E8, 4, 0, DDE_DYN_KIND_CONST, 0x00, 0x00, 0x00, 1, 1000,
     {0x86, 0x78, 0xFE, 0xFE}, {0x86, 0x78, 0xFE, 0xFE}, "E85"},
};

int DDE_RAM(dde_core_dyn_pdu_for)(unsigned dslot, unsigned cycle)
{
    for (int p = 0; p < DDE_DYN_N_PDU; p++)
        if (dde_dyn_pdu[p].dslot == dslot && dde_dyn_pdu[p].cyc_mod8 == (cycle & 7u)) return p;
    return -1;
}

int DDE_RAM(dde_core_dyn_slot_of_id)(unsigned id)
{
    for (int s = 0; s < DDE_DYN_N_SLOT; s++) if (dde_dyn_slot_id[s] == id) return s;
    return -1;
}

unsigned DDE_RAM(dde_core_build_dyn_pdu)(unsigned p, unsigned cycle, unsigned counter, bool preign, uint8_t *out)
{
    const dde_dyn_pdu_t *d = &dde_dyn_pdu[p];
    const unsigned plen = d->plen;
    dde_core_header(0x04u, d->id, plen / 2u, cycle, out);
    uint8_t *pl = out + 5;
    memcpy(pl, preign ? d->payload_pre : d->payload, plen);
    uint8_t hi = preign ? d->hi_pre : d->hi;
    if (d->kind == DDE_DYN_KIND_E2E) {
        pl[1] = (uint8_t)(hi | (counter & 0x0Fu));
        pl[0] = (uint8_t)(dde_core_crc8(&pl[1], plen - 1u) ^ d->k);
    } else if (d->kind == DDE_DYN_KIND_CTR) {
        pl[0] = (uint8_t)(hi | (counter & 0x0Fu));
    }
    uint32_t c = dde_core_frame_crc(out, 5u + plen);
    out[5 + plen] = (uint8_t)(c >> 16);
    out[6 + plen] = (uint8_t)(c >> 8);
    out[7 + plen] = (uint8_t)c;
    return 8u + plen;
}

// Run 5 fit (all senders): payload bytes -> minislots from the frame's AP to
// the next frame's AP (frame end + idle detection, rounded up, + 2).
unsigned DDE_RAM(dde_core_dyn_ms_used)(unsigned payload_bytes)
{
    switch (payload_bytes) {
    case 2: case 4: return 4;
    case 6: case 8: return 5;
    case 14: return 6;
    default: return 0;
    }
}

unsigned dde_core_dyn_dts_bits(unsigned nbytes, unsigned tss_bits)
{
    // frame bits: TSS + FSS + 10 per byte + FES(2); 10 ticks per bit
    uint32_t frame_q8 = (uint32_t)(tss_bits + 1u + 10u * nbytes + 2u) * 10u * 256u;
    uint32_t m = (frame_q8 + 20u * 256u + DDE_DYN_MS_Q8 - 1u) / DDE_DYN_MS_Q8;   // >= 0.2 us DTS
    uint32_t dts_q8 = m * DDE_DYN_MS_Q8 - frame_q8;
    return dts_q8 / (10u * 256u);
}

int32_t DDE_RAM(dde_core_elapsed_ticks)(uint32_t us0, uint16_t fine0, uint32_t us1, uint16_t fine1)
{
    int32_t dus = (int32_t)(us1 - us0);
    int32_t approx = dus * 100;                      // coarse ticks (+-100)
    int32_t lo = (int32_t)(int16_t)(uint16_t)((uint32_t)fine1 - (uint32_t)fine0 - (uint32_t)approx);
    return approx + lo;                              // exact mod 65536 near approx
}

void DDE_RAM(dde_dyn_track_start)(dde_dyn_track_t *t, unsigned cycle, bool mark_ok, uint8_t seq)
{
    t->active = true;
    t->cycle = (uint8_t)cycle;
    t->anchor_ok = mark_ok;
    t->why = mark_ok ? DDE_DYN_OK : DDE_SKIP_NOANCHOR;
    t->acc_seq = seq;
    t->have_dyn = false;
    t->last_k = 0;
    t->last_id = 0;
    t->last_ms = 0;
    t->max_id = 0;
}

void DDE_RAM(dde_dyn_track_header)(dde_dyn_track_t *t, unsigned id, unsigned payload_bytes, bool mark_ok,
                                   uint8_t seq, int64_t x_q8)
{
    if (!t->active) return;
    if (id < DDE_DYN_FIRST_ID) {
        // static frame: only extends the explained marks
        if (mark_ok) {
            if (!t->have_dyn && !t->anchor_ok && t->why == DDE_SKIP_NOANCHOR) { t->anchor_ok = true; t->why = 0; }
            if (t->anchor_ok) t->acc_seq = seq;
        } else if (t->anchor_ok) {
            t->anchor_ok = false; t->why = DDE_SKIP_NOANCHOR;
        }
        return;
    }
    if (id > t->max_id) t->max_id = (uint16_t)id;
    if (!mark_ok) { t->anchor_ok = false; t->why = DDE_SKIP_NOANCHOR; return; }
    int64_t rel = x_q8 - DDE_DYN_BASE_Q8;
    int64_t n = rel >= 0 ? (rel + DDE_DYN_MS_Q8 / 2) / DDE_DYN_MS_Q8 : -1;
    int64_t frac = rel - n * DDE_DYN_MS_Q8;
    if (n < (int64_t)(id - DDE_DYN_FIRST_ID) || n > 1000 || frac > DDE_DYN_GRID_TOL_Q8 || frac < -DDE_DYN_GRID_TOL_Q8) {
        t->anchor_ok = false; t->why = DDE_SKIP_GRID; return;
    }
    unsigned k = dde_core_dyn_ms_used(payload_bytes);
    if (!k) { t->anchor_ok = false; t->why = DDE_SKIP_LEN; return; }
    t->have_dyn = true;
    t->anchor_ok = true;
    t->why = DDE_DYN_OK;
    t->last_id = (uint16_t)id;
    t->last_ms = (uint16_t)n;
    t->last_k = (uint8_t)k;
    t->acc_seq = seq;
}

int DDE_RAM(dde_dyn_predict)(const dde_dyn_track_t *t, unsigned id, unsigned *m_out)
{
    if (!t->active) return DDE_SKIP_NOANCHOR;
    if (t->max_id >= id) return DDE_SKIP_PASSED;
    if (!t->anchor_ok) return t->why ? t->why : DDE_SKIP_NOANCHOR;
    unsigned m = t->have_dyn ? (unsigned)t->last_ms + t->last_k + (id - t->last_id - 1u) : id - DDE_DYN_FIRST_ID;
    if (m > DDE_DYN_MS_LIMIT) return DDE_SKIP_LIMIT;
    *m_out = m;
    return DDE_DYN_OK;
}
