// Host test of the DDE emulator core (src/dde_emu_core.c) against captured
// run 5 DDE frames (text vectors: "id cycle rawhex"), plus schedule maths and
// the reference-lock state machine.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dde_emu_core.h"
#include "dde_emu_tables.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; if (fails < 20) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static int hexval(char c) { return c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10; }

static void test_vectors(const char *path, bool strict)
{
    FILE *f = fopen(path, "r");
    if (!f) { printf("FAIL: cannot open %s\n", path); fails++; return; }
    char line[512];
    unsigned n_rebuild_diff = 0;
    unsigned n = 0, n_hcrc = 0, n_fcrc = 0, n_e2e = 0, n_build = 0, n_dyn = 0, n_pack = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        unsigned id, cycle;
        char hex[400];
        if (sscanf(line, "%u %u %399s", &id, &cycle, hex) != 3) continue;
        uint8_t raw[200];
        unsigned len = (unsigned)strlen(hex) / 2;
        for (unsigned i = 0; i < len; i++) raw[i] = (uint8_t)(hexval(hex[2 * i]) << 4 | hexval(hex[2 * i + 1]));
        n++;
        unsigned ind = raw[0] >> 3, lw = raw[2] >> 1;
        unsigned hc = ((raw[2] & 1u) << 10) | ((unsigned)raw[3] << 2) | (raw[4] >> 6);
        CHECK(dde_core_header_crc((ind >> 1) & 1, ind & 1, id, lw) == hc, "hcrc id %u c %u", id, cycle);
        n_hcrc++;
        uint32_t fc = ((uint32_t)raw[len - 3] << 16) | ((uint32_t)raw[len - 2] << 8) | raw[len - 1];
        CHECK(dde_core_frame_crc(raw, len - 3) == fc, "fcrc id %u c %u", id, cycle);
        n_fcrc++;
        int t = dde_core_target_index(id);
        if (t >= 0 && (ind & 4)) {
            // E2E: re-apply with the captured counter -> must reproduce the captured payload
            uint8_t p[16];
            memcpy(p, raw + 5, 16);
            unsigned ctr = 0;
            for (unsigned g = 0; g < DDE_EMU_N_E2E; g++)
                if (dde_emu_e2e[g][0] == t && dde_emu_e2e[g][1] == (cycle & 3)) { ctr = p[dde_emu_e2e[g][2] + 1] & 15; break; }
            uint8_t q[16];
            memcpy(q, p, 16);
            unsigned ng = dde_core_e2e_apply(q, (unsigned)t, cycle, ctr);
            CHECK(ng > 0 && memcmp(p, q, 16) == 0, "e2e id 0x%X c %u", id, cycle);
            n_e2e++;
        }
        if (t >= 0) {
            // full frame from the tables (captured sync bits kept) == captured raw frame
            unsigned ctr = 0;
            for (unsigned g = 0; g < DDE_EMU_N_E2E; g++)
                if (dde_emu_e2e[g][0] == t && dde_emu_e2e[g][1] == (cycle & 3)) { ctr = raw[5 + dde_emu_e2e[g][2] + 1] & 15; break; }
            uint8_t b[DDE_MAX_FRAME_BYTES];
            unsigned bn = dde_core_build_static((unsigned)t, cycle, ctr, true, b);
            bool same = bn == len && memcmp(b, raw, len) == 0;
            if (!same) n_rebuild_diff++;
            CHECK(same || !strict, "build id 0x%X c %u", id, cycle);
            n_build++;
            // emulator header: sync/startup cleared, header CRC recomputed
            unsigned en = dde_core_build_static((unsigned)t, cycle, ctr, false, b);
            unsigned eind = b[0] >> 3;
            unsigned ehc = ((b[2] & 1u) << 10) | ((unsigned)b[3] << 2) | (b[4] >> 6);
            CHECK((eind & 3) == 0 && (!same || (eind & 4) == (ind & 4)), "emu ind id 0x%X", id);
            CHECK(ehc == dde_core_header_crc(0, 0, id, 8), "emu hcrc id 0x%X", id);
            CHECK((b[4] & 63) == cycle, "cycle field");
            CHECK(!same || memcmp(b + 5, raw + 5, 16) == 0, "emu payload id 0x%X c %u", id, cycle);
            uint32_t efc = ((uint32_t)b[en - 3] << 16) | ((uint32_t)b[en - 2] << 8) | b[en - 1];
            CHECK(dde_core_frame_crc(b, en - 3) == efc, "emu fcrc");
            // PIO words round trip
            uint32_t w[DDE_TX_MAX_WORDS];
            unsigned nw = dde_core_pack(b, en, EMU_TSS_BITS, 0, w);
            CHECK(nw == 2 + en / 4 && w[0] == ((uint32_t)(EMU_TSS_BITS - 2) << 16 | (en - 1)) && w[1] == 0, "pack hdr");
            for (unsigned i = 0; i < en; i++) CHECK(((w[2 + i / 4] >> (24 - 8 * (i % 4))) & 0xFF) == b[i], "pack byte");
            n_pack++;
        }
        if (id == DDE_EMU_DYN_ID && dde_core_dyn_cycle(cycle)) {
            uint8_t b[DDE_MAX_FRAME_BYTES];
            unsigned bn = dde_core_build_dyn(cycle, b);
            CHECK(!strict || (bn == len && memcmp(b, raw, len) == 0), "dyn build c %u", cycle);
            uint32_t w[DDE_TX_MAX_WORDS];
            CHECK(dde_core_pack(b, bn, EMU_TSS_BITS, DDE_EMU_DYN_DTS_BITS, w) == 2 + bn / 4 &&
                  w[1] == DDE_EMU_DYN_DTS_BITS - 2, "dyn pack");
            n_dyn++;
        }
    }
    fclose(f);
    printf("%s: %u frames: hcrc %u, fcrc %u, e2e %u, table rebuild %u (%u differ%s), pack %u, dyn 0x5C %u\n",
           path, n, n_hcrc, n_fcrc, n_e2e, n_build, n_rebuild_diff,
           strict ? "" : ": other vehicle states, informational", n_pack, n_dyn);
    CHECK(n > 1000 && n_e2e > 100 && n_build > 500, "too few vectors");
}

static void test_pack_args(void)
{
    uint8_t b[24] = {0};
    uint32_t w[DDE_TX_MAX_WORDS];
    CHECK(dde_core_pack(b, 21, 7, 0, w) == 0, "odd length rejected");
    CHECK(dde_core_pack(b, 24, 2, 0, w) == 0, "tss 2 rejected");
    CHECK(dde_core_pack(b, 24, 16, 0, w) == 0, "tss 16 rejected");
    CHECK(dde_core_pack(b, 24, 7, 2, w) == 0, "dts 2 rejected");
    CHECK(dde_core_pack(b, 36, 7, 0, w) == 0, "too long rejected");
    CHECK(dde_core_pack(b, 24, 7, 0, w) == 8, "24 bytes = 8 words (TX FIFO depth)");
}

static void test_schedule(void)
{
    dde_sched_t s;
    // 0x28 from 0x27: one slot later
    int32_t delta = dde_emu_static_off_q8[1] - dde_emu_ref_off_q8[2];
    dde_core_schedule(0xFF00, 1000000u, delta, 25, 6, &s);
    int32_t dt = (delta + 128) >> 8;
    CHECK(abs(dt - 3300) <= 3, "0x27 -> 0x28 = one slot, 3300 ticks (got %d)", dt);
    CHECK(s.bus_fine == (uint16_t)(0xFF00 + dt), "bus wraps mod 2^16");
    CHECK(s.txen_fine == (uint16_t)(0xFF00 + dt - 25), "txen = bus - loop");
    CHECK(s.alarm_us == 1000000u + 32u - 6u, "alarm us (got %u)", (unsigned)s.alarm_us);
    // a long delta (0x3D from 0x21, 28 slots = 924 us > 655 us fine wrap) still lands right mod 2^16
    delta = dde_emu_static_off_q8[3] - dde_emu_ref_off_q8[1];
    dde_core_schedule(1234, 5u, delta, 0, 6, &s);
    dt = (delta + 128) >> 8;
    CHECK(s.bus_fine == (uint16_t)(1234 + dt), "long delta mod 2^16");
    CHECK(s.alarm_us == 5u + (uint32_t)(dt / 100) - 6u, "long delta alarm");
    // fire delay wraps
    CHECK(dde_core_fire_delay(10, 65530, 30) == 16 - 30, "fire delay across wrap");
    CHECK(dde_core_fire_delay(600, 0, 30) == 570, "fire delay");
    CHECK(dde_core_fire_delay(0, 600, 30) == -630, "late is negative");
    // targets: slot grid sanity (slot ids 38, 40, 41, 61 at 33.00285 us/slot)
    int32_t d26_28 = dde_emu_static_off_q8[1] - dde_emu_static_off_q8[0];
    CHECK(abs(d26_28 - 2 * DDE_EMU_SLOT_Q8) < 256 * 2, "0x26 -> 0x28 = 2 slots");
    int32_t d29_3d = dde_emu_static_off_q8[3] - dde_emu_static_off_q8[2];
    CHECK(abs(d29_3d - 20 * DDE_EMU_SLOT_Q8) < 256 * 4, "0x29 -> 0x3D = 20 slots");
    // every target has a reference at least 20 us before it
    for (int t = 0; t < DDE_EMU_N_STATIC; t++) {
        int ok = 0;
        for (int r = 0; r < DDE_EMU_N_REFS; r++)
            if (dde_emu_static_off_q8[t] - dde_emu_ref_off_q8[r] >= 20 * 100 * 256) ok = 1;
        CHECK(ok, "target %d has a reference", t);
    }
    // no reference is one of our IDs
    for (int r = 0; r < DDE_EMU_N_REFS; r++) CHECK(dde_core_target_index(dde_emu_ref_id[r]) < 0, "ref is target");
}

static void test_lock(void)
{
    dde_lock_t l;
    dde_lock_reset(&l);
    uint32_t t = 100000;
    unsigned c = 10;
    CHECK(dde_lock_on_ref(&l, c, t) == DDE_LOCK_NEW_WAKE, "first ref = wake");
    for (int i = 1; i < 7; i++) CHECK(dde_lock_on_ref(&l, c + i, t + 5000u * i) == DDE_LOCK_NEW_CYCLE, "cycle");
    CHECK(!l.locked, "not locked after 7");
    CHECK(dde_lock_on_ref(&l, c + 6, t + 5000u * 6 + 300) == DDE_LOCK_SAME_CYCLE, "second ref same cycle");
    CHECK(dde_lock_on_ref(&l, c + 7, t + 5000u * 7 - 1) == DDE_LOCK_NEW_CYCLE && l.locked, "locked after 8");
    // 2 missing cycles: still locked
    unsigned cc = c + 10; uint32_t tt = t + 5000u * 10;
    CHECK(dde_lock_on_ref(&l, cc, tt) == DDE_LOCK_NEW_CYCLE && l.locked, "gap of 2 keeps lock");
    CHECK(!dde_lock_poll(&l, tt + 19000), "poll within 3 cycles ok");
    // 3 missing cycles: lost
    cc += 4; tt += 20000;
    dde_lock_on_ref(&l, cc, tt);
    CHECK(!l.locked && l.lock_losses == 1, "gap of 3 loses lock");
    for (int i = 1; i < 7; i++) dde_lock_on_ref(&l, (cc + i) & 63, tt + 5000u * i);
    CHECK(!l.locked, "re-arm needs 8");
    dde_lock_on_ref(&l, (cc + 7) & 63, tt + 5000u * 7);
    CHECK(l.locked, "re-armed after 8 good cycles");
    cc += 7; tt += 35000;
    // poll timeout
    CHECK(!dde_lock_poll(&l, tt + 20000), "poll at 4 cycles - not yet");
    CHECK(dde_lock_poll(&l, tt + 20001) && !l.locked && l.lock_losses == 2, "poll loses lock");
    // cycle counter jump
    dde_lock_reset(&l);
    for (int i = 0; i < 8; i++) dde_lock_on_ref(&l, (60 + i) & 63, 1000u + 5000u * i);
    CHECK(l.locked, "locked across 63 -> 0 wrap");
    CHECK(dde_lock_on_ref(&l, 30, 1000u + 5000u * 8) == DDE_LOCK_BAD && !l.locked, "cycle jump");
    // new wake after silence
    dde_lock_reset(&l);
    for (int i = 0; i < 8; i++) dde_lock_on_ref(&l, i, 1000u + 5000u * i);
    CHECK(dde_lock_on_ref(&l, 0, 1000u + 5000u * 8 + 2000000u) == DDE_LOCK_NEW_WAKE && !l.locked && l.wakes == 2,
          "silence > 1 s = new wake");
}


// ---------------------------------------------------------------- v1c dynamic segment
// Vectors from tools/dde_emu/extract_dyn.py: "t_ticks ch id cycle plen hw payloadhex|-",
// header "# ign_ticks N".
typedef struct { int64_t t; uint8_t ch, hw, plen, cycle; uint16_t id; uint8_t pl[32]; bool has_pl; } dv_t;

static dv_t *load_dyn(const char *path, unsigned *n_out, int64_t *ign)
{
    FILE *f = fopen(path, "r");
    if (!f) { printf("FAIL: cannot open %s\n", path); fails++; *n_out = 0; return NULL; }
    unsigned cap = 4096, n = 0;
    dv_t *v = malloc(cap * sizeof *v);
    char line[256];
    *ign = -1;
    while (fgets(line, sizeof line, f)) {
        long long x;
        if (sscanf(line, "# ign_ticks %lld", &x) == 1) { *ign = x; continue; }
        if (line[0] == '#') continue;
        long long t; unsigned ch, id, cy, pl, hw; char hex[80];
        if (sscanf(line, "%lld %u %u %u %u %u %79s", &t, &ch, &id, &cy, &pl, &hw, hex) != 7) continue;
        if (n == cap) { cap *= 2; v = realloc(v, cap * sizeof *v); }
        dv_t *d = &v[n++];
        memset(d, 0, sizeof *d);
        d->t = t; d->ch = (uint8_t)ch; d->id = (uint16_t)id; d->cycle = (uint8_t)cy; d->plen = (uint8_t)pl; d->hw = (uint8_t)hw;
        if (hex[0] != '-') {
            d->has_pl = true;
            for (unsigned i = 0; i < pl && i < 32; i++) d->pl[i] = (uint8_t)(hexval(hex[2 * i]) << 4 | hexval(hex[2 * i + 1]));
        }
    }
    fclose(f);
    *n_out = n;
    return v;
}

static int pdu_of(unsigned id, unsigned plen)
{
    for (int p = 0; p < DDE_DYN_N_PDU; p++) if (dde_dyn_pdu[p].id == id && dde_dyn_pdu[p].plen == plen) return p;
    return -1;
}

// Builder vs the real DDE's 0xE6/0xE8/0xFB (run 5 FR2): E2E key, counter sequence,
// content, cycle classes.
static void test_dyn_build(const char *path)
{
    unsigned n;
    int64_t ign;
    dv_t *v = load_dyn(path, &n, &ign);
    if (!v) return;
    CHECK(ign > 0, "ign time in %s", path);
    unsigned cnt[DDE_DYN_N_PDU] = {0}, exact[DDE_DYN_N_PDU] = {0}, e2e_ok = 0, e2e_n = 0, ctr_ok = 0, ctr_n = 0;
    unsigned diffpos[DDE_DYN_N_PDU][8] = {{0}}, other = 0, ncmp[DDE_DYN_N_PDU] = {0};
    int64_t first_t = n ? v[0].t : 0;
    unsigned cyc_mask[DDE_DYN_N_PDU] = {0};
    int last_ctr[DDE_DYN_N_PDU];
    int64_t last_t[DDE_DYN_N_PDU];
    for (int p = 0; p < DDE_DYN_N_PDU; p++) { last_ctr[p] = -1; last_t[p] = 0; }
    for (unsigned i = 0; i < n; i++) {
        dv_t *d = &v[i];
        if (d->ch != 2 || !d->has_pl) continue;
        int p = pdu_of(d->id, d->plen);
        if (p < 0) { other++; continue; }
        const dde_dyn_pdu_t *q = &dde_dyn_pdu[p];
        cnt[p]++;
        cyc_mask[p] |= 1u << (d->cycle & 7);
        // pre-ign content until ign + 60 ms (the DDE switches 37-87 ms after ign; ignore that window)
        int64_t dt = d->t - ign;
        if (ign > 0 && dt > 3000000 && dt < 9000000) continue;
        // content: compare in the steady states (pre-ign after the ~1 s start-up, ign-on idle
        // up to 50 s; later in run 5 the drive state changes 0xFB L9 b2 / 0xE8 / 0xFB L7 values)
        bool cmp_win = ign > 0 && ((dt < 0 && d->t > first_t + 100000000) || (dt > 25000000 && dt < 5000000000LL));
        bool preign = ign < 0 || dt < 6000000;
        unsigned ctr = 0;
        if (q->kind == DDE_DYN_KIND_E2E) {
            ctr = d->pl[1] & 15;
            e2e_n++;
            if (d->pl[0] == (uint8_t)(dde_core_crc8(d->pl + 1, d->plen - 1u) ^ q->k)) e2e_ok++;
        } else if (q->kind == DDE_DYN_KIND_CTR) {
            ctr = d->pl[0] & 15;
        }
        if (q->kind != DDE_DYN_KIND_CONST) {
            // counter: +1 per transmission (0..14); a frame the capture lost = +2 after 2 periods
            int64_t per_t = (int64_t)q->period * 500000;
            int steps = (int)((d->t - last_t[p] + per_t / 2) / per_t);
            if (last_ctr[p] >= 0 && steps >= 1 && steps <= 4) {
                ctr_n++;
                if ((int)ctr == (last_ctr[p] + steps) % 15) ctr_ok++;
                else printf("  counter %s %d -> %u after %d periods (t %.3f s)\n", q->name, last_ctr[p], ctr, steps,
                            (double)d->t / 1e8);
            }
            last_ctr[p] = (int)ctr; last_t[p] = d->t;
            CHECK(ctr <= 14, "counter 15 seen id 0x%X", d->id);
        }
        uint8_t b[DDE_MAX_FRAME_BYTES];
        unsigned bn = dde_core_build_dyn_pdu((unsigned)p, d->cycle, ctr, preign, b);
        CHECK(bn == 5u + d->plen + 3u, "dyn length %s", q->name);
        // header: indicators 0x04 (payload preamble off, sync off, startup off), id, length, cycle
        CHECK(((b[0] & 7u) << 8 | b[1]) == d->id && (b[2] >> 1) == d->plen / 2u && (b[4] & 63u) == d->cycle,
              "dyn header %s", q->name);
        if (!cmp_win) continue;
        ncmp[p]++;
        if (memcmp(b + 5, d->pl, d->plen) == 0) exact[p]++;
        else for (unsigned j = 0; j < d->plen; j++) if (b[5 + j] != d->pl[j]) diffpos[p][j]++;
    }
    printf("dyn build vs real DDE (%s):\n", path);
    for (int p = 0; p < DDE_DYN_N_PDU; p++) {
        const dde_dyn_pdu_t *q = &dde_dyn_pdu[p];
        printf("  %-4s %4u frames, compared %4u, bit-exact %4u, cycle%%8 seen 0x%02X (ours %u); differing bytes:", q->name,
               cnt[p], ncmp[p], exact[p], cyc_mask[p], q->cyc_mod8);
        for (unsigned j = 0; j < q->plen; j++) if (diffpos[p][j]) printf(" b%u x%u", j + 1, diffpos[p][j]);
        printf("\n");
        CHECK(cnt[p] > 0, "%s present in run 5", q->name);
        // our cycle%8 lies in the PDU's stable class (period 40: %4, 200/1000: %2 at least)
        unsigned cls = q->period == 40 ? 3u : 1u;
        for (unsigned cy = 0; cy < 8; cy++)
            if (cyc_mask[p] & (1u << cy)) CHECK((cy & cls) == (q->cyc_mod8 & cls), "%s cycle class %u", q->name, cy);
    }
    printf("  E2E (0xE6 b1 = crc8(b2..b8) ^ 0x%02X): %u/%u; counter +1 mod 15: %u/%u; other PDUs %u\n",
           dde_dyn_pdu[DDE_PDU_E6].k, e2e_ok, e2e_n, ctr_ok, ctr_n, other);
    CHECK(e2e_n > 0 && e2e_ok == e2e_n, "E2E key");
    CHECK(ctr_n > 0 && ctr_ok * 100 >= ctr_n * 99, "counter sequence (>= 99%%, rest = capture timing)");
    CHECK(other == 0, "unknown 0xE6/0xE8/0xFB layout");
    // constant PDUs must be bit-exact (spec: constant content)
    CHECK(exact[DDE_PDU_FB9] == ncmp[DDE_PDU_FB9] && ncmp[DDE_PDU_FB9] > 5, "FB9 content");
    CHECK(exact[DDE_PDU_FB5] == ncmp[DDE_PDU_FB5] && ncmp[DDE_PDU_FB5] > 5, "FB5 content");
    CHECK(exact[DDE_PDU_E85] == ncmp[DDE_PDU_E85] && ncmp[DDE_PDU_E85] > 5, "E8 L5 content");
    free(v);
}

// Tracker vs the run 5 vehicle bus: predict every FR1 dynamic frame from the
// frames before it in the same cycle; ours must never be later than the real one
// (= our frame would start inside the real one).
static void test_dyn_tracker(const char *path)
{
    unsigned n;
    int64_t ign;
    dv_t *v = load_dyn(path, &n, &ign);
    if (!v) return;
    dde_dyn_track_t trk;
    memset(&trk, 0, sizeof trk);
    int64_t cs = 0;
    int cur = -1;
    bool prev_fr1 = true;
    uint8_t seq = 0;
    unsigned n_eval = 0, n_ok = 0, n_later = 0, n_early = 0, n_skip = 0, n_grid = 0, hist[8] = {0};
    for (unsigned i = 0; i < n; i++) {
        dv_t *d = &v[i];
        if (d->cycle != cur) {
            if (d->ch != 1 || d->id != dde_emu_ref_id[0]) continue;   // a cycle starts at its 0x07
            cur = d->cycle;
            cs = d->t * 256 - dde_emu_ref_off_q8[0];
            dde_dyn_track_start(&trk, d->cycle, d->hw != 0, ++seq);
            prev_fr1 = true;
            continue;
        }
        int64_t x = d->t * 256 - cs;
        bool dyn = d->id >= DDE_DYN_FIRST_ID;
        if (dyn && d->ch == 1 && prev_fr1 && d->hw) {
            unsigned m;
            int r = dde_dyn_predict(&trk, d->id, &m);
            double am = (double)(x - DDE_DYN_BASE_Q8) / DDE_DYN_MS_Q8;
            int ai = (int)(am + (am >= 0 ? 0.5 : -0.5));
            if (fabs(am - ai) > 0.25) n_grid++;
            else if (r != DDE_DYN_OK) n_skip++;
            else {
                n_eval++;
                int dm = ai - (int)m;
                if (dm == 0) n_ok++;
                else if (dm > 0) { n_later++; hist[dm < 7 ? dm : 7]++; }
                else n_early++;
            }
        }
        if (dyn) prev_fr1 = d->ch == 1;
        dde_dyn_track_header(&trk, d->id, d->plen, d->hw != 0, ++seq, x);
    }
    printf("dyn tracker vs run 5 vehicle frames (%s): %u predicted, %u exact, %u real later (capture gap; +1..7:",
           path, n_eval, n_ok, n_later);
    for (int k = 1; k < 8; k++) printf(" %u", hist[k]);
    printf("), %u real EARLIER, %u skip, %u off-grid\n", n_early, n_skip, n_grid);
    CHECK(n_eval > 10000, "tracker evaluated");
    CHECK(n_early == 0, "prediction later than the real frame");
    CHECK(n_ok * 100 >= n_eval * 95, "prediction exact >= 95%%");
    free(v);
}

static void test_dyn_basics(void)
{
    // DTS low bits (TSS 7): payload 4 -> 7, 6 -> 56, 8 -> 36 (fitted on run 5 FR2)
    CHECK(dde_core_dyn_dts_bits(12, 7) == 7, "dts 4 B (got %u)", dde_core_dyn_dts_bits(12, 7));
    CHECK(dde_core_dyn_dts_bits(14, 7) == 56, "dts 6 B (got %u)", dde_core_dyn_dts_bits(14, 7));
    CHECK(dde_core_dyn_dts_bits(16, 7) == 36, "dts 8 B (got %u)", dde_core_dyn_dts_bits(16, 7));
    CHECK(dde_core_dyn_ms_used(4) == 4 && dde_core_dyn_ms_used(6) == 5 && dde_core_dyn_ms_used(8) == 5 &&
          dde_core_dyn_ms_used(14) == 6 && dde_core_dyn_ms_used(20) == 0, "minislots used");
    // 14-byte frame: 2 control words + 4 data words, last half word = pad
    uint8_t b[16];
    for (int i = 0; i < 16; i++) b[i] = (uint8_t)(0x11 * (i + 1));
    uint32_t w[DDE_TX_MAX_WORDS], w2[DDE_TX_MAX_WORDS];
    unsigned nw = dde_core_pack(b, 14, 7, 56, w);
    CHECK(nw == 6, "14 bytes = 6 words (got %u)", nw);
    b[14] = 0xEE; b[15] = 0xEE;
    unsigned nw2 = dde_core_pack(b, 16, 7, 56, w2);
    CHECK(nw2 == 6 && memcmp(w + 2, w2 + 2, 3 * sizeof w[0]) == 0, "14-byte data words 1-3 = 16-byte");
    CHECK(((w[5] ^ w2[5]) & 0xFFFFu) == 0 || ((w[5] ^ w2[5]) & 0xFFFF0000u) == 0, "14-byte: one half word differs");
    // PDU table sanity: counters/E2E only where the spec has them, %8 unique per slot
    for (int p = 0; p < DDE_DYN_N_PDU; p++)
        for (int q = p + 1; q < DDE_DYN_N_PDU; q++)
            CHECK(!(dde_dyn_pdu[p].dslot == dde_dyn_pdu[q].dslot && dde_dyn_pdu[p].cyc_mod8 == dde_dyn_pdu[q].cyc_mod8),
                  "two PDUs of one slot in one cycle class");
    // E6 ign-on bytes = Fred's spec "CRC 1<ctr> 80 64 45 44 23 47", K = 0x04
    uint8_t f[DDE_MAX_FRAME_BYTES];
    dde_core_build_dyn_pdu(DDE_PDU_E6, 2, 7, false, f);
    static const uint8_t e6[8] = {0, 0x17, 0x80, 0x64, 0x45, 0x44, 0x23, 0x47};
    CHECK(memcmp(f + 6, e6 + 1, 7) == 0 && f[5] == (uint8_t)(dde_core_crc8(e6 + 1, 7) ^ 0x04), "E6 spec bytes");
    CHECK(dde_core_frame_crc(f, 13) == ((uint32_t)f[13] << 16 | (uint32_t)f[14] << 8 | f[15]), "E6 frame crc");
    // tracker: no anchor -> unsure; after 0x5C-less cycle with only statics: X - 0x5C
    dde_dyn_track_t t;
    unsigned m;
    dde_dyn_track_start(&t, 2, true, 1);
    dde_dyn_track_header(&t, 0x3D, 16, true, 2, 0);
    CHECK(dde_dyn_predict(&t, 0xE6, &m) == DDE_DYN_OK && m == 0xE6 - 0x5C, "empty dynamic segment");
    dde_dyn_track_header(&t, 0x80, 6, true, 3, DDE_DYN_BASE_Q8 + 50LL * DDE_DYN_MS_Q8);
    CHECK(dde_dyn_predict(&t, 0xE6, &m) == DDE_DYN_OK && m == 50 + 5 + (0xE6 - 0x80 - 1), "after 0x80 L6");
    dde_dyn_track_header(&t, 0x90, 20, true, 4, DDE_DYN_BASE_Q8 + 80LL * DDE_DYN_MS_Q8);
    CHECK(dde_dyn_predict(&t, 0xE6, &m) == DDE_SKIP_LEN, "unknown length -> skip");
    dde_dyn_track_start(&t, 4, true, 1);
    dde_dyn_track_header(&t, 0x80, 6, true, 2, DDE_DYN_BASE_Q8 + 30LL * DDE_DYN_MS_Q8);
    CHECK(dde_dyn_predict(&t, 0xE6, &m) == DDE_SKIP_GRID, "dynamic frame before its own minislot -> skip");
    dde_dyn_track_start(&t, 5, true, 1);
    dde_dyn_track_header(&t, 0x80, 6, true, 2, DDE_DYN_BASE_Q8 + 50LL * DDE_DYN_MS_Q8 + 200 * 256);
    CHECK(dde_dyn_predict(&t, 0xE6, &m) == DDE_SKIP_GRID, "2 us off the grid -> skip");
    dde_dyn_track_start(&t, 6, true, 1);
    dde_dyn_track_header(&t, 0x80, 6, false, 2, DDE_DYN_BASE_Q8 + 50LL * DDE_DYN_MS_Q8);
    CHECK(dde_dyn_predict(&t, 0xE6, &m) == DDE_SKIP_NOANCHOR, "dynamic header without mark -> skip");
    dde_dyn_track_start(&t, 3, false, 1);
    CHECK(dde_dyn_predict(&t, 0xE6, &m) != DDE_DYN_OK, "no anchor mark -> skip");
}

// Cross-check against Ian's DDE flash decode (dde_flash_e2e.md, dde_flash_pdu_layouts.md,
// dde_flash_eray_config.md): E2E as the DDE computes it, base/rep cycle filters, periods,
// 0xFF fill of unused bits, FB L7 12-bit field at its substitute 0xFFF, no CRC on E8/FB.
static uint8_t flash_crc_tab[256];
static uint8_t flash_e2e(uint16_t data_id, const uint8_t *pdu, unsigned len)
{
    // FUN_80125AC8: crc = T[id lo]; crc = T[crc ^ id hi]; then PDU bytes 1..len-1; no xorout
    uint8_t c = flash_crc_tab[data_id & 0xFFu];
    c = flash_crc_tab[c ^ (data_id >> 8)];
    for (unsigned i = 1; i < len; i++) c = flash_crc_tab[c ^ pdu[i]];
    return c;
}

static void test_dyn_flash(void)
{
    for (unsigned i = 0; i < 256; i++) {
        uint8_t c = (uint8_t)i;
        for (int b = 0; b < 8; b++) c = (uint8_t)((c & 0x80u) ? ((unsigned)c << 1) ^ 0x1Du : (unsigned)c << 1);
        flash_crc_tab[i] = c;
    }
    // Ian's three verified 0xE6 capture frames (data ID 0x37)
    static const uint8_t cap[3][8] = {{0x02, 0x15, 0x80, 0x64, 0x45, 0x44, 0x23, 0x47},
                                      {0xc8, 0x11, 0x00, 0x34, 0x44, 0x30, 0x00, 0x47},
                                      {0xd0, 0x1d, 0x80, 0x64, 0x45, 0x44, 0x23, 0x47}};
    for (int i = 0; i < 3; i++) CHECK(flash_e2e(0x37, cap[i], 8) == cap[i][0], "flash E2E on capture E6 #%d", i);
    // K form used by the firmware = flash CRC of (ID, 7 zero bytes)
    static const uint8_t z[8] = {0};
    CHECK(flash_e2e(0x37, z, 8) == dde_dyn_pdu[DDE_PDU_E6].k, "E6 K 0x%02X = flash ID 0x37 (0x%02X)",
          dde_dyn_pdu[DDE_PDU_E6].k, flash_e2e(0x37, z, 8));
    // Every E6 we can send (pre-ign and ign-on, all counters): B0 = DDE's CRC, counter 0..14
    uint8_t f[DDE_MAX_FRAME_BYTES];
    for (int pre = 0; pre < 2; pre++)
        for (unsigned ctr = 0; ctr < 15; ctr++) {
            dde_core_build_dyn_pdu(DDE_PDU_E6, 2, ctr, pre, f);
            CHECK(flash_e2e(0x37, f + 5, 8) == f[5] && (f[6] & 0x0F) == ctr, "E6 flash E2E pre=%d ctr=%u", pre, ctr);
        }
    // Static groups (unchanged statics): CRC byte = flash E2E with the flash data ID and CRC length
    static const struct { uint8_t tgt, cyc, off, len; uint16_t id; } sg[] = {
        {0, 1, 0, 4, 0x0D}, {1, 1, 0, 8, 0x09}, {1, 1, 8, 8, 0x43}, {1, 3, 0, 7, 0x5B}, {1, 3, 8, 6, 0x5A},
        {2, 3, 0, 8, 0x20}, {2, 3, 8, 8, 0x58}, {3, 3, 0, 8, 0x59}, {3, 3, 8, 4, 0xA3}};
    unsigned ngroups = 0;
    for (unsigned g = 0; g < sizeof sg / sizeof sg[0]; g++)
        for (unsigned cyc = sg[g].cyc; cyc < 64; cyc += 4)
            for (unsigned ctr = 0; ctr < 15; ctr += 7) {
                if (!(dde_core_emu_indicators(sg[g].tgt, cyc, false) & 0x04u)) continue;   // null frame
                dde_core_build_static(sg[g].tgt, cyc, ctr, false, f);
                const uint8_t *pl = f + 5 + sg[g].off;
                CHECK(flash_e2e(sg[g].id, pl, sg[g].len) == pl[0], "static tgt %u c%u +%u flash E2E ID 0x%02X",
                      sg[g].tgt, cyc, sg[g].off, sg[g].id);
                ngroups++;
            }
    CHECK(ngroups > 100, "static E2E groups checked (%u)", ngroups);
    // base/rep (flash MB table) and period; every TX cycle of every PDU must satisfy cycle % rep == base.
    // The tracker only retries a skipped PDU 8 cycles later, i.e. in the same cycle class.
    static const struct { uint8_t p, base, rep; uint16_t period; } br[] = {
        {DDE_PDU_E6, 0, 2, 200}, {DDE_PDU_FB5, 1, 4, 1000}, {DDE_PDU_FB7, 2, 4, 40},
        {DDE_PDU_FB9, 3, 4, 1000}, {DDE_PDU_E89, 1, 2, 200}, {DDE_PDU_E85, 0, 2, 1000}};
    for (unsigned i = 0; i < sizeof br / sizeof br[0]; i++) {
        const dde_dyn_pdu_t *d = &dde_dyn_pdu[br[i].p];
        CHECK(d->cyc_mod8 % br[i].rep == br[i].base, "%s cycle%%8=%u not base %u / rep %u", d->name, d->cyc_mod8,
              br[i].base, br[i].rep);
        CHECK(d->period == br[i].period && d->period % 8 == 0, "%s period %u (flash %u)", d->name, d->period, br[i].period);
        for (unsigned cyc = 0; cyc < 64; cyc++)
            if (dde_core_dyn_pdu_for(d->dslot, cyc) == (int)br[i].p)
                CHECK(cyc % br[i].rep == br[i].base, "%s due in cycle %u", d->name, cyc);
        // E2E only on 0xE6
        CHECK((d->kind == DDE_DYN_KIND_E2E) == (br[i].p == DDE_PDU_E6), "%s E2E kind", d->name);
    }
    // Only 0xE6 / 0xE8 / 0xFB, one 0xE6 PDU (base0 = DT_PT_2; base1 ID 0x8C and 0x88 are never on the bus)
    int ne6 = 0;
    for (int p = 0; p < DDE_DYN_N_PDU; p++) {
        CHECK(dde_dyn_pdu[p].id == 0xE6 || dde_dyn_pdu[p].id == 0xE8 || dde_dyn_pdu[p].id == 0xFB, "unexpected id");
        ne6 += dde_dyn_pdu[p].id == 0xE6;
    }
    CHECK(ne6 == 1, "one 0xE6 PDU");
    // Unused bits = 1 (flash fills PDUs with 0xFF, LSB-first) in every frame we can send
    static const struct { uint8_t p; uint8_t must1[8]; } fill[] = {
        {DDE_PDU_FB5, {0x00, 0x00, 0xF0, 0xFF}},
        {DDE_PDU_FB7, {0xF0, 0xFF, 0x00, 0x00, 0x00, 0xF0}},     // B0.4-B1 = 0xFFF (substitute) + B5 fill
        {DDE_PDU_FB9, {0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF}},
        {DDE_PDU_E89, {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xF0}},
        {DDE_PDU_E85, {0x00, 0x00, 0x00, 0xF0}}};
    for (unsigned i = 0; i < sizeof fill / sizeof fill[0]; i++) {
        const dde_dyn_pdu_t *d = &dde_dyn_pdu[fill[i].p];
        for (int pre = 0; pre < 2; pre++)
            for (unsigned ctr = 0; ctr < 15; ctr++) {
                dde_core_build_dyn_pdu(fill[i].p, d->cyc_mod8, ctr, pre, f);
                for (unsigned b = 0; b < d->plen; b++)
                    CHECK((f[5 + b] & fill[i].must1[b]) == fill[i].must1[b], "%s pre=%d B%u=0x%02X fill", d->name, pre,
                          b, f[5 + b]);
                if (d->kind == DDE_DYN_KIND_CTR) CHECK((f[5] & 0x0F) == ctr, "%s counter at B0.0", d->name);
            }
    }
    // FB L5 B0.4-B1 is a 12-bit value (0x604), not "0x40 | ctr": the high nibble must never follow the counter
    dde_core_build_dyn_pdu(DDE_PDU_FB5, 5, 14, false, f);
    CHECK(((f[5] >> 4) | (f[6] << 4)) == 0x604, "FB L5 12-bit value 0x604");
}

int main(int argc, char **argv)
{
    dde_core_init();
    // args: vector files; "strict:FILE" = every target frame must be rebuilt bit-exactly
    // from the tables (true for the ign-on/engine-off window the tables came from).
    // "dyn:FILE" = DDE dynamic frames (builder test), "dyntrack:FILE" = whole bus (tracker test)
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "dyn:", 4) == 0) { test_dyn_build(argv[i] + 4); continue; }
        if (strncmp(argv[i], "dyntrack:", 9) == 0) { test_dyn_tracker(argv[i] + 9); continue; }
        bool strict = strncmp(argv[i], "strict:", 7) == 0;
        test_vectors(strict ? argv[i] + 7 : argv[i], strict);
    }
    test_dyn_basics();
    test_dyn_flash();
    test_pack_args();
    test_schedule();
    test_lock();
    if (fails) { printf("test_dde_emu_core: %d FAILURES\n", fails); return 1; }
    printf("test_dde_emu_core: all OK\n");
    return 0;
}
