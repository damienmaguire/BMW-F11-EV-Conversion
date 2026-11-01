// Host tests for the v1e PT-CAN emulation core (src/ptcan_emu_core.c).
//   test_ptcan_emu unit
//   test_ptcan_emu e2e  VECTORS        (lines: "id_hex data_hex", real captured frames)
//   test_ptcan_emu replay CANFILE MASK_HEX OUT [end_extra_s]
//        CANFILE lines "t_us id_hex data_hex" (PT-CAN RX as the emulator would see it).
//        Writes our TX ("T t_us id data") and gate state changes ("S t_us state") to OUT.
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ptcan_emu_core.h"
#include "ptcan_emu.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static int hexbytes(const char *s, uint8_t *out, int max)
{
    int n = 0;
    while (s[0] && s[1] && n < max) {
        unsigned v;
        if (sscanf(s, "%2x", &v) != 1) break;
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}

// ---- E2E vectors -----------------------------------------------------------------
static int run_e2e(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { printf("cannot open %s\n", path); return 1; }
    char line[256];
    int n = 0, bad = 0, per[PTE_N_FRAMES] = {0};
    while (fgets(line, sizeof line, f)) {
        unsigned id;
        char hex[64];
        if (sscanf(line, "%x %63s", &id, hex) != 2) continue;
        uint8_t d[8];
        int len = hexbytes(hex, d, 8);
        int i = pte_index_of((uint16_t)id);
        if (i < 0 || pte_defs[i].data_id == 0xFFFF) continue;
        CHECK(len == pte_defs[i].dlc, "0x%03x len %d", id, len);
        uint8_t c = pte_e2e_crc(pte_defs[i].data_id, d, (uint8_t)len);
        if (c != d[0]) { if (bad < 5) printf("  E2E mismatch 0x%03x %s calc %02x\n", id, hex, c); bad++; }
        if ((d[1] >> 4) != pte_defs[i].ctr_hi) { if (bad < 5) printf("  b1 high nibble 0x%03x %s\n", id, hex); bad++; }
        if ((d[1] & 0xF) == 0xF) { bad++; }
        n++; per[i]++;
    }
    fclose(f);
    printf("E2E vectors %s: %d frames, %d mismatches (", path, n, bad);
    for (int i = 0; i < PTE_N_FRAMES; i++)
        if (pte_defs[i].data_id != 0xFFFF) printf(" 0x%03x:%d", pte_defs[i].id, per[i]);
    printf(" )\n");
    for (int i = 0; i < PTE_N_FRAMES; i++)
        if (pte_defs[i].data_id != 0xFFFF) CHECK(per[i] > 0, "no vectors for 0x%03x", pte_defs[i].id);
    return (bad || fails) ? 1 : 0;
}

// ---- replay -------------------------------------------------------------------------
typedef struct { uint64_t t; uint16_t id; uint8_t len; uint8_t d[8]; } rx_t;

static rx_t *load(const char *path, size_t *n_out)
{
    FILE *f = fopen(path, "r");
    if (!f) return NULL;
    size_t cap = 1 << 16, n = 0;
    rx_t *v = malloc(cap * sizeof *v);
    char line[256];
    while (fgets(line, sizeof line, f)) {
        unsigned long long t;
        unsigned id;
        char hex[64] = "";
        if (sscanf(line, "%llu %x %63s", &t, &id, hex) < 2) continue;
        if (n == cap) v = realloc(v, (cap *= 2) * sizeof *v);
        v[n].t = t;
        v[n].id = (uint16_t)id;
        v[n].len = (uint8_t)hexbytes(hex, v[n].d, 8);
        n++;
    }
    fclose(f);
    *n_out = n;
    return v;
}

typedef struct {
    FILE *out;
    pte_state_t last_state;
    uint64_t last_rx;
    uint32_t tx, tx_while_silent, tx_bad_id;
} sim_t;

// Firmware loop model for the replay (v1f): at most g_burst frames per poll (v1e: 4,
// v1f: 16 = the TX FIFO depth), and optionally a main-loop stall of g_stall_us every
// g_stall_every_us (v1e: the blocking 5 s UART status print, ~120-160 ms).
static int g_burst = 32;
static uint64_t g_stall_every_us, g_stall_us;

static void step(pte_t *p, sim_t *s, uint64_t now)
{
    pte_state_t st = pte_state(p, now);
    if (st != s->last_state) {
        if (s->out) fprintf(s->out, "S %" PRIu64 " %s\n", now, pte_state_name(st));
        s->last_state = st;
    }
    pte_tx_t tx;
    int guard = 0;
    while (guard++ < g_burst && pte_next(p, now, &tx)) {
        if (pte_id_forbidden(tx.id)) s->tx_bad_id++;
        if (!p->have_rx || now - p->last_rx_us > PTE_SILENCE_US) s->tx_while_silent++;
        pte_commit(p, &tx, now);
        s->tx++;
        if (s->out) {
            fprintf(s->out, "T %" PRIu64 " %03x ", now, tx.id);
            for (int i = 0; i < tx.dlc; i++) fprintf(s->out, "%02x", tx.data[i]);
            fprintf(s->out, "\n");
        }
    }
}

// Feed frames, polling the core every 1 ms (and at every RX), up to t_end.
static void simulate(pte_t *p, sim_t *s, const rx_t *v, size_t n, uint64_t t_from, uint64_t t_end)
{
    uint64_t now = t_from;
    size_t k = 0;
    while (k < n && v[k].t < t_from) k++;
    while (now <= t_end) {
        while (k < n && v[k].t <= now) {
            pte_on_rx(p, v[k].id, v[k].d, v[k].len, v[k].t);
            k++;
        }
        bool stalled = g_stall_every_us && ((now - t_from) % g_stall_every_us) < g_stall_us;
        if (!stalled) step(p, s, now);
        now += 1000;
    }
}

static int run_replay(const char *path, uint32_t mask, const char *outp, double extra_s)
{
    size_t n = 0;
    rx_t *v = load(path, &n);
    if (!v || !n) { printf("cannot load %s\n", path); return 1; }
    FILE *out = fopen(outp, "w");
    pte_t p;
    pte_init(&p);
    pte_set_mask(&p, mask);
    sim_t s = {.out = out, .last_state = (pte_state_t)-1};
    uint64_t t0 = v[0].t > 2000000 ? v[0].t - 2000000 : 0;   // 2 s of sleeping bus first
    simulate(&p, &s, v, n, t0, v[n - 1].t + (uint64_t)(extra_s * 1e6));
#ifndef PTE_V1E_COMPARE
    fprintf(out, "E fault=%d fault_id=%03x wakes=%u kl15_on=%u kl15_off=%u tx=%u silent_tx=%u bad_id=%u "
            "late=%u late_max_us=%u resyncs=%u\n",
            p.fault, p.fault_id, p.wakes, p.kl15_on_edges, p.kl15_off_edges, s.tx, s.tx_while_silent, s.tx_bad_id,
            p.late_sends, p.late_max_us, p.resyncs);
#else   // replay only, built against the v1e core (git) to reproduce the run 6 loss
    fprintf(out, "E fault=%d fault_id=%03x wakes=%u kl15_on=%u kl15_off=%u tx=%u silent_tx=%u bad_id=%u\n",
            p.fault, p.fault_id, p.wakes, p.kl15_on_edges, p.kl15_off_edges, s.tx, s.tx_while_silent, s.tx_bad_id);
#endif
    fclose(out);
    free(v);
    CHECK(s.tx_while_silent == 0, "TX while PT-CAN silent: %u", s.tx_while_silent);
    CHECK(s.tx_bad_id == 0, "forbidden ID sent");
    printf("replay %s mask %05x: %u frames sent, fault=%s, wakes %u, KL15 edges on %u off %u\n", path, mask, s.tx,
           pte_fault_name(p.fault), p.wakes, p.kl15_on_edges, p.kl15_off_edges);
    return fails ? 1 : 0;
}

// ---- unit tests ----------------------------------------------------------------------
static void feed_bus(pte_t *p, sim_t *s, uint64_t from, uint64_t to, uint8_t stkl, uint16_t nm_id)
{
    // synthetic PT-CAN: 0x12F every 100 ms, a filler frame every 10 ms, foreign NM every 640 ms
    for (uint64_t t = from; t < to; t += 1000) {
        if ((t - from) % 10000 == 0) { uint8_t d[8] = {0}; pte_on_rx(p, 0x1A1, d, 8, t); }
        if ((t - from) % 100000 == 0) { uint8_t d[8] = {0, 0, stkl, 0, 0, 0, 0, 0}; pte_on_rx(p, 0x12F, d, 8, t); }
        if (nm_id && (t - from) % 640000 == 0) { uint8_t d[8] = {0}; pte_on_rx(p, nm_id, d, 8, t); }
        step(p, s, t);
    }
}

static void unit(void)
{
    // never-send list
    const uint16_t bad[] = {0x137, 0x080, 0x085, 0x351, 0x380, 0x580, 0x5C0, 0x600, 0x618, 0x6F1, 0x6FF, 0x27A, 0x598};
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++) CHECK(pte_id_forbidden(bad[i]), "0x%03x allowed", bad[i]);
    for (int i = 0; i < PTE_N_FRAMES; i++) CHECK(!pte_id_forbidden(pte_defs[i].id), "table ID 0x%03x forbidden", pte_defs[i].id);
    CHECK(pte_index_of(0x598) < 0 && pte_index_of(0x351) < 0 && pte_index_of(0x137) < 0, "excluded ID in table");

    // KL15 rule
    CHECK(pte_kl15_from_stkl(0xA, false) == 1 && pte_kl15_from_stkl(9, true) == 1 && pte_kl15_from_stkl(9, false) == 0 &&
          pte_kl15_from_stkl(8, true) == 0 && pte_kl15_from_stkl(6, true) == 0 && pte_kl15_from_stkl(0xE, true) == -1,
          "KL15 from ST_KL");

    // disabled: nothing, whatever the bus does
    pte_t p;
    sim_t s = {0};
    pte_init(&p);
    feed_bus(&p, &s, 1000000, 5000000, 0xA, 0x510);
    CHECK(s.tx == 0, "TX while disabled: %u", s.tx);

    // enabled, bus asleep: nothing
    pte_init(&p);
    pte_set_mask(&p, PTE_MASK_ALL);
    memset(&s, 0, sizeof s);
    for (uint64_t t = 0; t < 10000000; t += 1000) step(&p, &s, t);
    CHECK(s.tx == 0 && pte_state(&p, 10000000) == PTE_ST_WAIT_BUS, "TX on a sleeping bus");

    // wake: first frame within 0.5 s, step1 mask = 0x3F9 only, counter starts at 1
    pte_init(&p);
    pte_set_mask(&p, 1u << PTE_F_3F9);
    memset(&s, 0, sizeof s);
    uint64_t W = 20000000;
    pte_tx_t tx;
    uint64_t first = 0;
    for (uint64_t t = W; t < W + 3000000; t += 1000) {
        if (t % 10000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x1A1, d, 8, t); }
        while (pte_next(&p, t, &tx)) {
            CHECK(tx.id == 0x3F9, "step1 sent 0x%03x", tx.id);
            if (!first) { first = t; CHECK((tx.data[1] & 0xF) == 1, "first counter %u", tx.data[1] & 0xF); }
            CHECK(tx.data[0] == pte_e2e_crc(0x37, tx.data, 8), "3F9 CRC");
            pte_commit(&p, &tx, t);
        }
    }
    CHECK(first && first - W <= 500000, "first TX %.3f s after wake", (double)(first - W) / 1e6);
    // 3F9 P-lock bits 28-29 = 2 once settled
    CHECK(((p.f[PTE_F_3F9].last[3] >> 4) & 3) == 2, "3F9 bits 28-29 = %u", (p.f[PTE_F_3F9].last[3] >> 4) & 3);

    // alive counter 1..14, 0, 1 ...
    pte_init(&p);
    pte_set_mask(&p, 1u << PTE_F_0A5);
    int seq[40], ns = 0;
    for (uint64_t t = W; t < W + 2000000 && ns < 40; t += 1000) {
        if (t % 10000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x1A1, d, 8, t); }
        while (pte_next(&p, t, &tx)) { if (ns < 40) seq[ns++] = tx.data[1] & 0xF; pte_commit(&p, &tx, t); }
    }
    bool ok = ns == 40;
    for (int i = 0; i < ns; i++) ok = ok && seq[i] == (i + 1) % 15;
    CHECK(ok, "alive counter sequence");

    // silence gate: 300 ms gap -> TX stops within 200 ms, resumes on traffic (same wake)
    pte_init(&p);
    pte_set_mask(&p, PTE_MASK_ALL);
    memset(&s, 0, sizeof s);
    feed_bus(&p, &s, W, W + 8000000, 0xA, 0x510);
    uint32_t before = s.tx;
    uint64_t last_tx = 0;
    for (uint64_t t = W + 8000000; t < W + 8300000; t += 1000) {
        while (pte_next(&p, t, &tx)) { pte_commit(&p, &tx, t); last_tx = t; }
    }
    CHECK(last_tx <= W + 8000000 - 1000 + PTE_SILENCE_US, "TX %.3f s into a silence", (double)(last_tx - (W + 7999000)) / 1e6);
    CHECK(pte_state(&p, W + 8300000) == PTE_ST_SILENT, "state after 300 ms gap: %s", pte_state_name(pte_state(&p, W + 8300000)));
    uint32_t wakes = p.wakes;
    feed_bus(&p, &s, W + 8300000, W + 9000000, 0xA, 0x510);
    CHECK(p.wakes == wakes && s.tx > before, "resume after short gap");
    // 3 s gap -> new wake: timelines restart, counters restart at 1
    {   // last RX at W+8.999 s: TX allowed up to +200 ms, never after
        uint32_t late = 0;
        for (uint64_t t = W + 9000000; t < W + 12000000; t += 1000)
            while (pte_next(&p, t, &tx)) { pte_commit(&p, &tx, t); if (t > W + 8999000 + PTE_SILENCE_US) late++; }
        CHECK(late == 0, "%u frames sent > 200 ms into a silence", late);
    }
    CHECK(pte_state(&p, W + 12000000) == PTE_ST_WAIT_BUS, "sleep state");
    { uint8_t d[8] = {0}; pte_on_rx(&p, 0x1A1, d, 8, W + 12000000); }
    CHECK(p.wakes == wakes + 1, "new wake after 3 s silence");
    CHECK(!pte_next(&p, W + 12000000 + 400000, &tx), "TX before W+0.435");

    // foreign sender of an enabled ID -> latched fault, TX stops; mask 0 clears
    pte_init(&p);
    pte_set_mask(&p, PTE_MASK_ALL);
    memset(&s, 0, sizeof s);
    feed_bus(&p, &s, W, W + 2000000, 0xA, 0x510);
    { uint8_t d[8] = {0}; pte_on_rx(&p, 0x3FD, d, 5, W + 2000000); }
    CHECK(pte_state(&p, W + 2000001) == PTE_ST_FAULT && p.fault == PTE_FAULT_FOREIGN && p.fault_id == 0x3FD, "foreign fault");
    CHECK(!pte_next(&p, W + 2100000, &tx), "TX after fault");
    pte_set_mask(&p, 0);
    pte_set_mask(&p, 1u << PTE_F_3F9);
    { uint8_t d[8] = {0}; pte_on_rx(&p, 0x3FD, d, 5, W + 2100000); }
    CHECK(p.fault == PTE_FAULT_NONE, "0x3FD from the real EGS is fine when 0x3FD is not emulated");

    // KL15 off: 0x518 stops OFF+29.91, 0x512 OFF+45.93, then (other NM gone) PREP_SLEEP
    pte_init(&p);
    pte_set_mask(&p, PTE_MASK_ALL);
    memset(&s, 0, sizeof s);
    feed_bus(&p, &s, W, W + 10000000, 0xA, 0x510);
    uint64_t OFF = W + 10000000;
    feed_bus(&p, &s, OFF, OFF + 50000000, 0x6, 0x510);    // others keep NM for 50 s
    uint64_t l512 = p.f[PTE_F_512].last_tx_us, l518 = p.f[PTE_F_518].last_tx_us;
    CHECK(l518 >= OFF + 29300000 && l518 < OFF + PTE_518_RELEASE_US, "0x518 last OFF+%.2f", (double)(l518 - OFF) / 1e6);
    CHECK(l512 >= OFF + 45300000 && l512 < OFF + PTE_512_RELEASE_US, "0x512 last OFF+%.2f", (double)(l512 - OFF) / 1e6);
    CHECK(pte_state(&p, OFF + 50000000) == PTE_ST_RUN, "still RUN while others hold NM");
    feed_bus(&p, &s, OFF + 50000000, OFF + 53000000, 0x6, 0);
    CHECK(pte_state(&p, OFF + 53000000 - 1000) == PTE_ST_PREP_SLEEP, "PREP_SLEEP 2 s after the last foreign NM");
    // foreign NM again -> RUN
    { uint8_t d[8] = {0}; pte_on_rx(&p, 0x510, d, 8, OFF + 53000000); }
    CHECK(pte_state(&p, OFF + 53000000) == PTE_ST_RUN, "back to RUN on a new NM frame");

    // door wake (KL15 never on): 0x512 only the 4 wake frames, 0x518 stops W+30 s, 0x592 never
    pte_init(&p);
    pte_set_mask(&p, PTE_MASK_ALL);
    memset(&s, 0, sizeof s);
    feed_bus(&p, &s, W, W + 40000000, 0x5, 0x510);
    CHECK(p.f[PTE_F_512].tx == 4, "0x512 door wake frames %u", p.f[PTE_F_512].tx);
    CHECK(p.f[PTE_F_592].tx == 0, "0x592 sent without KL15");
    CHECK(p.f[PTE_F_518].last_tx_us <= W + 30000000, "0x518 door wake stop");

    // NM hard ceiling: KL15 evidence stops (0x12F gone, bus otherwise alive) -> no NM after 60 s
    pte_init(&p);
    pte_set_mask(&p, PTE_MASK_ALL);
    memset(&s, 0, sizeof s);
    feed_bus(&p, &s, W, W + 5000000, 0xA, 0x510);
    for (uint64_t t = W + 5000000; t < W + 80000000; t += 1000) {
        if (t % 10000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x1A1, d, 8, t); }
        if (t % 640000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x510, d, 8, t); }
        step(&p, &s, t);
    }
    CHECK(p.f[PTE_F_512].last_tx_us <= W + 4900000 + PTE_NM_CEILING_US &&
          p.f[PTE_F_518].last_tx_us <= W + 4900000 + PTE_NM_CEILING_US, "NM ceiling");

    // 0x592: only ever 40 22 00 14
    CHECK(p.f[PTE_F_592].tx > 0 && memcmp(p.f[PTE_F_592].last, "\x40\x22\x00\x14\xff\xff\xff\xff", 8) == 0, "0x592 content");

#ifndef PTE_V1E_COMPARE
    // v1f: a 150 ms main-loop stall must not cost frames: the late 0x0A5 slots are all
    // sent (late, back to back) once the loop runs again, the counter stays continuous,
    // and the grid is kept (next slots on time again).
    {
        pte_init(&p);
        pte_set_mask(&p, 1u << PTE_F_0A5);
        uint32_t n0 = 0, n_stall = 0;
        uint8_t prev = 0;
        bool cont = true;
        for (uint64_t t = W; t < W + 3000000; t += 1000) {
            if (t % 10000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x1A1, d, 8, t); }
            if (t % 100000 == 0) { uint8_t d[8] = {0, 0, 0xA, 0, 0, 0, 0, 0}; pte_on_rx(&p, 0x12F, d, 8, t); }
            if (t % 640000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x510, d, 8, t); }
            if (t >= W + 1500000 && t < W + 1650000) continue;   // loop stalled 150 ms
            while (pte_next(&p, t, &tx)) {
                uint8_t c = tx.data[1] & 0xF;
                if (n0 && c != (prev >= 14 ? 0 : prev + 1)) cont = false;
                prev = c;
                if (t == W + 1650000) n_stall++;
                pte_commit(&p, &tx, t);
                n0++;
            }
        }
        // first slot W+0.459, period 20 ms: slots up to W+2.999
        uint32_t want = (uint32_t)((3000000 - 459000 - 1) / 20000 + 1);
        CHECK(n0 == want, "0x0A5 with a 150 ms stall: %u sent, %u slots", n0, want);
        CHECK(n_stall >= 7 && cont, "catch-up burst %u frames, counter continuous %d", n_stall, cont);
        CHECK(p.resyncs == 0 && p.late_sends >= 7 && p.late_max_us >= 120000, "late %u max %u resyncs %u",
              p.late_sends, p.late_max_us, p.resyncs);
        // a 400 ms stall (> PTE_CATCHUP_MAX_US) is hopeless: one late frame, then the grid
        // restarts from now (no 20-frame burst)
        pte_init(&p);
        pte_set_mask(&p, 1u << PTE_F_0A5);
        uint32_t burst = 0;
        for (uint64_t t = W; t < W + 3000000; t += 1000) {
            if (t % 10000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x1A1, d, 8, t); }
            if (t % 100000 == 0) { uint8_t d[8] = {0, 0, 0xA, 0, 0, 0, 0, 0}; pte_on_rx(&p, 0x12F, d, 8, t); }
            if (t % 640000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x510, d, 8, t); }
            if (t >= W + 1500000 && t < W + 1900000) continue;
            uint32_t k = 0;
            while (pte_next(&p, t, &tx)) { pte_commit(&p, &tx, t); k++; }
            if (k > burst) burst = k;
        }
        CHECK(burst == 1 && p.resyncs == 1, "burst after a 400 ms stall %u, resyncs %u", burst, p.resyncs);
    }

    // 0x397: e3 blips and the post-OFF walk
    {
        pte_init(&p);
        pte_set_mask(&p, 1u << PTE_F_397);
        memset(&s, 0, sizeof s);
        feed_bus(&p, &s, W, W + 10000000, 0xA, 0x510);
        uint32_t n_on = p.f[PTE_F_397].tx;
        CHECK(n_on >= 18 && n_on <= 30, "0x397 frames in 10 s: %u (real ~26)", n_on);
        uint64_t OFF2 = W + 10000000;
        feed_bus(&p, &s, OFF2, OFF2 + 300000, 0x6, 0x510);
        CHECK(p.f[PTE_F_397].last[1] == 0xdd, "0x397 OFF+0.3 s: %02x", p.f[PTE_F_397].last[1]);
        feed_bus(&p, &s, OFF2 + 300000, OFF2 + 4200000, 0x6, 0x510);
        CHECK(p.f[PTE_F_397].last[1] == 0xe2, "0x397 OFF+4.2 s: %02x", p.f[PTE_F_397].last[1]);
    }
#endif

    printf("unit tests: %s\n", fails ? "FAILED" : "ok");
}

// Known-value ptcan_stats_t blob for the C vs frts.py layout check (run.sh).
static int write_stats(const char *path)
{
    ptcan_stats_t st;
    memset(&st, 0, sizeof st);
    st.version = 1; st.state = 2; st.fault = 1; st.flags = 0xA5; st.mask = 0x7FFFF; st.fault_id = 0x3FD;
    st.st_kl = 0xA; st.n_frames = PTE_N_FRAMES; st.tec = 7; st.rec = 9; st.opmod = 6;
    st.rx_frames = 123456; st.n12f = 789; st.wakes = 3;
    st.ms_since_wake = 1000; st.ms_since_rx = 2; st.ms_since_on = -1; st.ms_since_off = 45930; st.ms_since_foreign_nm = 1950;
    st.tx_total = 99999; st.tx_fifo_full = 11; st.tx_fail = 12; st.mode_switches = 13;
    for (int i = 0; i < PTE_N_FRAMES; i++) { st.id[i] = pte_defs[i].id; st.tx[i] = 1000u + (uint32_t)i; }
    st.loop_gap_max_us = 4321; st.loop_gaps_20ms = 5; st.late_max_us = 131000; st.late_sends = 77; st.resyncs = 1;
    st.tx_logged = 36000; st.tx_log_lost = 2; st.console_dropped = 9; st.burst_max = 8;
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return 2; }
    fwrite(&st, sizeof st, 1, f);
    fclose(f);
    printf("ptcan_stats_t: %zu bytes -> %s\n", sizeof st, path);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 3 && !strcmp(argv[1], "stats")) return write_stats(argv[2]);
    if (argc >= 2 && !strcmp(argv[1], "unit")) { unit(); return fails ? 1 : 0; }
    if (argc >= 3 && !strcmp(argv[1], "e2e")) return run_e2e(argv[2]);
    if (argc >= 5 && !strcmp(argv[1], "replay")) {
        // optional: burst per poll, stall every ms, stall length ms (firmware loop model)
        if (argc >= 7) g_burst = atoi(argv[6]);
        if (argc >= 9) { g_stall_every_us = 1000ull * strtoull(argv[7], NULL, 10); g_stall_us = 1000ull * strtoull(argv[8], NULL, 10); }
        return run_replay(argv[2], (uint32_t)strtoul(argv[3], NULL, 16), argv[4], argc >= 6 ? atof(argv[5]) : 5.0);
    }
    printf("usage: %s unit | e2e FILE | replay CANFILE MASK OUT [extra_s [burst [stall_every_ms stall_ms]]]\n", argv[0]);
    return 2;
}
