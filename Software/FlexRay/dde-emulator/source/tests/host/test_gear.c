// v1h.1 gear shifting (src/dde_gear.c) host checks, against Ian's egs_gear_report_tx.md:
//  - the lever table (P/R/N/D, forward/back 1/2 notches, P button) and the refusals
//    (brake not pressed, engine not running), with the exact log texts;
//  - timing (v1h.2, Ian's run 9 comparison): act 40 ms after a notch, a 21 ms pass-through
//    notch is skipped (P -> R straight to R, no N), P released (E1 / bits 1) 20 ms after
//    the first notch, gear code on 0x39A 2..21 ms after 0x3FD, N only when held >= 40 ms,
//    P button: 0x39A 01 +21, 0x3F9 +42, 0x39A 03 ~19 ms before 0x3FD P at +900 ms,
//    pawl 340 ms after brake release or 3 s;
//  - lever timeout (> 90 ms without a valid 0x197): hold the gear, logged once;
//  - bad 0x197 (CRC / length) ignored;
//  - gear off (runtime / EMU_GEAR=0 behaviour): fixed P, bytes identical to v1h;
//  - the bytes on the bus via the real paths: PT-CAN pte_next (0x3FD/0x39A/0x3F9/0xDC,
//    every E2E CRC valid, 0x3FD extra frame on change, 0x197 never sent), FlexRay 0x28 c3
//    and 0xE6 (frame CRC + E2E valid);
//  - writes the timeline to argv[1] (CSV) when given.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "dde_emu_core.h"
#include "dde_emu_tables.h"
#include "dde_engine.h"
#include "dde_gear.h"
#include "ptcan_emu_core.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static FILE *csv;
static uint64_t T;          // us
static FILE *can_csv;   // argv[3]: a fr_ts_logger-style CAN CSV of the shift sequence, for the checker test
static void can_row(uint16_t id, const uint8_t *d, unsigned n, bool tx)
{
    if (!can_csv) return;
    fprintf(can_csv, "%llu,0x%X,0,", (unsigned long long)T, id);
    for (unsigned i = 0; i < n; i++) fprintf(can_csv, "%02x", d[i]);
    fprintf(can_csv, ",%s\n", tx ? "Tx" : "Rx");
}
static eng_t E;
static gear_t G;
static pte_t P;
static uint8_t lv_pos, lv_pb = 0xC0, lv_alive;
static bool lv_on = true, brake, lv_badcrc;
static uint8_t stkl = 0xA;
static uint64_t t_first_frame[5];   // when a 0x197 with this lever position was first sent (0 = reset)
static uint64_t t_first_pb;         // v1h.2: first 0x197 with the P button pressed (0 = reset)
static uint8_t last3fd, last39a3, last39a4, last3f9_3, last3f9_6, lastdc2, lastdc5;
static unsigned n3fd_frames, n_crc_ok, n_tx;

static uint8_t crc8_1d(const uint8_t *b, unsigned n)
{
    uint8_t c = 0;
    for (unsigned i = 0; i < n; i++) {
        c ^= b[i];
        for (int k = 0; k < 8; k++) c = (uint8_t)((c & 0x80) ? (c << 1) ^ 0x1D : c << 1);
    }
    return c;
}
static void send_lever(void)
{
    uint8_t d[4] = {0, (uint8_t)(0xF0u | lv_alive), (uint8_t)(lv_pos << 4 | 0x0F), lv_pb};
    lv_alive = (uint8_t)((lv_alive + 1u) % 15u);
    if (lv_pos < 5 && !t_first_frame[lv_pos]) t_first_frame[lv_pos] = T;
    if (lv_pb == 0xD5 && !t_first_pb) t_first_pb = T;
    uint8_t b[5] = {0x3F, 0x00, d[1], d[2], d[3]};
    d[0] = crc8_1d(b, 5);
    if (lv_badcrc) d[0] ^= 0x55;
    gear_input_lever(&G, d, 4, (uint32_t)T);
    can_row(0x197, d, 4, false);
}

// one 1 ms step: inputs, FlexRay cycle work every 5 ms, PT-CAN TX
static void step1(void)
{
    T += 1000;
    uint64_t ms = T / 1000;
    if (lv_on && ms % 20 == 0) send_lever();
    if (brake && ms % 20 == 10) gear_input_brake(&G, (uint32_t)T);
    if (ms % 20 == 10) { uint8_t d[8] = {0, 0, 0, 0, 0, 0, 0, (uint8_t)(brake ? 3 : 0)}; can_row(0x173, d, 8, false); }
    if (ms % 100 == 0) {   // ST_KL C (engine running after the start below) + brake bit
        uint8_t d[8] = {0, 0, (uint8_t)(0xF0u | stkl), 0, 0, 0, 3, 0};
        pte_on_rx(&P, 0x12F, d, 8, T);
        eng_input_stkl(&E, stkl, true, (uint32_t)T);
    }
    if (ms % 10 == 0) { uint8_t d[8] = {0}; pte_on_rx(&P, 0x1A1, d, 8, T); }
    if (ms % 640 == 0) { uint8_t d[8] = {0}; pte_on_rx(&P, 0x510, d, 8, T); }
    if (ms % 5 == 0) {
        if (ms % 1000 == 0) eng_set_mode(&E, ENG_MODE_CAS, (uint32_t)T);
        eng_step(&E, (uint32_t)T);
    }
    // v1h.2: the gear runs every 1 ms in the core 1 background (the engine per FlexRay cycle)
    gear_step(&G, (uint32_t)T, E.phase == ENG_RUN);
    gear_fill_out(&G, &E.out);
    P.eng = E.out.active ? &E.out : NULL;
    pte_tx_t tx;
    while (pte_next(&P, T, &tx)) {
        CHECK(!pte_id_forbidden(tx.id) && tx.id != 0x197, "forbidden id %X", tx.id);
        const pte_def_t *def = &pte_defs[tx.idx];
        if (def->data_id != 0xFFFF) {
            CHECK(tx.data[0] == pte_e2e_crc(def->data_id, tx.data, tx.dlc), "PT-CAN CRC %X", tx.id);
            n_crc_ok++;
        }
        if (tx.id == 0x3FD) { last3fd = tx.data[2]; n3fd_frames++; }
        if (tx.id == 0x39A) { last39a3 = tx.data[3]; last39a4 = tx.data[4]; }
        if (tx.id == 0x3F9) { last3f9_3 = tx.data[3]; last3f9_6 = tx.data[6]; }
        if (tx.id == 0x0DC) { lastdc2 = tx.data[2]; lastdc5 = tx.data[5]; }
        can_row(tx.id, tx.data, tx.dlc, true);
        pte_commit(&P, &tx, T);
        n_tx++;
    }
    if (csv && ms % 5 == 0)
        fprintf(csv, "%.3f,%u,%u,%d,%s,%u,%02X,%02X,%02X,%02X,%X,%02X,%02X\n", T / 1e6, lv_pos, lv_pb == 0xD5,
                brake, gear_name(G.target), G.stage, last3fd, last39a3, last39a4, (last3f9_3 >> 4) & 3u,
                last3f9_6 & 15u, lastdc2, lastdc5);
}
static void run_ms(unsigned n) { for (unsigned i = 0; i < n; i++) step1(); }

// v1h.2: record the last PT-CAN bytes every 1 ms (index = ms from the start of rec())
typedef struct { uint8_t f3fd, a3, a4, park, nib, dc5; } rec_t;
static rec_t R[5000];
static uint64_t rec_t0;
static void rec(unsigned n)
{
    rec_t0 = T;
    for (unsigned i = 0; i < n && i < 5000; i++) {
        step1();
        R[i] = (rec_t){last3fd, last39a3, last39a4, (uint8_t)((last3f9_3 >> 4) & 3u), (uint8_t)(last3f9_6 & 15u), lastdc5};
    }
}
// first ms index >= from where the field equals v (-1 = never), relative to a frame time t
#define FIRST(field, v, n) first_idx(offsetof(rec_t, field), (v), (n))
static int first_idx(size_t off, uint8_t v, unsigned n)
{
    for (unsigned i = 0; i < n && i < 5000; i++) if (((const uint8_t *)&R[i])[off] == v) return (int)i + 1;
    return -1;
}
// ms from frame time t (us) to rec index i (+1 = the step at rec_t0 + 1 ms)
static int rel_ms(int idx, uint64_t t) { return idx < 0 ? -9999 : (int)(((int64_t)rec_t0 + idx * 1000 - (int64_t)t) / 1000); }

static char last_text[96];
static const gear_ev_t *last_ev(void) { return &G.ev[GEAR_N_EV - 1]; }
static const char *ev_text(void) { gear_ev_text(last_ev(), last_text, sizeof last_text); return last_text; }

// push the lever to pos for hold_ms, back to centre for 300 ms
static void push(uint8_t pos, unsigned hold_ms) { lv_pos = pos; run_ms(hold_ms); lv_pos = 0; run_ms(300); }
static void pbutton(void) { lv_pb = 0xD5; run_ms(100); lv_pb = 0xC0; }
// wait (up to max_ms) until the last PT-CAN 0x3FD shows code; return the ms it took
static int wait_3fd(uint8_t code, unsigned max_ms)
{
    for (unsigned i = 0; i < max_ms; i++) { if (last3fd == code) return (int)i; step1(); }
    return -1;
}

static void check_fr(const char *what, uint8_t dc2, uint8_t dc5, uint8_t park, uint8_t nib)
{
    uint8_t f[DDE_MAX_FRAME_BYTES];
    dde_core_eng = &E.out;
    unsigned n = dde_core_build_static_ph(1, 3, 3, false, DDE_PH_ON, f);
    uint32_t crc = dde_core_frame_crc(f, n - 3u);
    CHECK(f[n - 3] == (uint8_t)(crc >> 16) && f[n - 2] == (uint8_t)(crc >> 8) && f[n - 1] == (uint8_t)crc, "%s: 0x28 c3 frame CRC", what);
    const uint8_t *pl = f + 5;
    CHECK(pl[10] == dc2 && pl[13] == dc5, "%s: FR 0x28 c3 b11/b14 %02X %02X, want %02X %02X", what, pl[10], pl[13], dc2, dc5);
    for (unsigned g = 0; g < DDE_EMU_N_E2E; g++) {
        const uint8_t *e = dde_emu_e2e[g];
        if (e[0] != 1 || e[1] != 3) continue;
        CHECK(pl[e[2]] == (uint8_t)(dde_core_crc8(&pl[e[2] + 1], e[3] - e[2]) ^ e[4]), "%s: 0x28 c3 E2E group %u", what, g);
    }
    n = dde_core_build_dyn_pdu_ph(DDE_PDU_E6, dde_dyn_pdu[DDE_PDU_E6].cyc_mod8, 5, 1u, f);
    crc = dde_core_frame_crc(f, n - 3u);
    CHECK(f[n - 1] == (uint8_t)crc && f[n - 2] == (uint8_t)(crc >> 8), "%s: E6 frame CRC", what);
    pl = f + 5;
    CHECK(pl[0] == (uint8_t)(dde_core_crc8(&pl[1], dde_dyn_pdu[DDE_PDU_E6].plen - 1u) ^ dde_dyn_pdu[DDE_PDU_E6].k), "%s: E6 E2E", what);
    CHECK(((pl[3] >> 4) & 3u) == park && (pl[6] & 15u) == nib, "%s: E6 b4 park %u b7 gear %u, want %u %u", what,
          (pl[3] >> 4) & 3u, pl[6] & 15u, park, nib);
}

// the steady bytes per gear (captured P/R/D; N = Ian's hypothesis)
static void check_steady(const char *what, uint8_t g)
{
    static const uint8_t b[4][7] = {
        {0x20, 0x03, 0xE2, 2, 3, 0x22, 0xFE}, {0x40, 0x02, 0xE0, 0, 2, 0x25, 0xF4},
        {0x60, 0x01, 0xE0, 0, 1, 0x22, 0xFE}, {0x80, 0x05, 0xE0, 0, 5, 0x24, 0xF4}};
    run_ms(1200);   // 0x39A / 0x3F9 at least once (1 s cycle)
    CHECK(G.target == g && G.stage == (g == GEAR_P ? GST_P_LOCKED : GST_STEADY), "%s: gear %s stage %u", what, gear_name(G.target), G.stage);
    CHECK(last3fd == b[g][0], "%s: 0x3FD d2 %02X want %02X", what, last3fd, b[g][0]);
    CHECK(last39a3 == b[g][1] && last39a4 == b[g][2], "%s: 0x39A %02X %02X want %02X %02X", what, last39a3, last39a4, b[g][1], b[g][2]);
    CHECK(((last3f9_3 >> 4) & 3u) == b[g][3] && (last3f9_6 & 15u) == b[g][4], "%s: 0x3F9 park %u gear %u", what,
          (last3f9_3 >> 4) & 3u, last3f9_6 & 15u);
    CHECK(lastdc2 == b[g][5] && lastdc5 == b[g][6], "%s: 0xDC %02X %02X", what, lastdc2, lastdc5);
    check_fr(what, b[g][5], b[g][6], b[g][3], b[g][4]);
}

static void setup(bool gear_on)
{
    T = 10000000ull;
    pte_init(&P);
    pte_set_mask(&P, PTE_MASK_ALL);
    eng_reset(&E);
    eng_set_mode(&E, ENG_MODE_CAS, (uint32_t)T);
    memset(&G, 0, sizeof G);
    gear_reset(&G, gear_on);
    lv_pos = 0; lv_pb = 0xC0; lv_on = true; brake = false; lv_badcrc = false;
    stkl = 0xA; run_ms(500);
    stkl = 0xD; run_ms(2000);   // fake start (ST_KL D, as test_engine) -> running
    stkl = 0xC; run_ms(2000);
}

static void test_shift_sequence(void)
{
    setup(true);
    CHECK(E.phase == ENG_RUN, "engine running before the shifts (%s)", eng_phase_name(E.phase));
    check_steady("start P", GEAR_P);
    uint16_t seq0 = G.ev_seq;

    // without the brake: refused, logged with the exact text
    push(LV_FWD2, 150);
    CHECK(G.target == GEAR_P && G.ev_seq == seq0 + 1 && last_ev()->kind == GEV_REFUSED, "P -> R without brake must be refused");
    CHECK(!strcmp(ev_text(), "GEAR: refused, brake not pressed (P -> N, lever forward 1)") ||
          !strcmp(ev_text(), "GEAR: refused, brake not pressed (P -> R, lever forward 2)"), "refusal text: %s", ev_text());
    printf("  %s\n", ev_text());
    check_steady("refused stays P", GEAR_P);

    // P -> R with the brake: forward 1 for ~30 ms, then forward 2 (real r10md10m: Up1 52.843,
    // Up2 52.873). v1h.2: P released (E1 / P-lock 1) 20 ms after the first notch, 0x3FD goes
    // straight to R 40 ms after forward 2 (no N: forward 1 was not held 40 ms), 0x39A 02
    // 2..21 ms after 0x3FD R; FR 0xE6 keeps 26 (P locked) until the gear code moves.
    brake = true; run_ms(100);
    memset(t_first_frame, 0, sizeof t_first_frame);
    while ((T / 1000) % 20 != 19) step1();    // the next 0x197 frame (every 20 ms) carries forward 1
    lv_pos = LV_FWD1;
    rec(30);
    lv_pos = LV_FWD2;
    { rec_t keep[30]; memcpy(keep, R, sizeof keep); uint64_t t0r = rec_t0; rec(300); memmove(R + 30, R, sizeof(rec_t) * 300);
      memcpy(R, keep, sizeof keep); rec_t0 = t0r; }
    uint64_t tf1 = t_first_frame[LV_FWD1], tf2 = t_first_frame[LV_FWD2];
    int t_e1 = rel_ms(FIRST(a4, 0xE1, 330), tf1), t_pk1 = rel_ms(FIRST(park, 1, 330), tf1);
    int t_n = FIRST(f3fd, 0x60, 330), t_r = rel_ms(FIRST(f3fd, 0x40, 330), tf2), t_02 = rel_ms(FIRST(a3, 0x02, 330), tf2);
    CHECK(G.target == GEAR_R, "P -> R");
    CHECK(t_e1 >= 19 && t_e1 <= 25 && t_pk1 >= 19 && t_pk1 <= 25, "P released: 0x39A E1 +%d, 0x3F9 P-lock 1 +%d ms after the first notch (want ~20)", t_e1, t_pk1);
    CHECK(t_n < 0, "P -> R: no N on the way (forward 1 held only ~30 ms)");
    CHECK(t_r >= 38 && t_r <= 45, "P -> R: R shown %d ms after the forward-2 frame (want ~40, real +39)", t_r);
    CHECK(t_02 - t_r >= 2 && t_02 - t_r <= 21, "P -> R: 0x39A 02 %d ms after 0x3FD R (real +21)", t_02 - t_r);
    CHECK(!strcmp(ev_text(), "GEAR: P -> R (brake on) lever forward 2"), "text: %s", ev_text());
    printf("  %s  (E1 +%d ms after forward 1, 0x3FD R +%d ms after forward 2, 0x39A 02 +%d)\n", ev_text(), t_e1, t_r, t_02);
    lv_pos = 0; run_ms(300);
    check_steady("R", GEAR_R);

    // R -> D: back 1 passed through in 21 ms (skipped), back 2 -> D 40 ms after it
    seq0 = G.ev_seq;
    lv_pos = LV_BACK1; run_ms(21); lv_pos = LV_BACK2;
    uint32_t t0 = (uint32_t)(T / 1000);
    int t_d = wait_3fd(0x80, 200);
    CHECK(t_d >= 38 && t_d <= 50, "R -> D 0x3FD at +%d ms after back 2 (want ~40)", t_d);
    CHECK(G.ev_seq == seq0 + 1 && G.target == GEAR_D, "R -> D: one change, no N in between (%u events)", G.ev_seq - seq0);
    printf("  %s  (+%d ms)\n", ev_text(), t_d);
    lv_pos = 0; run_ms(300);
    check_steady("D", GEAR_D);

    // D -> N (forward 1), N -> D without the brake: refused; with the brake: D
    push(LV_FWD1, 100);
    CHECK(G.target == GEAR_N, "D -> N");
    printf("  %s\n", ev_text());
    check_steady("N (hypothesis bytes)", GEAR_N);
    brake = false; run_ms(400);
    push(LV_BACK1, 100);
    CHECK(G.target == GEAR_N && last_ev()->kind == GEV_REFUSED && last_ev()->why == GREF_NO_BRAKE, "N -> D without brake refused");
    printf("  %s\n", ev_text());
    brake = true; run_ms(100);
    push(LV_BACK1, 100);
    CHECK(G.target == GEAR_D, "N -> D with brake");
    // D -> R (forward 2) is not gated (only P and N need the brake)
    run_ms(300);
    push(LV_FWD2, 100);
    CHECK(G.target == GEAR_R, "D -> R");
    push(LV_BACK1, 100);
    CHECK(G.target == GEAR_N, "R -> N (back 1)");
    push(LV_FWD1, 100);
    CHECK(G.target == GEAR_R, "N -> R (forward 1)");
    push(LV_BACK2, 100);
    CHECK(G.target == GEAR_D, "R -> D (back 2)");
    run_ms(500);

    // D -> P (P button). v1h.2 (Ian, real R->P / D->P): from the first pressed frame 0x39A 01
    // +21 ms, 0x3F9 gear 1 +42 ms, 0x39A 03 ~19 ms BEFORE 0x3FD P, 0x3FD P +900 ms;
    // pawl 340 ms after the brake is released
    t_first_pb = 0;
    while ((T / 1000) % 20 != 19) step1();
    lv_pb = 0xD5; rec(60); lv_pb = 0xC0;
    { rec_t keep[60]; memcpy(keep, R, sizeof keep); uint64_t t0r = rec_t0; rec(1200); memmove(R + 60, R, sizeof(rec_t) * 1200);
      memcpy(R, keep, sizeof keep); rec_t0 = t0r; }
    int t39 = rel_ms(FIRST(a3, 0x01, 1260), t_first_pb), tnib = rel_ms(FIRST(nib, 0x01, 1260), t_first_pb);
    int tp = rel_ms(FIRST(f3fd, 0x20, 1260), t_first_pb), t03 = rel_ms(FIRST(a3, 0x03, 1260), t_first_pb);
    CHECK(G.target == GEAR_P && G.stage == GST_P_UNLOCKED, "P button -> P, pawl not yet (brake held), stage %u", G.stage);
    CHECK(t39 >= 20 && t39 <= 25, "0x39A gear 01 at +%d ms (real +21)", t39);
    CHECK(tnib - t39 >= 18 && tnib - t39 <= 25, "0x3F9 gear 1 %d ms after 0x39A 01 (real ~21)", tnib - t39);
    CHECK(tp >= 898 && tp <= 905, "P shown at +%d ms (real +900)", tp);
    CHECK(tp - t03 >= 15 && tp - t03 <= 23, "0x39A 03 %d ms BEFORE 0x3FD P (real 19)", tp - t03);
    CHECK(last39a4 == 0xE0 && ((last3f9_3 >> 4) & 3u) == 0, "P shown, pawl not confirmed: E0 / bits 0");
    printf("  P button: 0x39A 01 +%d ms, 0x3F9 1 +%d, 0x39A 03 +%d, 0x3FD P +%d ms\n", t39, tnib, t03, tp);
    brake = false;
    { uint64_t tb = T; unsigned tl = 0;
      for (unsigned i = 0; i < 600 && !tl; i++) { step1(); if (G.stage == GST_P_LOCKED) tl = (unsigned)((T - tb) / 1000); }
      CHECK(tl >= 320 && tl <= 345, "pawl %u ms after the brake release (want +340, last pressed 0x173 up to 20 ms earlier)", tl);
      printf("  pawl confirmed %u ms after brake release\n", tl); }
    CHECK(G.stage == GST_P_LOCKED && last_ev()->kind == GEV_LOCKED, "pawl confirmed after brake release");
    run_ms(100);
    check_steady("back in P", GEAR_P);

    // P button again with the brake held: pawl 3 s after the act at the latest
    brake = true; run_ms(300);
    // v1h.2 P -> D (Fred: v1h.1 showed D on 0x3FD 65 ms before 0x39A/0x3F9; Ian: 0x39A 05
    // 2..21 ms after 0x3FD D); FR 0xE6 b4 never shows "16" (stays 26 until the code moves)
    memset(t_first_frame, 0, sizeof t_first_frame);
    while ((T / 1000) % 20 != 19) step1();
    lv_pos = LV_BACK1; rec(100); lv_pos = 0;
    {
        int td = rel_ms(FIRST(f3fd, 0x80, 100), t_first_frame[LV_BACK1]);
        int t05 = rel_ms(FIRST(a3, 0x05, 100), t_first_frame[LV_BACK1]);
        int tn5 = rel_ms(FIRST(nib, 0x05, 100), t_first_frame[LV_BACK1]);
        CHECK(G.target == GEAR_D, "P -> D (back 1)");
        CHECK(td >= 38 && td <= 45, "P -> D: 0x3FD D +%d ms after back 1", td);
        CHECK(t05 - td >= 2 && t05 - td <= 21 && tn5 - td >= 2 && tn5 - td <= 21,
              "P -> D: 0x39A 05 %d ms / 0x3F9 5 %d ms after 0x3FD D (want 2..21)", t05 - td, tn5 - td);
        CHECK(E.out.g_fr_park == 0, "in D: FR 0xE6 P lock bits 0");
        printf("  P -> D: 0x3FD D +%d ms, 0x39A 05 +%d ms after it\n", td, t05 - td);
    }
    run_ms(300);
    t0 = (uint32_t)(T / 1000);
    pbutton();
    unsigned tl = 0;
    for (unsigned i = 0; i < 4000 && !tl; i++) { step1(); if (G.stage == GST_P_LOCKED) tl = (unsigned)(T / 1000) - t0; }
    CHECK(tl >= 3000 && tl <= 3100, "pawl at +%u ms with the brake held (want 3 s)", tl);
    CHECK(n3fd_frames > 0, "0x3FD sent");

    // v1h.2: P -> N with forward 1 HELD (> 40 ms): N shown 40 ms after it, 0x39A keeps 03/E1
    // for 100 ms (hypothesis: the real N transit kept 03/E1), then N -> R with forward 2
    brake = true; run_ms(500);
    CHECK(G.stage == GST_P_LOCKED, "back in P before P -> N");
    memset(t_first_frame, 0, sizeof t_first_frame);
    while ((T / 1000) % 20 != 19) step1();
    lv_pos = LV_FWD1; rec(200);
    {
        int tn = rel_ms(FIRST(f3fd, 0x60, 200), t_first_frame[LV_FWD1]);
        int t01 = rel_ms(FIRST(a3, 0x01, 200), t_first_frame[LV_FWD1]);
        CHECK(G.target == GEAR_N && tn >= 38 && tn <= 45, "P -> N (forward 1 held): N at +%d ms", tn);
        CHECK(t01 - tn >= 95 && t01 - tn <= 110, "N: 0x39A gear 01 %d ms after 0x3FD N (100 ms hold of 03/E1)", t01 - tn);
        printf("  P -> N held: 0x3FD N +%d ms, 0x39A 01 +%d ms after it\n", tn, t01 - tn);
    }
    lv_pos = 0; run_ms(300);
    check_steady("N held", GEAR_N);
}

static void test_refusals_timeout_bad(void)
{
    setup(true);
    // engine not running: refused
    E.phase = ENG_OFF; eng_set_mode(&E, ENG_MODE_OFF, (uint32_t)T); brake = true;
    run_ms(200);
    push(LV_BACK1, 100);
    CHECK(G.target == GEAR_P && last_ev()->kind == GEV_REFUSED && last_ev()->why == GREF_NOT_RUNNING,
          "not running: refused (%s)", ev_text());
    printf("  %s\n", ev_text());

    setup(true);
    brake = true; run_ms(100);
    push(LV_BACK1, 100);
    CHECK(G.target == GEAR_D, "P -> D");
    // lever lost: hold D, logged once; back: logged
    uint16_t s0 = G.ev_seq;
    lv_on = false; run_ms(300);
    CHECK(G.target == GEAR_D && G.ev_seq == s0 + 1 && last_ev()->kind == GEV_LEVER_LOST, "lever lost: hold D, one event");
    printf("  %s\n", ev_text());
    lv_on = true; run_ms(100);
    CHECK(last_ev()->kind == GEV_LEVER_OK, "lever back logged");
    // bad CRC frames: ignored (counted), no shift
    uint32_t bad0 = G.lever_bad;
    lv_badcrc = true; lv_pos = LV_FWD1; run_ms(200); lv_pos = 0; run_ms(100); lv_badcrc = false;
    CHECK(G.target == GEAR_D && G.lever_bad > bad0, "bad CRC 0x197 ignored (%u bad)", (unsigned)(G.lever_bad - bad0));
    { uint8_t d[3] = {0}; uint32_t b = G.lever_bad; gear_input_lever(&G, d, 3, (uint32_t)T); CHECK(G.lever_bad == b + 1, "short 0x197 counted"); }
    // engine stops in D: auto P
    eng_set_mode(&E, ENG_MODE_OFF, (uint32_t)T);
    run_ms(5000);
    CHECK(G.target == GEAR_P, "engine off in D -> P (%s)", gear_name(G.target));
}

static void test_off_fixed_p(void)
{
    // gear off: the lever does nothing, and every byte is the v1h fixed-P content
    setup(false);
    brake = true;
    push(LV_FWD2, 150); push(LV_BACK1, 150);
    CHECK(G.target == GEAR_P && G.changes == 0 && E.out.g_set == 0, "gear off: stays P, nothing patched");
    uint8_t a[DDE_MAX_FRAME_BYTES], b[DDE_MAX_FRAME_BYTES];
    eng_out_t ref = E.out;   // same engine bytes, gear fields cleared = v1h
    ref.g_set = 0; ref.g_3fd = ref.g_39a_d3 = ref.g_39a_d4 = ref.g_park = ref.g_nib = ref.g_dc_b2 = ref.g_dc_b5 = 0;
    ref.g_fr_park = 0;
    for (unsigned tg = 0; tg < 4; tg++) for (unsigned c = 0; c < 64; c++) {
        dde_core_eng = &ref; unsigned n1 = dde_core_build_static_ph(tg, c, 3, false, DDE_PH_ON, a);
        dde_core_eng = &E.out; unsigned n2 = dde_core_build_static_ph(tg, c, 3, false, DDE_PH_ON, b);
        CHECK(n1 == n2 && !memcmp(a, b, n1), "off: static %u/%u differs", tg, c);
    }
    for (unsigned p = 0; p < DDE_DYN_N_PDU; p++) {
        dde_core_eng = &ref; unsigned n1 = dde_core_build_dyn_pdu_ph(p, 2, 3, 1, a);
        dde_core_eng = &E.out; unsigned n2 = dde_core_build_dyn_pdu_ph(p, 2, 3, 1, b);
        CHECK(n1 == n2 && !memcmp(a, b, n1), "off: dyn %u differs", p);
    }
    for (unsigned id = 0; id < 0x800; id++) {
        uint8_t d1[8] = {1, 2, 0x20, 0x26, 5, 6, 0x13, 8}, d2[8], c1 = 1, c2 = 1;
        memcpy(d2, d1, 8);
        eng_patch_can(&ref, (uint16_t)id, d1, &c1); eng_patch_can(&E.out, (uint16_t)id, d2, &c2);
        CHECK(!memcmp(d1, d2, 8) && c1 == c2, "off: CAN %X differs", id);
    }
    // and in P (pawl in) with gear ON, also nothing patched (= v1h content)
    setup(true);
    CHECK(E.out.g_set == 0, "gear on, P locked: nothing patched");
    // runtime switch: on in D -> off = fixed P straight away, logged
    brake = true; run_ms(100); push(LV_BACK1, 100);
    gear_set_enabled(&G, false, (uint32_t)T);
    run_ms(10);
    CHECK(G.target == GEAR_P && E.out.g_set == 0 && last_ev()->kind == GEV_ONOFF, "switched off in D: fixed P");
    printf("  %s\n", ev_text());
    printf("gear off: FlexRay static/dynamic and PT-CAN bytes identical to v1h (fixed P)\n");
}

// Replay the real lever (0x197) and brake (0x173) of r10md10m (real EGS on the car) and
// compare our 0x3FD gear code with the real EGS 0x3FD, change by change.
static void test_replay(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); fails++; return; }
    memset(&G, 0, sizeof G);
    gear_reset(&G, true);
    eng_out_t o; memset(&o, 0, sizeof o);
    char line[128];
    uint8_t real = 0, ours = 0x20;
    struct { uint32_t t; uint8_t v; } rc[32], oc[32];
    unsigned nr = 0, no = 0;
    uint32_t next_step = 0, t_disp = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        unsigned long t; unsigned id; char hex[40];
        if (sscanf(line, "%lu %x %39s", &t, &id, hex) != 3) continue;
        uint8_t d[8] = {0}; unsigned n = 0;
        for (; n < 8 && hex[2 * n] && hex[2 * n + 1]; n++) sscanf(hex + 2 * n, "%2hhx", &d[n]);
        uint32_t tu = (uint32_t)t + 1000000u;
        while (next_step <= tu) {   // FlexRay cycles up to this frame
            gear_step(&G, next_step, t_disp && next_step > t_disp);   // engine: running once the display is on
            gear_fill_out(&G, &o);
            uint8_t v = o.g_set ? o.g_3fd : 0x20;
            if (real && v != ours && no < 32) { oc[no].t = next_step; oc[no].v = v; no++; }
            if (real) ours = v;
            next_step += 5000u;
        }
        if (id == 0x197) gear_input_lever(&G, d, (uint8_t)n, tu);
        if (id == 0x173 && (d[7] & 3u) == 3u) gear_input_brake(&G, tu);
        if (id == 0x3FD && d[2] != real) {
            if (real && d[2] && nr < 32) { rc[nr].t = tu; rc[nr].v = d[2]; nr++; }
            if (!real && d[2]) t_disp = tu;
            real = d[2];
        }
    }
    fclose(f);
    printf("replay r10md10m (real lever + brake -> our 0x3FD vs the real EGS 0x3FD):\n");
    unsigned matched = 0;
    for (unsigned i = 0; i < nr; i++) {
        int best = -1; long bd = 0;
        for (unsigned k = 0; k < no; k++) {
            if (oc[k].v != rc[i].v) continue;
            long dd = (long)oc[k].t - (long)rc[i].t;
            if (best < 0 || labs(dd) < labs(bd)) { best = (int)k; bd = dd; }
        }
        printf("  real %8.3f s -> %02X   ours %s%+ld ms\n", (rc[i].t - 1000000u) / 1e6, rc[i].v,
               best < 0 ? "MISSING " : "", best < 0 ? 0 : bd / 1000);
        if (best >= 0 && labs(bd) <= 30000) matched++;
        CHECK(best >= 0 && labs(bd) <= 30000, "replay: real 0x3FD %02X at %.3f s: ours %ld ms off", rc[i].v,
              (rc[i].t - 1000000u) / 1e6, best < 0 ? 99999 : bd / 1000);
    }
    CHECK(no == nr, "replay: we changed 0x3FD %u times, the real EGS %u times", no, nr);
    printf("  %u of %u real 0x3FD changes matched within 30 ms, ours changed %u times, %u events, %u refusals\n",
           matched, nr, no, G.ev_seq, G.refusals);
    for (unsigned i = 0; i < GEAR_N_EV; i++) if (G.ev[i].seq) { char b[96]; gear_ev_text(&G.ev[i], b, sizeof b); printf("    %s\n", b); }
}

int main(int argc, char **argv)
{
    if (argc > 1) {
        csv = fopen(argv[1], "w");
        if (csv) fprintf(csv, "t_s,lever,pbtn,brake,gear,stage,3fd_d2,39a_d3,39a_d4,3f9_park,3f9_gear,dc_d2,dc_d5\n");
    }
    if (argc > 3 && (can_csv = fopen(argv[3], "w")) != NULL) fprintf(can_csv, "t_us,can_id,ide,data_hex,dir\n");
    test_shift_sequence();
    if (can_csv) { fclose(can_csv); can_csv = NULL; }
    test_refusals_timeout_bad();
    test_off_fixed_p();
    if (argc > 2) test_replay(argv[2]);
    printf("PT-CAN via pte_next: %u frames, %u E2E CRCs valid, 0x3FD frames %u\n", n_tx, n_crc_ok, n3fd_frames);
    if (csv) fclose(csv);
    printf("gear: %s\n", fails ? "FAILED" : "ok");
    return fails ? 1 : 0;
}
