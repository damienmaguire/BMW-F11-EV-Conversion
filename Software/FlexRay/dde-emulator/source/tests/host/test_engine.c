// v1g fake engine start (src/dde_engine.c) host checks:
//  - off by default: every FlexRay / PT-CAN frame is bit-identical to v1f;
//  - CAS start (ST_KL D + brake): the start / run / stop timeline of Ian's and Fred's
//    specs (times checked with tolerances), idle 780 rpm = 30 0C;
//  - every E2E CRC, alive counter and frame CRC of every frame built during the run is
//    valid (FlexRay static + dynamic, PT-CAN via the real pte_next path);
//  - fallback trigger (ST_KL A + brake + command), rejects, stop paths (ST_KL 9 / 8,
//    stale ST_KL, PC gone, stop command), restart, new-wake reset;
//  - writes the timeline to argv[1] (CSV) when given.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dde_emu_core.h"
#include "dde_emu_tables.h"
#include "dde_engine.h"
#include "ptcan_emu_core.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static FILE *csv;
static uint32_t T;               // simulated time (us)
static eng_t E;

// One FlexRay cycle: inputs every 100 ms (0x12F / 0x74 alternate is not needed: same path)
static uint8_t in_stkl = 0xA; static bool in_brake = false; static bool in_feed = true;
static void cycle5(void)
{
    T += 5000u;
    if (in_feed && T % 100000u == 0) eng_input_stkl(&E, in_stkl, in_brake, T);
    if (T % 1000000u == 0) eng_set_mode(&E, E.mode, T);   // logger keepalive (if mode != off)
    eng_step(&E, T);
    if (csv) {
        const eng_out_t *o = &E.out;
        fprintf(csv, "%.3f,%u,%s,%u,%u,%02X,%02X,%u,%u,%u,%02X,%02X,%u,%u,%u,%u\n", T / 1e6, in_stkl,
                eng_phase_name(o->phase), o->rpm4 / 4u, o->torque, o->e6_b3, o->e6_b4,
                o->e6_ready == 0xFF ? 9u : o->e6_ready, o->cnt_hi, o->fb9_b2, o->b142_b8, o->b142_b23,
                o->fb7_mv, o->f3_val, o->fuel, o->e8_fast);
    }
}
static void run_ms(uint32_t ms) { for (uint32_t k = 0; k < ms / 5u; k++) cycle5(); }
static uint32_t ms_since(uint32_t t) { return (T - t) / 1000u; }

// Validity of every frame built with the current overlay.
static unsigned e2e_checked;
static void check_frames(const char *what)
{
    uint8_t f[DDE_MAX_FRAME_BYTES];
    for (unsigned tg = 0; tg < 4; tg++) {
        for (unsigned c = 0; c < 64; c++) {
            unsigned n = dde_core_build_static_ph(tg, c, c % 15u, false, DDE_PH_ON, f);
            uint32_t crc = dde_core_frame_crc(f, n - 3u);
            CHECK(f[n - 3] == (uint8_t)(crc >> 16) && f[n - 2] == (uint8_t)(crc >> 8) && f[n - 1] == (uint8_t)crc,
                  "%s: frame CRC target %u cycle %u", what, tg, c);
            if (!(dde_core_emu_indicators(tg, c, false) & 0x04u)) continue;
            const uint8_t *pl = f + 5;
            for (unsigned g = 0; g < DDE_EMU_N_E2E; g++) {
                const uint8_t *e = dde_emu_e2e[g];
                if (e[0] != tg || e[1] != (c & 3u)) continue;
                CHECK(pl[e[2]] == (uint8_t)(dde_core_crc8(&pl[e[2] + 1], e[3] - e[2]) ^ e[4]),
                      "%s: E2E CRC target %u cycle %u group %u", what, tg, c, g);
                CHECK((pl[e[2] + 1] & 0x0F) == c % 15u, "%s: alive counter target %u cycle %u", what, tg, c);
                uint8_t hi = pl[e[2] + 1] >> 4;
                CHECK(hi == 1 || hi == 3 || hi == 0xF, "%s: CounterHi %X target %u cycle %u", what, hi, tg, c);
                e2e_checked++;
            }
        }
    }
    for (unsigned p = 0; p < DDE_DYN_N_PDU; p++) {
        unsigned n = dde_core_build_dyn_pdu_ph(p, dde_dyn_pdu[p].cyc_mod8, 5, p == DDE_PDU_142 ? DDE_142_P2 : 1u, f);
        uint32_t crc = dde_core_frame_crc(f, n - 3u);
        CHECK(f[n - 1] == (uint8_t)crc && f[n - 2] == (uint8_t)(crc >> 8), "%s: dyn frame CRC pdu %u", what, p);
        if (dde_dyn_pdu[p].kind == DDE_DYN_KIND_E2E) {
            const uint8_t *pl = f + 5;
            CHECK(pl[0] == (uint8_t)(dde_core_crc8(&pl[1], dde_dyn_pdu[p].plen - 1u) ^ dde_dyn_pdu[p].k),
                  "%s: E6 E2E CRC", what);
            CHECK((pl[1] & 0x0F) == 5, "%s: E6 alive counter", what);
            e2e_checked++;
        }
    }
}

static void fr_payload(unsigned tg, unsigned c, uint8_t *pl)
{
    uint8_t f[DDE_MAX_FRAME_BYTES];
    dde_core_build_static_ph(tg, c, 1, false, DDE_PH_ON, f);
    memcpy(pl, f + 5, 16);
}
static void dyn_payload(unsigned p, uint8_t *pl)
{
    uint8_t f[DDE_MAX_FRAME_BYTES];
    dde_core_build_dyn_pdu_ph(p, dde_dyn_pdu[p].cyc_mod8, 1, p == DDE_PDU_142 ? DDE_142_P2 : 1u, f);
    memcpy(pl, f + 5, dde_dyn_pdu[p].plen);
}

// Off by default: identical bytes with and without the overlay pointer.
static void test_off_identical(void)
{
    eng_reset(&E);
    E.mode = ENG_MODE_OFF;
    in_stkl = 0xD; in_brake = true;
    run_ms(2000);                                   // a D without the mode: nothing
    CHECK(E.phase == ENG_OFF && !E.out.active, "mode off: D must not start");
    uint8_t a[DDE_MAX_FRAME_BYTES], b[DDE_MAX_FRAME_BYTES];
    for (unsigned tg = 0; tg < 4; tg++) for (unsigned c = 0; c < 64; c++) {
        dde_core_eng = NULL; unsigned n1 = dde_core_build_static_ph(tg, c, 3, false, DDE_PH_ON, a);
        dde_core_eng = &E.out; unsigned n2 = dde_core_build_static_ph(tg, c, 3, false, DDE_PH_ON, b);
        CHECK(n1 == n2 && !memcmp(a, b, n1), "off: static %u/%u differs", tg, c);
    }
    for (unsigned p = 0; p < DDE_DYN_N_PDU; p++) for (unsigned ph = 0; ph < 3; ph++) {
        dde_core_eng = NULL; unsigned n1 = dde_core_build_dyn_pdu_ph(p, 2, 3, ph, a);
        dde_core_eng = &E.out; unsigned n2 = dde_core_build_dyn_pdu_ph(p, 2, 3, ph, b);
        CHECK(n1 == n2 && !memcmp(a, b, n1), "off: dyn %u differs", p);
    }
    uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8}, d2[8]; uint8_t chi = 1;
    memcpy(d2, d, 8);
    for (unsigned id = 0; id < 0x800; id++) CHECK(!eng_patch_can(&E.out, (uint16_t)id, d2, &chi) && !memcmp(d, d2, 8) && chi == 1, "off: CAN %X", id);
    printf("off by default: FlexRay static/dynamic and PT-CAN bytes identical to v1f\n");
}

// v1h: expected start timeline per profile (ms after the ST_KL D input; the engine steps
// every 5 ms, so +0..5 ms of latency on top of the profile time)
typedef struct { int blip0, blip1, end0, end1, rpm0, rpm1, e81_0, e81_1, e82_0, e82_1, b4_0, b4_1, rdy0, rdy1,
                 inj0, inj1, t92_0, t92_1; } expect_t;
static const expect_t EXP[ENG_PROF_N] = {
    [ENG_PROF_SHORT]     = {0, 10, 100, 115, 0, 10, 0, 10, 150, 200, 0, 0, 0, 0, 40, 50, 0, 10},
    [ENG_PROF_REALISTIC] = {40, 55, 145, 160, 85, 95, 85, 105, 830, 860, 0, 0, 10, 25, 170, 185, 0, 10},
    [ENG_PROF_RUN8]      = {45, 60, 145, 165, 90, 105, 90, 110, 850, 1200, 245, 260, 15, 30, -1, -1, 15, 25},
};

static void test_cas_start_stop(unsigned prof)
{
    const expect_t *X = &EXP[prof];
    eng_reset(&E);
    const uint32_t s0 = E.starts;
    T += 1000000u;
    eng_set_mode(&E, ENG_MODE_CAS, T);
    eng_set_profile(&E, (uint8_t)prof);
    in_stkl = 0x6; run_ms(200);   // a fresh ST_KL sequence (the previous test may have left D)
    in_stkl = 0xA; in_brake = false;
    run_ms(1000);
    CHECK(E.phase == ENG_OFF, "KL15 only: no start");
    in_brake = true;
    run_ms(300);
    in_stkl = 0xD;                                  // CAS starter request
    uint32_t t_d = 0, t_blip = 0, t_blip_end = 0, t_rpm = 0, t_81 = 0, t_82 = 0, t_b4_26 = 0, t_ready0 = 0,
             t_f97c = 0, t_b23 = 0, t_fb9 = 0, t_fb9_00 = 0, t_e8 = 0, t_c8 = 0, t_inj = 0;
    bool e8c_ok = true;
    for (int k = 0; k < 2400; k++) {               // 12 s
        if (E.out.e6_b3 == 0x82 && t_82 && ms_since(t_82) > 300 && in_stkl == 0xD) in_stkl = 0xC;   // CAS: D -> C after 82
        cycle5();
        const eng_out_t *o = &E.out;
        if (!t_d && E.phase == ENG_CRANK) t_d = T - 5000u;   // trigger in the previous step's input
        if (!t_blip && o->cnt_hi == 3) t_blip = T;
        if (t_blip && !t_blip_end && o->cnt_hi != 3) t_blip_end = T;
        if (!t_rpm && o->rpm4) t_rpm = T;
        if (!t_81 && o->e6_b3 == 0x81) t_81 = T;
        if (!t_82 && o->e6_b3 == 0x82) t_82 = T;
        if (!t_b4_26 && o->e6_b4 == 0x26) t_b4_26 = T;
        if (t_82 && !t_ready0 && o->e6_ready == 0) t_ready0 = T;
        if (!t_f97c && o->f97c) t_f97c = T;
        if (!t_b23 && o->b142_b23 == 0x24) t_b23 = T;
        if (!t_fb9 && o->fb9_b2) t_fb9 = T;
        if (!t_c8 && o->fb9_b2 == 0xC8) t_c8 = T;
        if (!t_fb9_00 && o->fb9_set && o->fb9_b3 == 0) t_fb9_00 = T;
        if (!t_e8 && o->e8_fast) t_e8 = T;
        if (!t_inj && o->inj) t_inj = T;
        // v1h: 0xE8 crank rate from the start until the fast rate (not in RUN8)
        if (E.phase == ENG_CRANK || E.phase == ENG_RUN)
            e8c_ok = e8c_ok && (o->e8_crank == (eng_profiles[prof].e8_crank && !o->e8_fast));
        // never "running" (82) while the profile says crank, never 82 before START (D)
        if (o->e6_b3 == 0x82) CHECK(o->rpm4 >= 4u * 950u || t_82, "82 below 950 rpm");
        if (k % 40 == 0) check_frames("running");
        // safety: the overlay never claims a CounterHi other than 1/3 and 0x592 is never touched
        uint8_t d[8] = {0x40, 0x22, 0x00, 0x14, 0xff, 0xff, 0xff, 0xff}; uint8_t chi = 0;
        CHECK(!eng_patch_can(o, 0x592, d, &chi), "0x592 must stay the MIL-off frame");
    }
    CHECK(t_d && t_blip && t_rpm && t_81 && t_82, "start milestones missing");
    int32_t rel = 0;
#define RELV(t) ((int32_t)((t) - t_d) / 1000)
#define REL(t) (rel = RELV(t))
    printf("[%s] CAS start (ms after ST_KL D): CounterHi=3 %d..%d, first rpm %d, E6 81 %d, E6 82 %d, b4 26 %d, ready 0 %d,"
           " F9 7C %d, 0x142 b23 24 %d, FB9 ramp %d (C8 %d), FB9 b3 00 %d, E8 fast %d, 0x3FB f0 %d\n",
           eng_prof_name(prof), RELV(t_blip), RELV(t_blip_end), RELV(t_rpm), RELV(t_81), RELV(t_82), RELV(t_b4_26), RELV(t_ready0), RELV(t_f97c),
           RELV(t_b23), RELV(t_fb9), RELV(t_c8), RELV(t_fb9_00), RELV(t_e8), t_inj ? RELV(t_inj) : -1);
    CHECK(REL(t_blip) >= X->blip0 && REL(t_blip) <= X->blip1, "CounterHi=3 start, got %d", rel);
    CHECK(REL(t_blip_end) >= X->end0 && REL(t_blip_end) <= X->end1, "CounterHi back to 1, got %d", rel);
    CHECK(REL(t_rpm) >= X->rpm0 && REL(t_rpm) <= X->rpm1, "first rpm, got %d", rel);
    CHECK(REL(t_81) >= X->e81_0 && REL(t_81) <= X->e81_1, "E6 81, got %d", rel);
    CHECK(REL(t_82) >= X->e82_0 && REL(t_82) <= X->e82_1, "E6 82, got %d", rel);
    CHECK(t_81 <= t_82 && t_rpm <= t_82, "crank before running");
    CHECK((int32_t)(t_b4_26 - t_82) / 1000 >= X->b4_0 && (int32_t)(t_b4_26 - t_82) / 1000 <= X->b4_1, "b4 26 after 82");
    CHECK((int32_t)(t_ready0 - t_82) / 1000 >= X->rdy0 && (int32_t)(t_ready0 - t_82) / 1000 <= X->rdy1, "ready 0 after 82");
    if (X->inj0 < 0) CHECK(!t_inj, "RUN8: 0x3FB d4 never f0");
    else CHECK(t_inj && REL(t_inj) >= X->inj0 && REL(t_inj) <= X->inj1, "0x3FB f0 at %d", t_inj ? REL(t_inj) : -1);
    CHECK(e8c_ok, "0xE8 crank rate flag");
    {   // 0x3FB d4 on PT-CAN while injecting: f1 -> f0, nothing else
        uint8_t d[8] = {0xd0, 0x10, 0x01, 0x10, 0xf1, 0x08, 0, 0}; uint8_t chi = 0;
        bool ch = eng_patch_can(&E.out, 0x3FB, d, &chi);
        CHECK(X->inj0 < 0 ? (!ch && d[4] == 0xf1) : (ch && d[4] == 0xf0 && d[0] == 0xd0 && d[5] == 0x08), "0x3FB d4 patch");
    }
    CHECK(E.cas_flags & ENG_CAS_D_ENDED, "D -> C after 82 recorded");
    CHECK((int32_t)(t_f97c - t_rpm) / 1000 >= 60 && (int32_t)(t_f97c - t_rpm) / 1000 <= 140, "F9 7C +0.07..0.13 s (Fred)");
    CHECK((int32_t)(t_fb9 - t_rpm) / 1000 >= 1990 && (int32_t)(t_fb9 - t_rpm) / 1000 <= 2010, "FB9 ramp +2.0 s");
    CHECK((int32_t)(t_c8 - t_fb9) / 1000 >= 3500 && (int32_t)(t_c8 - t_fb9) / 1000 <= 4300, "FB9 C8 in ~4 s");
    CHECK((int32_t)(t_fb9_00 - t_rpm) / 1000 >= 4995 && (int32_t)(t_fb9_00 - t_rpm) / 1000 <= 5100, "FB9 00 00 at +5.0 s");
    CHECK((int32_t)(t_e8 - t_rpm) / 1000 >= 2600 && (int32_t)(t_e8 - t_rpm) / 1000 <= 3100, "E8 fast +2.6..3.1 s");
    // idle 780 rpm (owner: 30 0C), rpm field bytes
    uint8_t pl[16];
    dde_core_eng = &E.out;
    fr_payload(1, 1, pl);
    unsigned rpm = (unsigned)(pl[5] | pl[6] << 8) / 4u;
    printf("idle after 12 s: 0x28 c1 b6..7 = %02X %02X (%u rpm), torque %02X %02X %02X, E6 %02X %02X ready %u\n",
           pl[5], pl[6], rpm, pl[2], pl[3], pl[4], E.out.e6_b3, E.out.e6_b4, E.out.e6_ready);
    CHECK(rpm >= 776 && rpm <= 790, "idle ~780 rpm, got %u", rpm);
    CHECK(pl[2] == 0x32 && pl[3] == 0x28 && pl[4] == 0x83, "idle torque 32 28 83");
    CHECK(E.out.e6_b3 == 0x82 && E.out.e6_b4 == 0x26 && E.out.e6_ready == 0, "running E6 82/26/0x");
    fr_payload(1, 3, pl);
    CHECK(pl[4] == 0x20 && pl[5] == 0x20 && pl[2] == 0x3E && pl[3] == 0x7D, "0x28 c3 running bytes");
    fr_payload(2, 3, pl);
    CHECK(pl[4] == 0x72 && pl[2] == 0xF9 && pl[3] == 0x7C && pl[10] == 0xF9 && pl[15] == 0x7C, "0x29 c3 running bytes");
    fr_payload(3, 3, pl);
    CHECK(pl[4] == 0xF9 && pl[5] == 0x7C && pl[6] == 0xF9 && pl[7] == 0x7C, "0x3D c3 running bytes");
    fr_payload(0, 1, pl);
    unsigned tu = pl[2] | pl[3] << 8;
    CHECK(tu >= 2000 + 776 - 56 && tu <= 2000 + 790 - 56, "turbine 2000 + rpm - 56, got %u", tu);
    uint8_t dp[32];
    dyn_payload(DDE_PDU_E6, dp);
    CHECK(dp[2] == 0x82 && dp[3] == 0x26 && (dp[6] >> 4) == 0 && (dp[1] >> 4) == 1, "0xE6 running bytes");
    dyn_payload(DDE_PDU_142, dp);
    CHECK(dp[7] == 0x84 && dp[22] == 0x24, "0x142 b8 84 / b23 24");
    dyn_payload(DDE_PDU_FB9, dp);
    CHECK(dp[1] == 0xC8 && dp[2] == 0 && dp[3] == 0, "FB L9 1F C8 00 00");
    dyn_payload(DDE_PDU_FB7, dp);
    CHECK(dp[2] == 0x6C && dp[3] == 0x39, "FB L7 14.70 V = 6C 39");
    dyn_payload(DDE_PDU_E89, dp);
    CHECK(dp[0] == 0x25 && dp[1] == 0x14 && dp[3] == 0xAC, "E8 L9 25 14 00 AC");

    // engine-off request with KL15 on: ST_KL 9 -> 0x142 b8 A4 at +50 ms, then 8 -> stop
    in_stkl = 0x9;
    uint32_t t9 = T + 100000u - (T % 100000u);    // next input instant
    uint32_t t_a4 = 0;
    for (int k = 0; k < 60; k++) { cycle5(); if (!t_a4 && E.out.b142_b8 == 0xA4) t_a4 = T; }
    CHECK(t_a4 && (int32_t)(t_a4 - t9) / 1000 >= 45 && (int32_t)(t_a4 - t9) / 1000 <= 60, "ST_KL 9: 0x142 A4 +50 ms (%d)",
          (int32_t)(t_a4 - t9) / 1000);
    CHECK(E.phase == ENG_RUN && E.out.e6_b3 == 0x82, "ST_KL 9 alone keeps running for up to 1 s");
    in_stkl = 0x8;
    uint32_t t_92 = 0, t_c0 = 0, t_ff7f = 0, t_80 = 0, t_ready1 = 0, t_e0 = 0, t_rpm0 = 0;
    uint32_t ts = 0;
    for (int k = 0; k < 1200; k++) {
        cycle5();
        const eng_out_t *o = &E.out;
        if (!ts && E.phase == ENG_STOP) ts = E.ts;
        if (!t_92 && o->e6_b3 == 0x92) t_92 = T;
        if (!t_c0 && o->r29_b8 == 0xC0) t_c0 = T;
        if (!t_e0 && o->c3_b56 == 3) t_e0 = T;
        if (!t_ff7f && o->fb9_b3 == 0xFF) t_ff7f = T;
        if (!t_rpm0 && ts && o->rpm4 == 0) t_rpm0 = T;
        if (!t_80 && t_92 && o->e6_b3 == 0x80) t_80 = T;
        if (t_80 && !t_ready1 && o->e6_ready == 1) t_ready1 = T;
        if (k % 40 == 0) check_frames("stopping");
    }
    printf("stop (ms after ST_KL 8): E6 92 %d, 0x29 b8 C0 %d, 0x28 c3 E0 E0 %d, FB9 FF 7F %d, rpm 0 %d, E6 80 %d, ready 1 %d\n",
           (int)(t_92 - ts) / 1000, (int)(t_c0 - ts) / 1000, (int)(t_e0 - ts) / 1000, (int)(t_ff7f - ts) / 1000,
           (int)(t_rpm0 - ts) / 1000, (int)(t_80 - ts) / 1000, (int)(t_ready1 - ts) / 1000);
    CHECK(ts && (int)(t_92 - ts) / 1000 >= X->t92_0 && (int)(t_92 - ts) / 1000 <= X->t92_1, "92 timing");
    // ST_KL 8 input -> 92 (engine steps every 5 ms; the real DDE: within 20 ms)
    if (prof != ENG_PROF_RUN8) CHECK((int)(t_92 - ts) / 1000 + 5 <= 20, "92 within 20 ms of ST_KL 8");
    CHECK(t_e0 == t_92, "0x28 c3 E0 E0 together with 92 (Fred)");
    CHECK((t_c0 - ts) / 1000 >= 30 && (t_c0 - ts) / 1000 <= 40, "0x29 b8 C0 +35 ms");
    CHECK((t_ff7f - ts) / 1000 >= 235 && (t_ff7f - ts) / 1000 <= 250, "FB9 FF 7F +0.24 s");
    CHECK((t_rpm0 - ts) / 1000 >= 1150 && (t_rpm0 - ts) / 1000 <= 1250, "rpm 0 at ~+1.2 s");
    CHECK(t_80 && (t_ready1 - t_80) / 1000 >= 15 && (t_ready1 - t_80) / 1000 <= 30, "ready 1 20 ms after 80");
    CHECK(E.phase == ENG_STOPPED && E.out.e6_b3 == 0x80 && E.out.e6_b4 == 0x64 && E.out.torque == 1978, "stopped state");
    fr_payload(2, 3, pl);
    CHECK(pl[2] == 0xF9 && pl[3] == 0x7C && pl[4] == 0x7C && pl[7] == 0xC0, "0x29 after stop: F9 7C sticky, b5 7C, b8 C0");
    fr_payload(1, 3, pl);
    CHECK(pl[2] == 0x1C && pl[3] == 0x7E && pl[4] == 0xE0, "0x28 c3 after stop 1C 7E E0 E0");
    // restart from STOPPED
    in_stkl = 0xA; run_ms(500);
    in_stkl = 0xD;
    uint32_t t_r = 0, t_r81 = 0, t_rfb9 = 0, t_re8 = 0; bool r_early = false;
    for (int k = 0; k < 1400; k++) {
        cycle5();
        if (!t_r && E.phase == ENG_CRANK) t_r = T;
        if (t_r && !E.cur_run && (E.out.fb9_b2 || E.out.e8_fast || E.out.e6_b3 == 0x82)) r_early = true;
        if (t_r && !t_r81 && E.out.e6_b3 == 0x81) t_r81 = T;
        if (t_r && !t_rfb9 && E.out.fb9_b2 == 0x2D) t_rfb9 = T;
        if (t_r && !t_re8 && E.out.e8_fast && E.out.e89 == 1) t_re8 = T;
    }
    printf("restart: E6 81 +%d ms, FB9 ramp +%d ms, E8 running +%d ms\n", (int)(t_r81 - t_r) / 1000,
           (int)(t_rfb9 - t_r) / 1000, (int)(t_re8 - t_r) / 1000);
    CHECK(E.phase == ENG_RUN && E.starts == s0 + 2u, "restart from stopped");
    CHECK(!r_early && t_r81 && (t_r81 - t_r) / 1000 <= 110 && (t_rfb9 - t_r) / 1000 >= 2000 && (t_re8 - t_r) / 1000 >= 2600,
          "restart replays the whole start timeline");
    // new FlexRay wake: everything off again, mode kept
    eng_reset(&E);
    eng_step(&E, T);
    CHECK(!E.out.active && E.mode == ENG_MODE_CAS, "new wake: overlay off, mode kept");
}

static void test_fallback_and_guards(void)
{
    eng_reset(&E);
    T += 1000000u;
    eng_set_mode(&E, ENG_MODE_FALLBACK, T);
    in_stkl = 0xA; in_brake = false;
    run_ms(500);
    eng_command(&E, ENG_CMD_START, T);
    run_ms(500);
    CHECK(E.phase == ENG_OFF && E.last_reject == ENG_REJ_NO_BRAKE, "fallback: no brake -> rejected");
    in_stkl = 0xD;     // fallback also ignores a plain D (no command)
    in_brake = true;
    run_ms(500);
    CHECK(E.phase == ENG_OFF, "fallback mode: D alone does not start");
    in_stkl = 0xA;
    run_ms(200);
    eng_command(&E, ENG_CMD_START, T);
    run_ms(1500);
    CHECK(E.phase == ENG_RUN, "fallback: A + brake + command -> running (phase %u)", E.phase);
    eng_command(&E, ENG_CMD_STOP, T);
    run_ms(2000);
    CHECK(E.phase == ENG_STOPPED, "stop command -> stopped");
    // PC goes away while running: mode lapses after 3 s -> stop
    eng_command(&E, ENG_CMD_START, T);
    run_ms(1500);
    CHECK(E.phase == ENG_RUN, "fallback restart");
    uint8_t mode = E.mode;
    E.mode_us = T - 2000000u;   // last keepalive 2 s ago, stop sending keepalives:
    for (int k = 0; k < 400 && E.phase == ENG_RUN; k++) { T += 5000u; if (T % 100000u == 0) eng_input_stkl(&E, 0xA, true, T); eng_step(&E, T); }
    CHECK(E.phase == ENG_STOP && E.mode == ENG_MODE_OFF, "keepalive lost -> stop (mode was %u)", mode);
    run_ms(2000);
    // stale ST_KL while running -> stop
    eng_set_mode(&E, ENG_MODE_CAS, T);
    in_stkl = 0xD; run_ms(1500);
    CHECK(E.phase == ENG_RUN, "CAS restart");
    in_feed = false;
    run_ms(700);
    CHECK(E.phase == ENG_STOP || E.phase == ENG_STOPPED, "ST_KL lost 600 ms -> stop");
    in_feed = true;
    // start aborted in the first 95 ms: no trace
    eng_reset(&E); in_stkl = 0xA; run_ms(300);
    in_stkl = 0xD; run_ms(100);
    CHECK(E.phase == ENG_CRANK, "crank");
    in_stkl = 0x8; run_ms(300);
    CHECK(E.phase == ENG_STOP || E.phase == ENG_STOPPED || E.phase == ENG_OFF, "ST_KL 8 while cranking stops");
    printf("fallback trigger, rejects, stop command, PC lost, ST_KL lost, crank abort: checked\n");
}

// PT-CAN through the real pte_next() path: CRCs/counters valid, bytes as specified.
static void test_ptcan(void)
{
    pte_t p;
    pte_init(&p);
    pte_set_mask(&p, PTE_MASK_ALL);
    eng_reset(&E);
    E.mode = ENG_MODE_CAS; E.mode_us = T;
    uint64_t t0 = 10000000ull, t = t0;
    uint32_t sent = 0, crc_ok = 0, a5_rpm_max = 0, f3_max = 0, d9_hi3 = 0, f9_hi3 = 0, f3_bad = 0;
    uint8_t last397 = 0, a5_last[8] = {0}, f3_last[8] = {0};
    unsigned n397 = 0; bool saw_3b3_c8 = false, saw_381_79 = false, saw_8f_72 = false;
    uint8_t stkl = 0xA;
    for (; t < t0 + 14000000ull; t += 1000) {
        uint64_t rel = t - t0;
        if (rel % 10000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x1A1, d, 8, t); }
        if (rel == 3000000) stkl = 0xD;
        if (rel == 5000000) stkl = 0xC;
        if (rel == 10000000) stkl = 0x8;
        if (rel % 100000 == 0) {
            uint8_t d[8] = {0, 0, stkl, 0, 0, 0, 3, 0};
            pte_on_rx(&p, 0x12F, d, 8, t);
            eng_input_stkl(&E, stkl, true, (uint32_t)t);
        }
        if (rel % 640000 == 0) { uint8_t d[8] = {0}; pte_on_rx(&p, 0x510, d, 8, t); }
        if (rel % 5000 == 0) { if (rel % 1000000 == 0) eng_set_mode(&E, ENG_MODE_CAS, (uint32_t)t); eng_step(&E, (uint32_t)t); }
        p.eng = E.out.active ? &E.out : NULL;
        pte_tx_t tx;
        while (pte_next(&p, t, &tx)) {
            CHECK(!pte_id_forbidden(tx.id), "forbidden id %X", tx.id);
            const pte_def_t *def = &pte_defs[tx.idx];
            if (def->data_id != 0xFFFF) {
                CHECK(tx.data[0] == pte_e2e_crc(def->data_id, tx.data, tx.dlc), "PT-CAN CRC %X", tx.id);
                crc_ok++;
            }
            if (tx.id == 0x0A5) {
                uint32_t r = (uint32_t)(tx.data[5] | tx.data[6] << 8) / 4u;
                if (r > a5_rpm_max) a5_rpm_max = r;
                memcpy(a5_last, tx.data, 8);
            }
            if (tx.id == 0x0F3) {
                uint32_t v = (uint32_t)(tx.data[1] >> 4) | (uint32_t)tx.data[2] << 4;
                if (v > f3_max) f3_max = v;
                if (v > 120) f3_bad++;
                memcpy(f3_last, tx.data, 8);
            }
            if (tx.id == 0x0D9 && (tx.data[1] >> 4) == 3) d9_hi3++;
            if (tx.id == 0x3F9 && (tx.data[1] >> 4) == 3) f9_hi3++;
            if (tx.id == 0x3B3 && tx.data[1] == 0xC8) saw_3b3_c8 = true;
            if (tx.id == 0x381 && tx.data[0] == 0x79) saw_381_79 = true;
            if (tx.id == 0x08F && tx.data[4] == 0x72 && tx.data[2] == 0xF9) saw_8f_72 = true;
            if (tx.id == 0x397 && tx.data[1] != last397) { n397++; last397 = tx.data[1]; }
            if (tx.id == 0x592) CHECK(tx.data[0] == 0x40 && tx.data[1] == 0x22, "0x592 MIL off");
            pte_commit(&p, &tx, t);
            sent++;
        }
        if (rel == 9000000) {
            uint32_t r = (uint32_t)(a5_last[5] | a5_last[6] << 8) / 4u;
            CHECK(r >= 776 && r <= 800, "PT-CAN 0xA5 idle rpm %u", r);
            CHECK(a5_last[2] == 0x32 && a5_last[3] == 0x28 && a5_last[4] == 0x83, "0xA5 idle torque 32 28 83");
            uint32_t v = (uint32_t)(f3_last[1] >> 4) | (uint32_t)f3_last[2] << 4;
            CHECK(v >= 76 && v <= 80, "0xF3 = rpm/10 at idle (%u)", v);
        }
    }
    CHECK(a5_rpm_max >= 1000 && a5_rpm_max <= 1070, "0xA5 flare peak %u", a5_rpm_max);
    CHECK(d9_hi3 >= 3 && d9_hi3 <= 8, "0xD9 CounterHi 3 for ~105 ms (%u frames)", d9_hi3);
    CHECK(f9_hi3 >= 1, "0x3F9 81 event with CounterHi 3");
    CHECK(saw_3b3_c8 && saw_381_79 && saw_8f_72, "0x3B3 C8 / 0x381 79 / 0x8F 72 F9 7C");
    CHECK(E.phase == ENG_STOPPED, "PT-CAN run ends stopped");
    printf("PT-CAN via pte_next: %u frames, %u E2E CRCs valid, 0xA5 peak %u rpm, 0xF3 max %u, D9 CounterHi=3 x%u, "
           "0x397 d1 changes %u, 0x592 MIL off only\n", sent, crc_ok, a5_rpm_max, f3_max, d9_hi3, n397);
}

// v1h: the CAS ends ST_KL D before our 82 (it refused / gave up on the short crank).
// short / realistic: the start is abandoned (no 82 ever), flags + abort counter say so.
// run8: the v1g.2 behaviour (keeps going to 82), but the early end is still flagged.
// Also: D held 300 ms after our 82 -> ENG_CAS_D_LONG.
static void test_cas_refusal(void)
{
    for (unsigned prof = ENG_PROF_SHORT; prof < ENG_PROF_N; prof++) {
        eng_reset(&E);
        E.phase = ENG_OFF; E.has_run = false;
        T += 1000000u;
        eng_set_mode(&E, ENG_MODE_CAS, T);
        eng_set_profile(&E, (uint8_t)prof);
        in_stkl = 0x6; in_brake = true; run_ms(300);
        in_stkl = 0xA; run_ms(300);
        uint32_t ab0 = E.aborts;
        in_stkl = 0xD; run_ms(100);
        CHECK(E.phase == ENG_CRANK, "[%s] cranking", eng_prof_name(prof));
        in_stkl = 0xC; run_ms(100);               // D ended after ~100 ms, before any 82
        bool saw82 = false;
        for (int k = 0; k < 400; k++) { cycle5(); if (E.out.e6_b3 == 0x82 && E.phase != ENG_STOP) saw82 = true; }
        CHECK(E.cas_flags & ENG_CAS_D_ENDED_EARLY, "[%s] early D end flagged", eng_prof_name(prof));
        if (eng_profiles[prof].need_d) {
            CHECK(!saw82 && E.aborts == ab0 + 1 && (E.cas_flags & ENG_CAS_ABANDONED) && E.phase != ENG_RUN,
                  "[%s] start abandoned, never running", eng_prof_name(prof));
        } else {
            CHECK(saw82 && E.aborts == ab0, "[%s] run8 keeps the v1g.2 behaviour", eng_prof_name(prof));
        }
        in_stkl = 0x6; run_ms(1500);
    }
    // D held long after 82
    eng_reset(&E); T += 1000000u;
    eng_set_mode(&E, ENG_MODE_CAS, T); eng_set_profile(&E, ENG_PROF_SHORT);
    in_stkl = 0xA; in_brake = true; run_ms(300);
    in_stkl = 0xD; run_ms(700);
    CHECK(E.phase == ENG_RUN && (E.cas_flags & ENG_CAS_D_LONG) && !(E.cas_flags & ENG_CAS_D_ENDED),
          "D still held 300 ms after 82 flagged");
    in_stkl = 0x6; run_ms(2000);
    printf("CAS refusal: D ended before 82 -> short/realistic abandon the start (no 82), run8 continues; "
           "D held 300 ms after 82 flagged\n");
}

int main(int argc, char **argv)
{
    dde_core_init();
    if (argc > 1) {
        csv = fopen(argv[1], "w");
        if (csv) fprintf(csv, "t_s,stkl,phase,rpm,torque_raw,e6_b3,e6_b4,ready,cnt_hi,fb9_b2,b142_b8,b142_b23,fb7_mv,f3,fuel,e8_fast\n");
    }
    test_off_identical();
    dde_core_eng = &E.out;
    test_cas_start_stop(ENG_PROF_SHORT);      // v1h default (timeline CSV = the short start)
    if (csv) { fclose(csv); csv = NULL; }
    test_cas_start_stop(ENG_PROF_REALISTIC);
    test_cas_start_stop(ENG_PROF_RUN8);
    test_cas_refusal();
    eng_set_profile(&E, ENG_PROF_RUN8);       // the v1g tests below were written for this timeline
    test_fallback_and_guards();
    test_ptcan();
    dde_core_eng = NULL;
    printf("frames checked: %u E2E groups valid\n", e2e_checked);
    printf("engine tests: %s\n", fails ? "FAILED" : "ok");
    return fails ? 1 : 0;
}
