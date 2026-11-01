// v1g fake engine start. See dde_engine.h for the plain-English overview.
// Every number below cites where it comes from:
//   Ian  = flexray-re-notes/analysis/dde_start_sequence.md/.csv
//   Fred = f11_fr_engine_running_diff.md/.csv
//   3krev = egs-logs/F11_4log_3krev.csv (real DDE: D at 4.792 s, 82 at ~5.80 s, stop ~19.3 s)
#include "dde_engine.h"
#include <string.h>
#include "dde_emu_core.h"   // DDE_PDU_* indices
#include "dde_emu_tables.h" // DDE_TAB_RAM

// Runs inside the FlexRay header ISR on core 1: code in RAM on the target (as dde_emu_core.c).
#if defined(__arm__)
#include "pico.h"
#define ENG_RAM(f) __time_critical_func(f)
#else
#define ENG_RAM(f) f
#endif

static uint32_t ENG_RAM(since_ms)(uint32_t now, uint32_t t) { return (now - t) / 1000u; }

// v1h start profiles (see dde_engine.h). All times in ms after the start trigger.
//                       blip_on/off  rpm  81   fire runup ready_off inj     t92  b4_26 e8_crank need_d
const eng_prof_t eng_profiles[ENG_PROF_N] DDE_TAB_RAM = {
    [ENG_PROF_DEFAULT]   = {0},   // never used directly: resolved to ENG_PROF_BUILD_DEFAULT
    // SHORT: everything in the first cycle after D; 950 rpm (= 82) at 50 + 750/800 * 130 = 172 ms
    [ENG_PROF_SHORT]     = {0, 105,   0,   0,   50,  130,  0,        40,     2,   1,    1,       1},
    // REALISTIC: 82 at 600 + 750/800 * 250 = 834 ms (+ <= 5 ms cycle) ~ D+840 (obd1: D+851)
    [ENG_PROF_REALISTIC] = {45, 150,  85,  95,  600, 250, 15,       175,     2,   1,    1,       1},
    // RUN8: the v1g.2 timeline (82 at 934 ms, measured D+941) and content
    [ENG_PROF_RUN8]      = {50, 155,  95, 100,  700, 250, 20,    0xFFFF,    19,   0,    0,       0},
};

const char *eng_prof_name(unsigned prof)
{
    static const char *const n[] = {"default", "short", "realistic", "run8"};
    return prof < ENG_PROF_N ? n[prof] : "?";
}

static unsigned ENG_RAM(prof_resolve)(unsigned prof)
{
    if (prof == ENG_PROF_DEFAULT || prof >= ENG_PROF_N) prof = ENG_PROF_BUILD_DEFAULT;
    return prof;
}

const char *eng_phase_name(unsigned phase)
{
    static const char *const n[] = {"off", "crank", "run", "stop", "stopped"};
    return phase < 5 ? n[phase] : "?";
}

void eng_reset(eng_t *e)
{
    uint8_t mode = e->mode; uint32_t mode_us = e->mode_us; bool mode_seen = e->mode_seen;
    uint32_t starts = e->starts, stops = e->stops, rejects = e->rejects, lr = e->last_reject;
    uint32_t aborts = e->aborts;
    uint8_t prof_req = e->prof_req, cas_flags = e->cas_flags;
    memset(e, 0, sizeof *e);
    e->mode = mode; e->mode_us = mode_us; e->mode_seen = mode_seen;
    e->prof_req = prof_req; e->prof = (uint8_t)prof_resolve(prof_req); e->pp = &eng_profiles[e->prof];
    e->aborts = aborts; e->cas_flags = cas_flags;
    e->starts = starts; e->stops = stops; e->rejects = rejects; e->last_reject = lr;
    e->out.e6_ready = 0xFF;
}

void ENG_RAM(eng_set_mode)(eng_t *e, uint8_t mode, uint32_t now_us)
{
    if (mode > ENG_MODE_FALLBACK) mode = ENG_MODE_OFF;
    e->mode = mode;
    e->mode_us = now_us;
    e->mode_seen = true;
}

void ENG_RAM(eng_set_profile)(eng_t *e, uint8_t prof)
{
    e->prof_req = prof < ENG_PROF_N ? prof : ENG_PROF_DEFAULT;
    if (e->phase != ENG_CRANK && e->phase != ENG_RUN && e->phase != ENG_STOP) {
        e->prof = (uint8_t)prof_resolve(e->prof_req);
        e->pp = &eng_profiles[e->prof];
    }
}

void ENG_RAM(eng_command)(eng_t *e, uint8_t cmd, uint32_t now_us)
{
    e->cmd = cmd;
    e->cmd_us = now_us;
}

void ENG_RAM(eng_input_stkl)(eng_t *e, uint8_t stkl, bool brake, uint32_t now_us)
{
    stkl &= 0x0F;
    if (stkl >= 0xE) return;          // 0xE/0xF = invalid / not available: ignore
    // v1h: note when the CAS ends its start request (D -> anything) during our start
    if (e->start_d && !e->d_end_have && stkl != 0xD && (e->phase == ENG_CRANK || e->phase == ENG_RUN)) {
        e->d_end_have = true;
        e->d_end_us = now_us;
        e->d_end_to = stkl;
        e->cas_flags = (uint8_t)(e->cas_flags | (e->t82_have ? ENG_CAS_D_ENDED : ENG_CAS_D_ENDED_EARLY));
    }
    e->stkl = stkl;
    e->stkl_us = now_us;
    e->stkl_have = true;
    if (brake) { e->brake_us = now_us; e->brake_have = true; }
}

void ENG_RAM(eng_input_brake)(eng_t *e, bool brake, uint32_t now_us)
{
    if (brake) { e->brake_us = now_us; e->brake_have = true; }
}

static bool ENG_RAM(brake_recent)(const eng_t *e, uint32_t now, uint32_t ms)
{
    return e->brake_have && since_ms(now, e->brake_us) <= ms;
}

// --- rpm shapes (rpm * 4) ----------------------------------------------------------
// exp(-x/10) * 1000 for x = 0..50 (tenths of the time constant)
static const uint16_t exp_tab[51] DDE_TAB_RAM = {
    1000, 905, 819, 741, 670, 607, 549, 497, 449, 407, 368, 333, 301, 273, 247, 223, 202, 183,
    165, 150, 135, 122, 111, 100, 91, 82, 74, 67, 61, 55, 50, 45, 41, 37, 33, 30, 27, 25,
    22, 20, 18, 17, 15, 14, 12, 11, 10, 9, 8, 7, 7};

static uint32_t ENG_RAM(tri)(uint32_t t_ms, uint32_t period_ms, uint32_t amp)   // 0..amp..0
{
    uint32_t ph = t_ms % period_ms, half = period_ms / 2u;
    return ph < half ? amp * ph / half : amp * (period_ms - ph) / half;
}

// Engine speed while starting / running, t = ms since the start trigger (T0).
static uint32_t ENG_RAM(rpm4_start)(const eng_prof_t *p, uint32_t t)
{
    if (t < p->rpm) return 0;
    if (t < p->fire) {
        // Cranking ~115..210 rpm with compression ripple (3krev A5: 114 -> 204 rpm)
        uint32_t base = 115u + (t - p->rpm) * 60u / (uint32_t)(p->fire - p->rpm);
        return 4u * (base + tri(t, 140u, 35u));
    }
    if (t < (uint32_t)p->fire + p->runup) {
        // First fire: run-up 200 -> 1000 rpm (3krev: 287 rpm at +0.74 s, 1005 rpm at +1.0 s)
        return 4u * (200u + (t - p->fire) * 800u / p->runup);
    }
    if (t < ENG_T_FLARE_END) return 4u * (1000u + tri(t, 300u, 60u));   // start flare 1000-1060
    // Settle to idle: 780 + 260 * exp(-(t - 1.5 s) / 1 s), then 780 rpm (30 0C) with a
    // +-1 rpm wobble so the value is alive like a real idle
    uint32_t x = (t - ENG_T_FLARE_END) / (ENG_TAU_IDLE_MS / 10u);
    uint32_t over = x > 50u ? 0u : 260u * exp_tab[x] / 1000u;
    return 4u * (ENG_IDLE_RPM - 1u + over) + tri(t, 1200u, 8u);
}

// Stop: idle held 380 ms, then the run-down captured in 3krev (stop at 19.28 s), as a
// fraction of the rpm at the stop request (per mille), t = ms since TS.
static const struct { uint16_t ms, pm; } stop_tab[] DDE_TAB_RAM = {
    {0, 1000}, {380, 1000}, {402, 912}, {542, 651}, {661, 441}, {762, 359}, {882, 92}, {1180, 77}, {1200, 0}};
#define ENG_T_STOP_ZERO 1200u

static uint32_t ENG_RAM(rpm4_stop)(uint32_t rpm4_at_stop, uint32_t t)
{
    for (unsigned k = 1; k < sizeof stop_tab / sizeof stop_tab[0]; k++) {
        if (t < stop_tab[k].ms) {
            uint32_t a = stop_tab[k - 1].ms, b = stop_tab[k].ms;
            uint32_t pa = stop_tab[k - 1].pm, pb = stop_tab[k].pm;
            uint32_t pm = pa + (uint32_t)((int32_t)(pb - pa) * (int32_t)(t - a) / (int32_t)(b - a));
            return rpm4_at_stop * pm / 1000u;
        }
    }
    return 0;
}

// --- triggers ------------------------------------------------------------------------
static void ENG_RAM(start_now)(eng_t *e, uint32_t now)
{
    e->phase = ENG_CRANK;
    e->t0 = now;
    e->cur_run = false;
    e->t9_have = false;
    e->b142_stop = false;
    e->starts++;
    e->cmd = ENG_CMD_NONE;
    e->last_reject = ENG_REJ_NONE;
    // v1h: the profile asked for by the PC applies from this start on
    e->prof = (uint8_t)prof_resolve(e->prof_req);
    e->pp = &eng_profiles[e->prof];
    e->start_d = e->stkl_have && e->stkl == 0xD;
    e->d_end_have = false;
    e->t82_have = false;
    e->cas_flags = 0;
}

static void ENG_RAM(stop_now)(eng_t *e, uint32_t now)
{
    if (!e->cur_run && e->phase == ENG_CRANK) {
        // aborted before the first rpm (first 95 ms): nothing was shown yet, just go back
        // (to "off" in a new wake, to "stopped" after an earlier run)
        e->phase = e->has_run ? ENG_STOPPED : ENG_OFF;
        e->cmd = ENG_CMD_NONE;
        return;
    }
    e->rpm_at_stop = e->out.rpm4;
    e->ts = now;
    e->phase = ENG_STOP;
    e->stops++;
    e->cmd = ENG_CMD_NONE;
}

static void ENG_RAM(reject)(eng_t *e, unsigned why)
{
    if (e->last_reject != why) e->rejects++;
    e->last_reject = why;
}

static void ENG_RAM(check_start)(eng_t *e, uint32_t now)
{
    if (e->phase != ENG_OFF && e->phase != ENG_STOPPED) return;
    bool stkl_fresh = e->stkl_have && since_ms(now, e->stkl_us) <= 300u;
    if (e->mode == ENG_MODE_CAS) {
        if (!stkl_fresh || e->stkl != 0xD) return;
        if (!brake_recent(e, now, ENG_BRAKE_MS_CAS)) { reject(e, ENG_REJ_NO_BRAKE); return; }
        start_now(e, now);
    } else if (e->mode == ENG_MODE_FALLBACK) {
        bool cmd = e->cmd == ENG_CMD_START && since_ms(now, e->cmd_us) <= ENG_CMD_MS;
        if (!cmd) return;
        // a refused command is used up: the driver has to type it again (no surprise start later)
        if (!stkl_fresh || (e->stkl != 0xA && e->stkl != 0xD)) { reject(e, ENG_REJ_STKL); e->cmd = ENG_CMD_NONE; return; }
        if (!brake_recent(e, now, ENG_BRAKE_MS_FB)) { reject(e, ENG_REJ_NO_BRAKE); e->cmd = ENG_CMD_NONE; return; }
        start_now(e, now);
    } else if (e->cmd == ENG_CMD_START) {
        reject(e, ENG_REJ_MODE);
        e->cmd = ENG_CMD_NONE;
    }
}

static void ENG_RAM(check_stop)(eng_t *e, uint32_t now)
{
    if (e->phase != ENG_CRANK && e->phase != ENG_RUN) return;
    if (e->cmd == ENG_CMD_STOP) { stop_now(e, now); return; }
    if (e->mode == ENG_MODE_OFF) { stop_now(e, now); return; }          // PC switched it off / went away
    // v1h: "never report running before START". With the short / realistic profile a start
    // that came from ST_KL D is given up when the CAS ends D before we sent 82: the CAS
    // refused or gave up on the crank, so we must not claim the engine started.
    if (e->pp->need_d && e->start_d && e->d_end_have && !e->t82_have) {
        e->aborts++;
        e->cas_flags = (uint8_t)(e->cas_flags | ENG_CAS_ABANDONED);
        stop_now(e, now);
        return;
    }
    if (!e->stkl_have || since_ms(now, e->stkl_us) > ENG_STKL_STALE_MS) { stop_now(e, now); return; }
    uint8_t k = e->stkl;
    if (k == 0xD || k >= 0xA) { e->t9_have = false; return; }   // A..D: keep going
    if (k == 9) {
        // Engine-off request (button with KL15 staying on): Ian: 0x142 b8 84 -> A4 at +50 ms,
        // 92 only when ST_KL drops further (or after 1 s of 9).
        if (!e->t9_have) { e->t9_have = true; e->t9 = now; }
        if (since_ms(now, e->t9) >= ENG_STOP_9_MS) stop_now(e, now);
        return;
    }
    stop_now(e, now);   // <= 8: ignition going off
}

// --- outputs -------------------------------------------------------------------------
static void ENG_RAM(set_outputs)(eng_t *e, uint32_t now, uint32_t dt_ms)
{
    eng_out_t *o = &e->out;
    const eng_prof_t *pp = e->pp ? e->pp : &eng_profiles[ENG_PROF_RUN8];
    eng_out_t prev = *o;
    uint32_t seq = o->seq;
    memset(o, 0, sizeof *o);
    o->seq = seq + 1u;
    o->mode = e->mode;
    o->phase = e->phase;
    o->e6_ready = 0xFF;
    o->e6_ev = prev.e6_ev;
    o->fb9_ev = prev.fb9_ev;
    o->f3_val = prev.f3_val;
    o->fuel = prev.fuel;
    if (e->phase == ENG_OFF && !e->has_run) {
        // Not started in this wake. A start that never reached first rpm (aborted in the
        // first 95 ms) leaves no trace.
        e->f3_q8 = 0;
        return;
    }
    o->active = 1;
    const uint32_t t = e->phase == ENG_CRANK || e->phase == ENG_RUN ? since_ms(now, e->t0) : 0;

    switch (e->phase) {
    case ENG_CRANK:
    case ENG_RUN: {
        o->rpm4 = (uint16_t)rpm4_start(pp, t);
        if (o->rpm4 && !e->cur_run) { e->cur_run = true; e->has_run = true; e->trpm = now; }
        if (!e->cur_run && !e->has_run) { o->active = 0; }
        // Crank marker: CounterHi 1 -> 3 on all DDE E2E groups (FR + PT-CAN D9/3F9), Ian +50..+155 ms
        if (t >= pp->blip_on && t < pp->blip_off) { o->cnt_hi = 3; o->active = 1; }
        if (e->phase == ENG_CRANK && o->rpm4 >= 4u * ENG_RPM_82) {
            e->phase = ENG_RUN; e->t82 = now; o->phase = ENG_RUN; e->t82_have = true;
        }
        // v1h: the CAS still asks for the starter 300 ms after our "running": say so
        if (e->phase == ENG_RUN && e->start_d && !e->d_end_have && since_ms(now, e->t82) >= 300u)
            e->cas_flags = (uint8_t)(e->cas_flags | ENG_CAS_D_LONG);
        // v1h: 0x3FB d4 bit0 0 = injecting (4log_3krev: f1 -> f0 at D+179 ms, run 8 stayed f1)
        if (t >= pp->inj) o->inj = 1;
        uint32_t tr = e->cur_run ? since_ms(now, e->trpm) : 0;     // ms since first rpm
        if (e->phase == ENG_CRANK) {
            // Ian: +100 ms state 0x81 (cranking), b4 0x65, start-ready 1 (b7 0x13)
            if (t >= pp->t81 || e->cur_run) { o->e6_b3 = 0x81; o->e6_b4 = 0x65; o->e6_ready = 1; }
            // torque: crank ~ +220 Nm (3krev A5 0x9B5), run-up falls to the idle level
            if (t < pp->fire) o->torque = 2560;   // Fred: crank raw 2540-2608
            else o->torque = (uint16_t)(2560u - (t - pp->fire) * 420u / pp->runup);
            // supply dips while the starter turns: FB7 ~10.6 V (Fred), 0x281 0x2C1 (3krev)
            o->fb7_mv = 10600; o->v281 = 0x2C1;
        } else {
            uint32_t t82 = since_ms(now, e->t82);
            // Ian/Fred: 0xE6 = 0x82 "running"; b4 0x66 for 250 ms then 0x26; start-ready 0 20 ms later.
            // Owner: CAS likely waits for 0x82 before releasing the starter (D -> C).
            // v1h: b4 26 from the first 82 frame (all 6 real starts); start-ready 0 together
            // with 82 (short) or 15 ms later (realistic; real 82+11..20). RUN8 keeps v1g.2.
            o->e6_b3 = 0x82;
            o->e6_b4 = (pp->b4_26 || t82 >= 250u) ? 0x26 : 0x66;
            o->e6_ready = t82 < pp->ready_off ? 1 : 0;
            o->torque = t < ENG_T_FLARE_END ? (uint16_t)2140 : (uint16_t)2098;   // idle 25.5 Nm = 32 28 83
            o->b142_b8 = 0x84;                       // Fred: 0x142 b8 A4 -> 84 while running
            o->r381_run = 1;
            o->fb7_mv = (uint16_t)(12400u + (t82 > 5000u ? 2300u : t82 * 2300u / 5000u));   // -> 14.70 V
            o->v281 = (uint16_t)(t82 > 1000u ? 0x339 : 0x2C1 + (0x339 - 0x2C1) * t82 / 1000u);
            // 0x397 (3krev): d0 c2, d1 e2 -> e1 (+60) -> dd (+120), d2 random 0x5f..0x79 / 60 ms
            o->r397_set = 1;
            o->r397_d0 = tr >= 2800u ? 0xD2 : 0xC2;
            o->r397_d1 = t82 < 60u ? 0xE2 : (t82 < 120u ? 0xE1 : 0xDD);
            uint32_t h = (now / 60000u) * 2654435761u;
            o->r397_d2 = (uint8_t)(0x5F + (h >> 24) % 27u);
            if (e->t9_have && since_ms(now, e->t9) >= 50u) o->b142_b8 = 0xA4;   // ST_KL 9: Ian +50 ms
        }
        if (e->cur_run) {
            // Fred: 0x28 c3 b5b6 24 20 at first rpm, 20 20 0.1 s later; b3..4 3E 7D
            o->c3_b56 = tr < 100u ? 1 : 2;
            o->c3_b34 = 1;
            // Fred/Ian: 0x29 b5 / PT-CAN 0x8F d4 0x7C -> 0x72 at first rpm (+7 ms)
            if (tr >= 7u) o->r29_b5 = 0x72;
            o->r29_b8 = 0x10;
            if (tr >= 100u) o->f97c = 1;           // Ian +170 ms: sensor values valid (sticky)
            if (tr >= 50u) o->b142_b23 = 0x24;     // Fred: 0x142 b23 20 -> 24 (sticky)
            // FB9 (PT-CAN 0x3B3 mirror): b2 ramps 0x2D -> 0xC8 from +2.0 s (+4 per 100 ms),
            // b3..4 04 00 -> 00 00 at +5.0 s (Fred; 3krev 0x3B3 +5.1 s)
            o->fb9_set = 1;
            o->fb9_b2 = 0;
            if (tr >= 2000u) {
                uint32_t v = 0x2Du + 4u * ((tr - 2000u) / 100u);
                o->fb9_b2 = (uint8_t)(v > 0xC8u ? 0xC8u : v);
            }
            o->fb9_b3 = tr >= 5000u ? 0x00 : 0x04;
            o->fb9_b4 = 0x00;
            // E8 L9 running content from +2.8 s (Fred), b1 ramp 12 18 1E 25, fast rate
            if (tr >= 2800u) {
                uint32_t s = (tr - 2800u) / 100u;
                o->e89 = 1;
                o->e89_b1 = s == 0 ? 0x12 : (s == 1 ? 0x18 : (s == 2 ? 0x1E : 0x25));
                o->e8_fast = 1;
            }
            o->turbine = (uint16_t)(2000u + (o->rpm4 / 4u > 56u ? o->rpm4 / 4u - 56u : 0u));
        } else if (e->has_run) {
            // restart after an earlier run, before this start's first rpm: the "after stop"
            // values stay (they only change again at first rpm)
            o->c3_b56 = 3; o->c3_b34 = 2;
            o->r29_b5 = 0x7C; o->r29_b8 = 0xC0; o->f97c = 1;
            o->b142_b8 = 0xA4; o->b142_b23 = 0x24;
            o->fb9_set = 1; o->fb9_b2 = 0x00; o->fb9_b3 = 0xFF; o->fb9_b4 = 0x7F;
            o->e89 = 2;
            o->turbine = 2000;
            if (!o->e6_b3) { o->e6_b3 = 0x80; o->e6_b4 = 0x64; o->e6_ready = 1; }
            if (!o->torque) o->torque = 1978;
        }
        if (o->rpm4 || e->cur_run) o->r381_run = o->r381_run || o->rpm4;
        // v1h: real DDE sends 0xE8 L9 4-7 times a second from D until the fast rate (Fred)
        if (pp->e8_crank && !o->e8_fast) o->e8_crank = 1;
        break;
    }
    case ENG_STOP: {
        uint32_t ts = since_ms(now, e->ts);
        o->rpm4 = (uint16_t)rpm4_stop(e->rpm_at_stop, ts);
        // Ian: +19 ms state 0x92 (stopping), b4 0x66, start-ready 0
        o->e6_b3 = ts >= pp->t92 ? 0x92 : 0x82;   // v1h: real 0x92 within 20 ms of ST_KL 8
        o->e6_b4 = 0x66;
        o->e6_ready = 0;
        o->torque = ts < 400u ? 2098 : 1947;      // Ian: -50 Nm drag torque while running down
        o->b142_b8 = 0xA4;
        o->c3_b56 = ts >= pp->t92 ? 3 : 2;        // Fred: E0 E0 with 92
        o->c3_b34 = 1;
        o->r29_b5 = ts >= pp->t92 ? 0x7C : 0x72;  // Fred/Ian: back to 7C, b8 C0 (+35 ms)
        // 4log_3krev: 0x3FB d4 back to f1 ~360 ms after 92 (run-down)
        o->inj = (pp->inj != 0xFFFFu && ts < 360u) ? 1 : 0;
        o->r29_b8 = ts >= 35u ? 0xC0 : 0x10;
        o->f97c = 1;
        o->b142_b23 = 0x24;
        o->fb9_set = 1;
        o->fb9_b2 = prev.fb9_set ? prev.fb9_b2 : 0;
        o->fb9_b3 = ts >= 240u ? 0xFF : prev.fb9_b3;   // 3krev 0x3B3: FF 7F at +0.24 s
        o->fb9_b4 = ts >= 240u ? 0x7F : prev.fb9_b4;
        o->e89 = prev.e89; o->e89_b1 = prev.e89_b1; o->e8_fast = prev.e8_fast;
        o->fb7_mv = 14700;
        o->v281 = 0x339;
        o->r381_run = 1;
        o->turbine = (uint16_t)(2000u + (o->rpm4 / 4u > 56u ? o->rpm4 / 4u - 56u : 0u));
        // 0x397 (3krev): dd until 92 + 120 ms, then a2, 2e (+60), 24 (+120); d2 -> 00 at +440 ms
        o->r397_set = 1;
        o->r397_d0 = since_ms(now, e->trpm) >= 2800u ? 0xD2 : 0xC2;
        o->r397_d1 = ts < 140u ? 0xDD : (ts < 200u ? 0xA2 : (ts < 260u ? 0x2E : 0x24));
        o->r397_d2 = ts < 440u ? (uint8_t)(0x5F + (((now / 60000u) * 2654435761u) >> 24) % 27u) : 0x00;
        if (o->rpm4 == 0 && ts >= ENG_T_STOP_ZERO) { e->phase = ENG_STOPPED; e->t80 = now; o->phase = ENG_STOPPED; }
        else break;
    }
    /* fall through */
    case ENG_STOPPED: {
        uint32_t t80 = since_ms(now, e->t80);
        o->rpm4 = 0;
        // Ian: state 0x80, b4 0x64, start-ready 1 (b7 0x13) 20 ms later; T1 -34.5 Nm (raw 1978)
        o->e6_b3 = 0x80; o->e6_b4 = 0x64; o->e6_ready = t80 >= 20u ? 1 : 0;
        o->torque = 1978;
        o->turbine = 2000;
        o->b142_b8 = 0xA4; o->b142_b23 = 0x24;
        o->c3_b56 = 3; o->c3_b34 = 2;             // Fred: b3..4 1C 7E after stop
        o->r29_b5 = 0x7C; o->r29_b8 = 0xC0; o->f97c = 1;
        o->fb9_set = 1; o->fb9_b2 = 0x00; o->fb9_b3 = 0xFF; o->fb9_b4 = 0x7F;
        o->e89 = 2; o->e8_fast = t80 < 1500u;     // Fred: back to ~1/s 1.5 s after the stop
        o->fb7_mv = (uint16_t)(t80 >= 5000u ? 0 : 14700u - (14700u - 12871u) * t80 / 5000u);
        // 0x397: d0 d0, d1 24 held until +2.09 s, then 29 af df de dd df e2 (3krev)
        o->r397_set = 1;
        o->r397_d0 = 0xD0;
        o->r397_d2 = 0x00;
        static const struct { uint16_t ms; uint8_t v; } walk[] = {
            {0, 0x24}, {2090, 0x29}, {2150, 0xAF}, {2210, 0xDF}, {2270, 0xDE}, {2370, 0xDD},
            {2854, 0xDF}, {2913, 0xE2}};
        uint8_t d1 = 0x24;
        for (unsigned k = 0; k < sizeof walk / sizeof walk[0]; k++) if (t80 >= walk[k].ms) d1 = walk[k].v;
        o->r397_d1 = d1;
        o->abs_stop_walk = t80 >= 2913u;
        o->r381_run = 0;
        o->v281 = 0;
        break;
    }
    default:
        break;
    }

    // 0xF3: rpm/10 with ~0.2 s lag (3krev: 66 = 660 rpm when the A5 shows 1000)
    {
        uint32_t target = (uint32_t)o->rpm4 * 256u / 40u;     // (rpm/10) * 256
        int32_t diff = (int32_t)target - (int32_t)e->f3_q8;
        e->f3_q8 = (uint32_t)((int32_t)e->f3_q8 + diff * (int32_t)(dt_ms > 200u ? 200u : dt_ms) / 200);
        if (o->rpm4 == 0 && e->f3_q8 < 256u) e->f3_q8 = 0;
        o->f3_val = (uint16_t)(e->f3_q8 / 256u);
    }
    // 0x2C4 fuel counter (3krev: ~0x0E / 100 ms cranking, ~0x28 / 100 ms at idle)
    if (o->rpm4) {
        uint32_t rpm = o->rpm4 / 4u;
        uint32_t per100 = (e->phase == ENG_CRANK && rpm < 400u) ? rpm / 10u : rpm / 20u;   // per 100 ms
        e->fuel_q10 += per100 * dt_ms * 10u / 100u;   // 1/10 units
        o->fuel = (uint16_t)(e->fuel_q10 / 10u);
    }
    // event flags
    uint8_t e6[3] = {o->e6_b3, o->e6_b4, o->e6_ready};
    if (memcmp(e6, e->last_e6, 3) != 0) { o->e6_ev++; memcpy(e->last_e6, e6, 3); }
    uint8_t f9[3] = {o->fb9_b2, o->fb9_b3, o->fb9_b4};
    if (o->fb9_set && memcmp(f9, e->last_fb9, 3) != 0) { o->fb9_ev++; memcpy(e->last_fb9, f9, 3); }
}

void ENG_RAM(eng_step)(eng_t *e, uint32_t now)
{
    uint32_t dt = e->last_have ? since_ms(now, e->last_us) : 5u;
    if (dt > 1000u) dt = 1000u;
    e->last_us = now; e->last_have = true;
    if (e->mode != ENG_MODE_OFF && (!e->mode_seen || since_ms(now, e->mode_us) > ENG_MODE_KEEP_MS))
        e->mode = ENG_MODE_OFF;                    // PC (logger) gone: no keepalive
    if (e->cmd != ENG_CMD_NONE && since_ms(now, e->cmd_us) > ENG_CMD_MS) e->cmd = ENG_CMD_NONE;
    check_stop(e, now);
    check_start(e, now);
    set_outputs(e, now, dt);
}

// --- byte patchers ---------------------------------------------------------------------
static void ENG_RAM(hi3)(uint8_t *b, uint8_t hi) { *b = (uint8_t)((*b & 0x0Fu) | (uint8_t)(hi << 4)); }
static void ENG_RAM(put_torque)(uint8_t *b, uint16_t t)   // b[0..2] = T1 (12 bit) + T2 (= T1), Ian
{
    b[0] = (uint8_t)t;
    b[1] = (uint8_t)(((t >> 8) & 0x0Fu) | ((t & 0x0Fu) << 4));
    b[2] = (uint8_t)(t >> 4);
}

static void ENG_RAM(patch_fr_static_engine)(const eng_out_t *o, unsigned target, unsigned cycle, uint8_t *pl)
{
    if (!o || !o->active) return;
    const unsigned c4 = cycle & 3u;
    switch (target) {
    case 0:   // 0x26 c1: b3..4 turbine speed (Fred: 2000 + rpm - 56 when running)
        if (c4 == 1 && o->turbine) { pl[2] = (uint8_t)o->turbine; pl[3] = (uint8_t)(o->turbine >> 8); }
        break;
    case 1:   // 0x28
        if (c4 == 1) {
            if (o->torque) put_torque(&pl[2], o->torque);          // b3..5 T1/T2
            pl[5] = (uint8_t)o->rpm4; pl[6] = (uint8_t)(o->rpm4 >> 8);   // b6..7 rpm*4 (= OBD 0x0C)
            if (o->cnt_hi) hi3(&pl[9], o->cnt_hi);                 // group 2 CounterHi
        } else if (c4 == 3) {
            if (o->cnt_hi) { hi3(&pl[1], o->cnt_hi); hi3(&pl[9], o->cnt_hi); }
            if (o->c3_b34 == 1) { pl[2] = 0x3E; pl[3] = 0x7D; }
            else if (o->c3_b34 == 2) { pl[2] = 0x1C; pl[3] = 0x7E; }
            if (o->c3_b56 == 1) { pl[4] = 0x24; pl[5] = 0x20; }
            else if (o->c3_b56 == 2) { pl[4] = 0x20; pl[5] = 0x20; }
            else if (o->c3_b56 == 3) { pl[4] = 0xE0; pl[5] = 0xE0; }
        }
        break;
    case 2:   // 0x29 c3
        if (c4 == 3) {
            if (o->cnt_hi) { hi3(&pl[1], o->cnt_hi); hi3(&pl[9], o->cnt_hi); }
            if (o->f97c) {
                pl[2] = 0xF9; pl[3] = 0x7C;
                pl[10] = 0xF9; pl[11] = 0x7C; pl[12] = 0xF9; pl[13] = 0x7C; pl[14] = 0xF9; pl[15] = 0x7C;
            }
            if (o->r29_b5) pl[4] = o->r29_b5;
            if (o->r29_b8) pl[7] = o->r29_b8;
        }
        break;
    case 3:   // 0x3D c3
        if (c4 == 3) {
            if (o->cnt_hi) { hi3(&pl[1], o->cnt_hi); hi3(&pl[9], o->cnt_hi); }
            if (o->f97c) { pl[4] = 0xF9; pl[5] = 0x7C; pl[6] = 0xF9; pl[7] = 0x7C; }
        }
        break;
    default:
        break;
    }
}

static void ENG_RAM(patch_fr_dyn_engine)(const eng_out_t *o, unsigned pdu, uint8_t *pl)
{
    if (!o || !o->active) return;
    switch (pdu) {
    case DDE_PDU_E6:   // b2 hi = CounterHi, b3 state, b4 phase, b7 hi = start-ready
        if (o->cnt_hi) hi3(&pl[1], o->cnt_hi);
        if (o->e6_b3) { pl[2] = o->e6_b3; pl[3] = o->e6_b4; }
        if (o->e6_ready != 0xFF) hi3(&pl[6], o->e6_ready);
        break;
    case DDE_PDU_FB7:  // b3..4 supply voltage mV LE
        if (o->fb7_mv) { pl[2] = (uint8_t)o->fb7_mv; pl[3] = (uint8_t)(o->fb7_mv >> 8); }
        break;
    case DDE_PDU_FB9:
        if (o->fb9_set) { pl[1] = o->fb9_b2; pl[2] = o->fb9_b3; pl[3] = o->fb9_b4; }
        break;
    case DDE_PDU_E89: {
        // Fred: running `25 14 00 AC 26 F3 58 F0` (b1 ramps up first); b1b2 back to 00 with E6=80
        static const uint8_t run[8] DDE_TAB_RAM = {0x25, 0x14, 0x00, 0xAC, 0x26, 0xF3, 0x58, 0xF0};
        static const uint8_t aft[8] DDE_TAB_RAM = {0x00, 0x00, 0x00, 0xAC, 0x26, 0xF3, 0x58, 0xF0};
        // v1h.2: byte loop, not memcpy: an 8-byte memcpy becomes loads from a flash literal pool
        if (o->e89 == 1) { for (unsigned i = 0; i < 8; i++) { pl[i] = run[i]; __asm__ volatile("" ::: "memory"); } pl[0] = o->e89_b1; }
        else if (o->e89 == 2) { for (unsigned i = 0; i < 8; i++) { pl[i] = aft[i]; __asm__ volatile("" ::: "memory"); } }
        break;
    }
    case DDE_PDU_142:
        if (o->b142_b8) pl[7] = o->b142_b8;
        if (o->b142_b23) pl[22] = o->b142_b23;
        break;
    default:
        break;
    }
}

static bool ENG_RAM(patch_can_engine)(const eng_out_t *o, uint16_t id, uint8_t *d, uint8_t *ctr_hi)
{
    if (!o || !o->active) return false;
    switch (id) {
    case 0x0A5:        // torque d2..4 + rpm d5..6 (same encoding as FR 0x28 c1)
        if (o->torque) put_torque(&d[2], o->torque);
        d[5] = (uint8_t)o->rpm4; d[6] = (uint8_t)(o->rpm4 >> 8);
        return true;
    case 0x0D9:
        if (o->cnt_hi) { *ctr_hi = o->cnt_hi; return true; }
        return false;
    case 0x3F9:        // PT-CAN copy of 0xE6: d2 state, d3 phase, d6 hi start-ready
        if (o->cnt_hi) *ctr_hi = o->cnt_hi;
        if (o->e6_b3) { d[2] = o->e6_b3; d[3] = o->e6_b4; }
        if (o->e6_ready != 0xFF) hi3(&d[6], o->e6_ready);
        return true;
    case 0x08F:        // copy of FR 0x29: d2..3 F9 7C, d4 run nibble
        if (o->f97c) { d[2] = 0xF9; d[3] = 0x7C; }
        if (o->r29_b5) d[4] = o->r29_b5;
        return true;
    case 0x0F3:        // rpm/10 lagged: b1 hi nibble = low 4 bits, b2 = high 8 bits
        *ctr_hi = (uint8_t)(o->f3_val & 0x0Fu);
        d[2] = (uint8_t)(o->f3_val >> 4);
        return true;
    case 0x2C4:
        d[0] = (uint8_t)o->fuel; d[1] = (uint8_t)(o->fuel >> 8);
        return true;
    case 0x281:
        if (!o->v281) return false;
        d[0] = (uint8_t)o->v281; d[1] = (uint8_t)(0xF0u | ((o->v281 >> 8) & 0x0Fu));
        return true;
    case 0x381:
        if (!o->r381_run) return false;
        d[0] = 0x79; d[1] = 0xF0;
        return true;
    case 0x3B3:        // mirror of FB L9 (`1f b2 b3 b4 00 03 f8`)
        if (!o->fb9_set) return false;
        d[1] = o->fb9_b2; d[2] = o->fb9_b3; d[3] = o->fb9_b4;
        return true;
    case 0x3FB:        // v1h: d4 bit0 0 while "injecting" (gateway copies d4 into FR 0xE9 b5)
        if (!o->inj) return false;
        d[4] = (uint8_t)(d[4] & 0xFEu);
        return true;
    case 0x397:
        if (!o->r397_set || o->abs_stop_walk) {
            if (o->abs_stop_walk) d[0] = 0xD0;
            return o->abs_stop_walk;
        }
        d[0] = o->r397_d0; d[1] = o->r397_d1; d[2] = o->r397_d2;
        return true;
    default:
        return false;
    }
}

// ---------------------------------------------------------------- v1h.1 gear bytes
// The gear (dde_gear.c) goes in AFTER the engine bytes (the engine writes 0x3F9 d3 /
// 0xE6 b4 whole; the gear only owns bits 4-5 of it and the low nibble of d6 / b7).
// g_set 0 = fixed P = nothing changes = exactly the v1h content.
void ENG_RAM(eng_patch_fr_static)(const eng_out_t *o, unsigned target, unsigned cycle, uint8_t *pl)
{
    if (!o) return;
    patch_fr_static_engine(o, target, cycle, pl);
    // 0x28 c3 b11 / b14 = FR copy of PT-CAN 0xDC d2 / d5 (inside E2E group 2, CRC computed later)
    if (o->g_set && target == 1 && (cycle & 3u) == 3u) { pl[10] = o->g_dc_b2; pl[13] = o->g_dc_b5; }
}

void ENG_RAM(eng_patch_fr_dyn)(const eng_out_t *o, unsigned pdu, uint8_t *pl)
{
    if (!o) return;
    patch_fr_dyn_engine(o, pdu, pl);
    if (o->g_set && pdu == DDE_PDU_E6) {   // b4 bits 4-5 P lock, b7 low nibble gear (= 0x3F9 d3 / d6)
        pl[3] = (uint8_t)((pl[3] & 0xCFu) | ((o->g_fr_park & 3u) << 4));   // v1h.2: no "16" in transit
        pl[6] = (uint8_t)((pl[6] & 0xF0u) | (o->g_nib & 0x0Fu));
    }
}

bool ENG_RAM(eng_patch_can)(const eng_out_t *o, uint16_t id, uint8_t *d, uint8_t *ctr_hi)
{
    if (!o) return false;
    bool ch = patch_can_engine(o, id, d, ctr_hi);
    if (!o->g_set) return ch;
    switch (id) {
    case 0x3FD:        // d2 gear display; only while the display is on (0 = off before KL15 / after)
        if (d[2] == 0) return ch;
        d[2] = o->g_3fd;
        return true;
    case 0x39A:        // d3 gear code, d4 P lock
        d[3] = o->g_39a_d3; d[4] = o->g_39a_d4;
        return true;
    case 0x3F9:        // d3 bits 4-5 (bits 28-29) P lock, d6 low nibble gear
        d[3] = (uint8_t)((d[3] & 0xCFu) | ((o->g_park & 3u) << 4));
        d[6] = (uint8_t)((d[6] & 0xF0u) | (o->g_nib & 0x0Fu));
        return true;
    case 0x0DC:        // d2 gear, d5 (d3..4 d0 07 kept: we are standing still)
        d[2] = o->g_dc_b2; d[5] = o->g_dc_b5;
        return true;
    default:
        return ch;
    }
}

// ---------------------------------------------------------------- v1h.2 snapshot reuse
const eng_out_t *eng_snap_select(eng_snap_cache_t *c, bool read_ok, const eng_out_t *fresh, uint32_t fresh_us,
                                 uint32_t now_us)
{
    if (read_ok) {
        c->last = *fresh; c->last_us = fresh_us; c->have = true;
    } else {
        if (!c->have) return 0;   // nothing published yet (not a failure)
        c->fails++;
    }
    if ((uint32_t)(now_us - c->last_us) > 500000u) return 0;   // FlexRay not stepping for 500 ms
    return c->last.active ? &c->last : 0;
}
