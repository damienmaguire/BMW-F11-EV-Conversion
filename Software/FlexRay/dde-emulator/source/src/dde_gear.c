// v1h.1 gear selection. See dde_gear.h for the plain-English overview.
// Sources: Ian's flexray-re-notes/analysis/egs_gear_report_tx.md/.csv (source of truth) and
// egs-logs/F11_4log_r10md10m.csv (real DDE + EGS, engine running, PT-CAN = bus 3):
//   P -> R  forward 1, 0x39A E1 +20 ms, 0x3FD 60 (N) +40 ms (only because forward 1 was held
//           61 ms), forward 2 -> 0x3FD 40 (R) +40 ms, 0x39A 02 21 ms after 0x3FD
//   R -> D  back 1 held 21 ms (skipped, no N shown), back 2 -> 0x3FD 80 (D) +40 ms
//   D -> P  P button; 0x39A gear 01 +21 ms, 0x3FD 20 (P) +900 ms, pawl (0x39A E2,
//           0x3F9 bits 28-29 = 2) +3.0 s (brake released +2.6 s)
#include "dde_gear.h"
#include <stdio.h>
#include <string.h>

#if defined(__arm__)
#include "pico.h"
#define GEAR_RAM(f) __time_critical_func(f)
#else
#define GEAR_RAM(f) f
#endif

static uint32_t GEAR_RAM(ms_since)(uint32_t now, uint32_t t) { return (now - t) / 1000u; }

// The bytes the real EGS/DDE send in each gear: 0x3FD d2, 0x39A d3, d4, park bits, gear nibble, 0xDC d2, d5.
typedef struct { uint8_t b3fd, a3, a4, park, nib, dc2, dc5; } gear_bytes_t;
static const gear_bytes_t gb_gear[4] = {
    [GEAR_P] = {0x20, 0x03, 0xE2, 2, 3, 0x22, 0xFE},   // capture, P with pawl (= v1h template)
    [GEAR_R] = {0x40, 0x02, 0xE0, 0, 2, 0x25, 0xF4},   // capture
    [GEAR_N] = {0x60, 0x01, 0xE0, 0, 1, 0x22, 0xFE},   // 0x3FD 60 capture; the rest HYPOTHESIS (Ian)
    [GEAR_D] = {0x80, 0x05, 0xE0, 0, 5, 0x24, 0xF4},   // capture
};
static const gear_bytes_t gb_p_unlocked = {0x20, 0x03, 0xE0, 0, 3, 0x22, 0xFE};   // P shown, pawl not yet confirmed

const char *gear_name(unsigned gear)
{
    static const char n[4][2] = {"P", "R", "N", "D"};
    return gear < 4 ? n[gear] : "?";
}

static bool brake_on(const gear_t *g)
{
    return g->brake_have && ms_since(g->now, g->brake_us) <= GEAR_BRAKE_MS;
}

static void GEAR_RAM(log_ev)(gear_t *g, uint8_t kind, uint8_t from, uint8_t to, uint8_t why)
{
    memmove(&g->ev[0], &g->ev[1], sizeof(gear_ev_t) * (GEAR_N_EV - 1));
    gear_ev_t *e = &g->ev[GEAR_N_EV - 1];
    e->seq = ++g->ev_seq;
    e->kind = kind;
    e->from_to = (uint8_t)((from & 0x0Fu) | (to << 4));
    e->why = why;
    e->flags = (uint8_t)((brake_on(g) ? 1u : 0u) | (g->running ? 2u : 0u) | (uint8_t)(g->act_why << 4));
    e->t_ms = (uint16_t)(g->now / 1000u);
    if (kind == GEV_CHANGE) g->changes++;
    if (kind == GEV_REFUSED) g->refusals++;
}

void gear_reset(gear_t *g, bool enabled)
{
    uint16_t seq = g->ev_seq, ch = g->changes, rf = g->refusals;
    uint32_t lf = g->lever_frames, lb = g->lever_bad;
    gear_ev_t ev[GEAR_N_EV];
    memcpy(ev, g->ev, sizeof ev);
    memset(g, 0, sizeof *g);
    // counters and the event list survive a new wake (the logger reads them)
    g->ev_seq = seq; g->changes = ch; g->refusals = rf; g->lever_frames = lf; g->lever_bad = lb;
    memcpy(g->ev, ev, sizeof ev);
    g->enabled = EMU_GEAR && enabled;
    g->target = g->from = GEAR_P;
    g->stage = GST_P_LOCKED;
    g->lv = 0xFF;
}

void gear_set_enabled(gear_t *g, bool on, uint32_t now_us)
{
    on = EMU_GEAR && on;
    if (on == g->enabled) return;
    g->now = now_us;
    uint8_t was = g->target;
    g->enabled = on;
    // off: straight back to the fixed v1h P content
    if (!on) { g->target = g->from = GEAR_P; g->stage = GST_P_LOCKED; }
    log_ev(g, GEV_ONOFF, was, g->target, on ? GWHY_ENABLED : GWHY_DISABLED);
}

void GEAR_RAM(gear_input_brake)(gear_t *g, uint32_t now_us)
{
    g->brake_have = true;
    g->brake_us = now_us;
}

// Start a transition from the gear we are in (or heading to) to 'to'.
static void GEAR_RAM(start)(gear_t *g, uint8_t to, uint8_t why)
{
    uint8_t cur = g->target;
    log_ev(g, GEV_CHANGE, cur, to, why);
    g->target = to;
    g->t_act = g->now;
    if (to == GEAR_P) {                                   // P button / engine off
        g->from = cur;
        g->stage = GST_TO_P; g->t_stage = g->now;
    } else if (g->stage == GST_P_LOCKED || g->stage == GST_P_UNLOCKED || g->stage == GST_TO_P ||
               g->stage == GST_P_RELEASED) {
        g->from = GEAR_P;
        g->stage = GST_LEAVE_P; g->t_stage = g->now;      // new gear on 0x3FD now, gear code 10 ms later
    } else if (g->stage == GST_LEAVE_P) {
        g->t_stage = g->now;                              // 2nd notch (P -> N -> R): R now, code 10 ms later
    } else {
        g->from = to;
        g->stage = GST_STEADY;                           // R / N / D: shown straight away
    }
}

// One lever action. dir 1 = forward, 2 = back (depth 1/2 notches); dir 3 = P button.
// The EGS table (Ian, egs_gear_report_tx.md):
//              forward 1   forward 2   back 1   back 2    P button
//   from P     N           R           D        D         (stays P)
//   from R     -           -           N        D         P
//   from N     R           R           D        D         P
//   from D     N           R           -        -         P
//   Out of P, and P/N -> R/D: engine running AND brake pressed, else refused (logged).
static void GEAR_RAM(gear_act)(gear_t *g, uint8_t dir, uint8_t depth)
{
    const uint8_t cur = g->target;
    uint8_t want = 0xFF, why;
    if (dir == 3) {
        why = GWHY_PBTN;
        if (cur != GEAR_P) want = GEAR_P;
    } else if (dir == 1) {
        why = depth >= 2 ? GWHY_FWD2 : GWHY_FWD1;
        switch (cur) {
        case GEAR_P: want = depth >= 2 ? GEAR_R : GEAR_N; break;
        case GEAR_N: want = GEAR_R; break;
        case GEAR_D: want = depth >= 2 ? GEAR_R : GEAR_N; break;
        default: break;                                    // R: nothing further forward
        }
    } else {
        why = depth >= 2 ? GWHY_BACK2 : GWHY_BACK1;
        switch (cur) {
        case GEAR_P: want = GEAR_D; break;
        case GEAR_N: want = GEAR_D; break;
        case GEAR_R: want = depth >= 2 ? GEAR_D : GEAR_N; break;
        default: break;                                    // D: nothing further back
        }
    }
    if (want == 0xFF || want == cur) return;              // nothing to do (already there)
    g->act_why = why;                                      // the lever move, for refusal texts
    if (!g->enabled) return;                               // fixed P (logged once when switched off)
    if (want != GEAR_P) {
        if (!g->running) { log_ev(g, GEV_REFUSED, cur, want, GREF_NOT_RUNNING); return; }
        // the brake: leaving P, and P/N -> R/D. The 2nd notch of the same push while we are
        // still releasing P (P -> N -> R) was already checked at the first notch.
        bool same_push = g->stage == GST_LEAVE_P;
        bool need = cur == GEAR_P || (cur == GEAR_N && (want == GEAR_R || want == GEAR_D) && !same_push);
        if (need && !brake_on(g)) { log_ev(g, GEV_REFUSED, cur, want, GREF_NO_BRAKE); return; }
    }
    start(g, want, why);
}

// PT-CAN 0x197 (GWS, 22/38 ms). B0 E2E CRC8 0x1D (data ID 0x3F, 4 bytes), B1 low = alive 0..14,
// B2 high nibble = lever (0 centre, 1 fwd 1, 2 fwd 2, 3 back 1, 4 back 2; low nibble masked),
// B3 = P button (C0 released, D5 pressed).
void GEAR_RAM(gear_input_lever)(gear_t *g, const uint8_t *d, uint8_t len, uint32_t now_us)
{
    g->now = now_us;
    if (len < 4) { g->lever_bad++; return; }
    static const uint8_t id[2] = {0x3F, 0x00};
    uint8_t crc = 0;
    for (unsigned i = 0; i < 5u; i++) {
        crc ^= i < 2 ? id[i] : d[i - 1];
        for (int b = 0; b < 8; b++) crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x1D : crc << 1);
    }
    if (crc != d[0]) { g->lever_bad++; return; }
    g->lever_frames++;
    if (g->lv_lost) { g->lv_lost = false; log_ev(g, GEV_LEVER_OK, g->target, g->target, 0); }
    const uint8_t pos = (uint8_t)((d[2] & 0xF0u) >> 4);
    const uint8_t pb = d[3] & 0x3Fu;
    g->lv_have = true; g->lv_us = now_us; g->lv = pos;
    g->pbtn = pb == 0x15u;
    // lever: remember the deepest notch of this push; gear_step() acts on it 40 ms after it appeared
    if (pos == LV_CENTRE) {
        g->gest_dir = 0; g->gest_depth = 0;
    } else if (pos >= LV_FWD1 && pos <= LV_BACK2) {
        uint8_t dir = pos <= LV_FWD2 ? 1 : 2, depth = (pos == LV_FWD2 || pos == LV_BACK2) ? 2 : 1;
        bool new_push = g->gest_dir == 0 && g->pend_dir == 0;
        bool deeper = (g->pend_dir ? g->pend_dir : g->gest_dir) == dir &&
                      depth > (g->pend_dir ? g->pend_depth : g->gest_depth);
        if (new_push || deeper) {
            if (g->pend_dir == 0) g->pend_us = now_us;   // a deeper notch replaces a waiting one
            else if (deeper) g->pend_us = now_us;
            g->pend_dir = dir; g->pend_depth = depth;
            if (new_push) { g->push_us = now_us; g->push_rel = false; }   // v1h.2: P release +20 ms
            if (g->gest_dir == 0) g->gest_dir = dir;       // push in progress
        }
    }   // other values (side gate M/S, +/-; not on this car): ignored
    // P button: v1h.2 acts on the first pressed frame (CRC-checked; the real EGS answers
    // 21 ms after it, v1h.1 waited for a 2nd frame = +20..38 ms), once per press
    if (pb == 0x15u) {
        if (g->pbtn_cnt < 255) g->pbtn_cnt++;
        if (g->pbtn_cnt >= 1 && !g->pbtn_done) { g->pbtn_done = true; gear_act(g, 3, 0); }
    } else if (pb == 0x00u) {
        g->pbtn_cnt = 0; g->pbtn_done = false;
    }
}

void GEAR_RAM(gear_step)(gear_t *g, uint32_t now_us, bool running)
{
    g->now = now_us;
    g->running = running;
    if (g->lv_have && ms_since(now_us, g->lv_us) > GEAR_LEVER_STALE_MS) {
        // lever timeout: hold the gear, forget the push, say so once
        g->lv_have = false; g->lv = 0xFF; g->gest_dir = 0; g->gest_depth = 0; g->pend_dir = 0;
        g->pbtn_cnt = 0;
        if (!g->lv_lost) { g->lv_lost = true; log_ev(g, GEV_LEVER_LOST, g->target, g->target, 0); }
    }
    // the waiting notch is due (40 ms after it first appeared)
    if (g->pend_dir && ms_since(now_us, g->pend_us) >= GEAR_ACT_MS) {
        uint8_t dir = g->pend_dir, depth = g->pend_depth;
        g->pend_dir = 0;
        g->gest_depth = depth;
        gear_act(g, dir, depth);
    }
    if (!g->enabled) return;
    // no engine: P (BMW auto-P when the engine goes off in R / N / D)
    if (!running && g->target != GEAR_P) { g->act_why = 0; start(g, GEAR_P, GWHY_ENGINE_OFF); }
    // v1h.2: P released 20 ms after the first notch of a push out of P, before the gear
    // decision (real EGS: 0x39A E1 +20 ms), only when the move would be allowed
    const bool pushing = g->gest_dir || g->pend_dir;
    if (pushing && !g->push_rel && g->target == GEAR_P &&
        (g->stage == GST_P_LOCKED || g->stage == GST_P_UNLOCKED) &&
        ms_since(now_us, g->push_us) >= GEAR_RELEASE_MS && running && brake_on(g)) {
        g->push_rel = true;
        g->rel_prev = g->stage;
        g->stage = GST_P_RELEASED; g->t_stage = now_us;
    }
    const uint32_t ts = ms_since(now_us, g->t_stage), ta = ms_since(now_us, g->t_act);
    switch (g->stage) {
    case GST_P_RELEASED:
        // the push ended (lever back in the centre) without a gear (refused): P as before
        if (!pushing) { g->stage = g->rel_prev; g->t_stage = now_us; }
        break;
    case GST_LEAVE_P:
        if (ts >= (g->target == GEAR_N ? GEAR_CODE_LAG_N_MS : GEAR_CODE_LAG_MS)) { g->stage = GST_STEADY; g->from = g->target; }
        break;
    case GST_TO_P:
        if (ts >= GEAR_TO_P_MS) { g->stage = GST_P_UNLOCKED; g->t_stage = now_us; g->from = GEAR_P; }
        break;
    case GST_P_UNLOCKED:
        // pawl confirmed: 340 ms after the brake was released, or 3 s after the P request at the latest
        if (!g->brake_have || ms_since(now_us, g->brake_us) >= GEAR_PAWL_BRAKE_MS || ta >= GEAR_PLOCK_MS) {
            g->stage = GST_P_LOCKED; log_ev(g, GEV_LOCKED, GEAR_P, GEAR_P, 0);
        }
        break;
    default:
        break;
    }
}

void GEAR_RAM(gear_fill_out)(const gear_t *g, eng_out_t *o)
{
    o->g_set = 0;
    if (!g->enabled || g->stage == GST_P_LOCKED) return;   // fixed P content (as v1h)
    gear_bytes_t b;
    switch (g->stage) {
    case GST_P_RELEASED:
        // v1h.2: P released before the gear decision: 0x39A 03 E1, 0x3F9 bits 28-29 = 1, 0x3FD still P
        b = gb_gear[GEAR_P];
        b.a4 = 0xE1; b.park = 1;
        break;
    case GST_LEAVE_P:
        // new gear on 0x3FD (v1h.2: no forced N on the way to R); 0x39A still 03 E1 and
        // 0x3F9 bits 1 until the gear code follows (GEAR_CODE_LAG_MS)
        b = gb_gear[GEAR_P];
        b.a4 = 0xE1; b.park = 1;
        b.b3fd = gb_gear[g->target & 3u].b3fd;
        break;
    case GST_STEADY:
        b = gb_gear[g->target & 3u];
        break;
    case GST_TO_P: {
        // "P coming" (real EGS, from the P button frame): 0x39A gear 01 +21 ms, 0x3F9 gear 1 and
        // 0xDC 22 FC +42 ms, 0x39A 03 +881 ms; 0x3FD keeps the old gear until +900 ms
        const uint32_t ts = ms_since(g->now, g->t_stage);
        b = gb_gear[g->from & 3u];
        if (g->from == GEAR_P) b = gb_p_unlocked;
        if (ts >= GEAR_PBTN_39A_MS) b.a3 = 0x01;
        if (ts >= GEAR_PBTN_3F9_MS) { b.nib = 1; b.dc2 = 0x22; b.dc5 = 0xFC; }
        if (ts >= GEAR_PBTN_39A_P_MS) b.a3 = 0x03;
        break;
    }
    default:   // GST_P_UNLOCKED
        b = gb_p_unlocked;
        break;
    }
    o->g_set = 1;
    o->g_3fd = b.b3fd; o->g_39a_d3 = b.a3; o->g_39a_d4 = b.a4;
    o->g_park = b.park; o->g_nib = b.nib; o->g_dc_b2 = b.dc2; o->g_dc_b5 = b.dc5;
    // v1h.2: FR 0xE6 keeps "P locked" (26/03) while P is being released; it moves with the gear code
    o->g_fr_park = (g->stage == GST_P_RELEASED || g->stage == GST_LEAVE_P) ? 2u : b.park;
}

void gear_fill_status(const gear_t *g, dde_gear_status_t *st)
{
    memset(st, 0, sizeof *st);
    st->version = 1;
    st->enabled = (uint8_t)((g->enabled ? 1u : 0u) | (EMU_GEAR ? 2u : 0u));
    st->gear = g->target;
    st->stage = g->stage;
    st->lever = g->lv_have ? g->lv : 0xFF;
    st->pbtn = g->lv_have && g->pbtn;
    st->brake = brake_on(g);
    st->running = g->running;
    st->ev_seq = g->ev_seq; st->changes = g->changes; st->refusals = g->refusals;
    st->lever_frames = g->lever_frames; st->lever_bad = g->lever_bad;
    memcpy(st->ev, g->ev, sizeof st->ev);
}

static const char *why_text(uint8_t why)
{
    switch (why) {
    case GWHY_FWD1: return "lever forward 1";
    case GWHY_FWD2: return "lever forward 2";
    case GWHY_BACK1: return "lever back 1";
    case GWHY_BACK2: return "lever back 2";
    case GWHY_PBTN: return "P button";
    case GWHY_ENGINE_OFF: return "engine not running: auto P";
    default: return "";
    }
}

int gear_ev_text(const gear_ev_t *e, char *buf, unsigned n)
{
    const char *f = gear_name(e->from_to & 0x0Fu), *t = gear_name(e->from_to >> 4);
    const char *br = (e->flags & 1u) ? "brake on" : "brake off";
    switch (e->kind) {
    case GEV_CHANGE:
        return snprintf(buf, n, "GEAR: %s -> %s (%s) %s", f, t, br, why_text(e->why));
    case GEV_REFUSED:
        return snprintf(buf, n, "GEAR: refused, %s (%s -> %s, %s)",
                        e->why == GREF_NO_BRAKE ? "brake not pressed" : "engine not running", f, t,
                        why_text((uint8_t)(e->flags >> 4)));
    case GEV_LOCKED:
        return snprintf(buf, n, "GEAR: P pawl confirmed (0x39A E2, 0x3F9 bits 28-29 = 2)");
    case GEV_LEVER_LOST:
        return snprintf(buf, n, "GEAR: lever 0x197 lost (no valid frame for 90 ms), holding %s", t);
    case GEV_LEVER_OK:
        return snprintf(buf, n, "GEAR: lever 0x197 back");
    case GEV_ONOFF:
        return snprintf(buf, n, "GEAR: shifting %s", e->why == GWHY_ENABLED ? "on" : "off (fixed P as v1h)");
    default:
        return snprintf(buf, n, "GEAR: event %u", e->kind);
    }
}
