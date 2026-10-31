// PT-CAN DDE+EGS emulation core (v1e). See ptcan_emu_core.h.
//
// All content timelines are the run 4 "ddecan" capture (real DDE + EGS, KL15 on with
// the engine off, selector in P), measured relative to:
//   W   = first PT-CAN frame after bus sleep (wake),
//   ON  = first 0x12F with ST_KL >= 0xA,
//   OFF = first 0x12F with ST_KL < 9 after ON.
#include <string.h>
#include "ptcan_emu_core.h"

#define MS(x) ((uint64_t)(x) * 1000u)
#define NO_E2E 0xFFFFu

const pte_def_t pte_defs[PTE_N_FRAMES] = {
    //            id  dlc kind             dataID  hi egs period first  evgap
    [PTE_F_3F9] = {0x3F9, 8, PTE_K_CYC_EVENT, 0x0037, 1, 0, 1000, 439, 1},
    [PTE_F_3FD] = {0x3FD, 5, PTE_K_CYCLIC, 0x0039, 0, 1, 100, 462, 0},
    [PTE_F_39A] = {0x39A, 6, PTE_K_CYC_EVENT, NO_E2E, 0, 1, 1000, 462, 20},
    [PTE_F_396] = {0x396, 2, PTE_K_CYCLIC, NO_E2E, 0, 1, 1000, 600, 0},
    [PTE_F_518] = {0x518, 8, PTE_K_NM518, NO_E2E, 0, 1, 645, 543, 0},
    [PTE_F_0A5] = {0x0A5, 8, PTE_K_CYCLIC, 0x0009, 0xF, 0, 20, 459, 0},
    [PTE_F_0D9] = {0x0D9, 8, PTE_K_CYCLIC, 0x0043, 1, 0, 20, 462, 0},
    [PTE_F_0DC] = {0x0DC, 6, PTE_K_CYCLIC, 0x005A, 1, 0, 20, 462, 0},
    [PTE_F_0F3] = {0x0F3, 8, PTE_K_CYCLIC, 0x00A6, 0, 0, 20, 462, 0},
    [PTE_F_08F] = {0x08F, 8, PTE_K_CYCLIC, 0x0020, 1, 0, 30, 459, 0},
    [PTE_F_2C4] = {0x2C4, 8, PTE_K_CYCLIC, NO_E2E, 0, 0, 100, 611, 0},
    [PTE_F_281] = {0x281, 2, PTE_K_CYCLIC, NO_E2E, 0, 0, 100, 612, 0},
    [PTE_F_1B9] = {0x1B9, 2, PTE_K_CYCLIC, NO_E2E, 0, 0, 1000, 611, 0},
    [PTE_F_381] = {0x381, 2, PTE_K_CYC_EVENT, NO_E2E, 0, 0, 10000, 435, 20},
    [PTE_F_3B3] = {0x3B3, 7, PTE_K_CYC_EVENT, NO_E2E, 0, 0, 5000, 437, 20},
    [PTE_F_397] = {0x397, 3, PTE_K_CYC_EVENT, NO_E2E, 0, 0, 1000, 437, 15},
    [PTE_F_3FB] = {0x3FB, 6, PTE_K_CYC_EVENT, NO_E2E, 0, 0, 1000, 437, 60},
    [PTE_F_512] = {0x512, 8, PTE_K_NM512, NO_E2E, 0, 0, 645, 529, 0},
    [PTE_F_592] = {0x592, 8, PTE_K_CC592, NO_E2E, 0, 0, 5100, 0, 0},
};

// 0x512 after a wake without KL15: four frames, then quiet until KL15 (run 4 ddecan).
static const uint32_t nm512_pre_ms[] = {529, 1179, 2459, 3109};
#define NM512_PRE_N (sizeof nm512_pre_ms / sizeof nm512_pre_ms[0])
#define NM512_ON_US   MS(106)   // first 0x512 after KL15 on
#define CC592_ON_US   MS(98)    // first 0x592 (CC) after KL15 on
#define CC592_OFF_US  MS(54)    // last 0x592 (MIL off) after KL15 off

// ---- helpers ------------------------------------------------------------------
uint8_t pte_crc8(const uint8_t *p, unsigned n, uint8_t crc)
{
    for (unsigned i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x1D : crc << 1);
    }
    return crc;
}

uint8_t pte_e2e_crc(uint16_t data_id, const uint8_t *frame, uint8_t dlc)
{
    uint8_t id[2] = {(uint8_t)data_id, (uint8_t)(data_id >> 8)};
    return pte_crc8(frame + 1, (unsigned)dlc - 1u, pte_crc8(id, 2, 0));
}

bool pte_id_forbidden(uint16_t id)
{
    if ((id & 0x700u) == 0x600u) return true;  // diagnostics 0x6xx (0x618 ...)
    switch (id) {
    case 0x137:                                 // never
    case 0x080: case 0x085: case 0x351:        // EWS / immobiliser (and any challenge/response)
    case 0x380: case 0x580: case 0x5C0:
    case 0x27A:
    case 0x598:                                 // EGS service / its own CC 419 rides on it
        return true;
    default:
        return false;
    }
}

int pte_kl15_from_stkl(uint8_t st_kl, bool prev)
{
    st_kl &= 0x0F;
    if (st_kl >= 0xE) return -1;
    if (st_kl >= 0xA) return 1;
    if (st_kl == 9) return prev ? 1 : 0;
    return 0;
}

int pte_index_of(uint16_t id)
{
    for (int i = 0; i < PTE_N_FRAMES; i++)
        if (pte_defs[i].id == id) return i;
    return -1;
}

const char *pte_state_name(pte_state_t s)
{
    switch (s) {
    case PTE_ST_OFF: return "OFF";
    case PTE_ST_WAIT_BUS: return "WAIT_BUS";
    case PTE_ST_RUN: return "RUN";
    case PTE_ST_SILENT: return "SILENT";
    case PTE_ST_PREP_SLEEP: return "PREP_SLEEP";
    case PTE_ST_FAULT: return "FAULT";
    }
    return "?";
}

const char *pte_fault_name(pte_fault_t f)
{
    switch (f) {
    case PTE_FAULT_NONE: return "none";
    case PTE_FAULT_FOREIGN: return "foreign sender of an emulated ID";
    case PTE_FAULT_BUS_OFF: return "bus-off";
    case PTE_FAULT_TX_PASSIVE: return "error passive (TEC >= 128)";
    case PTE_FAULT_MODE: return "controller mode change refused";
    case PTE_FAULT_SPI: return "SPI error";
    }
    return "?";
}

// ---- state --------------------------------------------------------------------
static void new_wake(pte_t *p, uint64_t t)
{
    p->wake_us = t;
    p->last_fnm_us = t;
    p->last_on_ev_us = t;
    p->wakes++;
    p->kl15 = p->kl15_ever = false;
    p->st_kl = 0xFF;
    for (int i = 0; i < PTE_N_FRAMES; i++) {
        uint32_t tx = p->f[i].tx;
        memset(&p->f[i], 0, sizeof p->f[i]);
        p->f[i].tx = tx;
        p->f[i].ctr = 0;     // first frame after wake carries 1
    }
}

void pte_init(pte_t *p)
{
    memset(p, 0, sizeof *p);
    p->st_kl = 0xFF;
}

uint32_t pte_set_mask(pte_t *p, uint32_t mask)
{
    mask &= PTE_MASK_ALL;
    if (mask == 0) {
        p->enabled = false;
        p->mask = 0;
        p->fault = PTE_FAULT_NONE;
        p->fault_id = 0;
        return 0;
    }
    p->mask = mask;
    p->enabled = true;
    return mask;
}

void pte_fault(pte_t *p, pte_fault_t f, uint16_t id)
{
    if (p->fault == PTE_FAULT_NONE) {
        p->fault = f;
        p->fault_id = id;
    }
}

static bool own_id_enabled(const pte_t *p, uint16_t id)
{
    int i = pte_index_of(id);
    return i >= 0 && (p->mask & (1u << i));
}

void pte_on_rx(pte_t *p, uint16_t id, const uint8_t *data, uint8_t len, uint64_t t)
{
    if (!p->have_rx || (t > p->last_rx_us && t - p->last_rx_us >= PTE_NEW_WAKE_US)) new_wake(p, t);
    p->have_rx = true;
    if (t > p->last_rx_us) p->last_rx_us = t;
    p->rx_frames++;

    if (p->enabled && own_id_enabled(p, id)) pte_fault(p, PTE_FAULT_FOREIGN, id);

    // Foreign NM: 0x500..0x57F, except the NM IDs we are sending ourselves.
    if (id >= 0x500 && id <= 0x57F && !(p->enabled && own_id_enabled(p, id))) p->last_fnm_us = t;

    if (id == 0x12F && len >= 3) {
        p->n12f++;
        p->st_kl = data[2] & 0x0F;
        int k = pte_kl15_from_stkl(p->st_kl, p->kl15);
        if (k == 1) {
            p->last_on_ev_us = t;
            if (!p->kl15) {
                p->kl15 = true;
                p->kl15_ever = true;
                p->on_us = t;
                p->kl15_on_edges++;
                pte_frame_t *f = &p->f[PTE_F_512];
                f->next_us = t + NM512_ON_US;
                p->f[PTE_F_592].next_us = t + CC592_ON_US;
                p->f[PTE_F_592].final_pending = false;
            }
        } else if (k == 0 && p->kl15) {
            p->kl15 = false;
            p->off_us = t;
            p->kl15_off_edges++;
            p->f[PTE_F_592].final_pending = true;
            p->f[PTE_F_592].next_us = t + CC592_OFF_US;
        }
    }
}

// ---- content ------------------------------------------------------------------
static inline uint64_t since(uint64_t now, uint64_t t) { return now >= t ? now - t : 0; }

// Is the frame active (allowed to be sent) at now? (Ignoring the bus gate.)
static bool frame_active(const pte_t *p, int i, uint64_t now)
{
    const pte_frame_t *f = &p->f[i];
    uint64_t ceil_ref = p->last_on_ev_us > p->wake_us ? p->last_on_ev_us : p->wake_us;
    switch (pte_defs[i].kind) {
    case PTE_K_NM512:
        if (now > ceil_ref + PTE_NM_CEILING_US) return false;
        if (p->kl15) return now >= p->on_us + NM512_ON_US;
        if (p->kl15_ever) return now < p->off_us + PTE_512_RELEASE_US;
        return f->nm_pre_idx < NM512_PRE_N;
    case PTE_K_NM518:
        if (now > ceil_ref + PTE_NM_CEILING_US) return false;
        if (p->kl15) return true;
        if (p->kl15_ever) return now < p->off_us + PTE_518_RELEASE_US;
        return now < p->wake_us + PTE_518_NOKL15_US;
    case PTE_K_CC592:
        return p->kl15 || f->final_pending;
    default:
        return true;
    }
}

// Captured payload for frame i at now (b0/b1 of E2E frames filled in later).
static void build(const pte_t *p, int i, uint64_t now, uint8_t *d)
{
    const uint64_t tw = since(now, p->wake_us);
    const bool on = p->kl15, ever = p->kl15_ever;
    const uint64_t ton = on ? since(now, p->on_us) : 0;
    const uint64_t toff = (!on && ever) ? since(now, p->off_us) : 0;
    memset(d, 0, 8);
    switch (i) {
    case PTE_F_3F9: {
        // p2..p7. Wake ramp: p6 00->10->20->23, p3 34->74->64 (bits 28-29: 3 -> 2 = P lock),
        // temperatures p4/p5 settling as captured. Final `80 64 3d 3d 23 47`.
        static const uint8_t base[6] = {0x80, 0x34, 0x3c, 0x30, 0x00, 0x47};
        memcpy(d + 2, base, 6);
        if (tw >= MS(459)) d[6] = 0x10;
        if (tw >= MS(509)) d[6] = 0x20;
        if (tw >= MS(529)) d[3] = 0x74;
        if (tw >= MS(609)) d[3] = 0x64;
        if (tw >= MS(709)) d[6] = 0x23;
        if (tw >= MS(1379)) d[5] = 0x43;
        if (tw >= MS(3219)) d[4] = 0x3d;
        if (tw >= MS(6379)) d[5] = 0x3c;
        if (tw >= MS(7379)) d[5] = 0x3d;
        break;
    }
    case PTE_F_3FD: {
        // `CC 0A 00 fc ff`, b2 = 0x20 (gear display P) from ON+27 ms to OFF+30.164 s
        bool p_on = on ? (ton >= MS(27)) : (ever && toff < PTE_3FD_P_HOLD_US);
        d[2] = p_on ? 0x20 : 0x00;
        d[3] = 0xfc;
        d[4] = 0xff;
        break;
    }
    case PTE_F_39A: {
        // b1 = ATF temperature (raw - 40 C): captured 0x35, constant. b2 = 0: no fault.
        // b4 bits 0-1 = 2: P engaged (relayed by the DDE into 0x3F9 bits 28-29).
        static const uint8_t fin[6] = {0x00, 0x35, 0x00, 0x03, 0xe2, 0x00};
        static const uint8_t s0[6] = {0x00, 0x00, 0x00, 0x00, 0xe3, 0x00};
        static const uint8_t s1[6] = {0x00, 0x00, 0x00, 0x00, 0xe2, 0x00};
        memcpy(d, tw < MS(583) ? s0 : (tw < MS(680) ? s1 : fin), 6);
        break;
    }
    case PTE_F_396:
        d[0] = 0xfd; d[1] = 0xff;
        break;
    case PTE_F_518: {
        // NM: `00 00 00 00 b0 30 00 18` with KL15 (and the very first frame of a wake),
        // `.. 44 2f ..` without. KL15 on: b0 30 from ON+157 ms; off: 44 2f from OFF+225 ms.
        bool kl = !p->f[i].started || (on ? ton >= MS(157) : (ever && toff < MS(225)));
        d[4] = kl ? 0xb0 : 0x44;
        d[5] = kl ? 0x30 : 0x2f;
        d[7] = 0x18;
        break;
    }
    case PTE_F_0A5: {
        // `CC Fx b8 87 7b 00 00 f1` before KL15; `CC Fx 1a aa a1 00 00 f1` from ON+55 ms;
        // after off `19 9a a1 ..` (the value the real DDE settles on; with KL15 it flips
        // between the two every few 100 ms late in the run, which is not reproduced).
        static const uint8_t pre[6] = {0xb8, 0x87, 0x7b, 0x00, 0x00, 0xf1};
        static const uint8_t kl[6] = {0x1a, 0xaa, 0xa1, 0x00, 0x00, 0xf1};
        static const uint8_t aft[6] = {0x19, 0x9a, 0xa1, 0x00, 0x00, 0xf1};
        memcpy(d + 2, on ? (ton >= MS(55) ? kl : pre) : (ever ? aft : pre), 6);
        break;
    }
    case PTE_F_0D9: {
        static const uint8_t c[6] = {0x00, 0x10, 0x00, 0xe0, 0x7f, 0xc0};
        memcpy(d + 2, c, 6);
        break;
    }
    case PTE_F_0DC: {
        static const uint8_t s0[4] = {0x80, 0xff, 0xff, 0xf0};
        static const uint8_t s1[4] = {0x20, 0xd0, 0x07, 0xf1};
        static const uint8_t s2[4] = {0x22, 0xd0, 0x07, 0xfe};
        memcpy(d + 2, tw < MS(502) ? s0 : (tw < MS(703) ? s1 : s2), 4);
        break;
    }
    case PTE_F_0F3: {
        static const uint8_t c[6] = {0x00, 0xc0, 0xf0, 0xff, 0xff, 0xff};
        memcpy(d + 2, c, 6);
        break;
    }
    case PTE_F_08F: {
        // `CC 1x 00 7d 8c 00 18 b7`: b7 80 before KL15, 10 from ON+15 ms, c0 from OFF+31 ms
        static const uint8_t c[5] = {0x00, 0x7d, 0x8c, 0x00, 0x18};
        memcpy(d + 2, c, 5);
        uint8_t b7 = 0x80;
        if (on) b7 = ton >= MS(15) ? 0x10 : (p->f[i].started ? p->f[i].last[7] : 0x80);
        else if (ever) b7 = toff >= MS(31) ? 0xc0 : 0x10;
        d[7] = b7;
        break;
    }
    case PTE_F_2C4: {
        static const uint8_t c[8] = {0x00, 0x00, 0xff, 0x64, 0x64, 0x64, 0x00, 0xf0};
        memcpy(d, c, 8);
        break;
    }
    case PTE_F_281:
        // supply voltage (captured modal values): 0x51 first 200 ms, 0x4e before KL15,
        // 0x33 with KL15, 0x36 after off. b1 = f3.
        d[0] = on ? (ton >= MS(100) ? 0x33 : 0x4e) : (ever ? (toff >= MS(100) ? 0x36 : 0x33)
                                                         : (tw < MS(811) ? 0x51 : 0x4e));
        d[1] = 0xf3;
        break;
    case PTE_F_1B9:
        d[0] = 0x00; d[1] = 0xff;
        break;
    case PTE_F_381:
        // `0c f0` -> `7c f0` (W+1.411); KL15: `79 f0` at ON+0.867, `0c f2` at ON+3.867;
        // off: `7c f0` from OFF+0.423.
        if (on) {
            if (ton < MS(867)) { d[0] = 0x7c; d[1] = 0xf0; }
            else if (ton < MS(3867)) { d[0] = 0x79; d[1] = 0xf0; }
            else { d[0] = 0x0c; d[1] = 0xf2; }
        } else if (ever) {
            if (toff < MS(423)) { d[0] = 0x0c; d[1] = 0xf2; }
            else { d[0] = 0x7c; d[1] = 0xf0; }
        } else {
            d[0] = tw < MS(1411) ? 0x0c : 0x7c;
            d[1] = 0xf0;
        }
        break;
    case PTE_F_3B3: {
        static const uint8_t s0[7] = {0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8};
        static const uint8_t nokl[7] = {0x1f, 0x00, 0xff, 0x7f, 0x00, 0x03, 0xf8};
        static const uint8_t kl[7] = {0x1f, 0x00, 0x04, 0x00, 0x00, 0x03, 0xf8};
        bool k = on ? (ton >= MS(79)) : (ever && toff < MS(234));
        memcpy(d, tw < MS(561) ? s0 : (k ? kl : nokl), 7);
        break;
    }
    case PTE_F_397: {
        // wake ramp fe e8 e9 e6 e4 -> e2 (the occasional 60 ms e3 blips are not reproduced)
        uint8_t b = 0xe2;
        if (tw < MS(461)) b = 0xfe;
        else if (tw < MS(481)) b = 0xe8;
        else if (tw < MS(541)) b = 0xe9;
        else if (tw < MS(601)) b = 0xe6;
        else if (tw < MS(661)) b = 0xe4;
        d[0] = 0xc0; d[1] = b; d[2] = 0x00;
        break;
    }
    case PTE_F_3FB: {
        // `d0 b1 01 10 f1 b5`: b1 ramps ~31/s from W+0.44 s to 0xb1 (W+6.2 s), b5 = b1/2.
        uint32_t r = 0;
        if (tw > MS(440)) r = (uint32_t)((tw - MS(440)) * 31u / 1000000u);
        if (r > 0xb1) r = 0xb1;
        d[0] = 0xd0; d[1] = (uint8_t)r; d[2] = 0x01; d[3] = 0x10; d[4] = 0xf1; d[5] = (uint8_t)(r >> 1);
        break;
    }
    case PTE_F_512: {
        static const uint8_t c[8] = {0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x12};
        memcpy(d, c, 8);
        break;
    }
    case PTE_F_592: {
        // only ever the MIL-OFF check-control frame
        static const uint8_t c[8] = {0x40, 0x22, 0x00, 0x14, 0xff, 0xff, 0xff, 0xff};
        memcpy(d, c, 8);
        break;
    }
    default:
        break;
    }
}

pte_state_t pte_state(const pte_t *p, uint64_t now)
{
    if (!p->enabled) return PTE_ST_OFF;
    if (p->fault != PTE_FAULT_NONE) return PTE_ST_FAULT;
    if (!p->have_rx) return PTE_ST_WAIT_BUS;
    uint64_t q = since(now, p->last_rx_us);
    if (q > PTE_SILENCE_US) return q >= PTE_NEW_WAKE_US ? PTE_ST_WAIT_BUS : PTE_ST_SILENT;
    if (!p->kl15 && since(now, p->last_fnm_us) >= PTE_SLEEP_PREP_US) {
        bool own_nm = false;
        if ((p->mask & (1u << PTE_F_512)) && frame_active(p, PTE_F_512, now)) own_nm = true;
        if ((p->mask & (1u << PTE_F_518)) && frame_active(p, PTE_F_518, now)) own_nm = true;
        if (!own_nm) return PTE_ST_PREP_SLEEP;
    }
    return PTE_ST_RUN;
}

static bool payload_differs(const pte_def_t *d, const uint8_t *a, const uint8_t *b)
{
    unsigned from = d->data_id != NO_E2E ? 2u : 0u;
    return memcmp(a + from, b + from, d->dlc - from) != 0;
}

bool pte_next(pte_t *p, uint64_t now, pte_tx_t *out)
{
    if (pte_state(p, now) != PTE_ST_RUN) return false;
    // most overdue first
    int best = -1;
    uint64_t best_due = 0;
    uint8_t best_d[8];
    for (int i = 0; i < PTE_N_FRAMES; i++) {
        if (!(p->mask & (1u << i))) continue;
        const pte_def_t *def = &pte_defs[i];
        pte_frame_t *f = &p->f[i];
        if (pte_id_forbidden(def->id)) continue;
        if (!frame_active(p, i, now)) continue;
        uint64_t due;
        uint8_t d[8];
        build(p, i, now, d);
        if (def->kind == PTE_K_NM512 && !p->kl15_ever) {
            due = p->wake_us + MS(nm512_pre_ms[f->nm_pre_idx]);
        } else if (def->kind == PTE_K_NM512 || def->kind == PTE_K_CC592) {
            due = f->next_us;
        } else if (!f->started) {
            due = p->wake_us + MS(def->first_ms);
        } else {
            due = f->next_us;
            if (def->kind == PTE_K_CYC_EVENT && payload_differs(def, d, f->last) &&
                now >= f->last_tx_us + MS(def->ev_gap_ms))
                due = now;   // on change
        }
        if (now < due) continue;
        if (best < 0 || due < best_due) {
            best = i;
            best_due = due;
            memcpy(best_d, d, 8);
        }
    }
    if (best < 0) return false;
    const pte_def_t *def = &pte_defs[best];
    out->idx = (uint8_t)best;
    out->id = def->id;
    out->dlc = def->dlc;
    memcpy(out->data, best_d, 8);
    if (def->data_id != NO_E2E) {
        uint8_t c = p->f[best].ctr;
        c = (c >= 14) ? 0 : (uint8_t)(c + 1);
        out->data[1] = (uint8_t)((def->ctr_hi << 4) | c);
        out->data[0] = pte_e2e_crc(def->data_id, out->data, def->dlc);
    }
    return true;
}

void pte_commit(pte_t *p, const pte_tx_t *tx, uint64_t now)
{
    if (tx->idx >= PTE_N_FRAMES) return;
    const pte_def_t *def = &pte_defs[tx->idx];
    pte_frame_t *f = &p->f[tx->idx];
    if (def->data_id != NO_E2E) f->ctr = tx->data[1] & 0x0F;
    uint64_t period = MS(def->period_ms);
    switch (def->kind) {
    case PTE_K_NM512:
        if (!p->kl15_ever) {
            f->nm_pre_idx++;
            f->next_us = now + period;
        } else {
            f->next_us = (f->next_us + period > now) ? f->next_us + period : now + period;
        }
        break;
    case PTE_K_CC592:
        if (!p->kl15) f->final_pending = false;
        f->next_us = now + period;
        break;
    default:
        if (!f->started) f->next_us = now + period;
        else if (now >= f->next_us) {
            // cyclic slot (events do not move the grid); if far behind, restart the grid
            f->next_us += period;
            if (f->next_us <= now) f->next_us = now + period;
        }
        break;
    }
    f->started = true;
    f->last_tx_us = now;
    memcpy(f->last, tx->data, 8);
    f->tx++;
    p->tx_total++;
}
