// PT-CAN DDE+EGS emulation core (v1e): frame table, captured content timelines,
// E2E seal, wake/sleep gate. Pure C (no SDK), so the host tests replay real captures
// through exactly this code. Firmware glue: ptcan_emu.c (V2 board, EMU_CAN_TX).
//
// Times are microseconds on one monotonic 64-bit clock (firmware: time_us_64()).
// Sources: run 4 ddecan capture (real DDE + EGS on PT-CAN, KL15 on, P, engine off),
// analysis/f11_full_emu_txlist.csv, Ian's f11_full_emu_ptcan.md/.csv. Where those
// disagree the captured values are used (see the v1e README / commit message).
#ifndef PTCAN_EMU_CORE_H
#define PTCAN_EMU_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include "dde_engine.h"

// ---- frame table (bit i of the host mask = table index i) ---------------------
enum {
    PTE_F_3F9 = 0, PTE_F_3FD, PTE_F_39A, PTE_F_396, PTE_F_518,
    PTE_F_0A5, PTE_F_0D9, PTE_F_0DC, PTE_F_0F3, PTE_F_08F,
    PTE_F_2C4, PTE_F_281, PTE_F_1B9, PTE_F_381, PTE_F_3B3,
    PTE_F_397, PTE_F_3FB, PTE_F_512, PTE_F_592,
    PTE_N_FRAMES
};
#define PTE_MASK_ALL ((1u << PTE_N_FRAMES) - 1u)
#define PTE_MASK_EGS ((1u << PTE_F_3FD) | (1u << PTE_F_39A) | (1u << PTE_F_396) | (1u << PTE_F_518))
#define PTE_MASK_DDE (PTE_MASK_ALL & ~PTE_MASK_EGS)

typedef enum { PTE_K_CYCLIC = 0, PTE_K_CYC_EVENT, PTE_K_NM512, PTE_K_NM518, PTE_K_CC592 } pte_kind_t;

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t kind;        // pte_kind_t
    uint16_t data_id;    // E2E data ID, 0xFFFF = no E2E (no CRC byte, no alive counter)
    uint8_t ctr_hi;      // static high nibble of b1 (E2E frames)
    uint8_t egs;         // 1 = EGS frame, 0 = DDE frame
    uint32_t period_ms;  // cyclic period
    uint32_t first_ms;   // first frame, ms after wake (W = first PT-CAN frame after sleep)
    uint32_t ev_gap_ms;  // event frames: minimum gap between sends
} pte_def_t;

extern const pte_def_t pte_defs[PTE_N_FRAMES];

// ---- timing constants ---------------------------------------------------------
#define PTE_SILENCE_US      200000u   // no foreign PT-CAN frame for 200 ms: stop all TX at once
#define PTE_NEW_WAKE_US     1000000u  // silence >= 1 s: the next frame is a new wake (restart timelines)
#define PTE_SLEEP_PREP_US   1950000u  // KL15 off, own NM released, no foreign NM for 1.95 s: stop cyclics
#define PTE_NM_CEILING_US   60000000u // NM (0x512/0x518) never later than 60 s after max(wake, last KL15-on 0x12F)
// NM release timers (v1f). The real DDE/EGS keep their 645 ms NM grid running through
// KL15 off and stop at a fixed time after OFF; the last frame is simply the last grid
// slot before that time. Run 4: last 0x512 OFF+45.929 (next slot would be +46.57),
// last 0x518 OFF+29.915 (ddecan) / +29.428 (noddecan; next slot +30.07). So the timers
// are 46.0 s and 30.0 s (v1e stopped at "last real frame" = 45.93/29.91 and therefore
// sent its last frame up to one period, ~0.25 s in run 6, earlier than the real ECU).
#define PTE_512_RELEASE_US  46000000u // DDE: no 0x512 from OFF+46.0 s
#define PTE_518_RELEASE_US  30000000u // EGS: no 0x518 from OFF+30.0 s
#define PTE_518_NOKL15_US   30000000u // wake without KL15: 0x518 stops W+30 s (assumption, no capture)
#define PTE_3FD_P_HOLD_US   30164000u // 0x3FD b2 0x20 -> 0x00 at OFF+30.164 s
// v1f: a cyclic frame that is due but could not be sent on time is sent late (the grid
// is kept, so the missed copies go out back to back with continuous alive counters),
// unless it is more than 250 ms behind; then the grid restarts from now (counted in
// 'resyncs'). A send more than PTE_LATE_US after its slot counts as a late send.
#define PTE_CATCHUP_MAX_US  250000u
#define PTE_LATE_US         5000u

// ---- state --------------------------------------------------------------------
typedef enum {
    PTE_ST_OFF = 0,      // host has not enabled CAN TX
    PTE_ST_WAIT_BUS,     // enabled, PT-CAN asleep / no traffic seen: never transmit
    PTE_ST_RUN,          // PT-CAN alive: frames scheduled
    PTE_ST_SILENT,       // was running, no traffic for > 200 ms: TX stopped
    PTE_ST_PREP_SLEEP,   // KL15 off, NM released, other NM gone: cyclics stopped (bus going to sleep)
    PTE_ST_FAULT,        // latched (foreign sender / bus-off / error passive / mode error)
} pte_state_t;

typedef enum {
    PTE_FAULT_NONE = 0,
    PTE_FAULT_FOREIGN = 1,   // one of our enabled IDs was received from another node
    PTE_FAULT_BUS_OFF = 2,
    PTE_FAULT_TX_PASSIVE = 3,
    PTE_FAULT_MODE = 4,      // controller refused Normal / Listen Only mode
    PTE_FAULT_SPI = 5,
} pte_fault_t;

typedef struct {
    bool started;          // sent at least once this wake
    bool final_pending;    // 0x592: one MIL-off frame after KL15 off
    uint8_t ctr;           // last alive counter sent (0..14)
    uint8_t nm_pre_idx;    // 0x512: index into the pre-KL15 wake pattern
    uint8_t last[8];
    uint8_t want[8];       // v1h.2: last INTENDED payload (event detection compares intent, not what went out)
    bool ev_pend;          // v1h.2: the intended payload changed and no frame has carried it yet
    uint64_t next_us;      // next cyclic send
    uint64_t last_tx_us;
    uint32_t tx;           // frames handed to the controller (all wakes)
} pte_frame_t;

typedef struct {
    uint32_t mask;         // host selection (0 = off)
    bool enabled;
    pte_fault_t fault;
    uint16_t fault_id;
    // bus / gate
    bool have_rx;
    uint64_t last_rx_us, wake_us, last_fnm_us;
    uint32_t wakes, rx_frames;
    // KL15 from 0x12F
    uint8_t st_kl;
    bool kl15, kl15_ever;
    uint64_t on_us, off_us, last_on_ev_us;
    uint32_t n12f, kl15_on_edges, kl15_off_edges;
    pte_frame_t f[PTE_N_FRAMES];
    uint32_t tx_total, refused;
    // v1f timing health
    uint32_t late_sends;   // cyclic sends more than PTE_LATE_US after their slot
    uint32_t late_max_us;  // worst lateness of a cyclic send
    uint32_t resyncs;      // grid restarts (frames skipped because too far behind)
    // v1g: fake-engine overlay (NULL = none). The glue points it at a fresh snapshot
    // before every pte_next(); eng_patch_can() changes bytes before the counter + CRC.
    const eng_out_t *eng;
} pte_t;

typedef struct {
    uint8_t idx;
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint64_t due_us;       // the slot this frame was due in (lateness = send time - due_us)
    bool cyclic_slot;      // true = a cyclic slot, false = an on-change (event) copy
} pte_tx_t;

// ---- API ----------------------------------------------------------------------
void pte_init(pte_t *p);
// Host selection. mask 0 = TX off and clear a latched fault. Returns the mask in use
// (unknown bits removed).
uint32_t pte_set_mask(pte_t *p, uint32_t mask);
// Every received PT-CAN frame (never our own: the controller does not receive what it sends).
void pte_on_rx(pte_t *p, uint16_t id, const uint8_t *data, uint8_t len, uint64_t t_us);
// Latch a fault (also used by the glue for bus-off / error passive / mode errors).
void pte_fault(pte_t *p, pte_fault_t f, uint16_t id);
pte_state_t pte_state(const pte_t *p, uint64_t now_us);
// Next frame due at now_us (false = nothing due). The frame is NOT consumed: call
// pte_commit() once the controller accepted it (FIFO full: just try again later).
bool pte_next(pte_t *p, uint64_t now_us, pte_tx_t *out);
void pte_commit(pte_t *p, const pte_tx_t *tx, uint64_t now_us);

// Hard never-send list (PT-CAN2 is not wired at all): 0x137, EWS/immobiliser
// 0x80/0x85/0x351/0x380/0x580/0x5C0, diagnostics 0x6xx, 0x27A, EGS service/CC 0x598.
bool pte_id_forbidden(uint16_t id);

// E2E: b1 low nibble = alive counter, b0 = CRC8 (poly 0x1D, init 0) over
// dataID lo, dataID hi, b1..b(dlc-1).
uint8_t pte_crc8(const uint8_t *p, unsigned n, uint8_t crc);
uint8_t pte_e2e_crc(uint16_t data_id, const uint8_t *frame, uint8_t dlc);

// 0x12F ST_KL -> KL15 (DDE rule: nibble >= 0xA on, 9 keeps the state, >= 0xE invalid)
int pte_kl15_from_stkl(uint8_t st_kl, bool prev);  // 1 on, 0 off, -1 invalid (keep)

const char *pte_state_name(pte_state_t s);
const char *pte_fault_name(pte_fault_t f);
int pte_index_of(uint16_t id);

#endif
