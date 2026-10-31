// DDE emulator, hardware side. Runs on core1 next to the FR1/FR2 streamer ISRs.
//
//  FR1 streamer header ISR (car frame, good header CRC)
//      -> reference? update the lock FSM, schedule our targets of THIS cycle:
//         expected echo mark = ref mark + (target offset - ref offset)  [10 ns ticks]
//         TXEN target        = echo mark - measured TX->RX loop delay
//         alarm              = ref us + same delta - DDE_EMU_LEAD_US
//      -> one of our IDs? echo of our own frame: measure loop delay and fire latency.
//  TIMER0 alarm ISR (priority 0, core1), ~DDE_EMU_LEAD_US before the slot
//      -> push the prepared frame (8 words) into the TX SM, read the clk_sys counter,
//         push the remaining delay (ticks) into the fire SM, which then raises
//         PIO irq 5 at exactly that clk_sys cycle; the TX SM drops TXEN and sends.
//      -> also housekeeping every 1 ms (and right after every frame's expected end):
//         lock timeout, keepalive, TX SM watchdog, fault auto re-arm.
//
// v1b (car test 1): the TX watchdog used to time "TX SM not idle" from the first
// non-idle observation and only cleared it when a later housekeeping pass saw the
// SM idle. Housekeeping only runs from the alarm ISR, and every pass between 0x26
// (1215 us) and 0x3D (1974 us) happened right after loading a frame, so the SM was
// never seen idle: at the 0x3D alarm the stale timestamp from 0x26 gave 759 us >
// 200 us and FAULT 1 stopped the SMs before 0x3D went out. Now the watchdog checks
// each committed frame against its own expected end, and one housekeeping pass is
// scheduled right after that end (tests/host/test_dde_emu_replay.c).
#include "dde_emu.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/structs/pwm.h"
#include "hardware/structs/timer.h"
#include "flexray_pins.h"
#include "flexray_ts.h"
#include "dde_emu_tables.h"
#include "dde_emu_tx.pio.h"

#ifndef EMU_DYN_5C
#define EMU_DYN_5C 1          // v1d: on, gated by KL15 / NM rules + 60 s hard ceiling
#endif
// v1d dynamic frames
#ifndef EMU_142
#define EMU_142 1
#endif
#ifndef EMU_10A
#define EMU_10A 1
#endif
#ifndef EMU_68
#define EMU_68 1
#endif
#ifndef EMU_KL15_F1
#define EMU_KL15_F1 1         // 1 = KL15 from 0xF1 b3 (fallback 0x103 until the first 0xF1); 0 = 0x103 only
#endif
// v1c dynamic frames (one switch per frame ID / slot) and pre-ign content
#ifndef EMU_DYN_E6
#define EMU_DYN_E6 1
#endif
#ifndef EMU_DYN_FB
#define EMU_DYN_FB 1
#endif
#ifndef EMU_DYN_E8
#define EMU_DYN_E8 1
#endif
#ifndef EMU_PREIGN
#define EMU_PREIGN 1
#endif
#ifndef DDE_EMU_SPIN_HOOK
#define DDE_EMU_SPIN_HOOK() tight_loop_contents()
#endif
// slot index of dynamic slot ds (after the 4 statics + 0x5C)
#define DSLOT(ds) (DDE_EMU_N_SLOTS + (unsigned)(ds))
#define N_ALL (DDE_EMU_N_SLOTS + DDE_DYN_N_SLOT)
#define DDE_EMU_R_DYN_CHECK 400         // dynamic: final frame-start check when <= 4 us are left
#define DDE_EMU_IGN_TIMEOUT_US 500000u  // 0x103 every 200-220 ms while ign is on
static inline bool dyn_enabled(unsigned ds)
{
    switch (ds) {
    case DDE_DSLOT_E6: return EMU_DYN_E6 != 0;
    case DDE_DSLOT_E8: return EMU_DYN_E8 != 0;
    case DDE_DSLOT_FB: return EMU_DYN_FB != 0;
    case DDE_DSLOT_10A: return EMU_10A != 0;
    case DDE_DSLOT_142: return EMU_142 != 0;
    case DDE_DSLOT_68: return EMU_68 != 0;
    default: return false;
    }
}

#define DDE_EMU_PIO pio0
#define DDE_EMU_LEAD_US 6u              // alarm this long before the TXEN target
#define DDE_EMU_LEAD_DYN_US 10u         // v1d: dynamic slots (run 3 FB7 lates: more ISR-entry margin)
#define DDE_EMU_HK_GUARD_US 200u        // v1d: no housekeeping alarm in the 200 us before an armed fire
#define DDE_EMU_TOPUP_US 50u            // v1d: > 8-word frames: rest of the words within 50 us of the fire
#define DDE_EMU_R_MIN 20                // fire SM delay window (ticks)
#define DDE_EMU_R_PRECHECK 120          // must still have 1.2 us left before loading the TX FIFO (v1: 2.5 us;
                                        // loading 8 words takes < 0.3 us, r is re-checked after)
#define DDE_EMU_R_MAX 2000
#define DDE_EMU_MIN_DELTA_Q8 (20 * 100 * 256)   // target at least 20 us after the reference
#define DDE_EMU_QUIET_CYCLES 8u         // cycles watched after enable before the first TX
#define DDE_EMU_ECHO_MISS_FAULT 8u
#define DDE_EMU_KEEPALIVE_US 20000000u  // host must re-send wValue 1 within 20 s
#define DDE_EMU_HK_US 1000u
#define DDE_EMU_K_INIT 30               // ticks: alarm-ISR counter read -> TXEN mark
#define DDE_EMU_LOOP_INIT 20            // ticks: TXEN mark -> FR1 RX echo mark (first guess)
#define DDE_EMU_WD_MARGIN_US 20u        // TX SM may be busy this long past a frame's computed end
#define DDE_EMU_WD_UNEXPECTED_US 200u   // TX SM busy with no committed frame: fault after this
#define DDE_EMU_ECHO_WINDOW_US 1000u    // echo header must arrive within this after our fire

enum { SL_IDLE = 0, SL_ARMED = 1, SL_DONE = 2, SL_WAIT = 3 };   // WAIT: dynamic, due, no certain minislot yet

typedef struct {
    uint8_t state;
    uint8_t cycle;
    uint8_t nwords;
    uint8_t is_data;
    int8_t pdu;             // dynamic: DDE_PDU_* (-1 static)
    uint16_t ms;            // dynamic: planned minislot
    uint16_t txen_fine;
    uint16_t bus_fine;
    uint32_t alarm_us;
    uint32_t words[DDE_TX_MAX_WORDS];
} emu_slot_t;

static emu_slot_t slots[N_ALL];
static uint8_t counter[DDE_EMU_N_STATIC][4];
static dde_lock_t lock;

static uint sm_tx, sm_fire, off_tx, off_fire;
static int alarm_num = -1;
static bool hw_ready;

// host control (core0 writes, core1 reads)
static volatile bool host_enabled;
static volatile uint32_t host_last_us;
static volatile uint32_t host_epoch;      // bumped on every 0 -> 1 enable
static volatile bool host_clear_req;

// core1 state
static volatile uint8_t fault;
static uint32_t seen_epoch;
static uint32_t quiet_cycles;
static bool was_locked;

// last fire per slot, for the echo check (one record per slot: our own echo is matched
// by slot + cycle + time window, so it can never be taken for a foreign frame)
typedef struct {
    bool pending;           // fired, echo not seen yet
    bool echoed;            // echo of the (cycle) fire seen
    uint8_t cycle;
    uint16_t txen_fine;
    uint16_t bus_fine;
    uint32_t us;
    uint32_t seq1;          // side-1 (our TXEN) ts entries produced before the fire
} fire_rec_t;
static fire_rec_t fired[N_ALL];
static int last_fired = -1;     // slot of the most recent fire (side-1 ts sequence check)
static uint32_t echo_miss_run;
static uint32_t echo_miss_run_cycles;
static uint8_t echo_miss_last_cycle;

// TX watchdog (see header comment)
static bool tx_expect;          // a committed frame may still be on the wire
static uint32_t tx_deadline_us; // ... until this time (+ margin)
static uint32_t tx_unexp_since; // first time the SM was seen busy with no frame committed (0 = none)

// fault bookkeeping
static uint8_t fault_count, auto_clears, last_fault_code;
static bool fault_latched;
static uint32_t fault_evidence_us;  // last time the fault condition was seen
static uint32_t last_fault_us;
static struct {
    uint8_t code, slot, cycle, flags;
    uint32_t us, sent_total, late, echo_ok, echo_miss, foreign;
    int32_t detail;
} first_fault;
static volatile uint32_t fault_seq;   // bumped at every new first fault (UART print once)
static uint16_t late_reason[DDE_EMU_LATE_N];
static uint8_t last_late_slot, last_late_reason;
static int16_t last_late_r;
static uint32_t echo_dup;
static uint8_t cur_cycle;

// calibration
static int32_t k_q4 = DDE_EMU_K_INIT * 16;
static int32_t loop_q4 = DDE_EMU_LOOP_INIT * 16;
static bool k_cal, loop_cal;
static int32_t echo_err_q4, fire_err_q4;
static int32_t echo_err_maxabs;

static volatile struct {
    uint32_t sent[DDE_EMU_N_SLOTS];
    uint32_t late, echo_ok, echo_miss, foreign, rearms;
} cnt;

// v1c dynamic state
static dde_dyn_track_t trk;                  // this cycle's dynamic-segment tracker
static uint16_t dref_fine;                   // latest reference of this cycle (mark)
static uint32_t dref_us;
static int32_t dref_off;
static uint32_t abs_cycle;                   // cycles since wake
static uint8_t abs_last_cycle;
static uint32_t pdu_due[DDE_DYN_N_PDU];      // abs cycle from which the PDU is due
static uint8_t pdu_ctr[DDE_DYN_N_PDU];       // 0..14 (0x142: 0..254), starts at 1 after wake
static bool pdu_started[DDE_DYN_N_PDU];      // sent since wake (period grid running)
static int8_t dyn_due[DDE_DYN_N_SLOT];       // PDU due in this cycle (-1 none)
static uint8_t dyn_reason[DDE_DYN_N_SLOT];   // why not (yet) armed in this cycle
static bool dyn_sent_cyc[DDE_DYN_N_SLOT];
static bool ign_seen;                        // 0x103 seen since wake
static uint32_t ign_abs;                     // abs cycle of the first 0x103
static uint32_t last_103_us;
static bool tx_parked_ok;                    // last frame had nbytes % 4 == 2: SM parks at idle+1
// v1d KL15 / NM
static dde_kl15_t kl15;
static bool nm_ok, nm_ceiling;               // 0x5C allowed in this cycle / stopped by the ceiling
static volatile struct { uint8_t b3; uint32_t us; uint32_t seq; } f1mb;   // core0 -> core1 (0xF1)
static uint32_t f1_seq_seen;
static uint8_t f1_last_b3;
static struct {
    int16_t isr_late_min, isr_late_max;      // dynamic fires: alarm ISR entry - alarm time (us)
    int16_t r_entry_min, r_final_min;        // dynamic fires: fire delay at ISR entry / after the final check
    uint16_t topup_timeouts, hk_deferred, nm_ceiling_stops;
    bool any;
} lat;
static volatile struct {
    uint32_t sent[DDE_DYN_N_PDU];
    uint16_t skip[DDE_DYN_N_PDU];
    uint16_t reason[DDE_SKIP_N];
    uint16_t defer, parks;
    uint16_t last_ms[DDE_DYN_N_SLOT];
} dcnt;
// 0x142 content phase for this cycle (see dde_emu_core.h)
static unsigned ph142(void)
{
    if (kl15.on && abs_cycle >= kl15.on_abs + DDE_142_P2_CYCLES) return DDE_142_P2;
    if (kl15.ever_on && !kl15.on) return DDE_142_P2;   // ign was on in this wake: settled content
    return abs_cycle >= DDE_142_P1_CYCLES ? DDE_142_P1 : DDE_142_P0;
}
#ifdef DDE_EMU_TEST_HOOKS
// host test: pick the PDU of dynamic slot ds for this cycle (-1 none, -2 = normal scheduler)
int (*dde_emu_test_dyn_pick)(unsigned ds, unsigned cycle);
#endif

static inline uint16_t slot_id(unsigned i)
{
    if (i < DDE_EMU_N_STATIC) return dde_emu_static_id[i];
    if (i == DDE_EMU_N_STATIC) return DDE_EMU_DYN_ID;
    return dde_dyn_slot_id[i - DDE_EMU_N_SLOTS];
}
static inline bool sat16_inc(volatile uint16_t *v) { if (*v != 0xFFFFu) { (*v)++; return true; } return false; }
static inline int32_t clamp32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline uint16_t fine_now(void) { return (uint16_t)pwm_hw->slice[FLEXRAY_TS_PWM_SLICE].ctr; }

static inline bool tx_allowed(void)
{
    return hw_ready && host_enabled && fault == DDE_EMU_FAULT_NONE && lock.locked &&
           seen_epoch == host_epoch && quiet_cycles >= DDE_EMU_QUIET_CYCLES;
}

// ---------------------------------------------------------------- pins
static void pins_sio_high(void)
{
    const uint pins[4] = {TXD_FR_1_PIN, TXEN_FR_1_PIN, TXD_FR_2_PIN, TXEN_FR_2_PIN};
    for (int i = 0; i < 4; i++) {
        gpio_put(pins[i], 1);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
    }
}

void dde_emu_early_pins(void)
{
    const uint pins[4] = {TXD_FR_1_PIN, TXEN_FR_1_PIN, TXD_FR_2_PIN, TXEN_FR_2_PIN};
    for (int i = 0; i < 4; i++) {
        gpio_init(pins[i]);
        gpio_put(pins[i], 1);
        gpio_set_dir(pins[i], GPIO_OUT);
    }
}

static uint32_t sent_total(void)
{
    uint32_t n = 0;
    for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) n += cnt.sent[i];
    return n;
}

static void __time_critical_func(count_late)(unsigned i, uint8_t reason, int32_t r)
{
    cnt.late++;
    if (late_reason[reason] != 0xFFFFu) late_reason[reason]++;
    last_late_slot = (uint8_t)slot_id(i);
    last_late_reason = reason;
    last_late_r = (int16_t)clamp32(r, -32768, 32767);
    if (i >= DDE_EMU_N_SLOTS) dyn_reason[i - DDE_EMU_N_SLOTS] = DDE_SKIP_LATE;
}

// slot: target index or -1. detail: see dde_emu_stats_t.ff_detail.
static void __time_critical_func(enter_fault)(uint8_t f, int slot, int32_t detail)
{
    uint32_t now = time_us_32();
    fault_evidence_us = now;
    if (fault == DDE_EMU_FAULT_NONE) {
        fault = f;
        last_fault_code = f;
        last_fault_us = now;
        if (fault_count < 255u) fault_count++;
        if (fault_count >= DDE_EMU_FAULT_LATCH_N) fault_latched = true;
        if (first_fault.code == DDE_EMU_FAULT_NONE) {
            first_fault.code = f;
            first_fault.slot = slot >= 0 ? (uint8_t)slot_id((unsigned)slot) : 0;
            first_fault.cycle = cur_cycle;
            first_fault.flags = (uint8_t)((lock.locked ? 1u : 0u) | (tx_expect ? 2u : 0u) |
                                          (f == DDE_EMU_FAULT_TX_STUCK ? 4u : 0u));
            first_fault.us = now;
            first_fault.sent_total = sent_total();
            first_fault.late = cnt.late;
            first_fault.echo_ok = cnt.echo_ok;
            first_fault.echo_miss = cnt.echo_miss;
            first_fault.foreign = cnt.foreign;
            first_fault.detail = detail;
            fault_seq++;
        }
    }
    if (f == DDE_EMU_FAULT_TX_STUCK && hw_ready) {
        // hard stop: take the pins away from PIO first, then stop the SMs
        pins_sio_high();
        pio_sm_set_enabled(DDE_EMU_PIO, sm_tx, false);
        pio_sm_set_enabled(DDE_EMU_PIO, sm_fire, false);
        hw_ready = false;
        tx_expect = false;
        tx_unexp_since = 0;
    }
    for (unsigned i = 0; i < N_ALL; i++) slots[i].state = SL_IDLE;
}

// ---------------------------------------------------------------- alarm
static inline void __time_critical_func(alarm_arm)(uint32_t t)
{
    uint32_t bit = 1u << alarm_num;
    timer_hw->alarm[alarm_num] = t;   // writing ARMS it
    if ((int32_t)(t - time_us_32()) <= 0) hw_set_bits(&timer_hw->intf, bit);  // past: force now
}

static void __time_critical_func(alarm_rearm_next)(uint32_t now)
{
    uint32_t hk = now + DDE_EMU_HK_US;
    if (tx_expect) {   // one housekeeping pass right after the frame's expected end
        uint32_t t = tx_deadline_us + DDE_EMU_WD_MARGIN_US + 1u;
        if ((int32_t)(t - hk) < 0) hk = t;
    }
    bool have = false;
    uint32_t sl = 0;
    for (unsigned i = 0; i < N_ALL; i++) {
        if (slots[i].state == SL_ARMED && (!have || (int32_t)(slots[i].alarm_us - sl) < 0)) {
            sl = slots[i].alarm_us;
            have = true;
        }
    }
    uint32_t best = hk;
    if (have && (int32_t)(sl - hk) <= 0) best = sl;
    else if (have && (int32_t)(sl - hk) < (int32_t)DDE_EMU_HK_GUARD_US) {
        // v1d: a housekeeping pass just before a fire delays the fire ISR (run 3 FB7 lates):
        // run it right after that fire instead (the fire ISR does housekeeping too)
        best = sl;
        sat16_inc(&lat.hk_deferred);
    }
    alarm_arm(best);
}

static inline bool tx_sm_idle(void)
{
    return pio_sm_get_pc(DDE_EMU_PIO, sm_tx) == off_tx + dde_emu_tx_offset_idle &&
           pio_sm_is_tx_fifo_empty(DDE_EMU_PIO, sm_tx);
}
static inline bool fire_sm_idle(void)
{
    return pio_sm_get_pc(DDE_EMU_PIO, sm_fire) == off_fire && pio_sm_is_tx_fifo_empty(DDE_EMU_PIO, sm_fire);
}

static void tx_sm_abort(void);
// Idle, or parked after a frame whose byte count is not a multiple of 4: the
// pad half-word is consumed by "out x, 16" and the SM waits for the FIFO at
// idle+1 with TXEN/TXD high. Restart it (only after the frame's computed end).
static bool __time_critical_func(tx_sm_ready)(uint32_t now)
{
    if (tx_sm_idle()) return true;
    if (tx_parked_ok && (int32_t)(now - tx_deadline_us) >= 0 &&
        pio_sm_get_pc(DDE_EMU_PIO, sm_tx) == off_tx + dde_emu_tx_offset_idle + 1u &&
        pio_sm_is_tx_fifo_empty(DDE_EMU_PIO, sm_tx)) {
        tx_sm_abort();
        tx_parked_ok = false;
        sat16_inc(&dcnt.parks);
        return tx_sm_idle();
    }
    return false;
}

static void __time_critical_func(tx_sm_abort)(void)
{
    // words were loaded but no fire was issued: put the SM back to idle with the pins high.
    pio_sm_set_enabled(DDE_EMU_PIO, sm_tx, false);
    pio_sm_clear_fifos(DDE_EMU_PIO, sm_tx);
    pio_sm_restart(DDE_EMU_PIO, sm_tx);
    pio_sm_exec(DDE_EMU_PIO, sm_tx, pio_encode_set(pio_pins, 3));
    pio_sm_exec(DDE_EMU_PIO, sm_tx, pio_encode_jmp(off_tx + dde_emu_tx_offset_idle));
    pio_interrupt_clear(DDE_EMU_PIO, 5);
    pio_sm_set_enabled(DDE_EMU_PIO, sm_tx, true);
}

// Bring the TX + fire SMs back after FAULT 1 (auto re-arm): both SMs reset to their
// idle instruction with TXEN/TXD driven high by the SM BEFORE the pins go back to PIO.
static void hw_reinit(void)
{
    PIO pio = DDE_EMU_PIO;
    pio_sm_set_enabled(pio, sm_tx, false);
    pio_sm_set_enabled(pio, sm_fire, false);
    pio_sm_clear_fifos(pio, sm_tx);
    pio_sm_clear_fifos(pio, sm_fire);
    pio_sm_restart(pio, sm_tx);
    pio_sm_restart(pio, sm_fire);
    uint32_t mask = (1u << TXEN_FR_1_PIN) | (1u << TXD_FR_1_PIN);
    pio_sm_set_pins_with_mask(pio, sm_tx, mask, mask);
    pio_sm_set_pindirs_with_mask(pio, sm_tx, mask, mask);
    pio_sm_exec(pio, sm_tx, pio_encode_jmp(off_tx + dde_emu_tx_offset_idle));
    pio_sm_exec(pio, sm_fire, pio_encode_jmp(off_fire));
    pio_interrupt_clear(pio, 5);
    pio_gpio_init(pio, TXEN_FR_1_PIN);
    pio_gpio_init(pio, TXD_FR_1_PIN);
    pio_sm_set_enabled(pio, sm_tx, true);
    pio_sm_set_enabled(pio, sm_fire, true);
    hw_ready = true;
}

static void clear_run_state(void)
{
    for (unsigned i = 0; i < N_ALL; i++) { fired[i].pending = false; fired[i].echoed = false; }
    echo_miss_run = 0;
    echo_miss_run_cycles = 0;
    quiet_cycles = 0;
}

static void __time_critical_func(housekeeping)(uint32_t now)
{
    if (dde_lock_poll(&lock, now)) {
        for (unsigned i = 0; i < N_ALL; i++) if (slots[i].state == SL_ARMED || slots[i].state == SL_WAIT) slots[i].state = SL_IDLE;
    }
    if (host_enabled && (int32_t)(now - host_last_us) > (int32_t)DDE_EMU_KEEPALIVE_US) host_enabled = false;
    if (host_clear_req) {   // wValue 0: full reset of the fault state, incl. the latch
        host_clear_req = false;
        if (!hw_ready) hw_reinit();
        fault = DDE_EMU_FAULT_NONE;
        fault_latched = false;
        fault_count = 0;
        auto_clears = 0;
        memset(&first_fault, 0, sizeof(first_fault));
        clear_run_state();
        echo_err_maxabs = 0;
    }
    if (seen_epoch != host_epoch) {
        seen_epoch = host_epoch;
        clear_run_state();
    }
    // watchdog. A committed frame must be off the wire by its computed end + margin.
    // Busy with nothing committed (should never happen) is a fault after 200 us of
    // continuous busy observations; any idle observation restarts that timer.
    if (hw_ready) {
        bool idle = tx_expect ? tx_sm_ready(now) : tx_sm_idle();
        if (tx_expect) {
            if (idle) tx_expect = false;
            else if ((int32_t)(now - (tx_deadline_us + DDE_EMU_WD_MARGIN_US)) > 0)
                enter_fault(DDE_EMU_FAULT_TX_STUCK, last_fired, (int32_t)(now - tx_deadline_us));
            tx_unexp_since = 0;
        } else if (idle) {
            tx_unexp_since = 0;
        } else if (tx_unexp_since == 0) {
            tx_unexp_since = now | 1u;
        } else if ((int32_t)(now - tx_unexp_since) > (int32_t)DDE_EMU_WD_UNEXPECTED_US) {
            enter_fault(DDE_EMU_FAULT_TX_STUCK, -1, -(int32_t)(now - tx_unexp_since));
        }
    }
    // auto re-arm: 2 s without new evidence, unless latched (3rd fault). TX then needs
    // 8 quiet cycles again (a still-present DDE re-faults FOREIGN during those).
    if (fault != DDE_EMU_FAULT_NONE && !fault_latched &&
        (int32_t)(now - fault_evidence_us) > (int32_t)DDE_EMU_FAULT_REARM_US) {
        if (!hw_ready) hw_reinit();
        fault = DDE_EMU_FAULT_NONE;
        if (auto_clears < 255u) auto_clears++;
        clear_run_state();
    }
}

static void __time_critical_func(fire_slot)(unsigned i, uint32_t now)
{
    emu_slot_t *s = &slots[i];
    s->state = SL_DONE;
    if (!tx_allowed()) return;
    int32_t k = k_q4 >> 4;
    if (!tx_sm_ready(now)) { count_late(i, DDE_EMU_LATE_TX_BUSY, 0); return; }
    if (!fire_sm_idle()) { count_late(i, DDE_EMU_LATE_FIRE_BUSY, 0); return; }
    int32_t r0 = dde_core_fire_delay(s->txen_fine, fine_now(), k);
    if (i >= DDE_EMU_N_SLOTS) {
        int32_t il = (int32_t)(now - s->alarm_us);
        if (!lat.any || il < lat.isr_late_min) lat.isr_late_min = (int16_t)clamp32(il, -32768, 32767);
        if (!lat.any || il > lat.isr_late_max) lat.isr_late_max = (int16_t)clamp32(il, -32768, 32767);
        if (!lat.any || r0 < lat.r_entry_min) lat.r_entry_min = (int16_t)clamp32(r0, -32768, 32767);
        if (!lat.any) lat.r_final_min = 32767;
        lat.any = true;
    }
    if (r0 < DDE_EMU_R_PRECHECK) { count_late(i, DDE_EMU_LATE_SHORT, r0); return; }
    uint32_t seq1 = 0;
    flexray_ts_t t1;
    uint32_t pos1;
    flexray_ts_peek_latest(1, &t1, &pos1, &seq1);

    pio_interrupt_clear(DDE_EMU_PIO, 5);
    // the joined TX FIFO holds 8 words: longer frames (0x142, 12 words) get the rest after the fire
    unsigned npre = s->nwords < 8u ? s->nwords : 8u;
    for (unsigned w = 0; w < npre; w++) pio_sm_put(DDE_EMU_PIO, sm_tx, s->words[w]);
    int32_t r = dde_core_fire_delay(s->txen_fine, fine_now(), k);
    if (i >= DDE_EMU_N_SLOTS) {
        // dynamic: wait until the latest possible start of a lower-ID frame has
        // passed, then require that every frame start (side-0 mark) since the
        // anchor was explained by a decoded header. Otherwise do not send.
        unsigned guard = 0;
        while (r > DDE_EMU_R_DYN_CHECK && guard++ < 4000u) {
            DDE_EMU_SPIN_HOOK();
            r = dde_core_fire_delay(s->txen_fine, fine_now(), k);
        }
        flexray_ts_t tm;
        uint32_t pm, sqm;
        unsigned m2 = 0;
        unsigned ds = i - DDE_EMU_N_SLOTS;
        bool ok = flexray_ts_peek_latest(0, &tm, &pm, &sqm) && (uint8_t)sqm == trk.acc_seq &&
                  trk.cycle == s->cycle &&
                  dde_dyn_predict(&trk, slot_id(i), &m2) == DDE_DYN_OK && m2 == s->ms;
        if (!ok) {
            tx_sm_abort();
            sat16_inc(&dcnt.defer);
            dyn_reason[ds] = DDE_SKIP_MARK;
            s->state = SL_WAIT;   // a later header of this cycle may still explain it
            return;
        }
        r = dde_core_fire_delay(s->txen_fine, fine_now(), k);
    }
    if (r < DDE_EMU_R_MIN || r > DDE_EMU_R_MAX) {
        tx_sm_abort();
        count_late(i, DDE_EMU_LATE_WINDOW, r);
        return;
    }
    pio_sm_put(DDE_EMU_PIO, sm_fire, (uint32_t)r);
    if (i >= DDE_EMU_N_SLOTS && r < lat.r_final_min) lat.r_final_min = (int16_t)r;
    if (s->nwords > npre) {
        // top-up: the SM drains a word per 3.2 us once the frame runs; the FIFO always has
        // >= 6 words queued here, so this never underruns unless the core stalls > 20 us.
        // Time guard: a stuck SM must never keep this ISR spinning (then: hard stop, pins high).
        uint32_t t0 = time_us_32();
        for (unsigned w = npre; w < s->nwords; w++) {
            while (pio_sm_is_tx_fifo_full(DDE_EMU_PIO, sm_tx)) {
                if ((uint32_t)(time_us_32() - t0) > DDE_EMU_TOPUP_US) {
                    sat16_inc(&lat.topup_timeouts);
                    enter_fault(DDE_EMU_FAULT_TX_STUCK, (int)i, -(int32_t)w);
                    return;
                }
            }
            pio_sm_put(DDE_EMU_PIO, sm_tx, s->words[w]);
        }
    }
    // committed: the frame is on the wire from now + (r + k) ticks for frame_bits * 100 ns
    {
        unsigned nbytes = (s->words[0] & 0xFFFFu) + 1u;
        unsigned bits = EMU_TSS_BITS + 1u + 10u * nbytes + 2u +
                        (s->words[1] ? s->words[1] + 3u : 0u);
        tx_deadline_us = now + 1u + (uint32_t)((r + k) / 100) + (bits + 9u) / 10u;
        tx_expect = true;
        tx_unexp_since = 0;
        tx_parked_ok = (nbytes & 3u) != 0;
    }
    if (i < DDE_EMU_N_SLOTS) cnt.sent[i]++;
    if (i < DDE_EMU_N_STATIC && s->is_data) {
        uint8_t *c = &counter[i][s->cycle & 3u];
        *c = (uint8_t)((*c + 1u) % DDE_EMU_COUNTER_MOD);
    }
    if (i >= DDE_EMU_N_SLOTS) {
        unsigned ds = i - DDE_EMU_N_SLOTS, p = (unsigned)s->pdu;
        dcnt.sent[p]++;
        dcnt.last_ms[ds] = s->ms;
        dyn_sent_cyc[ds] = true;
        pdu_ctr[p] = (uint8_t)((pdu_ctr[p] + 1u) % dde_core_dyn_ctr_mod(p));
        // periodic grid from the first TX after wake; a retry (skip, +8 cycles) keeps the
        // grid, anything later than half a period restarts it from now
        uint32_t per = dde_dyn_pdu[p].period;
        bool keep = pdu_started[p] && (int32_t)(abs_cycle - pdu_due[p]) < (int32_t)(per / 2u);
        pdu_due[p] = keep ? pdu_due[p] + per : abs_cycle + per;
        pdu_started[p] = true;
    }
    fire_rec_t *f = &fired[i];
    f->pending = true;
    f->echoed = false;
    f->cycle = s->cycle;
    f->txen_fine = s->txen_fine;
    f->bus_fine = s->bus_fine;
    f->us = now;
    f->seq1 = seq1;
    last_fired = (int)i;
}

static void __time_critical_func(alarm_isr)(void)
{
    uint32_t bit = 1u << alarm_num;
    hw_clear_bits(&timer_hw->intf, bit);
    timer_hw->intr = bit;
    uint32_t now = time_us_32();
    for (unsigned i = 0; i < N_ALL; i++) {
        if (slots[i].state == SL_ARMED && (int32_t)(slots[i].alarm_us - now) <= 0) {
            fire_slot(i, now);
            break;   // one per alarm; the next one is >= 27 us away
        }
    }
    housekeeping(now);
    alarm_rearm_next(time_us_32());
}

// ---------------------------------------------------------------- header hook
// Pre-ign content until DDE_IGN_STATIC_CYCLES after the first 0x103 since wake.
static inline bool preign_static(void)
{
    // v1e: KL15 on (0xF1) also ends the pre-ign content, not only the first 0x103.
    if (kl15.on && abs_cycle >= kl15.on_abs + DDE_IGN_STATIC_CYCLES) return false;
    return EMU_PREIGN && (!ign_seen || abs_cycle < ign_abs + DDE_IGN_STATIC_CYCLES);
}

// v1e: content follows KL15, not only the (sticky) "0x103 seen in this wake" flag:
// after KL15 off the real DDE moves 0x29 c%4==3 b8 to c0 (+30 ms) and 0xFB L9 back to
// 1f 00 ff 7f (+0.23 s); 0x142 already has its own after-off phase (ph142).
static inline bool kl15_off_for(uint32_t cycles)
{
    return kl15.ever_on && !kl15.on && abs_cycle >= kl15.off_abs + cycles;
}

static inline unsigned static_phase(void)
{
    if (kl15_off_for(DDE_OFF_29_CYCLES)) return DDE_PH_OFF;
    return preign_static() ? DDE_PH_PREIGN : DDE_PH_ON;
}

static inline unsigned dyn_phase(unsigned p)
{
    if (p == DDE_PDU_FB9 && kl15_off_for(DDE_OFF_FB9_CYCLES)) return 0u; // pre-ign content = ff 7f
    return preign_static() ? 0u : 1u;
}

static void __time_critical_func(prepare_slot)(unsigned i, unsigned cycle)
{
    emu_slot_t *s = &slots[i];
    uint8_t frame[DDE_MAX_FRAME_BYTES];
    unsigned n, dts = 0;
    s->pdu = -1;
    if (i < DDE_EMU_N_STATIC) {
        n = dde_core_build_static_ph(i, cycle, counter[i][cycle & 3u], EMU_SYNC_BITS != 0, static_phase(), frame);
        s->is_data = (dde_core_emu_indicators(i, cycle, false) & 0x04u) != 0;
    } else if (i >= DDE_EMU_N_SLOTS) {
        unsigned p = (unsigned)dyn_due[i - DDE_EMU_N_SLOTS];
        unsigned ph = p == DDE_PDU_142 ? ph142() : dyn_phase(p);
        n = dde_core_build_dyn_pdu_ph(p, cycle, pdu_ctr[p], ph, frame);
        dts = dde_core_dyn_dts_bits(n, EMU_TSS_BITS);
        s->is_data = 1;
        s->pdu = (int8_t)p;
    } else {
        n = dde_core_build_dyn(cycle, frame);
        dts = DDE_EMU_DYN_DTS_BITS;
        s->is_data = 1;
    }
    s->nwords = (uint8_t)dde_core_pack(frame, n, EMU_TSS_BITS, dts, s->words);
    s->cycle = (uint8_t)cycle;
}

// A fire whose echo did not arrive (called at the next cycle's first reference).
static void __time_critical_func(echo_missed)(unsigned i)
{
    fired[i].pending = false;
    cnt.echo_miss++;
    if (echo_miss_run == 0 || fired[i].cycle != echo_miss_last_cycle) echo_miss_run_cycles++;
    echo_miss_last_cycle = fired[i].cycle;
    echo_miss_run++;
    // refs are present (we are called from a reference frame) and the lock holds
    if (lock.locked && echo_miss_run >= DDE_EMU_ECHO_MISS_FAULT && echo_miss_run_cycles >= 2u)
        enter_fault(DDE_EMU_FAULT_NO_ECHO, (int)i, (int32_t)echo_miss_run);
}

static void __time_critical_func(on_echo)(unsigned i, unsigned cycle, const flexray_ts_t *t0, bool t0_ok)
{
    fire_rec_t *f = &fired[i];
    uint32_t dt = time_us_32() - f->us;
    bool ours_cycle = f->cycle == cycle && dt < DDE_EMU_ECHO_WINDOW_US;
    if (ours_cycle && f->echoed) { echo_dup++; return; }   // same frame reported twice: never foreign
    if (!(ours_cycle && f->pending)) {
        // an ID we own, in a cycle/slot where we did not transmit: another DDE on the bus
        cnt.foreign++;
        enter_fault(DDE_EMU_FAULT_FOREIGN, (int)i, f->cycle == cycle ? (int32_t)dt : -1);
        return;
    }
    f->pending = false;
    f->echoed = true;
    echo_miss_run = 0;
    echo_miss_run_cycles = 0;
    cnt.echo_ok++;
    if (!t0_ok) return;
    flexray_ts_t t1;
    uint32_t pos1, seq1;
    if (!flexray_ts_peek_latest(1, &t1, &pos1, &seq1)) return;
    if (last_fired != (int)i) return;   // side-1 mark belongs to a later frame
    if (((seq1 - f->seq1) & (FLEXRAY_TS_RING_ENTRIES - 1u)) != 1u) return;
    int32_t fire_err = (int16_t)(uint16_t)(t1.fine - f->txen_fine);
    int32_t loop = (int16_t)(uint16_t)(t0->fine - t1.fine);
    int32_t echo_err = (int16_t)(uint16_t)(t0->fine - f->bus_fine);
    if (fire_err < -300 || fire_err > 300 || loop < -100 || loop > 200) return;  // not plausible
    echo_err_q4 += ((echo_err * 16) - echo_err_q4) / 8;
    fire_err_q4 += ((fire_err * 16) - fire_err_q4) / 8;
    int32_t ae = echo_err < 0 ? -echo_err : echo_err;
    if (ae > echo_err_maxabs) echo_err_maxabs = ae;
    // fire latency: we fired fire_err ticks late -> subtract that much more next time
    if (!k_cal) { k_q4 += fire_err * 16; k_cal = true; }
    else k_q4 += fire_err * 2;               // 1/8 gain
    k_q4 = clamp32(k_q4, 0, 200 * 16);
    if (!loop_cal) { loop_q4 = loop * 16; loop_cal = true; }
    else loop_q4 += ((loop * 16) - loop_q4) / 8;
    loop_q4 = clamp32(loop_q4, 0, 60 * 16);
}

// ---- v1c dynamic segment
static void __time_critical_func(dyn_skip)(unsigned ds, unsigned reason)
{
    int p = dyn_due[ds];
    if (p < 0) return;
    sat16_inc(&dcnt.skip[p]);
    if (reason == 0 || reason >= DDE_SKIP_N) reason = DDE_SKIP_NOANCHOR;
    sat16_inc(&dcnt.reason[reason]);
}

// New cycle (first reference of it): close the old cycle, pick what is due.
static void __time_critical_func(dyn_new_cycle)(int ev, unsigned cycle, uint8_t seq0)
{
    for (unsigned ds = 0; ds < DDE_DYN_N_SLOT; ds++) {
        if (dyn_due[ds] >= 0 && !dyn_sent_cyc[ds])
            dyn_skip(ds, slots[DSLOT(ds)].state == SL_ARMED ? DDE_SKIP_LATE : dyn_reason[ds]);
        dyn_due[ds] = -1;
        dyn_sent_cyc[ds] = false;
    }
    uint32_t now = time_us_32();
    if (ev == DDE_LOCK_NEW_WAKE) {
        abs_cycle = 0;
        ign_seen = false;
        for (unsigned p = 0; p < DDE_DYN_N_PDU; p++) { pdu_due[p] = 0; pdu_ctr[p] = 1; pdu_started[p] = false; }
        dde_kl15_reset(&kl15, now);
        f1_seq_seen = f1mb.seq;   // an 0xF1 of the previous wake does not count
    } else {
        abs_cycle += (cycle - abs_last_cycle) & 63u;
    }
    abs_last_cycle = (uint8_t)cycle;
    // v1d KL15 (0xF1 b3 from core0, 0x103 from the header hook) and the 0x5C NM gate
    uint32_t sq = f1mb.seq;
    if (sq != f1_seq_seen) {
        __dmb();
        uint8_t b3 = f1mb.b3;
        uint32_t fus = f1mb.us;
        f1_seq_seen = sq;
        if (EMU_KL15_F1) dde_kl15_f1(&kl15, b3, fus);
        f1_last_b3 = b3;
    }
    dde_kl15_eval(&kl15, now, abs_cycle, EMU_KL15_F1 != 0);
    nm_ok = dde_nm_allowed(&kl15, abs_cycle, now, &nm_ceiling);
    if (nm_ceiling && dde_core_dyn_cycle(cycle)) sat16_inc(&lat.nm_ceiling_stops);
    dde_dyn_track_start(&trk, cycle, true, seq0);
    if (!tx_allowed()) return;
    for (unsigned ds = 0; ds < DDE_DYN_N_SLOT; ds++) {
        if (!dyn_enabled(ds)) continue;
        int p = -2;
#ifdef DDE_EMU_TEST_HOOKS
        if (dde_emu_test_dyn_pick) p = dde_emu_test_dyn_pick(ds, cycle);
        if (p == -1) continue;
#endif
        if (p == -2) {
            p = dde_core_dyn_pdu_for(ds, cycle);
            if (p < 0) continue;
            bool due = (int32_t)(abs_cycle - pdu_due[p]) >= 0;
            // KL15-gated PDUs (0xE8 L5, 0x10A): only while KL15 is on, from start_cyc after the on edge
            if (due && dde_dyn_pdu[p].ign_only) due = kl15.on && abs_cycle >= kl15.on_abs + dde_dyn_pdu[p].start_cyc;
            if (!due) continue;
        }
        dyn_due[ds] = (int8_t)p;
        dyn_reason[ds] = DDE_DYN_OK;   // nothing has stopped it yet (a skip without a reason counts as noanchor)
        prepare_slot(DSLOT(ds), cycle);
        slots[DSLOT(ds)].state = SL_WAIT;
    }
}

// (Re)plan the due dynamic frames of this cycle from the tracker.
static bool __time_critical_func(dyn_plan)(void)
{
    if (!tx_allowed()) return false;
    bool changed = false;
    int32_t loop = loop_q4 >> 4;
    for (unsigned ds = 0; ds < DDE_DYN_N_SLOT; ds++) {
        emu_slot_t *s = &slots[DSLOT(ds)];
        if (dyn_due[ds] < 0 || dyn_sent_cyc[ds] || (s->state != SL_WAIT && s->state != SL_ARMED)) continue;
        unsigned m = 0;
        int why = dde_dyn_predict(&trk, dde_dyn_slot_id[ds], &m);
        if (why != DDE_DYN_OK) {
            if (s->state == SL_ARMED) changed = true;
            s->state = why == DDE_SKIP_PASSED ? SL_DONE : SL_WAIT;
            // PASSED (a higher ID came first) keeps the reason that stopped us before it
            if (why != DDE_SKIP_PASSED || dyn_reason[ds] == DDE_DYN_OK) dyn_reason[ds] = (uint8_t)why;
            continue;
        }
        if (s->state == SL_ARMED && s->ms == m) continue;
        int32_t delta = dde_core_dyn_ap_q8(m) - dref_off;
        if (delta < DDE_EMU_MIN_DELTA_Q8 / 4) { s->state = SL_WAIT; dyn_reason[ds] = DDE_SKIP_LATE; continue; }
        dde_sched_t sc;
        dde_core_schedule(dref_fine, dref_us, delta, loop, DDE_EMU_LEAD_DYN_US, &sc);
        // a late header (IRQ latency) can move the slot to a time already passed: not a "late"
        if ((int32_t)(sc.alarm_us - time_us_32()) < 0) {
            if (s->state == SL_ARMED) changed = true;
            s->state = SL_WAIT;
            dyn_reason[ds] = DDE_SKIP_LATE;
            continue;
        }
        s->bus_fine = sc.bus_fine;
        s->txen_fine = sc.txen_fine;
        s->alarm_us = sc.alarm_us;
        s->ms = (uint16_t)m;
        s->state = SL_ARMED;
        changed = true;
    }
    return changed;
}

// Every FR1 header of the current cycle (ours included) goes through the tracker.
static void __time_critical_func(dyn_feed)(unsigned id, unsigned plen, unsigned cycle, const flexray_ts_t *t0,
                                          bool t0_ok, uint8_t seq0)
{
    if (!trk.active || cycle != trk.cycle) return;
    int64_t x_q8 = 0;
    if (id >= DDE_DYN_FIRST_ID && t0_ok)
        x_q8 = (int64_t)dde_core_elapsed_ticks(dref_us, dref_fine, t0->us, t0->fine) * 256 + dref_off;
    dde_dyn_track_header(&trk, id, plen, t0_ok, seq0, x_q8);
    if (dyn_plan()) alarm_rearm_next(time_us_32());
}

void __time_critical_func(dde_emu_on_fr1_header)(const uint8_t h[5], uint32_t start_pos, uint32_t ring_mask)
{
    if (!hw_ready && fault == DDE_EMU_FAULT_NONE) return;
    unsigned id = ((unsigned)(h[0] & 7u) << 8) | h[1];
    unsigned cycle = h[4] & 0x3Fu;
    unsigned plen = ((h[2] >> 1) & 0x7Fu) * 2u;
    flexray_ts_t t0;
    uint32_t pos0, seq0;
    bool t0_ok = flexray_ts_peek_latest(0, &t0, &pos0, &seq0) && ((pos0 - start_pos) & ring_mask) == 0;

    if (id == 0x103u) {   // ign-on indicator (run 5: every 200-220 ms while ign is on)
        uint32_t irq = save_and_disable_interrupts();
        last_103_us = time_us_32();
        if (!ign_seen && lock.have) { ign_seen = true; ign_abs = abs_cycle; }
        if (lock.have) dde_kl15_103(&kl15, last_103_us);
        restore_interrupts(irq);
    }

    int ti = dde_core_target_index(id);
    if (EMU_DYN_5C && ti < 0 && id == DDE_EMU_DYN_ID && dde_core_dyn_cycle(cycle)) ti = DDE_EMU_N_STATIC;
    if (ti < 0) {
        int ds = dde_core_dyn_slot_of_id(id);
        if (ds >= 0 && dyn_enabled((unsigned)ds)) ti = (int)DSLOT(ds);
    }
    if (ti >= 0) {
        uint32_t irq = save_and_disable_interrupts();
        on_echo((unsigned)ti, cycle, &t0, t0_ok);
        dyn_feed(id, plen, cycle, &t0, t0_ok, (uint8_t)seq0);
        restore_interrupts(irq);
        return;
    }
    int ri = dde_core_ref_index(id);
    if (ri < 0 || !t0_ok) {
        uint32_t irq = save_and_disable_interrupts();
        dyn_feed(id, plen, cycle, &t0, t0_ok, (uint8_t)seq0);
        restore_interrupts(irq);
        return;
    }

    int32_t ref_off = dde_emu_ref_off_q8[ri];
    uint32_t cs_us = t0.us - (uint32_t)(ref_off / 25600);
    uint32_t irq = save_and_disable_interrupts();
    int ev = dde_lock_on_ref(&lock, cycle, cs_us);
    if (ev == DDE_LOCK_NEW_WAKE) {
        for (unsigned i = 0; i < DDE_EMU_N_STATIC; i++) for (unsigned c = 0; c < 4; c++) counter[i][c] = 1;
    }
    if (ev != DDE_LOCK_SAME_CYCLE) {
        cur_cycle = (uint8_t)cycle;
        // dynamic bookkeeping of the cycle that ended (before the slots are reset)
        dref_fine = t0.fine; dref_us = t0.us; dref_off = ref_off;
        if (ev != DDE_LOCK_BAD) dyn_new_cycle(ev, cycle, (uint8_t)seq0);
        else trk.active = false;
        for (unsigned i = 0; i < N_ALL; i++) {
            if (slots[i].state == SL_ARMED && slots[i].cycle != cycle) count_late(i, DDE_EMU_LATE_MISSED, 0);
            if (slots[i].cycle != cycle && i < DDE_EMU_N_SLOTS) slots[i].state = SL_IDLE;
            if (i >= DDE_EMU_N_SLOTS && dyn_due[i - DDE_EMU_N_SLOTS] < 0) slots[i].state = SL_IDLE;
            if (fired[i].pending && fired[i].cycle != cycle) {
                if (ev == DDE_LOCK_NEW_CYCLE) echo_missed(i);
                else fired[i].pending = false;   // wake / bad ref: no verdict
            }
        }
        if (ev == DDE_LOCK_NEW_CYCLE && fault == DDE_EMU_FAULT_NONE && host_enabled &&
            seen_epoch == host_epoch && lock.locked && quiet_cycles < DDE_EMU_QUIET_CYCLES) {
            quiet_cycles++;
        }
    }
    if (lock.locked && !was_locked) cnt.rearms++;
    was_locked = lock.locked;
    if (trk.active && trk.cycle == cycle) { dref_fine = t0.fine; dref_us = t0.us; dref_off = ref_off; }

    bool changed = false;
    if (tx_allowed()) {
        int32_t loop = loop_q4 >> 4;
        for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) {
            int32_t tgt;
            if (i < DDE_EMU_N_STATIC) tgt = dde_emu_static_off_q8[i];
            else {
                // v1d: NM rules + hard ceiling (dyn_new_cycle) - never wakes / holds the bus
                if (!EMU_DYN_5C || !dde_core_dyn_cycle(cycle) || !nm_ok || !trk.active || trk.cycle != cycle) continue;
                tgt = dde_emu_dyn_off_q8;
            }
            int32_t delta = tgt - ref_off;
            if (delta < DDE_EMU_MIN_DELTA_Q8) continue;
            emu_slot_t *s = &slots[i];
            if (s->state == SL_DONE) continue;
            if (s->state == SL_IDLE) prepare_slot(i, cycle);
            dde_sched_t sc;
            dde_core_schedule(t0.fine, t0.us, delta, loop, DDE_EMU_LEAD_US, &sc);
            s->bus_fine = sc.bus_fine;
            s->txen_fine = sc.txen_fine;
            s->alarm_us = sc.alarm_us;
            s->state = SL_ARMED;
            changed = true;
        }
    }
    // the reference is a frame too: tracker + dynamic plan from this reference
    if (trk.active && trk.cycle == cycle) {
        dde_dyn_track_header(&trk, id, plen, true, (uint8_t)seq0, 0);
        changed |= dyn_plan();
    }
    if (changed) alarm_rearm_next(time_us_32());
    restore_interrupts(irq);
}

// ---------------------------------------------------------------- init
void dde_emu_init_core1(void)
{
    dde_core_init();
    dde_lock_reset(&lock);
    for (unsigned i = 0; i < DDE_EMU_N_STATIC; i++) for (unsigned c = 0; c < 4; c++) counter[i][c] = 1;
    for (unsigned p = 0; p < DDE_DYN_N_PDU; p++) { pdu_ctr[p] = 1; pdu_due[p] = 0; }
    for (unsigned d = 0; d < DDE_DYN_N_SLOT; d++) dyn_due[d] = -1;
    for (unsigned i = 0; i < N_ALL; i++) slots[i].pdu = -1;

    PIO pio = DDE_EMU_PIO;
    off_tx = pio_add_program(pio, &dde_emu_tx_program);
    off_fire = pio_add_program(pio, &dde_emu_fire_program);
    sm_tx = (uint)pio_claim_unused_sm(pio, true);
    sm_fire = (uint)pio_claim_unused_sm(pio, true);

    // TX SM: set pins = TXEN (base) + TXD (base+1), out pin = TXD. Idle level high on both
    // BEFORE the pins are handed to PIO, so there is no glitch.
    _Static_assert(TXD_FR_1_PIN == TXEN_FR_1_PIN + 1, "set pins need TXEN, TXD consecutive");
    pio_sm_config c = dde_emu_tx_program_get_default_config(off_tx);
    sm_config_set_set_pins(&c, TXEN_FR_1_PIN, 2);
    sm_config_set_out_pins(&c, TXD_FR_1_PIN, 1);
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, 1, 0);
    pio_sm_init(pio, sm_tx, off_tx, &c);
    uint32_t mask = (1u << TXEN_FR_1_PIN) | (1u << TXD_FR_1_PIN);
    pio_sm_set_pins_with_mask(pio, sm_tx, mask, mask);
    pio_sm_set_pindirs_with_mask(pio, sm_tx, mask, mask);
    pio_gpio_init(pio, TXEN_FR_1_PIN);
    pio_gpio_init(pio, TXD_FR_1_PIN);

    pio_sm_config cf = dde_emu_fire_program_get_default_config(off_fire);
    sm_config_set_out_shift(&cf, false, true, 32);
    sm_config_set_clkdiv_int_frac8(&cf, 1, 0);
    pio_sm_init(pio, sm_fire, off_fire, &cf);

    pio_interrupt_clear(pio, 5);
    pio_sm_set_enabled(pio, sm_tx, true);
    pio_sm_set_enabled(pio, sm_fire, true);

    alarm_num = hardware_alarm_claim_unused(true);
    uint irq = timer_hardware_alarm_get_irq_num(timer_hw, (uint)alarm_num);
    irq_set_exclusive_handler(irq, alarm_isr);
    irq_set_priority(irq, 0);
    hw_set_bits(&timer_hw->inte, 1u << alarm_num);
    irq_set_enabled(irq, true);
    hw_ready = true;
    alarm_arm(time_us_32() + DDE_EMU_HK_US);
}

// ---------------------------------------------------------------- host side (core0)
void dde_emu_host_set(uint16_t wvalue)
{
    if (wvalue == 0) {
        host_enabled = false;
        host_clear_req = true;
        printf("emu: TX disabled by host\n");
        return;
    }
    host_last_us = time_us_32();
    if (!host_enabled) {
        host_epoch++;
        host_enabled = true;
        printf("emu: TX enabled by host (TSS %u, sync bits %u, dyn 0x5C %u, E6 %u, FB %u, E8 %u, pre-ign %u, "
               "10A %u, 142 %u, 68 %u, KL15 F1 %u)\n",
               (unsigned)EMU_TSS_BITS, (unsigned)EMU_SYNC_BITS, (unsigned)EMU_DYN_5C, (unsigned)EMU_DYN_E6,
               (unsigned)EMU_DYN_FB, (unsigned)EMU_DYN_E8, (unsigned)EMU_PREIGN, (unsigned)EMU_10A,
               (unsigned)EMU_142, (unsigned)EMU_68, (unsigned)EMU_KL15_F1);
    }
}

void dde_emu_fill_stats(dde_emu_stats_t *st)
{
    memset(st, 0, sizeof(*st));
    st->version = DDE_EMU_STATS_VERSION;
    st->enabled = host_enabled;
    st->fault = fault;
    if (fault) st->state = DDE_EMU_ST_FAULT;
    else if (!host_enabled) st->state = DDE_EMU_ST_DISABLED;
    else if (!lock.locked) st->state = DDE_EMU_ST_NO_LOCK;
    else if (seen_epoch != host_epoch || quiet_cycles < DDE_EMU_QUIET_CYCLES) st->state = DDE_EMU_ST_ARMING;
    else st->state = DDE_EMU_ST_ACTIVE;
    for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) st->sent[i] = cnt.sent[i];
    st->lock_losses = lock.lock_losses;
    st->late = cnt.late;
    st->echo_ok = cnt.echo_ok;
    st->echo_miss = cnt.echo_miss;
    st->foreign = cnt.foreign;
    st->cycles_good = lock.good_cycles;
    st->echo_err_avg_ns = (int16_t)((echo_err_q4 * 10) / 16);
    st->echo_err_maxabs_ns = (int16_t)clamp32(echo_err_maxabs * 10, 0, 32767);
    st->fire_err_avg_ns = (int16_t)((fire_err_q4 * 10) / 16);
    st->loop_delay_ns = (uint16_t)((loop_q4 * 10) / 16);
    st->fire_k_ticks = (uint16_t)(k_q4 >> 4);
    st->tss_bits = EMU_TSS_BITS;
    st->flags = (uint8_t)((EMU_SYNC_BITS ? 1u : 0u) | (EMU_DYN_5C ? 2u : 0u) | (lock.locked ? 4u : 0u));
    st->rearms = cnt.rearms;
    // v2
    st->ff_code = first_fault.code;
    st->ff_slot = first_fault.slot;
    st->ff_cycle = first_fault.cycle;
    st->ff_flags = first_fault.flags;
    st->ff_us = first_fault.us;
    st->ff_sent_total = first_fault.sent_total;
    st->ff_late = first_fault.late;
    st->ff_echo_ok = first_fault.echo_ok;
    st->ff_echo_miss = first_fault.echo_miss;
    st->ff_foreign = first_fault.foreign;
    st->ff_detail = first_fault.detail;
    st->fault_count = fault_count;
    st->auto_clears = auto_clears;
    st->latched = fault_latched ? 1u : 0u;
    st->last_fault = last_fault_code;
    for (unsigned r = 1; r < DDE_EMU_LATE_N; r++) st->late_reason[r - 1] = late_reason[r];
    st->last_late_slot = last_late_slot;
    st->last_late_reason = last_late_reason;
    st->last_late_r = last_late_r;
    st->echo_dup = (uint16_t)(echo_dup > 0xFFFFu ? 0xFFFFu : echo_dup);
    st->last_fault_us = last_fault_us;
    st->now_us = time_us_32();
    // v3
    for (unsigned p = 0; p < DDE_DYN_V3_N_PDU; p++) { st->dyn_sent[p] = dcnt.sent[p]; st->dyn_skip[p] = dcnt.skip[p]; }
    for (unsigned r = 1; r < DDE_SKIP_N; r++) st->dyn_skip_reason[r - 1] = dcnt.reason[r];
    st->dyn_defer = dcnt.defer;
    for (unsigned d = 0; d < DDE_DYN_V3_N_SLOT; d++) st->dyn_last_ms[d] = dcnt.last_ms[d];
    bool ign_now = ign_seen && (uint32_t)(st->now_us - last_103_us) < DDE_EMU_IGN_TIMEOUT_US;
    st->ign = (uint8_t)((ign_seen ? 1u : 0u) | (!preign_static() ? 2u : 0u) |
                        (kl15.on && abs_cycle >= kl15.on_abs + DDE_IGN_E8L5_CYCLES ? 4u : 0u) | (ign_now ? 8u : 0u));
    st->dyn_flags = (uint8_t)((EMU_DYN_E6 ? 1u : 0u) | (EMU_DYN_FB ? 2u : 0u) | (EMU_DYN_E8 ? 4u : 0u) |
                              (EMU_PREIGN ? 8u : 0u));
    st->dyn_parks = dcnt.parks;
    // v4
    for (unsigned j = 0; j < 3; j++) {
        st->dyn_sent2[j] = dcnt.sent[DDE_PDU_10A + j];
        st->dyn_skip2[j] = dcnt.skip[DDE_PDU_10A + j];
        st->dyn_last_ms2[j] = dcnt.last_ms[DDE_DSLOT_10A + j];
    }
    st->kl15 = (uint8_t)((kl15.on ? 1u : 0u) | (kl15.ever_on ? 2u : 0u) | (kl15.f1_seen ? 4u : 0u) |
                         (kl15.c103_seen ? 8u : 0u) | (nm_ok ? 16u : 0u) | (nm_ceiling ? 32u : 0u) |
                         (kl15.evid_seen ? 64u : 0u));
    st->f1_b3 = f1_last_b3;
    st->kl15_on_edges = (uint16_t)(kl15.on_edges > 0xFFFFu ? 0xFFFFu : kl15.on_edges);
    st->kl15_off_edges = (uint16_t)(kl15.off_edges > 0xFFFFu ? 0xFFFFu : kl15.off_edges);
    st->nm_ceiling_stops = lat.nm_ceiling_stops;
    st->kl15_off_ago_ms = (kl15.off_edges && !kl15.on) ? (abs_cycle - kl15.off_abs) * 5u : 0xFFFFFFFFu;
    {
        uint32_t ref = kl15.wake_us;
        if (kl15.evid_seen && (int32_t)(kl15.evid_us - ref) > 0) ref = kl15.evid_us;
        uint32_t el = st->now_us - ref;
        st->ceil_left_ms = el >= DDE_NM_CEIL_US ? 0u : (DDE_NM_CEIL_US - el) / 1000u;
    }
    st->isr_late_min_us = lat.isr_late_min;
    st->isr_late_max_us = lat.isr_late_max;
    st->r_entry_min = lat.r_entry_min;
    st->r_final_min = lat.any ? lat.r_final_min : 0;
    st->topup_timeouts = lat.topup_timeouts;
    st->hk_deferred = lat.hk_deferred;
    st->dyn_flags2 = (uint8_t)((EMU_10A ? 1u : 0u) | (EMU_142 ? 2u : 0u) | (EMU_68 ? 4u : 0u) |
                               (EMU_KL15_F1 ? 8u : 0u) | (EMU_DYN_5C ? 16u : 0u));
    st->ph142 = (uint8_t)ph142();
    st->abs_cycle = abs_cycle;
}

void dde_emu_on_frame(unsigned id, unsigned cycle, const uint8_t *payload, unsigned nbytes)
{
    (void)cycle;
    if (id != 0xF1u || nbytes < 3u) return;
    f1mb.b3 = payload[2];
    f1mb.us = time_us_32();
    __dmb();
    f1mb.seq = f1mb.seq + 1u;
}

const char *dde_emu_fault_name(unsigned code)
{
    static const char *const fn[] = {"none", "TX state machine stuck", "foreign DDE frame", "no echo"};
    return code < 4u ? fn[code] : "?";
}

void dde_emu_print_stats(void)
{
    static const char *const sn[] = {"disabled", "no-lock", "arming", "ACTIVE", "FAULT"};
    dde_emu_stats_t s;
    dde_emu_fill_stats(&s);
    static uint32_t printed_seq;
    if (s.ff_code && fault_seq != printed_seq) {   // once per first fault
        printed_seq = fault_seq;
        printf("EMU FAULT %u \"%s\" slot=0x%02X cycle=%u t=%luus (%lu ms ago) detail=%ld "
               "sent=%lu late=%lu echo_ok=%lu miss=%lu foreign=%lu flags=0x%02X\n",
               s.ff_code, dde_emu_fault_name(s.ff_code), s.ff_slot, s.ff_cycle, (unsigned long)s.ff_us,
               (unsigned long)((s.now_us - s.ff_us) / 1000u), (long)s.ff_detail,
               (unsigned long)s.ff_sent_total, (unsigned long)s.ff_late, (unsigned long)s.ff_echo_ok,
               (unsigned long)s.ff_echo_miss, (unsigned long)s.ff_foreign, s.ff_flags);
    }
    printf("emu: state=%s fault=%u sent[26=%lu 28=%lu 29=%lu 3D=%lu 5C=%lu] lock_loss=%lu late=%lu "
           "echo ok=%lu miss=%lu foreign=%lu err avg=%dns max=%dns fire_err=%dns loop=%uns K=%u\n",
           sn[s.state], s.fault, (unsigned long)s.sent[0], (unsigned long)s.sent[1], (unsigned long)s.sent[2],
           (unsigned long)s.sent[3], (unsigned long)s.sent[4], (unsigned long)s.lock_losses,
           (unsigned long)s.late, (unsigned long)s.echo_ok, (unsigned long)s.echo_miss,
           (unsigned long)s.foreign, s.echo_err_avg_ns, s.echo_err_maxabs_ns, s.fire_err_avg_ns,
           s.loop_delay_ns, s.fire_k_ticks);
    printf("emu: faults=%u auto_clears=%u latched=%u late[busy=%u fire=%u short=%u win=%u missed=%u] "
           "last_late=0x%02X/%u r=%d\n", s.fault_count, s.auto_clears, s.latched, s.late_reason[0],
           s.late_reason[1], s.late_reason[2], s.late_reason[3], s.late_reason[4], s.last_late_slot,
           s.last_late_reason, s.last_late_r);
    printf("emu: dyn E6=%lu/%u FB5=%lu/%u FB7=%lu/%u FB9=%lu/%u E89=%lu/%u E85=%lu/%u (sent/skip) "
           "skip[anchor=%u mark=%u passed=%u len=%u grid=%u limit=%u late=%u] defer=%u ms[E6=%u E8=%u FB=%u] "
           "ign=0x%X build=0x%X parks=%u\n",
           (unsigned long)s.dyn_sent[0], s.dyn_skip[0], (unsigned long)s.dyn_sent[1], s.dyn_skip[1],
           (unsigned long)s.dyn_sent[2], s.dyn_skip[2], (unsigned long)s.dyn_sent[3], s.dyn_skip[3],
           (unsigned long)s.dyn_sent[4], s.dyn_skip[4], (unsigned long)s.dyn_sent[5], s.dyn_skip[5],
           s.dyn_skip_reason[0], s.dyn_skip_reason[1], s.dyn_skip_reason[2], s.dyn_skip_reason[3],
           s.dyn_skip_reason[4], s.dyn_skip_reason[5], s.dyn_skip_reason[6], s.dyn_defer,
           s.dyn_last_ms[0], s.dyn_last_ms[1], s.dyn_last_ms[2], s.ign, s.dyn_flags, s.dyn_parks);
    printf("emu: v1d 10A=%lu/%u 142=%lu/%u 68=%lu/%u ms[10A=%u 142=%u 68=%u] kl15=0x%02X f1=%02X edges=%u/%u "
           "off_ago=%ldms ceil_left=%lums ceil_stops=%u ph142=%u isr_late=%d..%dus r_entry_min=%d r_final_min=%d "
           "topup_to=%u hk_def=%u build2=0x%X\n",
           (unsigned long)s.dyn_sent2[0], s.dyn_skip2[0], (unsigned long)s.dyn_sent2[1], s.dyn_skip2[1],
           (unsigned long)s.dyn_sent2[2], s.dyn_skip2[2], s.dyn_last_ms2[0], s.dyn_last_ms2[1], s.dyn_last_ms2[2],
           s.kl15, s.f1_b3, s.kl15_on_edges, s.kl15_off_edges,
           s.kl15_off_ago_ms == 0xFFFFFFFFu ? -1L : (long)s.kl15_off_ago_ms, (unsigned long)s.ceil_left_ms,
           s.nm_ceiling_stops, s.ph142, s.isr_late_min_us, s.isr_late_max_us, s.r_entry_min, s.r_final_min,
           s.topup_timeouts, s.hk_deferred, s.dyn_flags2);
}
