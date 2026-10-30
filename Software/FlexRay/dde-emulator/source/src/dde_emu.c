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
#define EMU_DYN_5C 0
#endif

#define DDE_EMU_PIO pio0
#define DDE_EMU_LEAD_US 6u              // alarm this long before the TXEN target
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

enum { SL_IDLE = 0, SL_ARMED = 1, SL_DONE = 2 };

typedef struct {
    uint8_t state;
    uint8_t cycle;
    uint8_t nwords;
    uint8_t is_data;
    uint16_t txen_fine;
    uint16_t bus_fine;
    uint32_t alarm_us;
    uint32_t words[DDE_TX_MAX_WORDS];
} emu_slot_t;

static emu_slot_t slots[DDE_EMU_N_SLOTS];
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
static fire_rec_t fired[DDE_EMU_N_SLOTS];
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

static inline uint8_t slot_id(unsigned i) { return (uint8_t)(i < DDE_EMU_N_STATIC ? dde_emu_static_id[i] : DDE_EMU_DYN_ID); }
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
    last_late_slot = slot_id(i);
    last_late_reason = reason;
    last_late_r = (int16_t)clamp32(r, -32768, 32767);
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
            first_fault.slot = slot >= 0 ? slot_id((unsigned)slot) : 0;
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
    for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) slots[i].state = SL_IDLE;
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
    uint32_t best = now + DDE_EMU_HK_US;
    if (tx_expect) {   // one housekeeping pass right after the frame's expected end
        uint32_t t = tx_deadline_us + DDE_EMU_WD_MARGIN_US + 1u;
        if ((int32_t)(t - best) < 0) best = t;
    }
    for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) {
        if (slots[i].state == SL_ARMED && (int32_t)(slots[i].alarm_us - best) < 0) best = slots[i].alarm_us;
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
    for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) { fired[i].pending = false; fired[i].echoed = false; }
    echo_miss_run = 0;
    echo_miss_run_cycles = 0;
    quiet_cycles = 0;
}

static void __time_critical_func(housekeeping)(uint32_t now)
{
    if (dde_lock_poll(&lock, now)) {
        for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) if (slots[i].state == SL_ARMED) slots[i].state = SL_IDLE;
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
        bool idle = tx_sm_idle();
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
    if (!tx_sm_idle()) { count_late(i, DDE_EMU_LATE_TX_BUSY, 0); return; }
    if (!fire_sm_idle()) { count_late(i, DDE_EMU_LATE_FIRE_BUSY, 0); return; }
    int32_t r0 = dde_core_fire_delay(s->txen_fine, fine_now(), k);
    if (r0 < DDE_EMU_R_PRECHECK) { count_late(i, DDE_EMU_LATE_SHORT, r0); return; }
    uint32_t seq1 = 0;
    flexray_ts_t t1;
    uint32_t pos1;
    flexray_ts_peek_latest(1, &t1, &pos1, &seq1);

    pio_interrupt_clear(DDE_EMU_PIO, 5);
    for (unsigned w = 0; w < s->nwords; w++) pio_sm_put(DDE_EMU_PIO, sm_tx, s->words[w]);
    int32_t r = dde_core_fire_delay(s->txen_fine, fine_now(), k);
    if (r < DDE_EMU_R_MIN || r > DDE_EMU_R_MAX) {
        tx_sm_abort();
        count_late(i, DDE_EMU_LATE_WINDOW, r);
        return;
    }
    pio_sm_put(DDE_EMU_PIO, sm_fire, (uint32_t)r);
    // committed: the frame is on the wire from now + (r + k) ticks for frame_bits * 100 ns
    {
        unsigned nbytes = (s->words[0] & 0xFFFFu) + 1u;
        unsigned bits = EMU_TSS_BITS + 1u + 10u * nbytes + 2u +
                        (s->words[1] ? s->words[1] + 3u : 0u);
        tx_deadline_us = now + 1u + (uint32_t)((r + k) / 100) + (bits + 9u) / 10u;
        tx_expect = true;
        tx_unexp_since = 0;
    }
    cnt.sent[i]++;
    if (i < DDE_EMU_N_STATIC && s->is_data) {
        uint8_t *c = &counter[i][s->cycle & 3u];
        *c = (uint8_t)((*c + 1u) % DDE_EMU_COUNTER_MOD);
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
    for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) {
        if (slots[i].state == SL_ARMED && (int32_t)(slots[i].alarm_us - now) <= 0) {
            fire_slot(i, now);
            break;   // one per alarm; the next one is >= 27 us away
        }
    }
    housekeeping(now);
    alarm_rearm_next(time_us_32());
}

// ---------------------------------------------------------------- header hook
static void __time_critical_func(prepare_slot)(unsigned i, unsigned cycle)
{
    emu_slot_t *s = &slots[i];
    uint8_t frame[DDE_MAX_FRAME_BYTES];
    unsigned n, dts = 0;
    if (i < DDE_EMU_N_STATIC) {
        n = dde_core_build_static(i, cycle, counter[i][cycle & 3u], EMU_SYNC_BITS != 0, frame);
        s->is_data = (dde_core_emu_indicators(i, cycle, false) & 0x04u) != 0;
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

void __time_critical_func(dde_emu_on_fr1_header)(const uint8_t h[5], uint32_t start_pos, uint32_t ring_mask)
{
    if (!hw_ready && fault == DDE_EMU_FAULT_NONE) return;
    unsigned id = ((unsigned)(h[0] & 7u) << 8) | h[1];
    unsigned cycle = h[4] & 0x3Fu;
    flexray_ts_t t0;
    uint32_t pos0, seq0;
    bool t0_ok = flexray_ts_peek_latest(0, &t0, &pos0, &seq0) && ((pos0 - start_pos) & ring_mask) == 0;

    int ti = dde_core_target_index(id);
    if (EMU_DYN_5C && ti < 0 && id == DDE_EMU_DYN_ID && dde_core_dyn_cycle(cycle)) ti = DDE_EMU_N_STATIC;
    if (ti >= 0) {
        uint32_t irq = save_and_disable_interrupts();
        on_echo((unsigned)ti, cycle, &t0, t0_ok);
        restore_interrupts(irq);
        return;
    }
    int ri = dde_core_ref_index(id);
    if (ri < 0 || !t0_ok) return;

    int32_t ref_off = dde_emu_ref_off_q8[ri];
    uint32_t cs_us = t0.us - (uint32_t)(ref_off / 25600);
    uint32_t irq = save_and_disable_interrupts();
    int ev = dde_lock_on_ref(&lock, cycle, cs_us);
    if (ev == DDE_LOCK_NEW_WAKE) {
        for (unsigned i = 0; i < DDE_EMU_N_STATIC; i++) for (unsigned c = 0; c < 4; c++) counter[i][c] = 1;
    }
    if (ev != DDE_LOCK_SAME_CYCLE) {
        cur_cycle = (uint8_t)cycle;
        for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) {
            if (slots[i].state == SL_ARMED && slots[i].cycle != cycle) count_late(i, DDE_EMU_LATE_MISSED, 0);
            if (slots[i].cycle != cycle) slots[i].state = SL_IDLE;
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

    if (tx_allowed()) {
        int32_t loop = loop_q4 >> 4;
        bool changed = false;
        for (unsigned i = 0; i < DDE_EMU_N_SLOTS; i++) {
            int32_t tgt;
            if (i < DDE_EMU_N_STATIC) tgt = dde_emu_static_off_q8[i];
            else {
                if (!EMU_DYN_5C || !dde_core_dyn_cycle(cycle)) continue;
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
        if (changed) alarm_rearm_next(time_us_32());
    }
    restore_interrupts(irq);
}

// ---------------------------------------------------------------- init
void dde_emu_init_core1(void)
{
    dde_core_init();
    dde_lock_reset(&lock);
    for (unsigned i = 0; i < DDE_EMU_N_STATIC; i++) for (unsigned c = 0; c < 4; c++) counter[i][c] = 1;

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
        printf("emu: TX enabled by host (TSS %u, sync bits %u, dyn 0x5C %u)\n",
               (unsigned)EMU_TSS_BITS, (unsigned)EMU_SYNC_BITS, (unsigned)EMU_DYN_5C);
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
}
