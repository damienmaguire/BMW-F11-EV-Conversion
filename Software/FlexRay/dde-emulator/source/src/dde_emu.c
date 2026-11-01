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
#include "dde_gear.h"
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
// v1h: do not move an already armed static slot when the new reference is less than this
// many us before the slot (e.g. 0x27 -> 0x28 = 33 us, 0x3C -> 0x3D = 33 us); keep the
// schedule made from the earlier reference instead.
// v1h.2: ON by default with 70 us. Run 9: every echo miss was on a slot re-armed by the
// reference right before it (0x28 by 0x27, 0x3D by 0x3C: alarm 14-15 us after the re-arm;
// 0x29 by 0x27: 47 us), 0x26 (scheduled from 0x21, ~165 us before) had 0 misses in 54 k
// frames. 70 us also keeps 0x27 off 0x29 (66 us). The earlier references are 66..240 us
// before the slot: the timing error from that is a few ns (clock rate +-100 ppm).
// -DDDE_EMU_RESCHED_MIN_US=0 = v1h.1 behaviour.
#ifndef DDE_EMU_RESCHED_MIN_US
#define DDE_EMU_RESCHED_MIN_US 70u
#endif
#define DDE_EMU_DIAG_CLOSE_US 40        // v1h diag: "re-armed shortly before the alarm"
#define DDE_EMU_E8_CRANK_CYCLES 35u     // v1h: 0xE8 L9 every 35 cycles (~5.7/s) while cranking (real 4-7/s)

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
// v1g fake engine (core 1 owns `eng`; core 0 talks to it through the mailboxes below)
static eng_t eng;
static uint8_t eng_e6_seen, eng_fb9_seen;    // last event counters acted on
static bool eng_e6_event;                    // send 0xE6 in the next possible cycle (any cycle)
static uint8_t eng_e8_alt;                   // 0xE8 L9 fast rate: alternate 8 / 16 cycles
// ST_KL inputs: [0] = FlexRay 0x74 (even cycles), [1] = PT-CAN 0x12F. Written by core 0.
static volatile struct { uint8_t stkl, brake; uint32_t us, seq; } stmb[2];
static uint32_t stmb_seen[2];
static uint8_t eng_stkl_src;
static volatile struct { uint32_t us, seq; } brmb;                 // PT-CAN 0x173 brake pressed
static uint32_t brmb_seen;
static volatile struct { uint8_t mode, cmd, prof, gear_off; uint32_t us, seq; } hostmb;   // USB 0xa7
// v1h.1 gear: core 1 owns `gear`; the lever 0x197 comes from core 0 (PT-CAN RX) through lvmb
static gear_t gear;
static volatile struct { uint8_t d[4]; uint8_t len; uint32_t us, seq; } lvmb;
static uint32_t lvmb_seen;
static uint8_t gear_e6_last[2];              // 0xE6 gear bits last sent (event on change)
static dde_gear_status_t eng_pub_gear;
static uint32_t hostmb_seen;
// core 1 -> core 0: snapshot for the PT-CAN emulator (seqlock: odd = writing).
// v1h.2: the write window holds only eng_pub + its time (a ~64-byte copy with interrupts
// off on core 1, < 1 us); everything is computed before the window opens. The status
// structs (USB/console only) have their own seqlock, so a PT-CAN read never waits on them.
static volatile uint32_t eng_pub_seq;
static eng_out_t eng_pub;
static volatile uint32_t eng_pub_us;
static volatile uint32_t eng_st_seq;          // seqlock 2: eng_pub_st + eng_pub_gear
static dde_engine_status_t eng_pub_st;
// v1h.2: the engine story and the gear run in core 1's background loop (dde_emu_core1_poll),
// with interrupts on, not in the FlexRay header ISR. The header ISR only asks for one
// step per FlexRay cycle; the ISR-side frame builders read eng_isr (copied with interrupts
// off, so a frame never mixes two states).
static eng_out_t eng_isr;
static volatile uint32_t bg_req_seq;          // header ISR: +1 per new FlexRay cycle
static volatile bool bg_new_wake;             // header ISR: a new wake (engine + gear reset)
static uint32_t bg_seen_seq;
static uint32_t bg_cyc_us;                    // last engine step (core 1 background)
static uint32_t bg_gear_us;                   // last gear-only step (1 ms between cycles)
static volatile uint16_t bg_max_us;           // longest background step (diag)
static volatile uint16_t snap_fail;           // core 0: snapshot reads that failed (last good one reused)
static struct {
    int16_t isr_late_min, isr_late_max;      // dynamic fires: alarm ISR entry - alarm time (us)
    int16_t r_entry_min, r_final_min;        // dynamic fires: fire delay at ISR entry / after the final check
    uint16_t topup_timeouts, hk_deferred, nm_ceiling_stops;
    bool any;
} lat;
// v1h echo-miss diagnostics (core 1 writes, core 0 copies; small tearing is acceptable)
static struct {
    uint16_t miss[DDE_EMU_N_STATIC], late[DDE_EMU_N_STATIC];
    // per static slot: the last (re)schedule and the last fire
    uint16_t sched_ref[DDE_EMU_N_STATIC];
    uint32_t sched_us[DDE_EMU_N_STATIC];
    int16_t f_r0[DDE_EMU_N_STATIC], f_r[DDE_EMU_N_STATIC], f_il[DDE_EMU_N_STATIC], f_lead[DDE_EMU_N_STATIC];
    uint16_t f_ref[DDE_EMU_N_STATIC], f_hdr_us[DDE_EMU_N_STATIC];
    uint8_t f_flags[DDE_EMU_N_STATIC];
    uint32_t f_crc[DDE_EMU_N_STATIC], f_nc[DDE_EMU_N_STATIC];
    // last miss
    uint8_t lm_slot, lm_cycle, lm_flags;
    int16_t lm_r0, lm_r, lm_il, lm_lead;
    uint16_t lm_ref, lm_hdr_us;
    uint16_t hdr_max, alarm_max, hdr_over10, miss_close, miss_crc, miss_preempt, guard_skips;
    uint32_t static_fired;
    uint16_t last_hdr_us;
} dg;
static volatile bool in_hdr_isr;     // core 1: the FR1 header hook is running
// Streamer header counters (flexray_bss_streamer.c has the real ones; host tests: 0)
__attribute__((weak)) void dde_emu_stream_counts(uint32_t *crc_fail, uint32_t *no_cand)
{
    *crc_fail = 0; *no_cand = 0;
}
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
    if (i < DDE_EMU_N_STATIC) sat16_inc(&dg.late[i]);
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
    if (i < DDE_EMU_N_STATIC) {
        dg.f_r0[i] = (int16_t)clamp32(r0, -32768, 32767);
        dg.f_il[i] = (int16_t)clamp32((int32_t)(now - s->alarm_us), -32768, 32767);
        dg.f_lead[i] = (int16_t)clamp32((int32_t)(s->alarm_us - dg.sched_us[i]), -32768, 32767);
        dg.f_ref[i] = dg.sched_ref[i];
        dg.f_hdr_us[i] = dg.last_hdr_us;
        dg.f_flags[i] = (uint8_t)((in_hdr_isr ? 1u : 0u) | (dg.f_lead[i] < DDE_EMU_DIAG_CLOSE_US ? 2u : 0u) |
                                  (tx_expect ? 16u : 0u));
        dde_emu_stream_counts(&dg.f_crc[i], &dg.f_nc[i]);
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
    if (i < DDE_EMU_N_STATIC) { dg.f_r[i] = (int16_t)r; dg.static_fired++; }
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
        // v1h.2: an 0xE6 event copy sent BEFORE its periodic slot does not move the 1 s grid.
        // v1h.1 pushed the grid a whole period on (event just after a periodic one = next
        // periodic ~2 s later; run 9: 1.9-2.6 s gaps, EPS D51744 "DT_PT_2" x3, real max ~2.0 s).
        bool ev_early = p == DDE_PDU_E6 && pdu_started[p] && (int32_t)(abs_cycle - pdu_due[p]) < 0;
        if (!ev_early) pdu_due[p] = keep ? pdu_due[p] + per : abs_cycle + per;
        pdu_started[p] = true;
        if (p == DDE_PDU_E6) eng_e6_event = false;
        // v1g: while the fake engine runs the DDE sends 0xE8 L9 ~15/s (Fred): every 8 / 16 cycles
        if (p == DDE_PDU_E89 && eng_isr.e8_fast) {
            eng_e8_alt ^= 1u;
            pdu_due[p] = abs_cycle + (eng_e8_alt ? 8u : 16u);
        } else if (p == DDE_PDU_E89 && eng_isr.e8_crank) {
            // v1h: while cranking the real DDE sends 0xE8 L9 4-7 times a second (Fred)
            pdu_due[p] = abs_cycle + DDE_EMU_E8_CRANK_CYCLES;
        }
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
    uint32_t end = time_us_32();
    alarm_rearm_next(end);
    if (end - now > dg.alarm_max) dg.alarm_max = (uint16_t)(end - now > 0xFFFFu ? 0xFFFFu : end - now);
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
    if (i < DDE_EMU_N_STATIC) {   // v1h: what was special about this fire?
        uint32_t crc, nc;
        dde_emu_stream_counts(&crc, &nc);
        uint8_t fl = dg.f_flags[i];
        if (crc != dg.f_crc[i]) { fl |= 4u; sat16_inc(&dg.miss_crc); }
        if (nc != dg.f_nc[i]) fl |= 8u;
        if (fl & 2u) sat16_inc(&dg.miss_close);
        if (fl & 1u) sat16_inc(&dg.miss_preempt);
        sat16_inc(&dg.miss[i]);
        dg.lm_slot = (uint8_t)slot_id(i);
        dg.lm_cycle = fired[i].cycle;
        dg.lm_flags = fl;
        dg.lm_r0 = dg.f_r0[i]; dg.lm_r = dg.f_r[i]; dg.lm_il = dg.f_il[i]; dg.lm_lead = dg.f_lead[i];
        dg.lm_ref = dg.f_ref[i]; dg.lm_hdr_us = dg.f_hdr_us[i];
    }
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

// v1h.2: lever + brake inputs from core 0, gear step, gear bytes into eng.out. Core 1
// background (dde_emu_core1_poll), every FlexRay cycle and every 1 ms in between.
static void __time_critical_func(gear_tick)(uint32_t now)
{
    uint32_t bs = brmb.seq;
    if (bs != brmb_seen) {
        __dmb(); uint32_t us = brmb.us; brmb_seen = bs;
        eng_input_brake(&eng, true, us);
        gear_input_brake(&gear, us);
    }
    uint32_t ls = lvmb.seq;
    if (ls != lvmb_seen && !(ls & 1u)) {
        __dmb();
        uint8_t d[4] = {lvmb.d[0], lvmb.d[1], lvmb.d[2], lvmb.d[3]}, len = lvmb.len;
        uint32_t us = lvmb.us;
        __dmb();
        lvmb_seen = ls;
        if (lvmb.seq == ls) gear_input_lever(&gear, d, len, us);   // torn read: skip, next one in 22-38 ms
    }
    gear_step(&gear, now, eng.phase == ENG_RUN);
    gear_fill_out(&gear, &eng.out);
}

// v1h.2: hand the new state to the ISR side (frame builders, event sends) and to core 0.
// Everything is computed BEFORE the shared windows open; each window is a plain copy.
static void __time_critical_func(engine_publish)(uint32_t now, bool full)
{
    // 1) ISR side (core 1): eng_isr + the event decisions that touch the ISR's PDU plan
    uint8_t ge[2] = {eng.out.g_set ? eng.out.g_fr_park : 2u, eng.out.g_set ? eng.out.g_nib : 3u};   // = what 0xE6 carries
    bool e6 = false, fb9 = false;
    if (memcmp(ge, gear_e6_last, 2) != 0) {   // 0xE6 carries the gear: send it as an event
        memcpy(gear_e6_last, ge, 2);
        if (eng.out.active) e6 = true;
    }
    if (eng.out.e6_ev != eng_e6_seen) { eng_e6_seen = eng.out.e6_ev; if (eng.out.active) e6 = true; }
    if (eng.out.fb9_ev != eng_fb9_seen) { eng_fb9_seen = eng.out.fb9_ev; if (eng.out.active) fb9 = true; }
    uint32_t irq = save_and_disable_interrupts();
    if (bg_new_wake) {   // a new wake came in while we computed: drop this result, reset first
        restore_interrupts(irq);
        return;
    }
    eng_isr = eng.out;
    if (e6) eng_e6_event = true;
    if (fb9) pdu_due[DDE_PDU_FB9] = abs_cycle;   // next 0xFB L9 cycle (cycle % 8 == 3)
    // E8 fast rate just started: do not wait for the 1 s grid
    if (eng.out.e8_fast && (int32_t)(pdu_due[DDE_PDU_E89] - abs_cycle) > 16) pdu_due[DDE_PDU_E89] = abs_cycle;
    // v1h: crank rate just started: first one now, not at the end of the 1 s period
    if (eng.out.e8_crank && (int32_t)(pdu_due[DDE_PDU_E89] - abs_cycle) > (int32_t)DDE_EMU_E8_CRANK_CYCLES)
        pdu_due[DDE_PDU_E89] = abs_cycle;
    // 2) core 0 snapshot (PT-CAN): seqlock window = one struct copy, interrupts still off
    eng_pub_seq = eng_pub_seq + 1u;   // odd: writing
    __dmb();
    eng_pub = eng.out;
    eng_pub_us = bg_cyc_us;           // time of the last FlexRay-cycle step (500 ms bus-asleep rule)
    __dmb();
    eng_pub_seq = eng_pub_seq + 1u;   // even: consistent
    restore_interrupts(irq);
    if (!full) return;
    // 3) status for USB / console (own seqlock, filled into locals first)
    dde_engine_status_t st;
    memset(&st, 0, sizeof st);
    st.version = 2;
    st.mode = eng.mode;
    st.phase = eng.phase;
    st.stkl = eng.stkl_have ? eng.stkl : 0xFF;
    st.stkl_src = eng_stkl_src;
    st.brake = eng.brake_have && (now - eng.brake_us) <= 300000u;
    st.last_reject = (uint8_t)eng.last_reject;
    st.e6_b3 = eng.out.e6_b3;
    st.rpm = (uint16_t)(eng.out.rpm4 / 4u);
    st.torque_raw = eng.out.torque;
    st.starts = eng.starts; st.stops = eng.stops; st.rejects = eng.rejects;
    st.ms_since_start = eng.starts ? (now - eng.t0) / 1000u : 0xFFFFFFFFu;
    st.ms_since_stkl = eng.stkl_have ? (now - eng.stkl_us) / 1000u : 0xFFFFFFFFu;
    st.cnt_hi = eng.out.cnt_hi; st.fb9_b2 = eng.out.fb9_b2; st.e6_b4 = eng.out.e6_b4;
    st.e6_ready = eng.out.e6_ready;
    st.seq = eng.out.seq;
    // v1h: start profile and what the CAS did with our start
    st.profile = eng.starts ? eng.prof : 0;
    st.profile_req = eng.prof_req;
    st.cas_flags = eng.cas_flags;
    st.out_flags = (uint8_t)((eng.out.inj ? 1u : 0u) | (eng.out.e8_crank ? 2u : 0u) | (eng.out.e8_fast ? 4u : 0u));
    st.d_to_82_ms = (eng.starts && eng.t82_have) ? (uint16_t)clamp32((int32_t)((eng.t82 - eng.t0) / 1000u), 0, 0xFFFE) : 0xFFFFu;
    st.d_len_ms = (eng.starts && eng.start_d && eng.d_end_have)
                      ? (uint16_t)clamp32((int32_t)((eng.d_end_us - eng.t0) / 1000u), 0, 0xFFFE) : 0xFFFFu;
    st.d_end_after_82_ms = (eng.starts && eng.start_d && eng.d_end_have && eng.t82_have)
                               ? (int16_t)clamp32((int32_t)(eng.d_end_us - eng.t82) / 1000, -32767, 32766)
                               : (eng.starts && eng.start_d && eng.d_end_have) ? (int16_t)-32768 : (int16_t)0x7FFF;
    st.t_rpm_ms = (eng.starts && eng.cur_run) ? (uint16_t)clamp32((int32_t)((eng.trpm - eng.t0) / 1000u), 0, 0xFFFE) : 0xFFFFu;
    st.aborts = eng.aborts;
    dde_gear_status_t gs;
    gear_fill_status(&gear, &gs);
    irq = save_and_disable_interrupts();
    eng_st_seq = eng_st_seq + 1u;
    __dmb();
    eng_pub_st = st;
    eng_pub_gear = gs;
    __dmb();
    eng_st_seq = eng_st_seq + 1u;
    restore_interrupts(irq);
}

// v1g: once per FlexRay cycle: take the core-0 inputs, step the engine story, turn content
// changes into event sends and publish a snapshot for core 0 (PT-CAN).
// v1h.2: core 1 background loop (was: inside the FR1 header ISR, up to 189 us in run 9).
static void __time_critical_func(engine_cycle)(uint32_t now)
{
    uint32_t hs = hostmb.seq;
    if (hs != hostmb_seen) {
        __dmb();
        uint8_t mode = hostmb.mode, cmd = hostmb.cmd, prof = hostmb.prof, goff = hostmb.gear_off;
        uint32_t us = hostmb.us;
        hostmb_seen = hs;
        gear_set_enabled(&gear, !goff, us);   // v1h.1: logger --no-gear = fixed P
        eng_set_profile(&eng, prof);   // v1h: takes effect at the next start
        eng_set_mode(&eng, mode, us);
        if (cmd) eng_command(&eng, cmd, us);
    }
    for (unsigned k = 0; k < 2; k++) {
        uint32_t sq = stmb[k].seq;
        if (sq == stmb_seen[k]) continue;
        __dmb();
        uint8_t v = stmb[k].stkl, b = stmb[k].brake;
        uint32_t us = stmb[k].us;
        stmb_seen[k] = sq;
        eng_input_stkl(&eng, v, b != 0, us);
        eng_stkl_src = (uint8_t)k;
    }
    eng_step(&eng, now);
    // v1h.1: gear after the engine (it needs "running"), bytes into the same snapshot
    gear_tick(now);
    bg_cyc_us = now;
    bg_gear_us = now;
    engine_publish(now, true);
}

// v1h.2: core 1 background (main.c core1_entry loop, after every interrupt; the 1 ms
// housekeeping alarm guarantees >= 1 kHz). Never called from an ISR.
void __time_critical_func(dde_emu_core1_poll)(void)
{
    uint32_t t0 = time_us_32();
    uint32_t irq = save_and_disable_interrupts();
    bool wake = bg_new_wake;
    bg_new_wake = false;
    uint32_t rq = bg_req_seq;
    restore_interrupts(irq);
    if (wake) {   // new wake: the engine starts "never started", the gear in P
        eng_reset(&eng);
        gear_reset(&gear, gear.enabled);
        gear_e6_last[0] = 2; gear_e6_last[1] = 3;
        eng_e6_seen = eng.out.e6_ev; eng_fb9_seen = eng.out.fb9_ev;
    }
    if (rq != bg_seen_seq) {
        bg_seen_seq = rq;
        engine_cycle(t0);
    } else if ((uint32_t)(t0 - bg_cyc_us) < 20000u && (uint32_t)(t0 - bg_gear_us) >= 1000u) {
        // between FlexRay cycles: the gear every 1 ms (lever timing), the engine stays per cycle
        bg_gear_us = t0;
        gear_tick(t0);
        engine_publish(t0, false);
    } else {
        return;
    }
    uint32_t d = time_us_32() - t0;
    if (d > bg_max_us) bg_max_us = (uint16_t)(d > 0xFFFFu ? 0xFFFFu : d);
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
        // v1g: a new wake starts with the engine "never started", v1h.1: and in P.
        // v1h.2: the reset itself runs in the core 1 background; until then nothing is patched.
        bg_new_wake = true;
        eng_isr.active = 0; eng_isr.g_set = 0;
        eng_e6_event = false;
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
    // v1h.2: the engine/gear step runs in the core 1 background (dde_emu_core1_poll), not here
    bg_req_seq = bg_req_seq + 1u;
    __sev();
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
        if (p == -2 && ds == DDE_DSLOT_E6 && eng_e6_event) p = DDE_PDU_E6;   // v1g: state change, send now
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

static void __time_critical_func(on_fr1_header)(const uint8_t h[5], uint32_t start_pos, uint32_t ring_mask);
// v1h: the header hook, timed (diagnostics: how long it keeps interrupts off / the core busy)
void __time_critical_func(dde_emu_on_fr1_header)(const uint8_t h[5], uint32_t start_pos, uint32_t ring_mask)
{
    uint32_t t0 = time_us_32();
    in_hdr_isr = true;
    on_fr1_header(h, start_pos, ring_mask);
    in_hdr_isr = false;
    uint32_t d = time_us_32() - t0;
    if (d > 0xFFFFu) d = 0xFFFFu;
    dg.last_hdr_us = (uint16_t)d;
    if (d > dg.hdr_max) dg.hdr_max = (uint16_t)d;
    if (d > 10u) sat16_inc(&dg.hdr_over10);
}

static void __time_critical_func(on_fr1_header)(const uint8_t h[5], uint32_t start_pos, uint32_t ring_mask)
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
            if (DDE_EMU_RESCHED_MIN_US && i < DDE_EMU_N_STATIC && s->state == SL_ARMED && s->cycle == cycle &&
                delta < (int32_t)(DDE_EMU_RESCHED_MIN_US * 25600u)) {
                sat16_inc(&dg.guard_skips);   // v1h option: keep the schedule from the earlier reference
                continue;
            }
            if (s->state == SL_IDLE) prepare_slot(i, cycle);
            dde_sched_t sc;
            dde_core_schedule(t0.fine, t0.us, delta, loop, DDE_EMU_LEAD_US, &sc);
            s->bus_fine = sc.bus_fine;
            s->txen_fine = sc.txen_fine;
            s->alarm_us = sc.alarm_us;
            s->state = SL_ARMED;
            if (i < DDE_EMU_N_STATIC) { dg.sched_ref[i] = (uint16_t)id; dg.sched_us[i] = time_us_32(); }
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
    eng_reset(&eng);
    gear_reset(&gear, true);   // v1h.1: shifting on unless the build (EMU_GEAR=0) or the logger says no
    gear_e6_last[0] = 2; gear_e6_last[1] = 3;
    eng_isr = eng.out;
    dde_core_eng = &eng_isr;   // v1g overlay: does nothing until a start (mode is off by default)
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
    if (id == 0x74u) {
        // v1g: 0x74 even cycles = same PDU as PT-CAN 0x12F (Fred): b3 low nibble ST_KL,
        // b7 bits 0-1 = 3 brake pressed. Odd cycles carry another PDU: ignore them.
        if ((cycle & 1u) || nbytes < 7u) return;
        stmb[0].stkl = payload[2] & 0x0Fu;
        stmb[0].brake = (payload[6] & 3u) == 3u;
        stmb[0].us = time_us_32();
        __dmb();
        stmb[0].seq = stmb[0].seq + 1u;
        return;
    }
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
    dde_emu_diag_t d;
    dde_emu_fill_diag(&d);
    printf("emu: diag miss[26=%u 28=%u 29=%u 3D=%u] late[26=%u 28=%u 29=%u 3D=%u] last_miss=0x%02X c%u flags=0x%02X "
           "r=%d->%d isr_late=%dus sched_by=0x%02X lead=%dus hdr=%uus | hdr_max=%uus alarm_max=%uus hdr>10us=%u "
           "miss[close=%u crc=%u preempt=%u] crc_fail=%lu no_cand=%lu guard=%u/%uus snap_fail=%u bg_max=%uus\n",
           d.miss[0], d.miss[1], d.miss[2], d.miss[3], d.late[0], d.late[1], d.late[2], d.late[3], d.lm_slot,
           d.lm_cycle, d.lm_flags, d.lm_r_entry, d.lm_r_final, d.lm_isr_late_us, d.lm_sched_ref, d.lm_sched_lead_us,
           d.lm_hdr_us, d.hdr_isr_max_us, d.alarm_isr_max_us, d.hdr_over_10us, d.miss_resched_close,
           d.miss_crc_near, d.miss_preempt, (unsigned long)d.crc_fail, (unsigned long)d.no_cand,
           d.resched_guard_skips, d.resched_min_us, d.snap_fail, d.bg_max_us);
}

// ---------------------------------------------------------------- v1g engine, core0 side
void dde_engine_on_can(uint16_t id, const uint8_t *d, uint8_t len)
{
    if (id == 0x12Fu && len >= 7u) {          // ST_KL d2 low nibble, brake d6 bits 0-1
        stmb[1].stkl = d[2] & 0x0Fu;
        stmb[1].brake = (d[6] & 3u) == 3u;
        stmb[1].us = time_us_32();
        __dmb();
        stmb[1].seq = stmb[1].seq + 1u;
    } else if (id == 0x197u && len >= 4u) {  // v1h.1 gear lever (read only, never sent by us)
        lvmb.seq = lvmb.seq + 1u;             // odd: writing
        __dmb();
        for (unsigned i = 0; i < 4u; i++) lvmb.d[i] = d[i];
        lvmb.len = len;
        lvmb.us = time_us_32();
        __dmb();
        lvmb.seq = lvmb.seq + 1u;
    } else if (id == 0x173u && len >= 8u && (d[7] & 3u) == 3u) {   // brake pressed
        brmb.us = time_us_32();
        __dmb();
        brmb.seq = brmb.seq + 1u;
    }
}

void dde_engine_host_set(uint16_t wvalue, uint16_t windex)
{
    static uint8_t last_mode = 0xFF, last_prof = 0xFF;
    // v1h.1: wIndex bit 8 = gear shifting off (fixed P as v1h); low byte = start profile
    uint8_t goff = (windex & 0x100u) ? 1u : 0u;
    windex &= 0xFFu;
    uint8_t prof = windex < ENG_PROF_N ? (uint8_t)windex : ENG_PROF_DEFAULT;
    static uint8_t last_goff = 0xFF;
    if (goff != last_goff) printf("gear: shifting %s\n", (goff || !EMU_GEAR) ? "off (fixed P)" : "on");
    last_goff = goff;
    hostmb.gear_off = goff;
    hostmb.prof = prof;
    if (prof != last_prof) printf("engine: start profile %s%s\n", eng_prof_name(prof),
                                  prof == ENG_PROF_DEFAULT ? " (build default)" : "");
    last_prof = prof;
    uint8_t mode = (uint8_t)(wvalue & 0xFFu), cmd = (uint8_t)(wvalue >> 8);
    if (mode > ENG_MODE_FALLBACK) mode = ENG_MODE_OFF;
    if (cmd > ENG_CMD_STOP) cmd = ENG_CMD_NONE;
    hostmb.mode = mode;
    hostmb.cmd = cmd;
    hostmb.us = time_us_32();
    __dmb();
    hostmb.seq = hostmb.seq + 1u;
    static const char *const mn[] = {"off", "cas", "fallback"};
    if (mode != last_mode) printf("engine: fake start mode %s\n", mn[mode]);
    if (cmd == ENG_CMD_START) printf("engine: start command\n");
    if (cmd == ENG_CMD_STOP) printf("engine: stop command\n");
    last_mode = mode;
}

// v1h.2 seqlock readers (core 0): retry with a short bounded wait (the writer holds the
// window < 1 us with interrupts off), instead of 8 back-to-back tries.
#define ENG_READ_WAIT_US 20u
static bool engine_read(eng_out_t *o, uint32_t *us)
{
    uint32_t t0 = time_us_32();
    do {
        uint32_t a = eng_pub_seq;
        if (!(a & 1u)) {
            __dmb();
            eng_out_t tmp = eng_pub;
            uint32_t u = eng_pub_us;
            __dmb();
            if (eng_pub_seq == a) {
                if (a == 0) return false;   // nothing published yet
                if (o) *o = tmp;
                if (us) *us = u;
                return true;
            }
        }
        tight_loop_contents();
    } while ((uint32_t)(time_us_32() - t0) <= ENG_READ_WAIT_US);
    return false;
}

static bool status_read(dde_engine_status_t *st, dde_gear_status_t *gs)
{
    uint32_t t0 = time_us_32();
    do {
        uint32_t a = eng_st_seq;
        if (!(a & 1u)) {
            __dmb();
            if (st) *st = eng_pub_st;
            if (gs) *gs = eng_pub_gear;
            __dmb();
            if (eng_st_seq == a) return a != 0;
        }
        tight_loop_contents();
    } while ((uint32_t)(time_us_32() - t0) <= ENG_READ_WAIT_US);
    return false;
}

// v1h.1 gear status (USB 0xab / console). Before the first FlexRay cycle: "P, nothing yet".
void dde_gear_fill_status(dde_gear_status_t *gs)
{
    if (!status_read(0, gs)) {
        memset(gs, 0, sizeof *gs);
        gs->version = 1; gs->enabled = EMU_GEAR ? 3u : 0u; gs->lever = 0xFF;
    }
}

// v1h.1: print every new gear event on the console ("GEAR: P -> R (brake on) ..."). Core 0
// main loop; cheap (one 64-byte copy every 20 ms), never waits.
void dde_gear_console_poll(void)
{
    static uint32_t next_us;
    static uint16_t seen;
    uint32_t now = time_us_32();
    if ((int32_t)(now - next_us) < 0) return;
    next_us = now + 20000u;
    dde_gear_status_t gs;
    dde_gear_fill_status(&gs);
    if (gs.ev_seq == seen) return;
    for (unsigned i = 0; i < GEAR_N_EV; i++) {
        const gear_ev_t *e = &gs.ev[i];
        if (!e->seq || (int16_t)(e->seq - seen) <= 0) continue;
        char b[112];
        gear_ev_text(e, b, sizeof b);
        printf("%s\n", b);
    }
    if ((uint16_t)(gs.ev_seq - seen) > GEAR_N_EV)
        printf("GEAR: (%u older gear events not shown)\n", (unsigned)(uint16_t)(gs.ev_seq - seen - GEAR_N_EV));
    seen = gs.ev_seq;
}

// PT-CAN emulator (core 0, every frame it builds). v1h.2: a failed / torn read reuses the
// last good snapshot - once the emulation has produced data we never fall back to the
// stock (fixed-P, engine-not-running) template because of a read race (run 9: 0x3F9
// 666x, 0x39A 163x, 0x3FD/0x0DC P glitches in D). Counted in the diag line (snap_fail).
bool dde_engine_snapshot(eng_out_t *out)
{
    static eng_snap_cache_t cache;
    eng_out_t o;
    uint32_t us = 0;
    bool ok = engine_read(&o, &us);
    const eng_out_t *s = eng_snap_select(&cache, ok, &o, us, time_us_32());
    snap_fail = (uint16_t)(cache.fails > 0xFFFFu ? 0xFFFFu : cache.fails);
    if (!s) return false;
    *out = *s;
    return true;
}

void dde_engine_fill_status(dde_engine_status_t *st)
{
    if (!status_read(st, 0)) {
        memset(st, 0, sizeof *st); st->version = 2; st->stkl = 0xFF;
        st->d_to_82_ms = st->d_len_ms = st->t_rpm_ms = 0xFFFFu; st->d_end_after_82_ms = 0x7FFF;
    }
}

void dde_engine_print(void)
{
    dde_engine_status_t s;
    dde_engine_fill_status(&s);
    if (s.mode == 0 && s.starts == 0 && s.rejects == 0) return;
    printf("engine: mode=%u phase=%s rpm=%u e6=%02X/%02X ready=%u stkl=%X(%s) brake=%u starts=%lu stops=%lu "
           "rejects=%lu last_reject=%u\n",
           s.mode, eng_phase_name(s.phase), s.rpm, s.e6_b3, s.e6_b4, s.e6_ready, s.stkl,
           s.stkl_src ? "12F" : "74", s.brake, (unsigned long)s.starts, (unsigned long)s.stops,
           (unsigned long)s.rejects, s.last_reject);
    // v1h: start profile + what the CAS did with our start, loud when it refused
    printf("engine: profile=%s cas=0x%02X d_to_82=%dms d_len=%dms d_end_after_82=%dms first_rpm=%dms aborts=%lu inj=%u e8=%s\n",
           eng_prof_name(s.profile ? s.profile : s.profile_req), s.cas_flags,
           s.d_to_82_ms == 0xFFFFu ? -1 : (int)s.d_to_82_ms, s.d_len_ms == 0xFFFFu ? -1 : (int)s.d_len_ms,
           (int)s.d_end_after_82_ms, s.t_rpm_ms == 0xFFFFu ? -1 : (int)s.t_rpm_ms, (unsigned long)s.aborts,
           s.out_flags & 1u, (s.out_flags & 4u) ? "fast" : (s.out_flags & 2u) ? "crank" : "1/s");
    static uint8_t last_flags;
    static uint32_t last_starts;
    if (s.starts != last_starts) { last_starts = s.starts; last_flags = 0; }
    uint8_t nf = (uint8_t)(s.cas_flags & ~last_flags);
    if (nf & ENG_CAS_D_ENDED_EARLY)
        printf("ENGINE: *** CAS ended the start request (ST_KL D -> %X) BEFORE our running report - "
               "CAS refused / gave up on our start ***\n", s.stkl);
    if (nf & ENG_CAS_ABANDONED) printf("ENGINE: *** start abandoned (no running report sent) ***\n");
    if (nf & ENG_CAS_D_LONG)
        printf("ENGINE: *** CAS still requesting start 300 ms after our running report ***\n");
    last_flags = s.cas_flags;
}

void dde_emu_fill_diag(dde_emu_diag_t *d)
{
    memset(d, 0, sizeof *d);
    d->version = 1;
    d->lm_slot = dg.lm_slot; d->lm_cycle = dg.lm_cycle; d->lm_flags = dg.lm_flags;
    for (unsigned i = 0; i < DDE_EMU_N_STATIC && i < 4u; i++) { d->miss[i] = dg.miss[i]; d->late[i] = dg.late[i]; }
    d->lm_r_entry = dg.lm_r0; d->lm_r_final = dg.lm_r; d->lm_isr_late_us = dg.lm_il; d->lm_sched_lead_us = dg.lm_lead;
    d->lm_sched_ref = dg.lm_ref; d->lm_hdr_us = dg.lm_hdr_us;
    d->hdr_isr_max_us = dg.hdr_max; d->alarm_isr_max_us = dg.alarm_max;
    d->miss_resched_close = dg.miss_close; d->miss_crc_near = dg.miss_crc; d->miss_preempt = dg.miss_preempt;
    d->resched_guard_skips = dg.guard_skips;
    uint32_t crc, nc;
    dde_emu_stream_counts(&crc, &nc);
    d->crc_fail = crc; d->no_cand = nc;
    d->static_fired = dg.static_fired;
    d->hdr_over_10us = dg.hdr_over10;
    d->resched_min_us = (uint16_t)DDE_EMU_RESCHED_MIN_US;
    d->version = 2;                       // v1h.2: + snapshot read failures, background step time
    d->snap_fail = snap_fail;
    d->bg_max_us = bg_max_us;
}
