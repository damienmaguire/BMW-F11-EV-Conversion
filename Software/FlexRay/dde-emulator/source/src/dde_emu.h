// DDE emulator (PICO_FLEXRAY_DDE_EMU build): transmits the DDE's static frames
// 0x26/0x28/0x29/0x3D on FR1 in the DDE's slots, timed from the car's own frames,
// and (v1c) the dynamic frames 0xE6, 0xFB (L5/L7/L9) and 0xE8 (L9/L5) in their
// minislots when the minislot is certain.
// See Software/FlexRay/dde-emulator/README.md.
#ifndef DDE_EMU_H
#define DDE_EMU_H
#include <stdint.h>
#include "dde_emu_core.h"

#define PICOFLEX_SET_EMU   0xa3u   // vendor OUT, wValue 1 = enable/keepalive, 0 = disable + clear fault
#define PICOFLEX_GET_EMU   0xa4u   // vendor IN, up to 256 bytes: dde_emu_stats_t (first 64 = v1, 128 = v2, 192 = v3 layout)

// Pin used as the FR1 streamer's private frame-start mark (side-set) in the emu
// build: GP25 = Pico 2 on-board LED, not on any header pin.
#define DDE_EMU_MARK_PIN 25u

#define DDE_EMU_STATS_VERSION 4u
enum {
    DDE_EMU_ST_DISABLED = 0,   // host has not enabled TX (boot state)
    DDE_EMU_ST_NO_LOCK  = 1,   // enabled, waiting for 8 good reference cycles
    DDE_EMU_ST_ARMING   = 2,   // locked, waiting for 8 quiet cycles after enable
    DDE_EMU_ST_ACTIVE   = 3,   // transmitting
    DDE_EMU_ST_FAULT    = 4,   // fault, TX off: auto re-arm after 2 s quiet, latched after the 3rd
};
enum {
    DDE_EMU_FAULT_NONE = 0,
    DDE_EMU_FAULT_TX_STUCK = 1,    // TX state machine not idle when it should be
    DDE_EMU_FAULT_FOREIGN = 2,     // a DDE frame ID seen that we did not send (DDE still connected?)
    DDE_EMU_FAULT_NO_ECHO = 3,     // 8 transmitted frames in a row (over >= 2 cycles) not seen back on FR1 RX
};
// Why a scheduled frame was not sent (dde_emu_stats_t.late_reason[] / last_late_reason).
enum {
    DDE_EMU_LATE_NONE = 0,
    DDE_EMU_LATE_TX_BUSY = 1,      // TX SM not idle at the alarm (previous frame still on the wire)
    DDE_EMU_LATE_FIRE_BUSY = 2,    // fire SM not idle at the alarm
    DDE_EMU_LATE_SHORT = 3,        // alarm ran too late: < 2.5 us left before TXEN (precheck)
    DDE_EMU_LATE_WINDOW = 4,       // delay out of window after loading the FIFO (aborted, not sent)
    DDE_EMU_LATE_MISSED = 5,       // alarm never ran before the next cycle started
    DDE_EMU_LATE_N = 6,
};
#define DDE_EMU_FAULT_REARM_US 2000000u   // a fault clears itself after 2 s without new evidence
#define DDE_EMU_FAULT_LATCH_N 3u          // ... unless it was the 3rd fault since enable: stays latched

#define DDE_EMU_N_SLOTS 5u   // 4 static + dynamic 0x5C

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t state;
    uint8_t enabled;
    uint8_t fault;
    uint32_t sent[DDE_EMU_N_SLOTS];  // 0x26, 0x28, 0x29, 0x3D, 0x5C
    uint32_t lock_losses;
    uint32_t late;                   // frames skipped: alarm too late / PIO busy / out of window
    uint32_t echo_ok;
    uint32_t echo_miss;
    uint32_t foreign;
    uint32_t cycles_good;
    int16_t echo_err_avg_ns;         // echo mark - target mark, IIR mean
    int16_t echo_err_maxabs_ns;      // since last enable
    int16_t fire_err_avg_ns;         // our TXEN mark - TXEN target, IIR mean
    uint16_t loop_delay_ns;          // TXEN -> FR1 RX echo mark
    uint16_t fire_k_ticks;           // calibrated fire latency (10 ns ticks)
    uint8_t tss_bits;
    uint8_t flags;                   // bit0 EMU_SYNC_BITS, bit1 EMU_DYN_5C, bit2 locked
    uint32_t rearms;
    // ---- v2 (bytes 64..127). A host that asks for 64 bytes gets the v1 layout above.
    // First fault since enable/clear (all zero when none). Counters are snapshots at the fault.
    uint8_t ff_code;                 // DDE_EMU_FAULT_*
    uint8_t ff_slot;                 // 0x26/0x28/0x29/0x3D/0x5C frame id involved, 0 = none
    uint8_t ff_cycle;                // FlexRay cycle (0..63) of the event
    uint8_t ff_flags;                // bit0 locked, bit1 tx was expected busy, bit2 hw (SMs) stopped
    uint32_t ff_us;                  // TIMER0 (time_us_32) of the fault; compare with now_us
    uint32_t ff_sent_total;
    uint32_t ff_late;
    uint32_t ff_echo_ok;
    uint32_t ff_echo_miss;
    uint32_t ff_foreign;
    int32_t ff_detail;               // TX_STUCK: us past the expected frame end (or busy-without-commit
                                     // time, negated); FOREIGN: us since our fire in that slot (-1 none);
                                     // NO_ECHO: miss run length
    uint8_t fault_count;             // faults since enable/clear (latched at DDE_EMU_FAULT_LATCH_N)
    uint8_t auto_clears;             // automatic re-arms after DDE_EMU_FAULT_REARM_US quiet
    uint8_t latched;                 // 1 = stays off until host wValue 0
    uint8_t last_fault;              // code of the most recent fault
    uint16_t late_reason[DDE_EMU_LATE_N - 1];   // per DDE_EMU_LATE_* (1..5), saturating
    uint8_t last_late_slot;          // frame id of the most recent late, 0 = none
    uint8_t last_late_reason;
    int16_t last_late_r;             // fire delay (ticks) seen at that late, 0 if not computed
    uint16_t echo_dup;               // repeated echo headers of a frame already matched (ignored)
    uint32_t last_fault_us;
    uint32_t now_us;                 // TIMER0 when these stats were taken
    uint32_t reserved;
    // ---- v3 (bytes 128..191): v1c dynamic frames. A host asking for 128 bytes gets v2.
    // PDU order: 0xE6 L9, 0xFB L5, 0xFB L7, 0xFB L9, 0xE8 L9, 0xE8 L5
    uint32_t dyn_sent[DDE_DYN_V3_N_PDU];
    uint16_t dyn_skip[DDE_DYN_V3_N_PDU];      // due but not sent in that cycle (retried 8 cycles later)
    uint16_t dyn_skip_reason[DDE_SKIP_N - 1];   // DDE_SKIP_* 1..7: noanchor, mark, passed, len, grid, limit, late
    uint16_t dyn_defer;              // final check found an unexplained frame start: not sent (re-planned if possible)
    uint16_t dyn_last_ms[DDE_DYN_V3_N_SLOT];   // minislot of the last TX: 0xE6, 0xE8, 0xFB
    uint8_t ign;                     // bit0 0x103 seen since wake, bit1 ign-on content, bit2 0xE8 L5 allowed,
                                     // bit3 0x103 seen within 500 ms
    uint8_t dyn_flags;               // build: bit0 EMU_DYN_E6, bit1 EMU_DYN_FB, bit2 EMU_DYN_E8, bit3 EMU_PREIGN
    uint16_t dyn_parks;              // TX SM restarts after 14-byte frames (expected, one per 0xFB L7)
    uint16_t reserved3;
    // ---- v4 (bytes 192..255): v1d. PDU / slot order: 0x10A L5, 0x142 L31, 0x68 L3
    uint32_t dyn_sent2[3];
    uint16_t dyn_skip2[3];
    uint16_t dyn_last_ms2[3];        // minislot of the last TX: 0x10A, 0x142, 0x68
    uint8_t kl15;                    // bit0 KL15 on, bit1 on earlier in this wake, bit2 0xF1 seen this wake,
                                     // bit3 0x103 seen this wake, bit4 0x5C allowed now (NM), bit5 0x5C stopped
                                     // by the 60 s ceiling now, bit6 KL15 on-evidence seen this wake
    uint8_t f1_b3;                   // last 0xF1 "byte 3" = payload[2] (1-based B3; 0x20 KL15 on, 0xE0 off)
    uint16_t kl15_on_edges;
    uint16_t kl15_off_edges;
    uint16_t nm_ceiling_stops;       // 0x5C cycles stopped by the hard ceiling (should stay 0)
    uint32_t kl15_off_ago_ms;        // since the last KL15 off edge (0xFFFFFFFF = none this wake)
    uint32_t ceil_left_ms;           // time left before the 60 s hard ceiling (0 = reached)
    int16_t isr_late_min_us;         // dynamic fires: alarm ISR entry - alarm time, min / max (us)
    int16_t isr_late_max_us;
    int16_t r_entry_min;             // dynamic fires: fire delay at ISR entry, min (ticks; < 120 = SHORT late)
    int16_t r_final_min;             // dynamic fires: fire delay after the final frame-start check, min (ticks)
    uint16_t topup_timeouts;         // > 8-word frame not fed in time (TX_STUCK fault, should stay 0)
    uint16_t hk_deferred;            // housekeeping moved after a fire (expected, frequent)
    uint8_t dyn_flags2;              // build: bit0 EMU_10A, bit1 EMU_142, bit2 EMU_68, bit3 EMU_KL15_F1,
                                     // bit4 EMU_DYN_5C
    uint8_t ph142;                   // current 0x142 content phase (0 wake, 1 +2.59 s, 2 ign settled)
    uint16_t reserved4;
    uint32_t abs_cycle;              // cycles since wake
    uint32_t reserved5;
} dde_emu_stats_t;
_Static_assert(sizeof(dde_emu_stats_t) == 256, "emu stats size");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, dyn_sent2) == 192, "v4 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, kl15) == 216, "v4 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, kl15_off_ago_ms) == 224, "v4 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, isr_late_min_us) == 232, "v4 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, dyn_flags2) == 244, "v4 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, abs_cycle) == 248, "v4 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, dyn_sent) == 128, "v3 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, dyn_skip) == 152, "v3 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, dyn_skip_reason) == 164, "v3 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, dyn_defer) == 178, "v3 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, ign) == 186, "v3 layout");
// offsets used by tools/ts_capture/frts.py (EMU_STATS_V2_STRUCT)
_Static_assert(__builtin_offsetof(dde_emu_stats_t, ff_code) == 64, "v2 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, ff_detail) == 92, "v2 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, fault_count) == 96, "v2 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, late_reason) == 100, "v2 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, last_late_r) == 112, "v2 layout");
_Static_assert(__builtin_offsetof(dde_emu_stats_t, now_us) == 120, "v2 layout");
#define DDE_EMU_STATS_V1_SIZE 64u
#define DDE_EMU_STATS_V2_SIZE 128u
#define DDE_EMU_STATS_V3_SIZE 192u

void dde_emu_early_pins(void);   // core0 setup_pins: all TXD/TXEN of FR1/FR2 = SIO output high
void dde_emu_init_core1(void);   // core1, after setup_stream(): PIO0 TX + fire SMs, alarm IRQ
// FR1/FR2 header ISR (core1), FR1 frames with a good header CRC.
void dde_emu_on_fr1_header(const uint8_t header[5], uint32_t start_pos, uint32_t ring_mask);
void dde_emu_host_set(uint16_t wvalue);         // USB 0xa3 (core0)
// core0, every valid FR1 frame (payload bytes): v1d uses 0xF1 byte 3 as the KL15 state
void dde_emu_on_frame(unsigned id, unsigned cycle, const uint8_t *payload, unsigned nbytes);
void dde_emu_fill_stats(dde_emu_stats_t *st);   // USB 0xa4 / UART (core0)
void dde_emu_print_stats(void);                 // UART line (core0); "EMU FAULT ..." once per first fault
const char *dde_emu_fault_name(unsigned code);
#endif
