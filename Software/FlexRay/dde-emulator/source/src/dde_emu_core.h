// DDE emulator: hardware-free core (frame build, CRCs, E2E, PIO word packing,
// schedule maths, reference-lock state machine). Shared by the firmware
// (dde_emu.c) and the host test (tests/host/test_dde_emu_core.c).
#ifndef DDE_EMU_CORE_H
#define DDE_EMU_CORE_H
#include <stdbool.h>
#include <stdint.h>

#ifndef EMU_SYNC_BITS
#define EMU_SYNC_BITS 0        // 1 = keep the DDE's sync/startup bits on 0x3D. UNSAFE, UNTESTED.
#endif
#ifndef EMU_TSS_BITS
#define EMU_TSS_BITS 7         // transmitted TSS length (receivers accept 1 .. gdTSSTransmitter+1)
#endif

#define DDE_STATIC_FRAME_BYTES 24u      // 5 header + 16 payload + 3 CRC
#define DDE_MAX_FRAME_BYTES 32u
#define DDE_TX_MAX_WORDS (2u + DDE_MAX_FRAME_BYTES / 4u)

void dde_core_init(void);                                   // CRC tables
uint16_t dde_core_header_crc(unsigned sync, unsigned startup, unsigned frame_id, unsigned len_words);
uint32_t dde_core_frame_crc(const uint8_t *p, unsigned n);  // channel A init 0xFEDCBA
uint8_t dde_core_crc8(const uint8_t *p, unsigned n);        // poly 0x1D, init 0

// Header (5 bytes) for indicators (5 bits), id, payload length in words, cycle.
void dde_core_header(uint8_t indicators, unsigned frame_id, unsigned len_words, unsigned cycle, uint8_t h[5]);

// E2E: set counter nibble + CRC byte of every group of (target index, cycle).
// Returns the number of groups (0 for null-frame cycles).
unsigned dde_core_e2e_apply(uint8_t *payload, unsigned target, unsigned cycle, unsigned counter);

// Emulator indicators for target/cycle (null-frame bit from the table,
// sync/startup cleared unless keep_sync).
uint8_t dde_core_emu_indicators(unsigned target, unsigned cycle, bool keep_sync);

// Full static frame (24 bytes) for target/cycle with alive counter `counter`.
// keep_sync = EMU_SYNC_BITS. Returns byte count.
unsigned dde_core_build_static(unsigned target, unsigned cycle, unsigned counter, bool keep_sync, uint8_t *out);
// Dynamic 0x5C frame (16 bytes). Returns byte count.
unsigned dde_core_build_dyn(unsigned cycle, uint8_t *out);
bool dde_core_dyn_cycle(unsigned cycle);

// PIO TX FIFO words (see dde_emu_tx.pio). nbytes % 4 == 0, 3 <= tss <= 15, dts 0 or >= 3.
// Returns the word count, 0 on bad arguments.
unsigned dde_core_pack(const uint8_t *frame, unsigned nbytes, unsigned tss_bits, unsigned dts_bits, uint32_t *words);

// Lookups (-1 when not found)
int dde_core_ref_index(unsigned frame_id);
int dde_core_target_index(unsigned frame_id);

// Schedule maths (header ISR). delta_q8 = expected mark(target) - mark(ref), 1/256 ticks.
typedef struct {
    uint16_t bus_fine;   // expected echo mark (FR1 RX, side 0)
    uint16_t txen_fine;  // TXEN mark target (side 1)
    uint32_t alarm_us;   // TIMER0 time to load + arm the PIO
} dde_sched_t;
void dde_core_schedule(uint16_t ref_fine, uint32_t ref_us, int32_t delta_q8,
                       int32_t loop_ticks, uint32_t lead_us, dde_sched_t *out);
// Alarm ISR maths: delay-loop count for the fire SM (may be negative = late).
static inline int32_t dde_core_fire_delay(uint16_t txen_fine, uint16_t fine_now, int32_t k_ticks)
{
    return (int32_t)(int16_t)(uint16_t)(txen_fine - fine_now) - k_ticks;
}

// Reference lock. Fed with one cycle-start estimate per reference frame.
typedef struct {
    bool have;
    bool locked;
    uint8_t last_cycle;
    uint32_t last_cs_us;     // cycle start estimate of the last good cycle (TIMER0 us)
    uint32_t good_run;       // consecutive good cycles
    uint32_t lock_losses;
    uint32_t cycle_jumps;
    uint32_t wakes;          // bus silent > DDE_LOCK_WAKE_US, then refs again
    uint32_t good_cycles;
} dde_lock_t;

#define DDE_CYCLE_US 5000u
#define DDE_LOCK_MAX_GAP 2u          // lock lost when more than 2 cycles have no reference
#define DDE_LOCK_REARM 8u            // re-armed after 8 consecutive good cycles
#define DDE_LOCK_WAKE_US 1000000u    // silence longer than this = new bus wake (counters restart)

enum { DDE_LOCK_SAME_CYCLE = 0, DDE_LOCK_NEW_CYCLE = 1, DDE_LOCK_NEW_WAKE = 2, DDE_LOCK_BAD = 3 };
void dde_lock_reset(dde_lock_t *l);
int dde_lock_on_ref(dde_lock_t *l, unsigned cycle, uint32_t cs_us);
// Time-based check without references; returns true when the lock was just lost.
bool dde_lock_poll(dde_lock_t *l, uint32_t now_us);

#endif
