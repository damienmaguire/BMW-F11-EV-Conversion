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


// ---------------------------------------------------------------------------
// v1c: DDE dynamic-segment PDUs (0xE6, 0xE8, 0xFB). Hand-written (the
// generated dde_emu_tables.h stays byte-identical to gen_dde_tables.py).
// Minislot grid fitted on the run 5 real DDE (16039 on-grid DDE frames):
//   AP(m) = cycle start + DDE_DYN_BASE_Q8 + m * DDE_DYN_MS_Q8, residual +-0.26 us.
#define DDE_DYN_MS_Q8 176017            // 6.87567 us per minislot, 1/256 ticks (10 ns)
#define DDE_DYN_BASE_Q8 76879874        // minislot 0 (id 0x5C) = 0x07 ref + 2805.127 us
#define DDE_DYN_FIRST_ID 0x5Cu
#define DDE_DYN_MS_LIMIT 260u           // never TX beyond this minislot (run 5: DDE 0x142 30 B up to 275)
#define DDE_DYN_GRID_TOL_Q8 (150 * 256) // a dynamic header must sit within 1.5 us of the grid

enum { DDE_DYN_KIND_CONST = 0, DDE_DYN_KIND_E2E = 1, DDE_DYN_KIND_CTR = 2 };
// PDU index order (also the stats order): E6, FB L5, FB L7, FB L9, E8 L9, E8 L5
enum { DDE_PDU_E6 = 0, DDE_PDU_FB5, DDE_PDU_FB7, DDE_PDU_FB9, DDE_PDU_E89, DDE_PDU_E85, DDE_DYN_N_PDU };
// Dynamic emulator slots (one frame ID each)
enum { DDE_DSLOT_E6 = 0, DDE_DSLOT_E8, DDE_DSLOT_FB, DDE_DYN_N_SLOT };

typedef struct {
    uint16_t id;
    uint8_t dslot;        // DDE_DSLOT_*
    uint8_t plen;         // payload bytes (4, 6, 8)
    uint8_t cyc_mod8;     // TX only in cycles with cycle % 8 == cyc_mod8
    uint8_t kind;         // DDE_DYN_KIND_*
    uint8_t k;            // E2E: crc8 xor constant (b1 = crc8(b2..bn) ^ k)
    uint8_t hi;           // counter byte high nibble (ign-on)
    uint8_t hi_pre;       // counter byte high nibble (pre-ign)
    uint8_t ign_only;     // 1 = only after ign-on (+DDE_IGN_E8L5_CYCLES)
    uint16_t period;      // cycles between transmissions
    uint8_t payload[8];       // ign-on content (counter/crc bytes are overwritten)
    uint8_t payload_pre[8];   // pre-ign content
    const char *name;
} dde_dyn_pdu_t;
extern const dde_dyn_pdu_t dde_dyn_pdu[DDE_DYN_N_PDU];
extern const uint16_t dde_dyn_slot_id[DDE_DYN_N_SLOT];

#define DDE_IGN_STATIC_CYCLES 12u       // switch to ign-on content 60 ms after the first 0x103 (DDE: 37-87 ms)
#define DDE_IGN_E8L5_CYCLES 45u         // 0xE8 L5 starts 225 ms after the first 0x103

// PDU of dynamic slot `dslot` due in `cycle` (by cycle % 8), -1 = none.
int dde_core_dyn_pdu_for(unsigned dslot, unsigned cycle);
int dde_core_dyn_slot_of_id(unsigned id);
// Build the full frame (5 + plen + 3 bytes) of PDU p. counter 0..14. Returns bytes.
unsigned dde_core_build_dyn_pdu(unsigned p, unsigned cycle, unsigned counter, bool preign, uint8_t *out);
// Static frame with optional pre-ign content (0x28 c%4==1 b3-b5, 0x29 c%4==3 b8).
unsigned dde_core_build_static_ex(unsigned target, unsigned cycle, unsigned counter, bool keep_sync,
                                  bool preign, uint8_t *out);
// Minislots a dynamic frame of `payload_bytes` occupies (0 = unknown length).
unsigned dde_core_dyn_ms_used(unsigned payload_bytes);
// DTS low bits so that the frame (TSS + bytes) plus DTS ends on a minislot action point.
unsigned dde_core_dyn_dts_bits(unsigned nbytes, unsigned tss_bits);
// Expected first-bit time of minislot m relative to cycle start (q8 ticks).
static inline int32_t dde_core_dyn_ap_q8(unsigned m) { return DDE_DYN_BASE_Q8 + (int32_t)m * DDE_DYN_MS_Q8; }
// Elapsed ticks from (us0, fine0) to (us1, fine1); us gives the coarse value,
// fine (10 ns, u16) the exact one. Valid for |dt| < 300 ms.
int32_t dde_core_elapsed_ticks(uint32_t us0, uint16_t fine0, uint32_t us1, uint16_t fine1);

// Skip reasons (stats order: index reason-1)
enum { DDE_DYN_OK = 0, DDE_SKIP_NOANCHOR = 1, DDE_SKIP_MARK, DDE_SKIP_PASSED, DDE_SKIP_LEN,
       DDE_SKIP_GRID, DDE_SKIP_LIMIT, DDE_SKIP_LATE, DDE_SKIP_N = 8 };

// Per-cycle dynamic-segment tracker. Fed with every FR1 header of the
// current cycle (after the cycle's first reference). Certain = every frame
// start (side-0 mark) since the anchor is explained by a decoded header.
typedef struct {
    bool active;
    bool anchor_ok;
    bool have_dyn;       // a dynamic frame anchors the prediction
    uint8_t why;         // reason when !anchor_ok
    uint8_t cycle;
    uint8_t acc_seq;     // side-0 mark seq of the last explained frame
    uint8_t last_k;
    uint16_t last_id, last_ms;
    uint16_t max_id;     // highest dynamic id seen this cycle (0 = none)
} dde_dyn_track_t;
void dde_dyn_track_start(dde_dyn_track_t *t, unsigned cycle, bool mark_ok, uint8_t seq);
// x_q8 = mark time of this header relative to cycle start (q8 ticks), only
// used for dynamic ids. mark_ok = this header's own side-0 mark was found.
void dde_dyn_track_header(dde_dyn_track_t *t, unsigned id, unsigned payload_bytes, bool mark_ok,
                          uint8_t seq, int64_t x_q8);
// Predict the minislot for our frame id X. Returns DDE_DYN_OK or a skip reason.
int dde_dyn_predict(const dde_dyn_track_t *t, unsigned id, unsigned *m_out);

#endif
