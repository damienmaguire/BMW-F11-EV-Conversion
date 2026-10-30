// DDE emulator (PICO_FLEXRAY_DDE_EMU build): transmits the DDE's static frames
// 0x26/0x28/0x29/0x3D on FR1 in the DDE's slots, timed from the car's own frames.
// See Software/FlexRay/dde-emulator/README.md.
#ifndef DDE_EMU_H
#define DDE_EMU_H
#include <stdint.h>
#include "dde_emu_core.h"

#define PICOFLEX_SET_EMU   0xa3u   // vendor OUT, wValue 1 = enable/keepalive, 0 = disable + clear fault
#define PICOFLEX_GET_EMU   0xa4u   // vendor IN, 64 bytes: dde_emu_stats_t

// Pin used as the FR1 streamer's private frame-start mark (side-set) in the emu
// build: GP25 = Pico 2 on-board LED, not on any header pin.
#define DDE_EMU_MARK_PIN 25u

#define DDE_EMU_STATS_VERSION 1u
enum {
    DDE_EMU_ST_DISABLED = 0,   // host has not enabled TX (boot state)
    DDE_EMU_ST_NO_LOCK  = 1,   // enabled, waiting for 8 good reference cycles
    DDE_EMU_ST_ARMING   = 2,   // locked, waiting for 8 quiet cycles after enable
    DDE_EMU_ST_ACTIVE   = 3,   // transmitting
    DDE_EMU_ST_FAULT    = 4,   // latched fault, TX off until wValue 0
};
enum {
    DDE_EMU_FAULT_NONE = 0,
    DDE_EMU_FAULT_TX_STUCK = 1,    // TX state machine not idle when it should be
    DDE_EMU_FAULT_FOREIGN = 2,     // a DDE frame ID seen that we did not send (DDE still connected?)
    DDE_EMU_FAULT_NO_ECHO = 3,     // 8 transmitted frames in a row not seen back on FR1 RX
};

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
} dde_emu_stats_t;
_Static_assert(sizeof(dde_emu_stats_t) == 64, "emu stats size");

void dde_emu_early_pins(void);   // core0 setup_pins: all TXD/TXEN of FR1/FR2 = SIO output high
void dde_emu_init_core1(void);   // core1, after setup_stream(): PIO0 TX + fire SMs, alarm IRQ
// FR1/FR2 header ISR (core1), FR1 frames with a good header CRC.
void dde_emu_on_fr1_header(const uint8_t header[5], uint32_t start_pos, uint32_t ring_mask);
void dde_emu_host_set(uint16_t wvalue);         // USB 0xa3 (core0)
void dde_emu_fill_stats(dde_emu_stats_t *st);   // USB 0xa4 / UART (core0)
void dde_emu_print_stats(void);                 // UART line (core0)
#endif
