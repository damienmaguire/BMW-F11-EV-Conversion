#ifndef FLEXRAY_TS_H
#define FLEXRAY_TS_H
// TS-CAPTURE: hardware frame-start timestamps for the passive FR1/FR2 capture.
//
// FIXED build (on the 9a9ddcc pass-through repeater): for every frame the stock
// BSS streamer starts decoding it pulls the other side's TXEN low (after the TSS
// falling edge and the FR1/FR2 pair mutex). A read-only tsmark PIO SM on the
// same block sees that TXEN edge and pushes a word, and a 4-channel
// DMA chain (no CPU involved) latches, in this order:
//   1. pop the tsmark FIFO word                       (paced by its DREQ)
//   2. PWM slice counter, free-running at clk_sys     -> fine  (16 bit, 10 ns @ 100 MHz)
//   3. TIMER0 TIMERAWL (time_us_32 clock, 1 MHz)      -> us    (32 bit)
//   4. capture data-DMA WRITE_ADDR                    -> where this frame's
//      first byte will land in the FR1/FR2 byte ring (used to match the entry
//      to the frame the CPU parses later, exactly, without any time guessing).
// Each value goes to its own 256-entry ring; the main loop matches entries to
// parsed frames by ring index (flexray_ts_match()).
#include <stdbool.h>
#include <stdint.h>
#include "hardware/pio.h"

#define FLEXRAY_TS_PWM_SLICE 11u   // RP2350A: slices 8..11 have no GPIO in QFN-60, never pinned out
#define FLEXRAY_TS_RING_ENTRIES 256u

#include "flexray_ts_match_core.h" // flexray_ts_t, flexray_ts_stats_t

// Call from setup_stream() after the streamer SMs and their data DMA channels
// exist and BEFORE they are enabled. txen_pin_fr1 is the TXEN pin driven by the
// FR1-receiving streamer SM (TXEN_FR_2), txen_pin_fr2 the one driven by the FR2 SM.
void flexray_ts_setup(PIO pio, uint txen_pin_fr1, uint txen_pin_fr2,
                      uint data_dma_fr1, uint data_dma_fr2,
                      volatile uint8_t *ring_fr1, volatile uint8_t *ring_fr2);

// Core0 main loop: timestamp of the frame whose first byte is at byte-ring
// index frame_start_idx (0..ring_mask) on side (0 = FR1, 1 = FR2). Consumes
// all entries up to and including the match. Returns out->valid.
bool flexray_ts_match(int side, uint16_t frame_start_idx, uint16_t ring_mask, flexray_ts_t *out);

void flexray_ts_get_stats(flexray_ts_stats_t *st);

// DDE emulator (core1, ISR context): newest entry on a side WITHOUT consuming it.
// *ring_pos = byte-ring position of the frame start ((idx - ring base), unmasked),
// *seq = entries produced so far mod FLEXRAY_TS_RING_ENTRIES. Returns false when
// the side is not set up.
bool flexray_ts_peek_latest(int side, flexray_ts_t *out, uint32_t *ring_pos, uint32_t *seq);
uint32_t flexray_ts_fine_hz(void); // counter rate of the fine field (= clk_sys)

#endif
