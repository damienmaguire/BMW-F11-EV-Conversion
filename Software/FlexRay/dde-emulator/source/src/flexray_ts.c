#include "flexray_ts.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/structs/pwm.h"
#include "hardware/structs/timer.h"
#include "flexray_tsmark.pio.h"

#define RING_BYTES (FLEXRAY_TS_RING_ENTRIES * 4u)
#define RING_BITS 10u // log2(RING_BYTES)
_Static_assert(RING_BYTES == (1u << RING_BITS), "ts ring size");

static volatile uint32_t ts_fine_ring[2][FLEXRAY_TS_RING_ENTRIES] __attribute__((aligned(RING_BYTES)));
static volatile uint32_t ts_us_ring[2][FLEXRAY_TS_RING_ENTRIES] __attribute__((aligned(RING_BYTES)));
static volatile uint32_t ts_idx_ring[2][FLEXRAY_TS_RING_ENTRIES] __attribute__((aligned(RING_BYTES)));
static volatile uint32_t ts_pop_dummy[2];

static int ch_idx[2] = {-1, -1};
static uint32_t ring_base_addr[2];
static uint32_t rd[2];
static flexray_ts_stats_t stats;

static void setup_side(PIO pio, int side, uint tsmark_sm, uint data_dma, volatile uint8_t *ring)
{
    uint ch_pop = dma_claim_unused_channel(true);
    uint ch_fine = dma_claim_unused_channel(true);
    uint ch_us = dma_claim_unused_channel(true);
    uint chi = dma_claim_unused_channel(true);
    ch_idx[side] = (int)chi;
    ring_base_addr[side] = (uint32_t)(uintptr_t)ring;

    // 1. pop the tsmark word (paced by the tsmark RX FIFO DREQ)
    dma_channel_config c = dma_channel_get_default_config(ch_pop);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, tsmark_sm, false));
    channel_config_set_high_priority(&c, true);
    channel_config_set_chain_to(&c, ch_fine);
    dma_channel_configure(ch_pop, &c, (void *)&ts_pop_dummy[side], &pio->rxf[tsmark_sm], 1, false);

    // 2. fine counter (PWM CTR at clk_sys)
    c = dma_channel_get_default_config(ch_fine);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RING_BITS);
    channel_config_set_high_priority(&c, true);
    channel_config_set_chain_to(&c, ch_us);
    dma_channel_configure(ch_fine, &c, (void *)ts_fine_ring[side],
                          &pwm_hw->slice[FLEXRAY_TS_PWM_SLICE].ctr, 1, false);

    // 3. 1 MHz system timer (same clock as time_us_32 / CAN records)
    c = dma_channel_get_default_config(ch_us);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RING_BITS);
    channel_config_set_high_priority(&c, true);
    channel_config_set_chain_to(&c, chi);
    dma_channel_configure(ch_us, &c, (void *)ts_us_ring[side], &timer_hw->timerawl, 1, false);

    // 4. capture data-DMA write pointer (frame start position in the byte ring), then re-arm 1.
    c = dma_channel_get_default_config(chi);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RING_BITS);
    channel_config_set_high_priority(&c, true);
    channel_config_set_chain_to(&c, ch_pop);
    dma_channel_configure(chi, &c, (void *)ts_idx_ring[side],
                          &dma_hw->ch[data_dma].write_addr, 1, false);

    dma_channel_start(ch_pop); // armed: waits for the first tsmark DREQ
}

void flexray_ts_setup(PIO pio, uint txen_pin_fr1, uint txen_pin_fr2,
                      uint data_dma_fr1, uint data_dma_fr2,
                      volatile uint8_t *ring_fr1, volatile uint8_t *ring_fr2)
{
    // Free-running fine counter: PWM slice at clk_sys / 1, wrap 0xFFFF, no pin.
    pwm_config pc = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&pc, 1);
    pwm_config_set_wrap(&pc, 0xFFFF);
    pwm_init(FLEXRAY_TS_PWM_SLICE, &pc, true);

    // Must be called AFTER the streamer program is loaded so the streamer keeps the
    // same instruction-memory offset (and so byte-identical words) as the stock build.
    uint offset = pio_add_program(pio, &flexray_tsmark_txen_program);
    uint sm_ts[2];
    sm_ts[0] = (uint)pio_claim_unused_sm(pio, true);
    sm_ts[1] = (uint)pio_claim_unused_sm(pio, true);
    // side 0 = frames received on FR1: the FR1 streamer SM drives TXEN of FR2.
    flexray_tsmark_txen_program_init(pio, sm_ts[0], offset, txen_pin_fr1);
    flexray_tsmark_txen_program_init(pio, sm_ts[1], offset, txen_pin_fr2);

    setup_side(pio, 0, sm_ts[0], data_dma_fr1, ring_fr1);
    setup_side(pio, 1, sm_ts[1], data_dma_fr2, ring_fr2);
    rd[0] = rd[1] = 0;

    pio_sm_set_enabled(pio, sm_ts[0], true);
    pio_sm_set_enabled(pio, sm_ts[1], true);
}

static inline uint32_t produced_index(int side)
{
    uint32_t wa = dma_hw->ch[ch_idx[side]].write_addr;
    return ((wa - (uint32_t)(uintptr_t)ts_idx_ring[side]) >> 2) & (FLEXRAY_TS_RING_ENTRIES - 1u);
}

bool flexray_ts_match(int side, uint16_t frame_start_idx, uint16_t ring_mask, flexray_ts_t *out)
{
    if (ch_idx[side] < 0) {
        out->valid = false;
        return false;
    }
    return flexray_ts_match_core(ts_us_ring[side], ts_fine_ring[side], ts_idx_ring[side],
                                 FLEXRAY_TS_RING_ENTRIES, &rd[side], produced_index(side),
                                 ring_base_addr[side], frame_start_idx, ring_mask,
                                 &stats, side, out);
}

void flexray_ts_get_stats(flexray_ts_stats_t *st) { *st = stats; }

bool __time_critical_func(flexray_ts_peek_latest)(int side, flexray_ts_t *out, uint32_t *ring_pos, uint32_t *seq)
{
    out->valid = false;
    if (ch_idx[side] < 0) return false;
    uint32_t w = produced_index(side);
    uint32_t i = (w - 1u) & (FLEXRAY_TS_RING_ENTRIES - 1u);
    out->us = ts_us_ring[side][i];
    out->fine = (uint16_t)ts_fine_ring[side][i];
    out->valid = true;
    *ring_pos = ts_idx_ring[side][i] - ring_base_addr[side];
    *seq = w;
    return true;
}

uint32_t flexray_ts_fine_hz(void) { return clock_get_hz(clk_sys); }
