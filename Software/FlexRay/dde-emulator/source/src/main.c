#include <stdio.h>
#include <limits.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "fw_version.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/iobank0.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include <unistd.h>
#include "hardware/regs/addressmap.h"

#include "flexray_frame.h"
#include "panda_usb.h"
#include "flexray_bss_streamer.h"
#include "flexray_forwarder.h"
#include "flexray_injector.h"
#include "flexray_pins.h"
#include "flexray_ts.h"
#include "pico_fr_record.h"
#include "console.h"
#if PICO_FLEXRAY_DDE_EMU
#include "dde_emu.h"
#include "dde_standalone.h"
#endif
#if PICO_FLEXRAY_BOARD_V2
#include "v2_cantest.h"
#endif
#if PICO_FLEXRAY_DDE_EMU
#include "dde_emu.h"
#endif

#define SRAM __attribute__((section(".data")))
#define FLASH __attribute__((section(".rodata")))

extern char __end__;
extern char __StackTop;
extern char __StackLimit;

static inline uintptr_t get_sp(void) {
	uintptr_t sp;
	__asm volatile ("mov %0, sp" : "=r"(sp));
	return sp;
}

static void print_ram_usage(void) {
	void *heap_end = sbrk(0);
	uintptr_t sp = get_sp();

	uintptr_t heap_start = (uintptr_t)&__end__;
	uintptr_t stack_top = (uintptr_t)&__StackTop;
	uintptr_t stack_limit = (uintptr_t)&__StackLimit;

	size_t heap_used = (uintptr_t)heap_end - heap_start;
	size_t stack_used = stack_top - sp;
	size_t gap_heap_to_sp = sp - (uintptr_t)heap_end;   // remaining space between heap and sp
	size_t stack_free = sp - stack_limit;               // remaining space in stack

	printf("RAM usage: heap_used=%lu B, stack_used=%lu B, gap(heap->sp)=%lu B, stack_free=%lu B\n",
	       (unsigned long)heap_used,
	       (unsigned long)stack_used,
	       (unsigned long)gap_heap_to_sp,
	       (unsigned long)stack_free);
}


// Forward declaration for the Core 1 counter
extern volatile uint32_t core1_sent_frame_count;


void print_pin_assignments(void)
{
    printf("BGE Pin: %02d\n", BGE_PIN);
    printf("STBN Pin: %02d\n", STBN_PIN);
    printf("FR1 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_1_PIN, TXD_FR_1_PIN, TXEN_FR_1_PIN);
    printf("FR2 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_2_PIN, TXD_FR_2_PIN, TXEN_FR_2_PIN);
    printf("FR3 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_3_PIN, TXD_FR_3_PIN, TXEN_FR_3_PIN);
    printf("FR4 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_4_PIN, TXD_FR_4_PIN, TXEN_FR_4_PIN);
}

typedef struct {
    uint32_t total_notif;
    uint32_t seq_gap;
    uint32_t crc_fail;
    uint32_t parsed_ok;
    uint32_t valid;
    uint32_t len_mismatch;
    uint32_t len_ok;
    uint32_t parse_fail;
    uint32_t source_fr1;
    uint32_t source_fr2;
    uint32_t source_fr3;
    uint32_t source_fr4;
    uint32_t overflow_len;
    uint32_t zero_len;
    uint32_t usb_fifo_dropped;
} stream_stats_t;

// TS-CAPTURE (FIXED build): file scope so the USB 0xa0/ts-info handler can read it.
static stream_stats_t stats;
static uint32_t capture_seq;   // accepted CRC-valid frames pushed towards USB

uint8_t FRAME_CACHE[262][10];

static void stats_print(const stream_stats_t *s, uint32_t prev_total, uint32_t prev_valid)
{
    uint32_t total_fps = (s->len_ok - prev_total) / 5;
    uint32_t valid_fps = (s->valid - prev_valid) / 5;

    printf("Ring Stats: total=%lu seq_gap=%lu src[FR1=%lu,FR2=%lu,FR3=%lu,FR4=%lu] len_ok=%lu len_mis=%lu overflow=%lu zero=%lu parse_fail=%lu valid=%lu | fps[frames=%lu/s,valid=%lu/s]\n",
           s->total_notif, s->seq_gap, s->source_fr1, s->source_fr2,
           s->source_fr3, s->source_fr4,
           s->len_ok, s->len_mismatch, s->overflow_len, s->zero_len,
           s->parse_fail, s->valid, total_fps, valid_fps);
    printf("Notify dropped=%lu\n", notify_queue_dropped());
    flexray_ts_stats_t ts;
    flexray_ts_get_stats(&ts);
    printf("TS: matched=%lu missing=%lu skipped=%lu marks[FR1=%lu,FR2=%lu] seq=%lu usb_fifo_dropped=%lu fine_hz=%lu\n",
           (unsigned long)ts.matched, (unsigned long)ts.missing, (unsigned long)ts.skipped,
           (unsigned long)ts.marks[0], (unsigned long)ts.marks[1], (unsigned long)capture_seq,
           (unsigned long)s->usb_fifo_dropped, (unsigned long)flexray_ts_fine_hz());
#if PICO_FLEXRAY_DDE_EMU
    dde_emu_print_stats();
    dde_engine_print();
#endif
}

// Reply for vendor request PICOFLEX_GET_TS_INFO (called from panda_usb.c).
void capture_fill_ts_info(pico_ts_info_t *ti)
{
    flexray_ts_stats_t ts;
    flexray_ts_get_stats(&ts);
    memset(ti, 0, sizeof(*ti));
    ti->info_version = PICO_TS_INFO_VERSION;
    ti->fine_bits = 16;
    ti->armed = 1; // no arm gate in the pass-through build: streams from boot like c5fa46a8
    ti->fine_hz = flexray_ts_fine_hz();
    ti->now_us = time_us_32();
    ti->seq_next = capture_seq;
    ti->usb_fifo_dropped = stats.usb_fifo_dropped;
    ti->notify_dropped = notify_queue_dropped();
    ti->ring_overflow = stats.overflow_len;
    ti->len_mismatch = stats.len_mismatch;
    ti->parse_fail = stats.parse_fail;
    ti->ts_matched = ts.matched;
    ti->ts_missing = ts.missing;
    ti->ts_skipped = ts.skipped;
    ti->held_pre_arm = 0;
    ti->valid_frames = stats.valid;
    ti->crc_fail = stats.crc_fail;
}

void core1_entry(void)
{
#if PICO_FLEXRAY_DDE_EMU
    // Emulator: no FR3/FR4 capture (pio0 hosts the emulator TX instead).
    setup_stream(pio1);
    dde_emu_init_core1();
#else
    setup_stream_fr34(pio0);

    setup_stream(pio1);
#endif
#if PICO_FLEXRAY_DDE_EMU
    // v1h.2: the emulator's engine story + gear run here, outside the ISRs. WFE: the FR1
    // header ISR raises SEV for a new cycle (latched, so none is lost between the poll and
    // the WFE); any interrupt also wakes it (1 ms housekeeping alarm = >= 1 kHz).
    while (1)
    {
        dde_emu_core1_poll();
        __wfe();
    }
#else
    while (1)
    {
        __wfi();
    }
#endif
}

void setup_pins(void)
{
    // disable transceiver
    gpio_init(BGE_PIN);
    gpio_set_dir(BGE_PIN, GPIO_OUT);
    gpio_put(BGE_PIN, 0);

    gpio_init(STBN_PIN);
    gpio_set_dir(STBN_PIN, GPIO_OUT);
    gpio_put(STBN_PIN, 0);

#if PICO_FLEXRAY_DDE_EMU
    // Emulator: FR1/FR2 TXD and TXEN are driven high (transmitter off) from here on.
    dde_emu_early_pins();
#endif
    gpio_pull_up(TXEN_FR_1_PIN);
    gpio_pull_up(TXEN_FR_2_PIN);
    gpio_pull_up(TXEN_FR_3_PIN);
    gpio_pull_up(TXEN_FR_4_PIN);

    gpio_init(RXD_FR_1_PIN);
    gpio_set_dir(RXD_FR_1_PIN, GPIO_IN);
    gpio_init(RXD_FR_2_PIN);
    gpio_set_dir(RXD_FR_2_PIN, GPIO_IN);

    gpio_init(RXD_FR_3_PIN);
    gpio_set_dir(RXD_FR_3_PIN, GPIO_IN);
    gpio_init(RXD_FR_4_PIN);
    gpio_set_dir(RXD_FR_4_PIN, GPIO_IN);

    gpio_pull_up(RXD_FR_1_PIN);
    gpio_pull_up(RXD_FR_2_PIN);
    gpio_pull_up(RXD_FR_3_PIN);
    gpio_pull_up(RXD_FR_4_PIN);

#if !PICO_FLEXRAY_BOARD_V2
    gpio_init(RELAY_FR_1_2);
    gpio_set_dir(RELAY_FR_1_2, GPIO_OUT);
    gpio_put(RELAY_FR_1_2, 1);
    sleep_ms(500);
    gpio_init(RELAY_FR_3_4);
    gpio_set_dir(RELAY_FR_3_4, GPIO_OUT);
    gpio_put(RELAY_FR_3_4, 1);
#else
    // V2 has no bypass relays (GP17/GP18 are the CAN SPI CS/SCK lines).
    v2_leds_init();
#endif

    // delay enabling pins to avoid glitch
    sleep_ms(100);

    // enable transceiver
    gpio_put(BGE_PIN, 1);
    gpio_put(STBN_PIN, 1);

#if !PICO_FLEXRAY_BOARD_V2
		// On-board LED
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
#endif
}

int main(void)
{
    setup_pins();

    bool clock_configured = set_sys_clock_khz(100000, true);
    stdio_init_all();
    // v1f: printf() no longer waits for the 115200 baud UART (the 5 s status print used to
    // stop this loop for 120-160 ms, which cost PT-CAN frames). See console.c.
    console_init();
    printf("\n=== pico-flexray firmware %s (built %s) ===\n", FW_VERSION, __DATE__);
    printf("static_used=%lu B\n", (unsigned long)((uintptr_t)&__end__ - (uintptr_t)SRAM_BASE));
    print_ram_usage();
    // Initialize Panda USB interface
    panda_usb_init();
    // Initialize cross-core notification queue before starting streams
    notify_queue_init();
    // --- Set system clock to 100MHz (RP2350) ---
    // make PIO clock div has no fraction, reduce jitter
    if (!clock_configured)
    {
        printf("Warning: Failed to set system clock, using default\n");
    }
    else
    {
        printf("System clock set to 125MHz\n");
    }

    print_pin_assignments();

    printf("Actual system clock: %lu Hz\n", clock_get_hz(clk_sys));
    printf("\n--- FlexRay Continuous Streaming Bridge (Forwarder Mode) ---\n");
#if PICO_FLEXRAY_BOARD_V2
    printf("Board: Pi_Flex_V2 (FR3/FR4 parked on unconnected pins), LED1=GP%d LED2=GP%d\n", V2_LED1_PIN, V2_LED2_PIN);
    printf("PT-CAN: MCP251863 LISTEN-ONLY 500 kbit/s classic, RX only (no TX), USB CAN records bus %d\n", V2_CAN_USB_BUS);
    // CAN controller init on core0 before the FlexRay machinery (bounded, < ~30 ms).
    v2_can_init();
#endif

    multicore_launch_core1(core1_entry);
    sleep_ms(500);


#if !PICO_FLEXRAY_DDE_EMU
    setup_forwarder_with_injector(pio2);
#else
#if EMU_STANDALONE
    dde_standalone_start();   // v1h.1: emulate from power-up, no host needed (see dde_standalone.h)
#else
    printf("--- DDE EMULATOR build: TX on FR1 disabled until host enables it (vendor 0xa3) ---\n");
#endif
#endif


    uint8_t temp_buffer[MAX_FRAME_BUF_SIZE_BYTES];

	absolute_time_t next_stats_print_time = make_timeout_time_ms(5000);
	absolute_time_t next_led_toggle_time = make_timeout_time_ms(500);
	bool led_on = false;
#if PICO_FLEXRAY_BOARD_V2
    (void)next_led_toggle_time;
    (void)led_on;
    v2_tick_start(); // 50 ms alarm-pool tick (core0): LED1 heartbeat, LED2, CAN housekeeping
#endif
    // Track previous len_ok to compute parsed-frames FPS
    uint32_t prev_total = 0;
    uint32_t prev_valid = 0;

    while (true)
    {
        panda_usb_task();
#if PICO_FLEXRAY_DDE_EMU
        dde_gear_console_poll();   // v1h.1: "GEAR: P -> R (brake on) ..." lines on the console
#if EMU_STANDALONE
        dde_standalone_poll();     // v1h.1: re-send the standalone settings every 1 s, LED state
#endif
#endif
#if PICO_FLEXRAY_BOARD_V2
        v2_cantest_poll(); // bounded CAN RX drain + 50 ms housekeeping, never waits on USB
#elif PICO_FLEXRAY_DDE_EMU && EMU_STANDALONE
        // V1 standalone: the LED is driven by dde_standalone.c (50 ms timer)
        (void)next_led_toggle_time;
        (void)led_on;
#else
		if (time_reached(next_led_toggle_time))
		{
			next_led_toggle_time = make_timeout_time_ms(500);
			led_on = !led_on;
			gpio_put(LED_PIN, led_on);
		}
#endif
        if (time_reached(next_stats_print_time))
        {
            next_stats_print_time = make_timeout_time_ms(5000);
            stats_print(&stats, prev_total, prev_valid);
#if PICO_FLEXRAY_BOARD_V2
            v2_cantest_print_status();
#endif
            prev_total = stats.len_ok;
            prev_valid = stats.valid;
            print_ram_usage();
        }

        // Consume frame-end notifications from core1 (FR1/FR2 only;
        // FR3/FR4 ISR just records frame IDs for channel demuxing)
        static uint16_t last_end_idx_fr1 = 0;
        static uint16_t last_end_idx_fr2 = 0;
        static uint32_t last_seq = 0;

        uint32_t encoded;
        if (!notify_queue_pop(&encoded))
        {
            panda_usb_task();
            __wfe();
            continue;
        }
        do {
            notify_info_t info; notify_decode(encoded, &info);

            stats.total_notif++;
            if (stats.total_notif > 1 && ((info.seq - last_seq) & 0x3FFFF) != 1) stats.seq_gap++;
            last_seq = info.seq;

            if (info.is_fr2) stats.source_fr2++;
            else stats.source_fr1++;

            volatile uint8_t *ring_base;
            uint16_t ring_mask;
            uint16_t prev_end;
            uint8_t source;

            if (info.is_fr2) {
                ring_base = fr2_ring_buffer;
                ring_mask = FR2_RING_MASK;
                prev_end = last_end_idx_fr2;
                source = FROM_FR2;
            } else {
                ring_base = fr1_ring_buffer;
                ring_mask = FR1_RING_MASK;
                prev_end = last_end_idx_fr1;
                source = FROM_FR1;
            }

            uint16_t len = (uint16_t)((info.end_idx - prev_end) & ring_mask);

            if (len == 0 || len > MAX_FRAME_BUF_SIZE_BYTES)
            {
                if (info.is_fr2) last_end_idx_fr2 = info.end_idx;
                else last_end_idx_fr1 = info.end_idx;
                if (len == 0) stats.zero_len++;
                else stats.overflow_len++;
                continue;
            }

            uint16_t start = (uint16_t)((info.end_idx - len) & ring_mask);
            uint16_t first = (uint16_t)((len <= (ring_mask + 1 - start)) ? len : (ring_mask + 1 - start));
            memcpy(temp_buffer, (const void *)(ring_base + start), first);
            if (first < len)
            {
                memcpy(temp_buffer + first, (const void *)ring_base, (size_t)(len - first));
            }

            uint16_t pos = 0;
            while ((uint16_t)(len - pos) >= 8)
            {
                uint8_t *header = temp_buffer + pos;
                uint8_t payload_len_words = (header[2] >> 1) & 0x7F;
                uint16_t expected_len = (uint16_t)(5 + (payload_len_words * 2) + 3);
                if (expected_len == 0 || expected_len > FRAME_BUF_SIZE_BYTES) {
                    stats.len_mismatch++;
                    break;
                }
                if ((uint16_t)(len - pos) < expected_len) {
                    break;
                }

                stats.len_ok++;

                flexray_frame_t frame;
                if (!parse_frame_from_slice(header, expected_len, source, &frame))
                {
                    stats.parse_fail++;
                    pos = (uint16_t)(pos + 1);
                    continue;
                }
                else if (is_valid_frame(&frame, header))
                {
                    stats.valid++;

                    uint8_t demuxed = info.fr34_source;
                    if (demuxed != FROM_UNKNOWN) {
                        frame.source |= demuxed;
                        if (demuxed & FROM_FR3) stats.source_fr3++;
                        if (demuxed & FROM_FR4) stats.source_fr4++;
                    } else {
                        uint32_t idx = flexray_unknown_source_idx++;
                        flexray_unknown_source_ids[idx & 31u] = frame.frame_id;
                    }
                    try_cache_last_target_frame(frame.frame_id, frame.cycle_count, expected_len, header);
#if PICO_FLEXRAY_DDE_EMU
                    // v1d: KL15 state from the vehicle's 0xF1 (FR1 only; core1 polls it once per cycle)
                    if (!info.is_fr2)
                        dde_emu_on_frame(frame.frame_id, frame.cycle_count, frame.payload,
                                         (unsigned)frame.payload_length_words * 2u);
#endif

                    // TS-CAPTURE: hardware frame-start timestamp, matched by byte-ring position.
                    flexray_ts_t ts;
                    uint16_t frame_start = (uint16_t)((start + pos) & ring_mask);
                    flexray_ts_match(info.is_fr2 ? 1 : 0, frame_start, ring_mask, &ts);
                    frame.ts_flags = PICO_FR_FLAG_FRAME_CRC_OK | PICO_FR_FLAG_HEADER_CRC_OK;
                    if (ts.valid) {
                        frame.ts_us = ts.us;
                        frame.ts_fine = ts.fine;
                        frame.ts_flags |= PICO_FR_FLAG_TS_HW;
                    } else {
                        frame.ts_us = time_us_32();
                        frame.ts_fine = 0;
                    }
                    frame.seq = capture_seq++;
                    if (!panda_flexray_fifo_push(&frame)) {
                        stats.usb_fifo_dropped++;
                    }
#if PICO_FR_USB_FAST || PICO_FLEXRAY_DDE_EMU
                    // Keep the USB link moving during long bursts of FlexRay frames: USB
                    // only sends the next 64-byte packet when panda_usb_task() runs, and
                    // this loop can otherwise run for a while without calling it.
                    // v1g.2: also in the DDE emulator builds (cheap: a few us, every 8 frames).
                    if ((capture_seq & 7u) == 0) panda_usb_task();
#endif
                }
                else
                {
                    stats.crc_fail++;   // header/frame CRC (or id/len) reject: dropped before seq is assigned
                }

                pos = (uint16_t)(pos + expected_len);
            }

            if (info.is_fr2) last_end_idx_fr2 = info.end_idx;
            else last_end_idx_fr1 = info.end_idx;
        } while (notify_queue_pop(&encoded));
    }

    return 0;
}
