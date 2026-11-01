// Non-blocking debug console (v1f).
//
// In plain words
// --------------
// Up to v1e every printf() waited until the UART (115200 baud, about 11.5 characters
// per millisecond) had sent the text. The 5-second status print is 1.5-2 KB long, so
// the main loop stood still for 120-160 ms every 5 s. During that time no PT-CAN frame
// could be queued, and the 20 ms frames (0x0A5/0x0D9/0x0DC/0x0F3) lost about 6 frames
// in every 5 s (run 6: 6,447 sent of 6,634 due).
//
// Now printf() only copies the text into a 16 KB memory ring and returns at once.
// A 1 ms timer interrupt (and the main loop) move the text to the UART hardware FIFO,
// only as much as fits without waiting. If the ring is ever full, the extra text is
// dropped (counted, and reported by the next line that fits) - the program never waits.
#include "console.h"

#include <stdio.h>
#include <string.h>
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
#include "pico/time.h"

#ifndef CONSOLE_RING_SIZE
#define CONSOLE_RING_SIZE 16384u   // power of two
#endif
_Static_assert((CONSOLE_RING_SIZE & (CONSOLE_RING_SIZE - 1)) == 0, "ring size must be a power of two");

static char ring[CONSOLE_RING_SIZE];
static volatile uint32_t head, tail;     // head: next write, tail: next read (free-running)
static volatile uint32_t dropped, dropped_reported;
static spin_lock_t *lock;
static repeating_timer_t drain_timer;
static bool active;

static void put_locked(const char *s, uint32_t n)
{
    uint32_t free_ = CONSOLE_RING_SIZE - (head - tail);
    if (n > free_) { dropped += n - free_; n = free_; }
    for (uint32_t i = 0; i < n; i++) ring[(head + i) & (CONSOLE_RING_SIZE - 1)] = s[i];
    head += n;
}

static void out_chars(const char *buf, int len)
{
    if (len <= 0) return;
    uint32_t irq = spin_lock_blocking(lock);
    if (dropped != dropped_reported) {
        char note[48];
        int k = snprintf(note, sizeof note, "\r\n[console: %lu chars dropped]\r\n",
                         (unsigned long)(dropped - dropped_reported));
        if ((uint32_t)k + (uint32_t)len <= CONSOLE_RING_SIZE - (head - tail)) {
            dropped_reported = dropped;
            put_locked(note, (uint32_t)k);
        }
    }
    put_locked(buf, (uint32_t)len);
    spin_unlock(lock, irq);
}

static void out_flush(void) { console_drain(); }

static stdio_driver_t console_driver = {
    .out_chars = out_chars,
    .out_flush = out_flush,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_DEFAULT_CRLF,
#endif
};

void console_drain(void)
{
    if (!active) return;
    uart_inst_t *u = uart_get_instance(PICO_DEFAULT_UART);
    uint32_t irq = spin_lock_blocking(lock);
    while (tail != head && uart_is_writable(u)) {
        uart_putc_raw(u, ring[tail & (CONSOLE_RING_SIZE - 1)]);
        tail++;
    }
    spin_unlock(lock, irq);
}

static bool drain_cb(repeating_timer_t *t)
{
    (void)t;
    console_drain();
    return true;
}

void console_init(void)
{
    lock = spin_lock_init(spin_lock_claim_unused(true));
    head = tail = dropped = dropped_reported = 0;
    active = true;
    // Swap the SDK's blocking UART driver for the ring (the UART itself and its pins
    // were already set up by stdio_init_all()).
    stdio_set_driver_enabled(&stdio_uart, false);
    stdio_set_driver_enabled(&console_driver, true);
    add_repeating_timer_ms(-1, drain_cb, NULL, &drain_timer);
}

uint32_t console_dropped(void) { return dropped; }

uint32_t console_pending(void) { return head - tail; }
