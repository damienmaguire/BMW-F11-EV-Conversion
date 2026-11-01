// v1h.1 standalone build: emulate at power-up, no laptop needed. See dde_standalone.h.
#include "dde_standalone.h"

bool dde_standalone_led(uint8_t state, uint32_t ms)
{
    switch (state) {
    case SA_LED_FAULT:     return ((ms / 100u) & 1u) == 0;    // fast blink: 0.1 s on, 0.1 s off
    case SA_LED_EMULATING: return ((ms / 500u) & 1u) == 0;    // slow blink: 0.5 s on, 0.5 s off
    default:               return (ms % 2000u) < 100u;        // short flash every 2 s: waiting for the car
    }
}

#if EMU_STANDALONE
#include <stdio.h>
#include "pico/stdlib.h"
#include "dde_emu.h"
#include "dde_engine.h"
#if PICO_FLEXRAY_BOARD_V2 && EMU_CAN_TX
#include "ptcan_emu.h"
#include "ptcan_emu_core.h"
#endif
#if !PICO_FLEXRAY_BOARD_V2
#include "hardware/gpio.h"
#include "hardware/sync.h"   // __sev()
#include "flexray_pins.h"
#endif

static uint32_t next_refresh_ms;       // when to re-send the settings
static uint32_t next_state_ms;         // when to look at the emulator state again
static volatile uint32_t host_cmd_ms;  // last switch from a laptop
static volatile bool host_cmd_seen;
static volatile uint8_t led_state = SA_LED_WAITING;
static bool announced;

static uint32_t ms_now(void) { return to_ms_since_boot(get_absolute_time()); }

void dde_standalone_host_cmd(void)
{
    host_cmd_ms = ms_now();
    host_cmd_seen = true;
}

uint8_t dde_standalone_led_state(void) { return led_state; }

// Send the logger's start settings to ourselves (the same calls the USB requests make).
static void apply_settings(void)
{
    dde_emu_host_set((uint16_t)SA_EMU_ON);                                    // FlexRay emulator TX on
#if PICO_FLEXRAY_BOARD_V2 && EMU_CAN_TX
    ptcan_emu_host_set(PTE_MASK_ALL);                                         // PT-CAN TX, all frames (= --can-tx)
#endif
    dde_engine_host_set((uint16_t)SA_ENGINE_MODE, (uint16_t)SA_ENGINE_PROFILE); // fake start cas, short, gear on
}

#if !PICO_FLEXRAY_BOARD_V2
// V1: the LED (GP20) is driven from a 50 ms timer so it keeps blinking even while the
// main loop sleeps (no car bus, no USB). The timer also wakes the main loop (__sev),
// so the settings are refreshed even when nothing else happens.
static repeating_timer_t v1_led_timer;
static bool v1_led_cb(repeating_timer_t *t)
{
    (void)t;
    gpio_put(LED_PIN, dde_standalone_led(led_state, ms_now()));
    __sev();
    return true;
}
#endif

void dde_standalone_start(void)
{
    printf("--- STANDALONE build: emulating from power-up with the settings of\n"
           "    fr_ts_logger.py --emulate --can-tx --fake-start cas --start-profile short (gear %s)\n"
           "    no USB host needed; LED: flash every 2 s = waiting for the car, slow blink = emulating,\n"
           "    fast blink = fault (power off and on to clear a latched fault) ---\n",
           EMU_GEAR ? "on" : "off (fixed P)");
    apply_settings();
    next_refresh_ms = ms_now() + SA_REFRESH_MS;
#if !PICO_FLEXRAY_BOARD_V2
    add_repeating_timer_ms(-50, v1_led_cb, NULL, &v1_led_timer);
#endif
}

void dde_standalone_poll(void)
{
    uint32_t now = ms_now();
    if ((int32_t)(now - next_refresh_ms) >= 0) {
        next_refresh_ms = now + SA_REFRESH_MS;
        // a laptop that is actively switching things wins; 25 s after its last switch
        // (longer than any firmware keepalive) the standalone settings come back
        if (!host_cmd_seen || (uint32_t)(now - host_cmd_ms) > SA_HOST_HOLD_MS) {
            if (host_cmd_seen) {
                host_cmd_seen = false;
                printf("standalone: no switches from the laptop for %u s, standalone settings back\n",
                       (unsigned)(SA_HOST_HOLD_MS / 1000u));
            }
            apply_settings();
        }
    }
    if ((int32_t)(now - next_state_ms) >= 0) {
        next_state_ms = now + 50u;
        static dde_emu_stats_t st;
        dde_emu_fill_stats(&st);
        uint8_t s = SA_LED_WAITING;
        if (st.fault || st.state == DDE_EMU_ST_FAULT) s = SA_LED_FAULT;
        else if (st.state == DDE_EMU_ST_ACTIVE) s = SA_LED_EMULATING;
#if PICO_FLEXRAY_BOARD_V2 && EMU_CAN_TX
        if (ptcan_emu_fault()) s = SA_LED_FAULT;
#endif
        if (s == SA_LED_EMULATING && !announced) {
            announced = true;
            printf("standalone: emulating (FlexRay TX active)\n");
        }
        led_state = s;
    }
}
#endif
