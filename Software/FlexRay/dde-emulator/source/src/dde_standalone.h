// v1h.1 standalone build: emulate at power-up, no laptop needed.
//
// In plain words
// --------------
// The normal emulator image does nothing on its own: the laptop logger switches the
// FlexRay emulator on (USB 0xa3), PT-CAN TX on (0xa5, V2 board), the fake engine start
// on (0xa7) and then repeats those switches every 1..2 s ("keepalive"). If the laptop
// stops, the firmware switches everything off again after 3..20 s.
//
// The standalone image (CMake -DEMU_STANDALONE=1) plays the laptop's part itself, with
// exactly the settings of
//     fr_ts_logger.py --emulate --can-tx --fake-start cas --start-profile short
// (gear shifting on): from power-up, without waiting for USB, and it keeps them up for
// as long as the board has power. A laptop may still be plugged in to watch the
// console or log. If the laptop sends its own switches, those win for as long as it
// keeps sending them (plus 25 s), then the standalone settings come back.
//
// What it does NOT do: it never clears a latched fault. A fault that latches (for
// example a second sender of one of our IDs = the real DDE is still on the bus, or
// the CAN chip went bus-off) stays until the board is powered off and on again.
// The LED says which state it is in (see dde_standalone_led()).
#ifndef DDE_STANDALONE_H
#define DDE_STANDALONE_H

#include <stdbool.h>
#include <stdint.h>

#ifndef EMU_STANDALONE
#define EMU_STANDALONE 0
#endif

// LED states
enum {
    SA_LED_WAITING = 0,   // powered, waiting for the car's FlexRay bus (one short flash every 2 s)
    SA_LED_EMULATING = 1, // emulating (FlexRay TX active): slow blink, 0.5 s on / 0.5 s off
    SA_LED_FAULT = 2,     // a fault (FlexRay emulator or PT-CAN TX): fast blink, 0.1 s on / 0.1 s off
};

#if EMU_STANDALONE
// The settings the logger normally sends at start (compile-time defaults).
#define SA_EMU_ON 1u                 // = --emulate (USB 0xa3 wValue 1)
#define SA_ENGINE_MODE 1u            // = --fake-start cas (ENG_MODE_CAS)
#define SA_ENGINE_PROFILE 1u         // = --start-profile short (ENG_PROF_SHORT); gear on (wIndex bit 8 clear)
#define SA_REFRESH_MS 1000u          // re-send them every second, like the logger
#define SA_HOST_HOLD_MS 25000u       // after the last switch from a laptop, leave it alone this long

void dde_standalone_start(void);         // core0, once, before the main loop
void dde_standalone_poll(void);          // core0 main loop: never blocks
void dde_standalone_host_cmd(void);      // panda_usb.c: the laptop sent 0xa3/0xa5/0xa7
uint8_t dde_standalone_led_state(void);  // SA_LED_*
#endif

// LED on/off for a state at time ms (pure, also used by the host test).
bool dde_standalone_led(uint8_t state, uint32_t ms);

#endif
