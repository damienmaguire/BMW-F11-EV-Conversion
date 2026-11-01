// v1h.1: gear selection for the DDE/EGS emulator (no real gearbox in the car).
//
// Plain English: with the real EGS gone nobody answers the gear lever, so the
// car always shows P. This module reads the driver's lever (GWS frame 0x197 on
// PT-CAN, received only - we never send 0x197) and the brake (PT-CAN 0x173),
// decides which gear the driver asked for, and tells the frame builders which
// bytes to put in the EGS/DDE frames the emulator ALREADY sends, so the dash and
// the selector show P / R / N / D like the real EGS would.
//
// Shift rules (Ian's egs_gear_report_tx.md = the real EGS table; state machine in
// gear_act() in dde_gear.c):
//   From P:  forward 1 = N, forward 2 = R, back (1 or 2) = D.
//   From R:  back 1 = N, back 2 = D.
//   From N:  forward = R, back = D.
//   From D:  forward 1 = N, forward 2 = R.
//   P button = P from R / N / D (we have no road speed here: standstill assumed).
//   * Leaving P needs the engine "running" (our fake engine in RUN) AND the brake
//     pressed (0x173 d7 bits 0-1 == 3 in the last 250 ms). P/N -> R/D also needs the
//     brake (BMW shiftlock). No brake-off shift was ever captured: HYPOTHESIS, logged.
//   * We act 40 ms after a lever notch first appears, on the deepest notch reached
//     in that push (the real EGS: 0x3FD changes 40 ms after the lever frame; a notch
//     held only ~21-30 ms on the way to the 2nd notch is skipped, so P -> R goes
//     straight to R ~40 ms after forward 2; N is shown only when forward 1 is held
//     >= 40 ms). One action per notch, a new push only after the lever was back in the centre.
//   * v1h.2 (Ian's run 9 comparison): P is released (0x39A E1, 0x3F9 P-lock 1) 20 ms
//     after the FIRST notch of a push out of P, before the gear decision; the gear code
//     on 0x39A / 0x3F9 / 0xDC follows 0x3FD 10 ms later (real +2..+21 ms).
//     P button: 0x39A gear 01 +21 ms, 0x3F9 gear 1 +42 ms, 0x39A 03 +881 ms (19 ms
//     BEFORE 0x3FD P), 0x3FD P +900 ms, all from the first pressed 0x197 frame.
//     Pawl (0x39A E2, 0x3F9 P-lock 2): 340 ms after the brake is released, 3.0 s after
//     the P button at the latest.
//   * No valid 0x197 for 90 ms (the EGS timeout): we stay in gear and log it.
//   * Engine not running: stays in P (moves are refused and logged); if the engine
//     stops while in R/N/D we go to P automatically (BMW auto-P).
//   * There is no unlock button in 0x197: the GWS gates the lever itself using 0x3FD,
//     so 0x3FD always shows the gear we have really accepted.
//   * Switched off (build EMU_GEAR=0 or logger --no-gear): fixed P exactly as v1h.
//
// Byte values: every value for P, R, D, the P release and the P-engage sequence is
// copied from the real EGS (F11_4log_r10md10m.csv, Ian's report). N was only ever
// seen for 60 ms on the way from P to R: 0x3FD 0x60 is real, the rest of N
// (0x39A gear 01, 0x3F9/0xE6 gear nibble 1, 0xDC/0x28 22 FE) is HYPOTHESIS.
//
// Pure C (no SDK): the firmware runs gear_step() on core 1 once per FlexRay cycle,
// the host test (tests/host/test_gear.c) runs it with simulated time and with the
// real lever/brake frames of the capture.
#ifndef DDE_GEAR_H
#define DDE_GEAR_H
#include <stdbool.h>
#include <stdint.h>
#include "dde_engine.h"

#ifndef EMU_GEAR
#define EMU_GEAR 1          // build option (CMake EMU_GEAR): 0 = compiled out, fixed P
#endif

enum { GEAR_P = 0, GEAR_R = 1, GEAR_N = 2, GEAR_D = 3 };
// what the bytes show right now
enum { GST_P_LOCKED = 0,    // P with the parking pawl in (= the v1h fixed-P content, nothing patched)
       GST_LEAVE_P,         // P released (0x39A E1, 0x3F9 bits 28-29 = 1), new gear on 0x3FD; v1h.2: gear code 10 ms later
       GST_STEADY,          // in R, N or D
       GST_TO_P,            // P button: 0x39A gear 01 (+21), 0x3F9 gear 1 (+42), 0x39A 03 (+881), old gear on 0x3FD until +900 ms
       GST_P_UNLOCKED,      // P shown, pawl confirmed 340 ms after brake release or 3 s after the button
       GST_P_RELEASED };    // v1h.2: P released (0x39A E1, 0x3F9 bits 1) 20 ms after the first notch, 0x3FD still P
// lever positions (0x197 B2 >> 4)
enum { LV_CENTRE = 0, LV_FWD1 = 1, LV_FWD2 = 2, LV_BACK1 = 3, LV_BACK2 = 4 };
// event kinds / reasons
enum { GEV_CHANGE = 1, GEV_REFUSED = 2, GEV_LOCKED = 3, GEV_ONOFF = 4, GEV_LEVER_LOST = 5, GEV_LEVER_OK = 6 };
enum { GWHY_FWD1 = 1, GWHY_FWD2, GWHY_BACK1, GWHY_BACK2, GWHY_PBTN, GWHY_ENGINE_OFF, GWHY_DISABLED,
       GWHY_ENABLED,
       GREF_NO_BRAKE = 16, GREF_NOT_RUNNING };

// timings (ms), from the capture / Ian's report
#define GEAR_BRAKE_MS        250u   // brake "pressed" if a pressed 0x173 came in the last 250 ms
#define GEAR_LEVER_STALE_MS   90u   // no valid 0x197 for this long: lever lost (EGS timeout 90 ms)
#define GEAR_ACT_MS           40u   // act 40 ms after a notch first appears (0x3FD +40 +-1 ms)
#define GEAR_RELEASE_MS       20u   // v1h.2: P released 20 ms after the first notch of a push (real 0x39A E1 +20)
#define GEAR_CODE_LAG_MS      10u   // v1h.2: 0x39A/0x3F9/0xDC gear code 10 ms after 0x3FD (real +2 R->D .. +21 P->R)
#define GEAR_CODE_LAG_N_MS   100u   // N: keep 03/E1 for 100 ms (the real N transit showed 03/E1 for 60 ms; N code HYPOTHESIS)
#define GEAR_PBTN_39A_MS      21u   // v1h.2 P button: 0x39A gear 01 21 ms after the first pressed frame (real +21)
#define GEAR_PBTN_3F9_MS      42u   //   0x3F9 gear 1 (and 0xDC 22 FC) 21 ms after that (real +42)
#define GEAR_PBTN_39A_P_MS   881u   //   0x39A 03 19 ms BEFORE 0x3FD P (real)
#define GEAR_TO_P_MS         900u   //   0x3FD P (real +900 ms after the button frame)
#define GEAR_PLOCK_MS       3000u   // pawl confirmed 3 s after the P button at the latest ...
#define GEAR_PAWL_BRAKE_MS   340u   // ... or 340 ms after the brake was released (last pressed 0x173; real +340, untested)
#define GEAR_N_EV 5
typedef struct __attribute__((packed)) {
    uint16_t seq;           // event number (1, 2, ...)
    uint8_t kind;           // GEV_*
    uint8_t from_to;        // low nibble from gear, high nibble to gear (GEAR_*)
    uint8_t why;            // GWHY_* / GREF_*
    uint8_t flags;          // bit0 brake pressed, bit1 engine running, bits 4-7 lever move (GWHY_*)
    uint16_t t_ms;          // low 16 bits of the firmware ms clock
} gear_ev_t;

// USB 0xab (PICOFLEX_GET_GEAR) / UART. 64 bytes.
typedef struct __attribute__((packed)) {
    uint8_t version;        // 1
    uint8_t enabled;        // bit0 on (0 = fixed P), bit1 compiled in
    uint8_t gear;           // GEAR_* we are in / heading to
    uint8_t stage;          // GST_*
    uint8_t lever;          // last valid 0x197 B2 >> 4 (0xFF = none)
    uint8_t pbtn;           // 1 = P button pressed in the last valid 0x197
    uint8_t brake;          // 1 = brake pressed (0x173, last 250 ms)
    uint8_t running;        // 1 = fake engine running
    uint16_t ev_seq;        // events so far (ev[] holds the last GEAR_N_EV, oldest first)
    uint16_t changes;
    uint16_t refusals;
    uint16_t reserved;
    uint32_t lever_frames;  // valid 0x197 frames
    uint32_t lever_bad;     // 0x197 with a bad length / E2E CRC
    gear_ev_t ev[GEAR_N_EV];
} dde_gear_status_t;
_Static_assert(sizeof(dde_gear_status_t) == 64, "gear status size");

typedef struct {
    bool enabled;
    uint8_t target;         // GEAR_*
    uint8_t from;           // gear shown before the current transition
    uint8_t stage;
    uint32_t t_stage, t_act;
    // lever
    bool lv_have; uint32_t lv_us; uint8_t lv, pbtn;
    bool lv_lost;                   // lever was seen and then timed out (logged once)
    uint8_t gest_dir, gest_depth;   // current push: 0 none, 1 forward, 2 back; deepest notch acted on
    uint8_t pend_dir, pend_depth;   // notch waiting for its 40 ms (0 = none)
    uint32_t pend_us;
    uint8_t pbtn_cnt; bool pbtn_done;
    uint32_t push_us;               // v1h.2: first notch of the current push (P release +20 ms)
    bool push_rel;                  // this push already released P
    uint8_t rel_prev;               // stage to go back to when the push ends without a gear
    bool brake_have; uint32_t brake_us;
    bool running;
    uint8_t act_why;                // GWHY_* of the move being handled
    uint32_t now;
    // bookkeeping
    uint16_t ev_seq, changes, refusals;
    uint32_t lever_frames, lever_bad;
    gear_ev_t ev[GEAR_N_EV];
} gear_t;

void gear_reset(gear_t *g, bool enabled);                    // power-up / new FlexRay wake: P
void gear_set_enabled(gear_t *g, bool on, uint32_t now_us);  // runtime switch (logger --no-gear)
void gear_input_lever(gear_t *g, const uint8_t *d, uint8_t len, uint32_t now_us);   // PT-CAN 0x197
void gear_input_brake(gear_t *g, uint32_t now_us);           // a PT-CAN 0x173 with d7 & 3 == 3
void gear_step(gear_t *g, uint32_t now_us, bool running);    // once per FlexRay cycle
void gear_fill_out(const gear_t *g, eng_out_t *o);           // bytes for the frame builders
void gear_fill_status(const gear_t *g, dde_gear_status_t *st);
const char *gear_name(unsigned gear);
// "GEAR: P -> R (brake on) lever forward 2" / "GEAR: refused, brake not pressed (P -> R, lever forward 2)"
int gear_ev_text(const gear_ev_t *e, char *buf, unsigned n);
#endif
