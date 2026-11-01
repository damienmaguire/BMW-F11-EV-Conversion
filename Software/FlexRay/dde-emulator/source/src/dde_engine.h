// v1g: opt-in "fake engine start" for the DDE emulator (EV conversion, no engine).
//
// Plain English: the real DDE tells the car when the diesel cranks, fires,
// idles and stops. With no engine the car never sees "engine running", so it
// stays in a half-started state. This module plays a recorded start/run/stop
// story (Ian's dde_start_sequence.md + Fred's f11_fr_engine_running_diff.md,
// checked against the F11_4log_3krev capture) when the driver asks for a start:
//
//   OFF ---(start trigger)---> CRANK ---(rpm >= 950)---> RUN
//    ^                          |                         |
//    |                          +------(stop request)-----+--> STOP --(rpm 0)--> STOPPED
//    +--------------------------- (new FlexRay wake) ------------------------------+
//   STOPPED --(start trigger)--> CRANK (restart is allowed)
//
// Start triggers (both need the mode switched on from the PC, OFF by default).
// Real start (F11_engstartthenstop): ST_KL A -> C -> D (starter) -> 0xE6 82 -> C (+8 ms) -> A.
//   ENG_MODE_CAS:      the car's own start request, ST_KL = 0xD (CAS "starter
//                      request") in FlexRay 0x74 or PT-CAN 0x12F, with the brake
//                      pressed. This is the normal button start.
//   ENG_MODE_FALLBACK: ST_KL = 0xA (ignition on; D also accepted), brake pressed in the last
//                      300 ms, and a "start" command typed into the logger in
//                      the last 2 s. Use it if the CAS never sends ST_KL = D
//                      (for example because it waits for something we do not send).
// Stop: ST_KL drops below 0xA (9 = engine-off request, <= 8 = ignition going
// off), ST_KL not seen for 600 ms, a "stop" command, or the mode being switched off.
//
// This file is pure C (no SDK): the firmware runs eng_step() on core 1 once per
// FlexRay cycle (5 ms) and the host tests run it with simulated time. It only
// decides values; the frame builders call the eng_patch_*() helpers which
// overwrite bytes BEFORE the counters and CRCs are computed, so every E2E CRC,
// alive counter and FlexRay frame CRC stays valid. Nothing here ever creates
// a new frame ID: only bytes of frames the emulator already sends change.
#ifndef DDE_ENGINE_H
#define DDE_ENGINE_H
#include <stdbool.h>
#include <stdint.h>

enum { ENG_MODE_OFF = 0, ENG_MODE_CAS = 1, ENG_MODE_FALLBACK = 2 };
enum { ENG_OFF = 0, ENG_CRANK = 1, ENG_RUN = 2, ENG_STOP = 3, ENG_STOPPED = 4 };
enum { ENG_CMD_NONE = 0, ENG_CMD_START = 1, ENG_CMD_STOP = 2 };
enum { ENG_SRC_FR74 = 0, ENG_SRC_CAN12F = 1 };

// v1h: start length ("profile"). Chosen by the PC (logger --start-profile, USB 0xa7 wIndex)
// and used from the next start on. wIndex 0 (older loggers) = the build default below.
//   SHORT     (v1h default): crank marker + first rpm in the first FlexRay cycle after
//             ST_KL D, quick run-up, "running" (0xE6/0x3F9 = 82) together with
//             start-ready 0 at about D+170 ms. Shorter than ANY real start we have
//             seen (fastest real: 82 at D+851 ms) - the CAS may refuse it; the logger
//             and the console then say so loudly ("CAS ended the start BEFORE ...").
//   REALISTIC first fire at +600 ms: 82 at about D+840 ms, just inside the fastest real
//             start (obd1: D+851 ms). Crank marker +45 ms, first rpm ~+90 ms (real
//             +51..59 / +94 ms).
//   RUN8      exactly the v1g.2 behaviour that passed run 8 (82 at D+941 ms), including
//             its known small differences (0xE6 b4 66 for 250 ms, 0xE8 1/s while cranking,
//             0x3FB d4 never f0, 0x92 19 ms after the stop). For A/B comparison.
enum { ENG_PROF_DEFAULT = 0, ENG_PROF_SHORT = 1, ENG_PROF_REALISTIC = 2, ENG_PROF_RUN8 = 3, ENG_PROF_N };
#ifndef ENG_PROF_BUILD_DEFAULT
#define ENG_PROF_BUILD_DEFAULT ENG_PROF_SHORT
#endif
typedef struct {
    uint16_t blip_on, blip_off;   // CounterHi 3 window (ms after the start trigger)
    uint16_t rpm, t81;            // first rpm, 0xE6 81 (cranking)
    uint16_t fire, runup;         // first fire, run-up 200 -> 1000 rpm duration
    uint16_t ready_off;           // start-ready 1 -> 0 this long after 82 (0 = in the same frame)
    uint16_t inj;                 // 0x3FB d4 bit0 1 -> 0 ("injecting") at this time, 0xFFFF = never
    uint16_t t92;                 // 0xE6 82 -> 92 this long after the stop trigger
    uint8_t b4_26;                // 1 = 0xE6 b4 = 26 from the first 82 frame (real DDE); 0 = 66 for 250 ms
    uint8_t e8_crank;             // 1 = 0xE8 L9 ~5.7/s from the start until the fast rate (real: 4-7/s)
    uint8_t need_d;               // 1 = a start triggered by ST_KL D is abandoned if D ends before 82
} eng_prof_t;
extern const eng_prof_t eng_profiles[ENG_PROF_N];
const char *eng_prof_name(unsigned prof);

// Timeline (ms). T0 = start trigger, TS = stop trigger. Sources in the comments.
#define ENG_T_BLIP_ON      50u    // Ian: CounterHi 1 -> 3 at +50 ms (crank marker)
#define ENG_T_BLIP_OFF    155u    //      back to 1 at +155 ms
#define ENG_T_RPM          95u    // Ian/Fred: first rpm at +95 ms (~115 rpm)
#define ENG_T_81          100u    // 0xE6 state 0x80 -> 0x81 (cranking) at +100 ms
#define ENG_T_FIRE        700u    // first fire, run-up to 1000 rpm (3krev: 82 at +1.00 s)
#define ENG_T_RUNUP       250u    // run-up duration 200 -> 1000 rpm
#define ENG_T_FLARE_END  1500u    // start flare 1000-1060 rpm until +1.5 s
#define ENG_IDLE_RPM      780u    // owner: idle ~780 rpm (OBD 0x0C 30 0C)
#define ENG_TAU_IDLE_MS  1000u    // flare -> idle settle time constant
#define ENG_RPM_82        950u    // 0xE6 = 0x82 (running) once rpm >= 950
#define ENG_STKL_STALE_MS 600u    // no ST_KL for this long while cranking/running = stop
#define ENG_BRAKE_MS_CAS 2000u    // CAS mode: brake pressed within this window of ST_KL = D
#define ENG_BRAKE_MS_FB   300u    // fallback mode: brake pressed within 300 ms
#define ENG_CMD_MS       2000u    // fallback start command valid for 2 s
#define ENG_MODE_KEEP_MS 3000u    // mode must be refreshed by the PC (logger does it every 1 s)
#define ENG_STOP_9_MS    1000u    // ST_KL 9 held this long = full stop even without <= 8

// Everything the frame builders need, one consistent snapshot. 0 / false = no
// override (use the emulator's normal content).
typedef struct {
    uint32_t seq;          // bumped by every eng_step (core 0 reads a snapshot)
    uint8_t active;        // 1 = engine story in progress or happened in this wake
    uint8_t phase;         // ENG_*
    uint8_t mode;          // ENG_MODE_*
    uint8_t cnt_hi;        // 0 = normal, 3 = crank CounterHi marker
    uint16_t rpm4;         // engine speed * 4 (0x28 c1 b6..7, PT-CAN 0xA5 d5..6)
    uint16_t torque;       // 12-bit raw (Nm*2 + 2047), 0 = no override
    uint16_t turbine;      // 0x26 c1 b3..4 raw, 0 = no override
    uint8_t e6_b3, e6_b4, e6_ready;   // 0xE6 / 0x3F9: state, phase byte, start-ready (b7 hi nibble); 0 = none (ready: 0xFF = none)
    uint8_t c3_b56;        // 0x28 c3 b5/b6 code: 0 none, 1 = 24 20, 2 = 20 20, 3 = E0 E0
    uint8_t c3_b34;        // 0x28 c3 b3..4: 0 none, 1 = 3E 7D (running), 2 = 1C 7E (after stop)
    uint8_t r29_b5;        // 0x29 c3 b5 / PT-CAN 0x8F d4 (0 = none)
    uint8_t r29_b8;        // 0x29 c3 b8 (0 = none)
    uint8_t f97c;          // 1 = 0x29/0x3D/0x8F "F9 7C" (valid sensor values, sticky)
    uint8_t b142_b8, b142_b23;   // 0x142 bytes (0 = none)
    uint8_t fb9_set, fb9_b2, fb9_b3, fb9_b4;   // 0xFB L9 b2..b4 (+ PT-CAN 0x3B3 d1..d3)
    uint16_t fb7_mv;       // 0xFB L7 b3..4 voltage in mV (0 = none)
    uint8_t e89;           // 0xE8 L9: 0 template, 1 running content, 2 after-stop content
    uint8_t e89_b1;        // running content b1 (ramp 0x12 -> 0x25)
    uint8_t e8_fast;       // 1 = send 0xE8 L9 ~15/s instead of 1/s
    uint16_t f3_val;       // PT-CAN 0xF3 rpm/10 (lagged)
    uint16_t v281;         // PT-CAN 0x281 12-bit voltage (0 = none)
    uint8_t r397_set, r397_d0, r397_d1, r397_d2;
    uint8_t r381_run;      // PT-CAN 0x381 79 f0
    uint16_t fuel;         // PT-CAN 0x2C4 b0..1 fuel counter
    uint8_t e6_ev, fb9_ev; // bumped when 0xE6 / 0xFB L9 content changes (send as event)
    uint8_t abs_stop_walk; // 1 = 0x397 stop walk finished (normal model again)
    uint8_t inj;           // v1h: 1 = "injecting": PT-CAN 0x3FB d4 bit0 = 0 (gateway copies it to FR 0xE9 b5)
    uint8_t e8_crank;      // v1h: 1 = 0xE8 L9 ~5.7/s (crank rate, before e8_fast)
    // v1h.1 gear (dde_gear.c fills these). g_set 0 = fixed P: the v1h content, nothing patched.
    // Patched even when the engine story is not active (the gear can only leave P while running).
    uint8_t g_set;         // 1 = put the gear bytes below in
    uint8_t g_3fd;         // PT-CAN 0x3FD d2 gear display (20 P, 40 R, 60 N, 80 D)
    uint8_t g_39a_d3, g_39a_d4;   // PT-CAN 0x39A d3 gear (3 P, 2 R, 5 D, 1 P coming), d4 P lock (e2 / e1 / e0)
    uint8_t g_park;        // 0x3F9 d3 / 0xE6 b4 bits 4-5 (2 P lock, 1 released, 0 in gear)
    uint8_t g_nib;         // 0x3F9 d6 / 0xE6 b7 low nibble (3 P, 2 R, 5 D, 1 P coming)
    uint8_t g_dc_b2, g_dc_b5;     // PT-CAN 0xDC d2/d5 = FR 0x28 c3 b11/b14 (22/fe P, 25/f4 R, 24/f4 D)
    uint8_t g_fr_park;     // v1h.2: FR 0xE6 b4 bits 4-5. Stays 2 while P is being released (real P -> R:
                           // E6 b4 26 -> 06 directly while PT-CAN 0x3F9 showed 1 = "16")
} eng_out_t;

typedef struct {
    // inputs
    uint8_t mode;
    uint32_t mode_us; bool mode_seen;
    uint8_t stkl; uint32_t stkl_us; bool stkl_have;
    uint32_t brake_us; bool brake_have;
    uint8_t cmd; uint32_t cmd_us;
    // state
    uint8_t phase;
    bool has_run;              // reached first rpm in this wake
    bool cur_run;              // reached first rpm in the current start attempt
    uint32_t t0, trpm, t82, ts, t9, t80;
    bool t9_have, b142_stop;
    uint32_t rpm_at_stop;      // rpm*4 when the stop started
    uint32_t last_us; bool last_have;
    uint32_t f3_q8, fuel_q10;
    uint8_t last_e6[3], last_fb9[3];
    uint8_t mode_prev;
    // v1h start profile and CAS reaction bookkeeping (times in us, *_have = valid)
    uint8_t prof_req;          // profile asked for by the PC (ENG_PROF_*; 0 = build default)
    uint8_t prof;              // profile of the current / last start
    const eng_prof_t *pp;      // = &eng_profiles[prof]
    bool start_d;              // current start was triggered by ST_KL D
    bool d_end_have;           // ST_KL left D during this start
    uint32_t d_end_us;
    uint8_t d_end_to;          // ST_KL it went to
    bool t82_have;
    uint32_t aborts;           // starts abandoned because D ended before our 82 (CAS refused?)
    uint8_t cas_flags;         // ENG_CAS_* of the last start
    // stats
    uint32_t starts, stops, rejects, last_reject;  // last_reject: ENG_REJ_*
    eng_out_t out;
} eng_t;

// cas_flags (status / logger): what the CAS did with our last start
enum { ENG_CAS_D_ENDED_EARLY = 1,   // ST_KL left D before we reported running (82): CAS refused / gave up
       ENG_CAS_D_ENDED = 2,         // ST_KL left D after 82 (normal: real cars 8-39 ms after 82)
       ENG_CAS_D_LONG = 4,          // ST_KL still D 300 ms after our 82 (CAS ignores our "running"?)
       ENG_CAS_ABANDONED = 8 };     // we abandoned the start (need_d profile) because of the early end
enum { ENG_REJ_NONE = 0, ENG_REJ_NO_BRAKE, ENG_REJ_STKL, ENG_REJ_NO_CMD, ENG_REJ_BUSY, ENG_REJ_MODE };

void eng_reset(eng_t *e);                 // power-up / new FlexRay wake: everything off, mode kept
void eng_set_mode(eng_t *e, uint8_t mode, uint32_t now_us);   // PC keepalive (every <= 3 s)
void eng_set_profile(eng_t *e, uint8_t prof);                  // v1h: ENG_PROF_*, used from the next start
void eng_command(eng_t *e, uint8_t cmd, uint32_t now_us);     // ENG_CMD_START / STOP from the PC
void eng_input_stkl(eng_t *e, uint8_t stkl, bool brake, uint32_t now_us);  // 0x74 / 0x12F
void eng_input_brake(eng_t *e, bool brake, uint32_t now_us);               // 0x173
void eng_step(eng_t *e, uint32_t now_us); // once per FlexRay cycle; updates e->out

// Byte patchers. Called with the payload BEFORE counters/CRCs are written.
// FlexRay static frame: target 0..3 = 0x26, 0x28, 0x29, 0x3D; pl = 16 payload bytes.
void eng_patch_fr_static(const eng_out_t *o, unsigned target, unsigned cycle, uint8_t *pl);
// FlexRay dynamic PDU (DDE_PDU_* index), pl = payload with counter byte already set
// (the E2E CRC byte is computed afterwards).
void eng_patch_fr_dyn(const eng_out_t *o, unsigned pdu, uint8_t *pl);
// PT-CAN frame (d = 8 data bytes, before counter/CRC). *ctr_hi may be changed.
// Returns true when it changed anything.
bool eng_patch_can(const eng_out_t *o, uint16_t id, uint8_t *d, uint8_t *ctr_hi);

// v1h.2: core 0's view of the core 1 snapshot. A failed / torn read reuses the last good
// one (never the stock template once emulation has produced data). Returns the snapshot
// to patch PT-CAN with, or NULL (nothing published yet, older than 500 ms = FlexRay not
// stepping, or the engine story is not active).
typedef struct { eng_out_t last; uint32_t last_us; bool have; uint32_t fails; } eng_snap_cache_t;
const eng_out_t *eng_snap_select(eng_snap_cache_t *c, bool read_ok, const eng_out_t *fresh, uint32_t fresh_us,
                                 uint32_t now_us);
const char *eng_phase_name(unsigned phase);
#endif
