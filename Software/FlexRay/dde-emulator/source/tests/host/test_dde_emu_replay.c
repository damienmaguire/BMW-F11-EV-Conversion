// Replay test for src/dde_emu.c (the real firmware file, built against the stand-ins
// in tests/host/emu_stubs). Car test 1 (f11_emu1.csv): the emulator sent 0x26/0x28/0x29
// in cycles 52 and 53, never 0x3D, then latched FAULT 1 "TX state machine stuck" and
// stayed off the bus for the rest of the run.
//
// The car's FR1 frames are replayed with their captured 10 ns timing. The sim models
// the parts dde_emu.c talks to: TIMER0 + alarm IRQ, the 100 MHz PWM fine counter,
// the PIO TX SM (busy from the first FIFO word to the end of the frame) and fire SM
// (TXEN falls r + K ticks after the counter read), the transceiver loop (echo mark
// LOOP ticks after TXEN) and the streamer's header hook (HDR_LAT after the mark).
//
//   test_dde_emu_replay VECTORS [scenario]
//   scenarios: run1 (default)  capture only; v1b must send all 4 slots every cycle,
//                              no fault, no late, every echo on its slot
//              run1-full       run1 checks on the whole capture (60.8 s; f11_emu1.csv extract,
//                              not committed): no fault, no late, every frame on its slot
//              run1-v1         same input against the v1 source: must reproduce
//                              FAULT 1 at the first 0x3D load (the car-test-1 failure)
//              run1-v1-spike   v1, and the alarm before cycle 52's 0x3D enters 4 us late
//                              (0x3D skipped, late=1): reproduces the car-test-1 console
//                              counters exactly (sent 2/2/2/1, late 1, echo ok 6, FAULT 1)
//              run1-spike      v1b with the same spike: 0x3D still sent (precheck 1.2 us)
//              run1-bigspike   v1b, 8 us spike: late 1 recorded as "short" for 0x3D, no fault
//              foreign         a 0x3D from "another DDE" appears 3 times, 3 s apart:
//                              FAULT 2 each time, auto re-arm twice, latched at the 3rd
//              stuck           the TX SM hangs once: FAULT 1, pins back to SIO,
//                              auto re-arm after 2 s quiet, TX resumes
//              noecho          echoes lost for 1 cycle: no fault; lost for 3 cycles: FAULT 3
//              lateecho        our echo headers reach the hook 400 us late: never foreign
//   v1c dynamic frames (binary built with the dynamic frames on, see run.sh):
//              dyn-run1, dyn-run2  whole car test 1 / 2 FR1 capture (extract_replay.py, all
//                              columns): statics as before, 0xE6/0xFB/0xE8 at the minislot the
//                              car frames before them give, never before or over a lower-ID
//                              frame, periods/cycles/counters/bytes/pre-ign as specified.
//                              Car frames with a higher ID move back by our frame's minislots.
//              dyn-run5 DDE    run 5 (FR1 + the real DDE's other FR2 frames), each PDU sent in
//                              the cycles the real DDE sent it: our placement vs the real DDE's
//              dyn-unsure      dyn-run2 plus: an unexplained frame start just before 0xE6
//                              (no header), an unknown-length frame before it, and headers
//                              without their mark: 0xE6 must be skipped (counted), not sent
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "dde_emu.h"
#include "dde_emu_tables.h"
#include "flexray_ts.h"
#include "flexray_pins.h"
// v1d: core 0 hands decoded frames (0xF1 KL15) to the emulator. Weak so that older
// sources without it (the v1 before-the-fix build) still link; then never called.
extern void dde_emu_on_frame(unsigned id, unsigned cycle, const uint8_t *payload, unsigned nbytes) __attribute__((weak));

#define K_TRUE 34       // ticks: fire-delay counter read -> TXEN mark (car test 1: K=34)
#define LOOP_TRUE 34    // ticks: TXEN mark -> FR1 RX echo mark (car test 1: loop=338 ns)
#define HDR_LAT 700     // ticks: mark -> header hook (5 header bytes + ISR entry)
#define ALARM_LAT 60    // ticks: alarm match -> ISR body
#define US_OFF 181679000u
#define FINE_OFF 32668u
#define RING_MASK 0x3FFFu

// ------------------------------------------------------------------ clock / events
static uint64_t T;   // 10 ns ticks
uint32_t time_us_32(void) { return (uint32_t)(T / 100u) + US_OFF; }
static uint64_t us_to_tick(uint32_t us) { return (uint64_t)(uint32_t)(us - US_OFF) * 100u; }

enum { EV_CAR, EV_HDR, EV_ALARM_MATCH, EV_ALARM_FORCE, EV_TXEN, EV_ECHO, EV_TXEND, EV_HOST, EV_CORE0 };
typedef struct { uint64_t t; uint32_t seq; int type; uint32_t gen; uint32_t pos; uint8_t h[5]; uint8_t ours; uint8_t flags; uint32_t mpos; uint16_t b3p1; } ev_t;   // b3p1: 0xF1 payload[2] + 1 (0 = none)
#define EVF_NOMARK 1u     // header whose mark is not the latest one (capture row without a hw timestamp)
#define EVF_MARKONLY 2u   // frame start without a decoded header
#define EVF_MARKED 4u     // mark already delivered by sim_spin()
static ev_t *heap; static size_t nheap, capheap; static uint32_t evseq;
static void push(ev_t e)
{
    if (nheap == capheap) { capheap = capheap ? capheap * 2 : 4096; heap = realloc(heap, capheap * sizeof(ev_t)); }
    e.seq = evseq++;
    size_t i = nheap++;
    heap[i] = e;
    while (i) {
        size_t p = (i - 1) / 2;
        if (heap[p].t < heap[i].t || (heap[p].t == heap[i].t && heap[p].seq < heap[i].seq)) break;
        ev_t x = heap[p]; heap[p] = heap[i]; heap[i] = x; i = p;
    }
}
static ev_t pop(void)
{
    ev_t top = heap[0];
    heap[0] = heap[--nheap];
    size_t i = 0;
    for (;;) {
        size_t l = 2 * i + 1, r = l + 1, m = i;
        if (l < nheap && (heap[l].t < heap[m].t || (heap[l].t == heap[m].t && heap[l].seq < heap[m].seq))) m = l;
        if (r < nheap && (heap[r].t < heap[m].t || (heap[r].t == heap[m].t && heap[r].seq < heap[m].seq))) m = r;
        if (m == i) break;
        ev_t x = heap[m]; heap[m] = heap[i]; heap[i] = x; i = m;
    }
    return top;
}

// ------------------------------------------------------------------ hardware stand-ins
sim_timer_hw_t sim_timer;
sim_pwm_hw_t sim_pwm;
sim_pio_t sim_pio0;
void (*sim_alarm_handler)(void);
static uint32_t alarm_val_seen; static bool alarm_armed; static uint32_t alarm_gen;
static int pin_fn[32];

void sim_gpio_put(uint pin, bool v) { (void)pin; (void)v; }
void sim_gpio_set_function(uint pin, int fn) { pin_fn[pin] = fn; }

static uint nprog;
uint sim_pio_add_program(PIO pio, const pio_program_t *p) { (void)pio; uint o = nprog; nprog += p->length; return o; }

// TX SM 0, fire SM 1 (claim order in dde_emu_init_core1)
static struct {
    uint32_t words[16]; unsigned nw;
    uint32_t frame[16];      // words of the frame being fired / sent
    unsigned fnw;            // words of frame[] delivered so far (v1d: preload <= 8, rest topped up after the fire)
    bool loaded, sending, fire_busy, enabled[2];
    uint32_t gen;
    uint64_t fire_t;
    bool hang_next;          // scenario stuck: the next frame never finishes
    bool hung;
    bool parked;             // after a frame with nbytes % 4 != 0: waits at idle+1 (pins high)
} pio;
static unsigned n_truncated, n_fire_without_frame, n_put_parked, n_parked, n_fifo_over, n_underrun, n_topup;

void sim_pio_sm_reset(PIO p, uint sm)
{
    (void)p;
    if (sm == 0) {
        if (pio.sending) n_truncated++;
        pio.nw = 0; pio.loaded = false; pio.sending = false; pio.hung = false; pio.parked = false;
    } else {
        pio.fire_busy = false;
    }
    pio.gen++;   // cancels any scheduled TXEN/echo/end of the frame in flight
}
void sim_pio_sm_set_enabled(PIO p, uint sm, bool en)
{
    (void)p;
    if (!en && sm == 0 && (pio.sending || pio.loaded)) sim_pio_sm_reset(p, 0);
    if (!en && sm == 1 && pio.fire_busy) sim_pio_sm_reset(p, 1);
    pio.enabled[sm] = en;
}
uint sim_pio_sm_get_pc(PIO p, uint sm)
{
    (void)p;
    if (sm == 0) return (pio.loaded || pio.sending || pio.hung) ? 3u : pio.parked ? 1u : 0u;
    return pio.fire_busy ? 22u : 21u;   // fire program loaded after the 21-instruction TX program
}
bool sim_pio_sm_tx_fifo_empty(PIO p, uint sm) { (void)p; (void)sm; return true; }  // SM pulls at once
// TX FIFO joined = 8 words; before the fire the SM blocks on its first pull of the frame
// after the header words, so at most 8 words can wait; once fired the SM drains it.
bool sim_pio_sm_tx_fifo_full(PIO p, uint sm) { (void)p; return sm == 0 && !pio.fire_busy && !pio.sending && pio.nw >= 8; }
static unsigned frame_words_of(const uint32_t *w) { return 2u + (((w[0] & 0xFFFFu) + 1u) + 3u) / 4u; }

static unsigned frame_bits_of(const uint32_t *w)
{
    unsigned nbytes = (w[0] & 0xFFFFu) + 1u, tss = (w[0] >> 16) + 2u;
    return tss + 1u + 10u * nbytes + 2u + (w[1] ? w[1] + 3u : 0u);
}
#define frame_bits() frame_bits_of(pio.frame)
void sim_pio_sm_put(PIO p, uint sm, uint32_t v)
{
    (void)p;
    if (sm == 0) {
        if (pio.parked) n_put_parked++;   // would be shifted out as the pad/next frame: a bug
        if (pio.fire_busy && pio.fnw < frame_words_of(pio.frame)) {   // v1d top-up of the fired frame
            pio.frame[pio.fnw++] = v; n_topup++;
            return;
        }
        if (pio.nw >= 8) n_fifo_over++;   // a 9th word before the fire: the real FIFO is full (put blocks)
        if (pio.nw < 16) pio.words[pio.nw++] = v;
        pio.loaded = true;
        return;
    }
    // fire SM: TXEN falls r + K ticks after the counter read (= now in the sim)
    if (!pio.loaded || pio.nw < 3) { n_fire_without_frame++; return; }
    pio.fire_busy = true;
    memcpy(pio.frame, pio.words, sizeof pio.words);
    pio.fnw = pio.nw;
    pio.nw = 0;   // the SM has pulled what was preloaded
    uint64_t tf = T + v + K_TRUE;
    pio.fire_t = tf;
    ev_t e = {0};
    e.gen = pio.gen;
    e.t = tf; e.type = EV_TXEN; push(e);
    e.t = tf + frame_bits() * 10u; e.type = EV_TXEND; push(e);
}

// flexray_ts: latest mark per side
static flexray_ts_t ts_last[2]; static uint32_t ts_pos[2], ts_seq[2];
static void ts_mark_at(int side, uint32_t pos, uint64_t t)
{
    ts_last[side].us = (uint32_t)(t / 100u) + US_OFF;
    ts_last[side].fine = (uint16_t)(t + FINE_OFF);
    ts_last[side].valid = true;
    ts_pos[side] = pos;
    ts_seq[side]++;
}
bool flexray_ts_peek_latest(int side, flexray_ts_t *out, uint32_t *ring_pos, uint32_t *seq)
{
    *out = ts_last[side]; *ring_pos = ts_pos[side]; *seq = ts_seq[side];
    return ts_last[side].valid;
}

static void after_callback(void)
{
    if (sim_timer.intf & 1u) {
        ev_t e = {0}; e.t = T; e.type = EV_ALARM_FORCE; push(e);
    }
    if (sim_timer.alarm[0] != alarm_val_seen) {   // a write with a new value arms the alarm
        alarm_val_seen = sim_timer.alarm[0];
        alarm_gen++;
        uint64_t at = us_to_tick(alarm_val_seen);
        alarm_armed = at > T;                      // a time already passed never matches
        if (alarm_armed) { ev_t e = {0}; e.t = at + ALARM_LAT; e.type = EV_ALARM_MATCH; e.gen = alarm_gen; push(e); }
    }
}
static void set_now(uint64_t t) { T = t; sim_pwm.slice[FLEXRAY_TS_PWM_SLICE].ctr = (uint16_t)(T + FINE_OFF); }

// ------------------------------------------------------------------ bookkeeping
typedef struct { uint64_t t; unsigned id, cycle, len; uint8_t hw; int16_t b3; } car_t;   // b3: 0xF1 payload[2], -1 = none
static car_t *car; static size_t ncar;
static size_t cfeed, nfeed;   // car[cfeed..nfeed) not delivered yet (fed in time order by run_until)
static uint32_t pos_ctr;
static bool use_nomark;       // dynamic scenarios: rows without a hw timestamp have no matching mark
static bool shift_on;         // dynamic scenarios: our dynamic frame pushes higher-ID car frames back

#define MAXTX 400000
static struct { uint64_t txen, echo, end; int64_t r07; unsigned id, cycle, plen; uint8_t pl[32]; } txl[MAXTX];
static unsigned ntx;
static int64_t ref07_tick[64];   // per cycle number: latest reference mark, moved to 0x07's offset (198 us)
static unsigned drop_echo_cycles_from = 999, drop_echo_cycles_n;  // scenario noecho
static unsigned drop_echo_seen_cycles;
static uint64_t echo_hdr_lat = HDR_LAT;
static unsigned last_car_cycle = 99;
static unsigned spike_us; static bool spiked;

static void car_frame(const ev_t *ep)
{
    ev_t e = *ep;
    uint32_t pos;
    if (e.flags & EVF_MARKED) pos = e.mpos;
    else { pos = (pos_ctr += 64) & RING_MASK; ts_mark_at(0, pos, e.t); }
    if (e.flags & EVF_MARKONLY) return;
    if (e.b3p1) {   // core 0 gets the decoded frame (~frame length + 200 us of USB/ring latency)
        ev_t c = e; c.type = EV_CORE0; c.t = e.t + 21200u; c.flags = 0; push(c);
    }
    {
        ev_t h = e; h.type = EV_HDR; h.t = e.t + HDR_LAT; h.flags = 0;
        h.pos = (e.flags & EVF_NOMARK) ? ((pos + 1u) & RING_MASK) : pos;
        push(h);
        unsigned id = ((unsigned)(e.h[0] & 7u) << 8) | e.h[1], cyc = e.h[4] & 63u;
        int ri = dde_core_ref_index(id);   // cycle start estimate from any reference of this cycle
        if (ri >= 0) ref07_tick[cyc] = (int64_t)e.t - (int64_t)(dde_emu_ref_off_q8[ri] - dde_emu_ref_off_q8[0]) / 256;
        if (cyc != last_car_cycle) {
            last_car_cycle = cyc;
            if (drop_echo_cycles_from != 999 && drop_echo_seen_cycles <= drop_echo_cycles_from + drop_echo_cycles_n) drop_echo_seen_cycles++;
        }
    }
}

static ev_t car_ev(size_t i)
{
    ev_t e = {0}; e.t = car[i].t; e.type = EV_CAR;
    dde_core_header(0, car[i].id, car[i].len ? car[i].len / 2u : 8u, car[i].cycle, e.h);
    if (use_nomark && !car[i].hw) e.flags = EVF_NOMARK;
    if (car[i].b3 >= 0) e.b3p1 = (uint16_t)(car[i].b3 + 1);
    return e;
}

static void shift_after_ours(uint64_t txen, unsigned id, unsigned cyc, unsigned plen);

// Busy-wait inside the firmware's ISR (DDE_EMU_SPIN_HOOK): time moves on by 50 ns and
// every frame start (car frame / our echo) of that time reaches the mark ring, in order.
static unsigned n_spin;
void sim_spin(void)
{
    n_spin++;
    set_now(T + 5u);
    for (;;) {
        size_t hi = (size_t)-1;
        for (size_t i = 0; i < nheap; i++)
            if ((heap[i].type == EV_CAR || heap[i].type == EV_ECHO) && !(heap[i].flags & EVF_MARKED) && heap[i].t <= T &&
                (hi == (size_t)-1 || heap[i].t < heap[hi].t)) hi = i;
        bool cv = cfeed < nfeed && car[cfeed].t <= T;
        if (cv && (hi == (size_t)-1 || car[cfeed].t <= heap[hi].t)) {
            ev_t e = car_ev(cfeed++);
            car_frame(&e);   // mark now, header HDR_LAT later
        } else if (hi != (size_t)-1) {
            uint32_t pos = (pos_ctr += 64) & RING_MASK;
            ts_mark_at(0, pos, heap[hi].t);
            heap[hi].flags |= EVF_MARKED;
            heap[hi].mpos = pos;
        } else break;
    }
}

static void handle(ev_t e)
{
    if (e.t > T) set_now(e.t);
    switch (e.type) {
    case EV_CAR:
        car_frame(&e);
        break;
    case EV_HDR:
        dde_emu_on_fr1_header(e.h, e.pos, RING_MASK);
        break;
    case EV_ALARM_MATCH:
        if (e.gen != alarm_gen || !alarm_armed) return;
        if (spike_us && !spiked && last_car_cycle == 52 && (int64_t)e.t - ref07_tick[52] > 175000 &&
            (int64_t)e.t - ref07_tick[52] < 400000) {
            // the alarm before cycle 52's 0x3D enters spike_us late (ISR latency spike)
            spiked = true;
            ev_t d = e; d.t += (uint64_t)spike_us * 100u; push(d);
            return;
        }
        alarm_armed = false;
        sim_alarm_handler();
        break;
    case EV_ALARM_FORCE:
        if (!(sim_timer.intf & 1u)) return;
        sim_alarm_handler();
        break;
    case EV_TXEN: {
        if (e.gen != pio.gen) return;
        pio.fire_busy = false;
        if (pin_fn[TXEN_FR_1_PIN] != GPIO_FUNC_PIO0) return;   // pins taken away: nothing on the bus
        pio.loaded = false; pio.sending = true;
        if (pio.fnw < frame_words_of(pio.frame)) n_underrun++;   // the SM would run out of words mid-frame
        ts_mark_at(1, 0, e.t);
        const uint32_t *w = pio.frame;
        uint8_t hb[5] = {(uint8_t)(w[2] >> 24), (uint8_t)(w[2] >> 16), (uint8_t)(w[2] >> 8), (uint8_t)w[2],
                         (uint8_t)(w[3] >> 24)};
        unsigned id = ((unsigned)(hb[0] & 7u) << 8) | hb[1], cyc = hb[4] & 63u;
        unsigned plen = (hb[2] >> 1 & 0x7Fu) * 2u;
        if (ntx < MAXTX) {
            txl[ntx].txen = e.t; txl[ntx].end = e.t + frame_bits() * 10u; txl[ntx].id = id; txl[ntx].cycle = cyc;
            txl[ntx].echo = 0; txl[ntx].r07 = ref07_tick[cyc]; txl[ntx].plen = plen;
            for (unsigned b = 0; b < 32 && b < plen; b++) txl[ntx].pl[b] = (uint8_t)(w[2 + (5 + b) / 4] >> (24 - 8 * ((5 + b) % 4)));
            ntx++;
        }
        if (shift_on && id >= 0x5Cu) shift_after_ours(e.t, id, cyc, plen);
        bool drop = drop_echo_cycles_from != 999 && drop_echo_seen_cycles > drop_echo_cycles_from &&
                    drop_echo_seen_cycles <= drop_echo_cycles_from + drop_echo_cycles_n;
        if (!drop) {
            ev_t x = {0}; x.t = e.t + LOOP_TRUE; x.type = EV_ECHO; x.gen = pio.gen; x.pos = ntx; memcpy(x.h, hb, 5); push(x);
        }
        if (pio.hang_next) { pio.hang_next = false; pio.hung = true; }
        break; }
    case EV_ECHO: {
        uint32_t pos;
        if (e.flags & EVF_MARKED) pos = e.mpos;
        else { pos = (pos_ctr += 64) & RING_MASK; ts_mark_at(0, pos, e.t); }
        if (e.pos >= 1 && e.pos <= MAXTX) txl[e.pos - 1].echo = e.t;
        ev_t h = e; h.type = EV_HDR; h.t = e.t + echo_hdr_lat; h.pos = pos; h.ours = 1; push(h);
        break; }
    case EV_TXEND:
        if (e.gen != pio.gen) return;
        pio.sending = false;   // a hung SM keeps pio.hung (not idle) until reset
        // the last "out x,16" takes the pad half-word, "out y,16" then waits for the FIFO
        if (((pio.frame[0] & 0xFFFFu) + 1u) & 3u) { pio.parked = true; n_parked++; }
        break;
    case EV_CORE0:
        if (dde_emu_on_frame) {   // v1d: core 0 hands 0xF1 (KL15) to the emulator
            uint8_t pl[4] = {0, 0, (uint8_t)(e.b3p1 - 1u), 0xFF};
            dde_emu_on_frame(0xF1, e.h[4] & 63u, pl, 4);
        }
        break;
    case EV_HOST:
        dde_emu_host_set(1);
        { ev_t n = e; n.t = e.t + 100000000ull; push(n); }   // keepalive 1 s
        break;
    }
    after_callback();
}

static void load(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); exit(2); }
    char line[256];
    size_t cap = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        unsigned long long t; unsigned id, cyc, len, fl, hw = 1; int b3 = -1;
        if (sscanf(line, "%llu %u %u %u %u %u %d", &t, &id, &cyc, &len, &fl, &hw, &b3) < 5) continue;
        if (ncar == cap) { cap = cap ? cap * 2 : 8192; car = realloc(car, cap * sizeof(car_t)); }
        car[ncar].t = t + 100000ull;   // 1 ms margin before the first frame
        car[ncar].id = id; car[ncar].cycle = cyc; car[ncar].len = len; car[ncar].hw = (uint8_t)(hw != 0); car[ncar].b3 = (int16_t)(id == 0xF1 ? b3 : -1); ncar++;
    }
    fclose(f);
}

static void push_car_ex(uint64_t t, unsigned id, unsigned cyc, unsigned len, uint8_t flags)
{
    ev_t e = {0}; e.t = t; e.type = EV_CAR; e.flags = flags;
    dde_core_header(0, id, len / 2u, cyc, e.h);
    push(e);
}
static inline void push_car(uint64_t t, unsigned id, unsigned cyc) { push_car_ex(t, id, cyc, 16, 0); }

// Extend the capture: cut it at its last cycle-0 0x07 and repeat the 64 cycles before
// that (one full cycle-counter period) n times. Returns the end tick.
static uint64_t t_capture_end;
static size_t extend(unsigned n, uint64_t *tend)
{
    int64_t a = -1, b = -1;
    for (size_t i = ncar; i-- > 0;) {
        if (car[i].id == 0x07 && car[i].cycle == 0) { if (b < 0) b = (int64_t)i; else { a = (int64_t)i; break; } }
    }
    if (a < 0) { fprintf(stderr, "extend: not enough cycles\n"); exit(2); }
    uint64_t period = car[b].t - car[a].t;
    for (unsigned k = 1; k <= n; k++)
        for (size_t i = (size_t)a; i < (size_t)b; i++) push_car_ex(car[i].t + period * k, car[i].id, car[i].cycle, car[i].len, 0);
    *tend = car[a].t + period * (n + 1);
    return (size_t)b;   // original frames [0, b) are replayed as captured
}

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void run_until(uint64_t tend)
{
    for (;;) {
        bool hv = nheap && heap[0].t <= tend;
        bool cv = cfeed < nfeed && car[cfeed].t <= tend;
        if (!hv && !cv) break;
        if (cv && (!hv || car[cfeed].t <= heap[0].t)) handle(car_ev(cfeed++));
        else handle(pop());
    }
}

static dde_emu_stats_t st(void) { dde_emu_stats_t s; dde_emu_fill_stats(&s); return s; }

static double bus_tol_us = 0.3;   // dyn-mitm: SavvyCAN 1 us timestamps -> 1.2
// Timing / completeness of what went on the bus from cycle index c_from (absolute tick) on.
static unsigned check_bus(uint64_t from, uint64_t to, unsigned *cycles_out)
{
    static const unsigned ids[5] = {0x26, 0x28, 0x29, 0x3D, 0x5C};   // 0x5C: v1d NM frame (c%16 == 1)
    double maxerr = 0;
    unsigned bad = 0, cyc_seen = 0;
    // count cycles (0x07 marks) in [from, to)
    for (unsigned k = 0; k < ntx; k++) {
        if (txl[k].txen < from || txl[k].txen >= to) continue;
        int ti = -1;
        for (int j = 0; j < 5; j++) if (ids[j] == txl[k].id) ti = j;
        if (txl[k].id == 0xE6 || txl[k].id == 0xE8 || txl[k].id == 0xFB ||
            txl[k].id == 0x10A || txl[k].id == 0x142 || txl[k].id == 0x68) continue;   // v1c/v1d dynamic: check_dyn()
        if (ti < 0) { bad++; continue; }
        int64_t r07 = txl[k].r07;   // this cycle's 0x07 mark
        double cs = (double)r07 - dde_emu_ref_off_q8[0] / 256.0;
        double off = ti < 4 ? dde_emu_static_off_q8[ti] / 256.0 : dde_emu_dyn_off_q8 / 256.0;
        double err = ((double)txl[k].echo - cs - off) / 100.0;   // us
        if (!txl[k].echo) { bad++; continue; }
        if (err < 0) err = -err;
        if (err > maxerr) maxerr = err;
        if (err > bus_tol_us) { if (bad < 5) printf("    off: id 0x%02X cycle %u err %.3f r07=%lld echo=%llu\n", txl[k].id, txl[k].cycle, err, (long long)r07, (unsigned long long)txl[k].echo); bad++; }
    }
    (void)cyc_seen;
    if (cycles_out) *cycles_out = 0;
    printf("  bus: %u frames, max |echo - slot| = %.3f us, %u bad\n", ntx, maxerr, bad);
    return bad;
}

static void print_stats(const char *tag)
{
    dde_emu_stats_t s = st();
    printf("  %s: state=%u fault=%u sent[%u %u %u %u] late=%u echo ok=%u miss=%u foreign=%u K=%u loop=%uns\n", tag,
           s.state, s.fault, (unsigned)s.sent[0], (unsigned)s.sent[1], (unsigned)s.sent[2], (unsigned)s.sent[3],
           (unsigned)s.late, (unsigned)s.echo_ok, (unsigned)s.echo_miss, (unsigned)s.foreign, s.fire_k_ticks, s.loop_delay_ns);
#ifdef DDE_EMU_STATS_V1_SIZE
    printf("  %s: first fault code=%u slot=0x%02X cycle=%u detail=%d sent=%u; faults=%u auto_clears=%u latched=%u "
           "late[busy=%u fire=%u short=%u win=%u missed=%u] dup=%u\n", tag, s.ff_code, s.ff_slot, s.ff_cycle,
           (int)s.ff_detail, (unsigned)s.ff_sent_total, s.fault_count, s.auto_clears, s.latched, s.late_reason[0],
           s.late_reason[1], s.late_reason[2], s.late_reason[3], s.late_reason[4], s.echo_dup);
    if (s.late) printf("  %s: last late slot 0x%02X reason %u r %d\n", tag, s.last_late_slot, s.last_late_reason, (int)s.last_late_r);
#endif
}

// ------------------------------------------------------------------ v1c dynamic frames
#ifdef DDE_EMU_STATS_V2_SIZE
extern int (*dde_emu_test_dyn_pick)(unsigned ds, unsigned cycle);
#define MS_T (DDE_DYN_MS_Q8 / 256.0)          // minislot in ticks
// minislots a frame of plen payload bytes takes (FES/DTS end of either sender's clock):
// ceil((100 (plen + 8) + o) / MS) + 2 for o = 126..175 ticks, -1 if the two ends differ
// (independent of dde_core_dyn_ms_used(); 2/4 -> 4, 6/8/10 -> 5, 14 -> 6, 20 -> 7, 30 -> 8, 32 -> 9)
static int k_of(unsigned plen)
{
    if (!plen || (plen & 1u) || plen > 254) return -1;
    double a = ceil((100.0 * (plen + 8) + 126) * 256.0 / DDE_DYN_MS_Q8), b = ceil((100.0 * (plen + 8) + 175) * 256.0 / DDE_DYN_MS_Q8);
    return a == b ? (int)a + 2 : -1;
}
static bool is_dyn_id(unsigned id) { return id == 0xE6 || id == 0xE8 || id == 0xFB || id == 0x10A || id == 0x142 || id == 0x68; }
static bool is_our_dyn(unsigned id) { return is_dyn_id(id) || id == 0x5C; }   // + the NM frame (minislot 0)
static unsigned n_shifted;
static void shift_after_ours(uint64_t txen, unsigned id, unsigned cyc, unsigned plen)
{
    int k = k_of(plen);
    if (k < 0 || !is_our_dyn(id)) return;
    uint64_t sh = (uint64_t)(k * MS_T + 0.5);
    for (size_t j = cfeed; j < nfeed && car[j].t < txen + 600000u; j++)
        if (car[j].cycle == cyc && car[j].id > id && car[j].t > txen) { car[j].t += sh; n_shifted++; }
}
static double cs_of(int64_t r07) { return (double)r07 - dde_emu_ref_off_q8[0] / 256.0; }
static double ms_at(double t, double cs) { return ((t - cs) * 256.0 - (double)DDE_DYN_BASE_Q8) / DDE_DYN_MS_Q8; }

// injected frames (dyn-unsure), also part of the order checks
typedef struct { uint64_t t; unsigned id, cycle, len; uint8_t flags; } inj_t;
static inj_t inj[4096]; static unsigned ninj;
static void inject(uint64_t t, unsigned id, unsigned cyc, unsigned len, uint8_t flags)
{
    if (ninj < 4096) { inj[ninj].t = t; inj[ninj].id = id; inj[ninj].cycle = cyc; inj[ninj].len = len; inj[ninj].flags = flags; ninj++; }
    push_car_ex(t, id, cyc, len, flags);
}

static uint64_t t_ign;          // first 0x103 of the replay (0 = none)
static size_t car_lb(uint64_t t)   // first car frame at or after t
{
    size_t lo = 0, hi = ncar;
    while (lo < hi) { size_t m = (lo + hi) / 2; if (car[m].t < t) lo = m + 1; else hi = m; }
    return lo;
}
static int pdu_of(unsigned id, unsigned plen)
{
    for (int p = 0; p < DDE_DYN_N_PDU; p++) if (dde_dyn_pdu[p].id == id && dde_dyn_pdu[p].plen == plen) return p;
    return -1;
}

typedef struct {
    unsigned fb9_off, n, noecho, offgrid, mexp_bad, order_bad, overlap, ctr_bad, bytes_bad, preign_bad, e85_early, cyc_bad, period_bad, spec_bad;
    unsigned per_pdu[DDE_DYN_N_PDU];
    double maxres;
    unsigned ph_bad, ph_n[3];   // v1d 0x142 content phase vs the KL15 model
} dynchk_t;

// ---- v1d: KL15 / NM model of the replay, from the vehicle frames only (independent of
// dde_kl15_*): 0xF1 payload[2] 0x20 = on / 0xE0 = off, 5 s without 0xF1 = off; before
// the first 0xF1 of a wake 0x103 within 500 ms = on. 0x5C allowed while on, 47 s after
// the off edge, or 1.76 s after wake if KL15 was not on in this wake; never 60 s after
// max(wake, last on-evidence). Wake = first reference frame after > 1 s of bus silence.
#define TK_S 100000000ull
typedef struct { uint64_t t; uint8_t v; } seg_t;
#define MAXSEG 8192
static seg_t kseg[MAXSEG], nseg[MAXSEG]; static unsigned nks, nns;
static uint64_t wakes[256]; static unsigned nwakes;
static uint64_t kl_first_on, kl_last_off, nm_end_model;
static void seg_add(seg_t *a, unsigned *n, uint64_t t, uint8_t v)
{
    if (*n && a[*n - 1].v == v) return;
    if (*n < MAXSEG) { a[*n].t = t; a[*n].v = v; (*n)++; }
}
static int seg_idx(const seg_t *a, unsigned n, uint64_t t)   // last segment starting at or before t
{
    int lo = 0, hi = (int)n - 1, r = -1;
    while (lo <= hi) { int m = (lo + hi) / 2; if (a[m].t <= t) { r = m; lo = m + 1; } else hi = m - 1; }
    return r;
}
static bool seg_at(const seg_t *a, unsigned n, uint64_t t) { int i = seg_idx(a, n, t); return i >= 0 && a[i].v; }
static uint64_t wake_of(uint64_t t)
{
    uint64_t w = 0;
    for (unsigned i = 0; i < nwakes && wakes[i] <= t; i++) w = wakes[i];
    return w;
}
static void build_kl15_model(void)
{
    struct { bool f1_seen, c103_seen, on, ever_on, nm; uint8_t b3; uint64_t f1_t, c103_t, evid_t, off_t, wake; } m;
    memset(&m, 0, sizeof m);
    uint64_t prev = 0;
    bool waiting = true;   // wake = the first reference frame (the emulator's lock) after > 1 s of silence
    for (size_t i = 0; i < ncar; i++) {
        uint64_t t = car[i].t;
        if (i && t - prev > (uint64_t)DDE_LOCK_WAKE_US * 100u) waiting = true;
        prev = t;
        if (waiting) {
            if (dde_core_ref_index(car[i].id) < 0) continue;
            waiting = false;
            memset(&m, 0, sizeof m);
            m.wake = m.evid_t = t;
            if (nwakes < 256) wakes[nwakes++] = t;
        }
        if (car[i].id == 0xF1 && car[i].b3 >= 0) {
            m.f1_seen = true; m.f1_t = t; m.b3 = (uint8_t)car[i].b3;
            if (m.b3 == 0x20) m.evid_t = t;
        }
        if (car[i].id == 0x103) {
            if (!m.f1_seen && (!m.c103_seen || t - m.c103_t > TK_S / 2)) m.evid_t = t;
            m.c103_seen = true; m.c103_t = t;
        }
        bool on = m.f1_seen ? (m.b3 == 0x20 && t - m.f1_t < 5 * TK_S) : (m.c103_seen && t - m.c103_t < TK_S / 2);
        if (on && !m.on && !kl_first_on) kl_first_on = t;
        if (m.on && !on) { m.off_t = t; kl_last_off = t; }
        if (on) m.ever_on = true;
        m.on = on;
        seg_add(kseg, &nks, t, on);
        bool nm = on || (m.ever_on ? t - m.off_t < 47 * TK_S : t - m.wake < 176000000ull);
        uint64_t base = m.evid_t > m.wake ? m.evid_t : m.wake;
        if (t - base >= 60 * TK_S) nm = false;
        if (m.nm && !nm) nm_end_model = t;
        m.nm = nm;
        seg_add(nseg, &nns, t, nm);
    }
}
static uint64_t kl_on_since(uint64_t t)   // start of the on-segment holding t (0 = off)
{
    int i = seg_idx(kseg, nks, t);
    return i >= 0 && kseg[i].v ? kseg[i].t : 0;
}
static bool kl_ever_on(uint64_t t)   // KL15 was on in this wake before t
{
    uint64_t w = wake_of(t);
    for (int i = seg_idx(kseg, nks, t); i >= 0 && kseg[i].t >= w; i--) if (kseg[i].v) return true;
    return false;
}
static bool nm_at(uint64_t t) { return seg_at(nseg, nns, t); }
// v1e: start of the KL15-off segment holding t, if KL15 was on earlier in this wake (0 = n/a)
static uint64_t kl_off_since(uint64_t t)
{
    int i = seg_idx(kseg, nks, t);
    if (i < 0 || kseg[i].v || !kl_ever_on(t)) return 0;
    return kseg[i].t;
}
// 0x142 content phase the model expects at t (DDE_142_P*)
static int ph142_model(uint64_t t)
{
    uint64_t on = kl_on_since(t);
    if (on && t >= on + 160000000ull) return 2;
    if (!on && kl_ever_on(t)) return 2;
    return t >= wake_of(t) + 259000000ull ? 1 : 0;
}
#define NM_MARGIN (10u * 500000u)   // 10 cycles: the emulator counts from its lock, 0xF1 reaches it ~0.2 ms late
static bool near_any(bool (*f)(uint64_t), uint64_t t, uint64_t m) { return f(t) || f(t > m ? t - m : 0) || f(t + m); }
static bool near_all(bool (*f)(uint64_t), uint64_t t, uint64_t m) { return f(t) && f(t > m ? t - m : 0) && f(t + m); }
static bool ph_ok(int ph, uint64_t t)
{
    return ph == ph142_model(t) || ph == ph142_model(t > NM_MARGIN ? t - NM_MARGIN : 0) || ph == ph142_model(t + NM_MARGIN);
}

#define CYC_T 500000.0   // ticks per cycle (5 ms)
// dyn-unsure: windows with injected frames; the period check skips sends around them
// (several skip/retry rounds in a row, e.g. 0x10A after 0xE6 in the same cycle class)
static uint64_t period_exempt[3][2];
static bool exempt_at(uint64_t t)
{
    for (int w = 0; w < 3; w++)
        if (period_exempt[w][1] && t + 100000000ull >= period_exempt[w][0] && t < period_exempt[w][1] + 200000000ull) return true;
    return false;
}
static dynchk_t check_dyn(bool picked)
{
    dynchk_t d; memset(&d, 0, sizeof d);
    int last_ctr[DDE_DYN_N_PDU]; uint64_t last_tx[DDE_DYN_N_PDU]; double last_iv[DDE_DYN_N_PDU];
    for (int p = 0; p < DDE_DYN_N_PDU; p++) { last_ctr[p] = 0; last_tx[p] = 0; last_iv[p] = 0; }
    for (unsigned k = 0; k < ntx; k++) {
        unsigned X = txl[k].id, cyc = txl[k].cycle, plen = txl[k].plen;
        if (!is_dyn_id(X)) continue;
        d.n++;
        int p = pdu_of(X, plen);
        if (p < 0) { d.bytes_bad++; continue; }
        d.per_pdu[p]++;
        if (!txl[k].echo) { d.noecho++; continue; }
        double cs = cs_of(txl[k].r07), tx = (double)txl[k].echo;
        double mf = ms_at(tx, cs);
        long m = lround(mf);
        double res = fabs(mf - (double)m) * MS_T / 100.0;
        if (res > d.maxres) d.maxres = res;
        if (res > 0.3) d.offgrid++;
        // expected minislot from the frames before ours (car, injected, our own dynamic ones)
        double tp = -1; unsigned P = 0, lp = 0;
        for (size_t j = car_lb((uint64_t)cs); j < ncar && (double)car[j].t < tx - 50; j++)
            if (car[j].cycle == cyc && car[j].id >= DDE_DYN_FIRST_ID && car[j].id < X && (double)car[j].t > tp) { tp = (double)car[j].t; P = car[j].id; lp = car[j].len; }
        for (unsigned j = 0; j < ninj; j++)
            if (!(inj[j].flags & EVF_MARKONLY) && inj[j].cycle == cyc && (double)inj[j].t > cs && (double)inj[j].t < tx - 50 &&
                inj[j].id >= DDE_DYN_FIRST_ID && inj[j].id < X && (double)inj[j].t > tp) { tp = (double)inj[j].t; P = inj[j].id; lp = inj[j].len; }
        for (unsigned j = 0; j < k; j++)
            if (is_our_dyn(txl[j].id) && txl[j].cycle == cyc && txl[j].echo && (double)txl[j].echo > cs && txl[j].id < X && (double)txl[j].echo > tp) { tp = (double)txl[j].echo; P = txl[j].id; lp = txl[j].plen; }
        long mexp = tp < 0 ? (long)(X - DDE_DYN_FIRST_ID) : lround(ms_at(tp, cs)) + k_of(lp) + (long)(X - P - 1);
        if (m != mexp) { if (d.mexp_bad < 5) printf("    0x%02X cycle %u: minislot %ld, frames before it give %ld (P=0x%02X len %u)\n", X, cyc, m, mexp, P, lp); d.mexp_bad++; }
        // no lower-ID dynamic frame after ours in this cycle, nothing on the bus while we send
        for (size_t j = car_lb(txl[k].txen - 100); j < ncar && (double)car[j].t < cs + CYC_T; j++) {
            if (car[j].cycle != cyc) continue;
            if (car[j].id >= DDE_DYN_FIRST_ID && car[j].id < X) d.order_bad++;
            if (car[j].t <= txl[k].end + 100) d.overlap++;
        }
        for (unsigned j = 0; j < ninj; j++)
            if (inj[j].cycle == cyc && inj[j].t + 100 >= txl[k].txen && (double)inj[j].t < cs + CYC_T && inj[j].id < X) d.order_bad++;
        // counter: 1, 2, .. 14, 0, 1 .. per PDU (CTR/E2E kinds)
        const dde_dyn_pdu_t *pd = &dde_dyn_pdu[p];
        int ctr = -1;
        if (pd->kind == DDE_DYN_KIND_E2E) ctr = txl[k].pl[1] & 0x0F;
        else if (pd->kind == DDE_DYN_KIND_CTR) ctr = txl[k].pl[0] & 0x0F;
        else if (pd->kind == DDE_DYN_KIND_CTR8) ctr = txl[k].pl[0];   // v1d 0x142: 0..254
        int cmod = pd->kind == DDE_DYN_KIND_CTR8 ? 255 : 15;
        int want = last_ctr[p] == 0 && !last_tx[p] ? 1 : (last_ctr[p] + 1) % cmod;
        if (ctr >= 0) { if (ctr != want) d.ctr_bad++; last_ctr[p] = ctr; }
        // content: pre-ign until 12 cycles after the first 0x103 (cycle-boundary ambiguity allowed)
        double since = t_ign ? (double)txl[k].txen - (double)t_ign : -1e18;
        bool pre_must = since < 11 * CYC_T, post_must = since > 13 * CYC_T;
        uint8_t fr[DDE_MAX_FRAME_BYTES]; bool okc = false;
        if (X == 0x142) {   // v1d: three content phases (wake / +2.59 s / KL15 on + 1.6 s)
            pre_must = post_must = false;
            for (int ph = 0; ph < 3 && !okc; ph++) {
                dde_core_build_dyn_pdu_ph((unsigned)p, cyc, ctr >= 0 ? (unsigned)ctr : 0, (unsigned)ph, fr);
                if (memcmp(fr + 5, txl[k].pl, plen) == 0) {
                    okc = true; d.ph_n[ph]++;
                    if (!ph_ok(ph, txl[k].txen)) {
                        if (d.ph_bad < 5) printf("    0x142 at %.3f s: phase %d, model %d\n", txl[k].txen / 1e8, ph, ph142_model(txl[k].txen));
                        d.ph_bad++;
                    }
                }
            }
        }
        if (p == DDE_PDU_FB9) {   // v1e: back to the pre-ign content 46 cycles after KL15 off
            uint64_t off = kl_off_since(txl[k].txen);
            if (off) {
                double a = (double)txl[k].txen - (double)off;
                if (a > (DDE_OFF_FB9_CYCLES + 1) * CYC_T) { pre_must = true; post_must = false; d.fb9_off++; }
                else if (a > (DDE_OFF_FB9_CYCLES - 1) * CYC_T) pre_must = post_must = false;
            }
        }
        for (int pre = 0; pre < 2 && !okc && X != 0x142; pre++) {
            if ((pre && post_must) || (!pre && pre_must)) continue;
            dde_core_build_dyn_pdu((unsigned)p, cyc, ctr >= 0 ? (unsigned)ctr : 0, pre != 0, fr);
            okc = memcmp(fr + 5, txl[k].pl, plen) == 0;
        }
        if (!okc) { if (d.bytes_bad + d.preign_bad < 5) printf("    0x%02X L%u cycle %u: bytes differ (ign +%.0f ms)\n", X, plen + 1, cyc, since / 1e5); if (pre_must || post_must) d.preign_bad++; else d.bytes_bad++; }
        // the spec's ign-on 0xE6 bytes, independent of the builder
        if (p == 0 && post_must) {
            static const uint8_t e6[6] = {0x80, 0x64, 0x45, 0x44, 0x23, 0x47};
            if ((txl[k].pl[1] & 0xF0) != 0x10 || memcmp(txl[k].pl + 2, e6, 6) != 0 ||
                txl[k].pl[0] != (dde_core_crc8(txl[k].pl + 1, 7) ^ 0x04)) d.spec_bad++;
        }
        if (getenv("DYN_TRACE") && p == atoi(getenv("DYN_TRACE"))) printf("    trace %s cycle %u at %.4f s m %ld\n", pd->name, cyc, txl[k].txen / 1e8, m);
        if (!picked && pd->ign_only) {   // v1d: KL15 on (model) and start_cyc cycles after its on edge
            uint64_t on = kl_on_since(txl[k].txen), on2 = kl_on_since(txl[k].txen - NM_MARGIN);
            bool ok = (on && (double)txl[k].txen >= (double)on + (pd->start_cyc - 1) * CYC_T) ||
                      (on2 && (double)txl[k].txen - NM_MARGIN >= (double)on2 + (pd->start_cyc - 1) * CYC_T - NM_MARGIN);
            if (!ok) {
                if (d.e85_early < 5) printf("    %s at %.3f s: KL15 model on since %.3f s (start %u cycles)\n", pd->name, txl[k].txen / 1e8, on / 1e8, pd->start_cyc);
                d.e85_early++;
            }
        }
        if (!picked) {
            if (pd->every4 ? (cyc & 3u) != (pd->cyc_mod8 & 3u) : (cyc & 7u) != pd->cyc_mod8) d.cyc_bad++;
            // period grid: a send that was late (retry after a skip / no lock) is followed by
            // one that is early by the same amount (the grid is kept)
            if (last_tx[p]) {
                double iv = (double)(txl[k].txen - last_tx[p]) / CYC_T * 1.0005, per = pd->period;
                bool ok = iv > per - 0.5 || (last_iv[p] > per + 0.5 && fabs(iv + last_iv[p] - 2 * per) < 1.0) ||
                          exempt_at(txl[k].txen) || exempt_at(last_tx[p]);
                if (!ok) {
                    if (d.period_bad < 5) printf("    %s cycle %u at %.3f s: %.2f cycles after the previous one\n", pd->name, cyc, txl[k].txen / 1e8, iv);
                    d.period_bad++;
                }
                last_iv[p] = iv;
            }
        }
        last_tx[p] = txl[k].txen;
    }
    return d;
}

static unsigned print_dyn(const char *tag, const dynchk_t *d)
{
    dde_emu_stats_t s = st();
    printf("  %s dyn: sent E6=%u FB5=%u FB7=%u FB9=%u E8L9=%u E8L5=%u skip[%u %u %u %u %u %u] reasons[anchor=%u mark=%u passed=%u len=%u grid=%u limit=%u late=%u] defer=%u parks=%u ms[E6=%u E8=%u FB=%u] ign=0x%x build=0x%x\n",
           tag, (unsigned)s.dyn_sent[0], (unsigned)s.dyn_sent[1], (unsigned)s.dyn_sent[2], (unsigned)s.dyn_sent[3], (unsigned)s.dyn_sent[4], (unsigned)s.dyn_sent[5],
           s.dyn_skip[0], s.dyn_skip[1], s.dyn_skip[2], s.dyn_skip[3], s.dyn_skip[4], s.dyn_skip[5],
           s.dyn_skip_reason[0], s.dyn_skip_reason[1], s.dyn_skip_reason[2], s.dyn_skip_reason[3], s.dyn_skip_reason[4], s.dyn_skip_reason[5], s.dyn_skip_reason[6],
           s.dyn_defer, s.dyn_parks, s.dyn_last_ms[0], s.dyn_last_ms[1], s.dyn_last_ms[2], s.ign, s.dyn_flags);
    printf("  %s check: %u dynamic frames, max |echo - minislot grid| %.3f us; offgrid %u, minislot != frames before %u, "
           "lower-ID after ours %u, overlap %u, no echo %u, counter %u, bytes %u, pre-ign %u, KL15-gated outside KL15 %u, cycle %u, period %u, spec E6 %u; "
           "car frames moved back %u, spins %u, parked %u/put while parked %u\n",
           tag, d->n, d->maxres, d->offgrid, d->mexp_bad, d->order_bad, d->overlap, d->noecho, d->ctr_bad, d->bytes_bad, d->preign_bad, d->e85_early,
           d->cyc_bad, d->period_bad, d->spec_bad, n_shifted, n_spin, n_parked, n_put_parked);
    printf("  %s v1e: 0xFB L9 after KL15 off + %u cycles checked as 1f 00 ff 7f: %u\n", tag, DDE_OFF_FB9_CYCLES, d->fb9_off);
#ifdef DDE_EMU_STATS_V3_SIZE
    printf("  %s v1d: sent 10A=%u 142=%u 68=%u skip[%u %u %u] ms[10A=%u 142=%u 68=%u] 0x142 phases [%u %u %u] bad %u; "
           "kl15=0x%02x f1_b3=0x%02x edges on %u off %u off_ago %u ms ceil_left %u ms ceiling stops %u; "
           "isr late %d..%d us r_entry_min %d r_final_min %d topup timeouts %u hk_deferred %u flags2=0x%02x ph142 %u abs %u; "
           "PIO fifo over %u underrun %u topups %u\n", tag,
           (unsigned)s.dyn_sent2[0], (unsigned)s.dyn_sent2[1], (unsigned)s.dyn_sent2[2], s.dyn_skip2[0], s.dyn_skip2[1], s.dyn_skip2[2],
           s.dyn_last_ms2[0], s.dyn_last_ms2[1], s.dyn_last_ms2[2], d->ph_n[0], d->ph_n[1], d->ph_n[2], d->ph_bad,
           s.kl15, s.f1_b3, s.kl15_on_edges, s.kl15_off_edges, (unsigned)s.kl15_off_ago_ms, (unsigned)s.ceil_left_ms, s.nm_ceiling_stops,
           s.isr_late_min_us, s.isr_late_max_us, s.r_entry_min, s.r_final_min, s.topup_timeouts, s.hk_deferred, s.dyn_flags2, s.ph142,
           (unsigned)s.abs_cycle, n_fifo_over, n_underrun, n_topup);
#endif
    return d->offgrid + d->mexp_bad + d->order_bad + d->overlap + d->noecho + d->ctr_bad + d->bytes_bad + d->preign_bad + d->e85_early +
           d->cyc_bad + d->period_bad + d->spec_bad + n_put_parked + d->ph_bad + n_fifo_over + n_underrun;
}

// static pre-ign content: 0x28 cycle%4==1 b3..b5 = c1 17 7c, 0x29 cycle%4==3 b8 = 80 until ign
static unsigned check_preign_static(void)
{
    unsigned bad = 0, npre = 0, npost = 0, noff = 0;
    for (unsigned k = 0; k < ntx; k++) {
        double since = t_ign ? (double)txl[k].txen - (double)t_ign : -1e18;
        bool pre = since < 11 * CYC_T, post = since > 13 * CYC_T;
        if (txl[k].id == 0x28 && (txl[k].cycle & 3u) == 1) {
            bool v = txl[k].pl[2] == 0xC1 && txl[k].pl[3] == 0x17 && txl[k].pl[4] == 0x7C;
            if (pre) { npre++; bad += !v; }
            if (post) { npost++; bad += v; }
        }
        if (txl[k].id == 0x29 && (txl[k].cycle & 3u) == 3) {
            bool v = txl[k].pl[7] == 0x80;
            // v1e: after KL15 off (+6 cycles) b8 = c0, before that 10
            uint64_t off = kl_off_since(txl[k].txen);
            double a = off ? (double)txl[k].txen - (double)off : -1.0;
            if (off && a > (DDE_OFF_29_CYCLES + 1) * CYC_T) { noff++; bad += txl[k].pl[7] != 0xC0; continue; }
            if (off && a > (DDE_OFF_29_CYCLES - 1) * CYC_T) continue;
            if (pre) { npre++; bad += !v; }
            if (post) { npost++; bad += v || txl[k].pl[7] == 0xC0; }
        }
    }
    printf("  statics: %u pre-ign / %u ign-on / %u after-off frames of 0x28 c1 / 0x29 c3 checked, %u wrong\n", npre, npost,
           noff, bad);
    return bad;
}

// dyn-run5: the real DDE's 0xE6/0xE8/0xFB (same time base as the vectors)
typedef struct { uint64_t t; unsigned id, cycle, len; bool used; } dde_t;
static dde_t *ddef; static size_t nddef;
static void load_dde(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); exit(2); }
    char line[256]; size_t cap = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        unsigned long long t; unsigned id, cyc, len, hw;
        if (sscanf(line, "%llu %u %u %u %u", &t, &id, &cyc, &len, &hw) < 4) continue;
        if (nddef == cap) { cap = cap ? cap * 2 : 8192; ddef = realloc(ddef, cap * sizeof(dde_t)); }
        ddef[nddef].t = t + 100000ull; ddef[nddef].id = id; ddef[nddef].cycle = cyc; ddef[nddef].len = len; ddef[nddef].used = false; nddef++;
    }
    fclose(f);
}
static size_t dde_lb(uint64_t t)
{
    size_t lo = 0, hi = nddef;
    while (lo < hi) { size_t m = (lo + hi) / 2; if (ddef[m].t < t) lo = m + 1; else hi = m; }
    return lo;
}
static unsigned picks_real;
static int pick_real(unsigned ds, unsigned cycle)
{
    // called at the cycle's first reference (~0.2 ms into the cycle)
    for (size_t j = dde_lb(T); j < nddef && ddef[j].t < T + 480000u; j++) {
        if (ddef[j].cycle != cycle || ddef[j].id != dde_dyn_slot_id[ds]) continue;
        int p = pdu_of(ddef[j].id, ddef[j].len);
        if (p >= 0) { picks_real++; return p; }
    }
    return -1;
}
static bool fr2_other_id(unsigned id) { return id == 0x5C || id == 0x68 || id == 0xBE || id == 0x10A || id == 0x142; }
static unsigned compare_real(void)
{
    // ours - real in whole minislots: 0 = same place. Ours later (+n): the real DDE's FR2 time is n
    // minislots early (repeater; the vehicle grid is the truth) - a vehicle frame after it in the
    // cycle decides. Ours earlier (-n): a frame before it that the capture did not decode (the
    // emulator sees its mark and does not send; the replay has no such frame).
    unsigned match = 0, other = 0, notsent = 0, cmp = 0, hist_lo[41] = {0}, hist_hi[8] = {0};
    unsigned conf_ours[2] = {0}, conf_real[2] = {0}, unconf[2] = {0};   // [0] ours later, [1] ours earlier
    double maxm = 0;
    for (size_t j = 0; j < nddef; j++) {
        if (pdu_of(ddef[j].id, ddef[j].len) < 0) continue;
        int best = -1; double bd = 1e18;
        for (unsigned k = 0; k < ntx; k++) {
            if (txl[k].id != ddef[j].id || txl[k].cycle != ddef[j].cycle || !txl[k].echo) continue;
            double dd = (double)txl[k].echo - (double)ddef[j].t;
            if (fabs(dd) < 300000 && fabs(dd) < fabs(bd)) { bd = dd; best = (int)k; }
        }
        if (best < 0) { notsent++; continue; }
        cmp++;
        double dm = bd / MS_T;
        long n = lround(dm);
        if (fabs(bd) <= 40) { match++; if (fabs(bd) / 100.0 > maxm) maxm = fabs(bd) / 100.0; continue; }
        if (fabs(dm - (double)n) > 0.1 || n == 0 || n > 7 || n < -40) {
            other++;
            if (other <= 5) printf("    0x%02X cycle %u at %.3f s: ours - real = %.2f us (%.2f minislots)\n", ddef[j].id, ddef[j].cycle, ddef[j].t / 1e8, bd / 100.0, dm);
            continue;
        }
        if (n > 0) hist_hi[n]++; else hist_lo[-n]++;
        if (getenv("DYN_TRACE") && n < 0 && n != -4) {
            double cs0 = cs_of(txl[best].r07);
            printf("    earlier %ld: 0x%X L%u cycle %u at %.4f s ours m%.2f real m%.2f:", n, ddef[j].id, ddef[j].len, ddef[j].cycle, ddef[j].t / 1e8, ms_at((double)txl[best].echo, cs0), ms_at((double)ddef[j].t, cs0));
            for (size_t i = car_lb((uint64_t)cs0); i < ncar && (double)car[i].t < cs0 + CYC_T; i++)
                if (car[i].cycle == ddef[j].cycle && car[i].id >= 0x5C) printf(" [0x%X m%.2f L%u]", car[i].id, ms_at((double)car[i].t, cs0), car[i].len);
            for (size_t i = 0; i < nddef; i++) if (ddef[i].cycle == ddef[j].cycle && fabs((double)ddef[i].t - cs0) < CYC_T)
                printf(" real[0x%X m%.2f L%u]", ddef[i].id, ms_at((double)ddef[i].t, cs0), ddef[i].len);
            printf("\n");
        }
        // the next vehicle (FR1) frame of the cycle after ours
        double cs = cs_of(txl[best].r07), mo = ms_at((double)txl[best].echo, cs);
        int ko = k_of(txl[best].plen);
        bool found = false;
        for (size_t i = car_lb(txl[best].echo + 50); i < ncar && (double)car[i].t < cs + CYC_T; i++) {
            if (car[i].cycle != ddef[j].cycle || car[i].id <= ddef[j].id || fr2_other_id(car[i].id)) continue;
            bool own_between = false;
            for (unsigned k = 0; k < ntx; k++)
                if (is_our_dyn(txl[k].id) && txl[k].cycle == ddef[j].cycle && txl[k].echo > txl[best].echo && txl[k].echo < car[i].t) own_between = true;
            if (own_between) break;
            double my = ms_at((double)car[i].t, cs);
            double want_ours = mo + ko + (car[i].id - ddef[j].id - 1), want_real = want_ours - (double)n;
            if (fabs(my - want_ours) < 0.2) conf_ours[n < 0]++;
            else if (fabs(my - want_real) < 0.2) conf_real[n < 0]++;
            else unconf[n < 0]++;
            found = true;
            break;
        }
        if (!found) unconf[n < 0]++;
    }
    printf("  vs real DDE: %u compared, %u at the same time (max %.3f us), %u other; %u real frames we did not send (picks %u)\n",
           cmp, match, maxm, other, notsent, picks_real);
    printf("  ours later than the real DDE's FR2 time by 1..7 minislots:");
    for (int i = 1; i < 8; i++) printf(" %u", hist_hi[i]);
    printf("\n  ours earlier (capture missed a frame before it), minislots:count");
    for (int i = 1; i < 41; i++) if (hist_lo[i]) printf(" %d:%u", i, hist_lo[i]);
    printf("\n  next vehicle frame after it: ours later: agrees with ours %u, with the real DDE's FR2 time %u, none %u;"
           " ours earlier: agrees with ours %u, with the real DDE %u, none %u\n",
           conf_ours[0], conf_real[0], unconf[0], conf_ours[1], conf_real[1], unconf[1]);
    CHECK(conf_real[0] == 0, "run 5: ours later than the real DDE and a vehicle frame confirms the real time (%u)", conf_real[0]);
    CHECK(cmp > 0 && match >= cmp * 90 / 100, "run 5: only %u of %u at the real DDE's time", match, cmp);
    CHECK(other == 0, "run 5: %u placements off the minislot grid", other);
    CHECK(notsent <= (cmp + notsent) / 50, "run 5: %u of the real DDE's frames not sent", notsent);
    return other;
}

// v1d NM frame 0x5C (c%16 == 1, minislot 0): only while the KL15/NM model allows it
// (10-cycle margin at the edges), in every allowed cycle the emulator is active in, and
// against the real DDE's 0x5C when its frames are loaded (dyn-run5 / dyn-mitm).
static unsigned check_nm(void)
{
    unsigned extra = 0, missing = 0, expected = 0, n5c = 0, badcyc = 0;
    uint64_t first = 0, last = 0;
    for (unsigned k = 0; k < ntx; k++) {
        if (txl[k].id != 0x5C) continue;
        n5c++;
        if (!first) first = txl[k].txen;
        last = txl[k].txen;
        if ((txl[k].cycle & 15u) != 1u) badcyc++;
        if (!near_any(nm_at, txl[k].txen, NM_MARGIN)) {
            if (extra < 5) printf("    0x5C at %.3f s cycle %u: the NM model does not allow it\n", txl[k].txen / 1e8, txl[k].cycle);
            extra++;
        }
    }
    for (unsigned k = 0; k < ntx; k++) {
        if (txl[k].id != 0x26 || (txl[k].cycle & 15u) != 1u || !near_all(nm_at, txl[k].txen, NM_MARGIN)) continue;
        expected++;
        bool f = false;
        for (unsigned j = k + 1; j < ntx && txl[j].txen < txl[k].txen + (uint64_t)CYC_T; j++)
            if (txl[j].id == 0x5C && txl[j].cycle == txl[k].cycle) f = true;
        if (!f) { if (missing < 5) printf("    cycle %u at %.3f s: 0x5C allowed, not sent\n", txl[k].cycle, txl[k].txen / 1e8); missing++; }
    }
    printf("  NM 0x5C: %u sent (%.3f .. %.3f s), model: KL15 first on %.3f s, last off %.3f s, NM end %.3f s; "
           "outside the model %u, wrong cycle %u, allowed cycles %u, missing %u\n",
           n5c, first / 1e8, last / 1e8, kl_first_on / 1e8, kl_last_off / 1e8, nm_end_model / 1e8, extra, badcyc, expected, missing);
    if (kl_last_off && last > kl_last_off) printf("  NM 0x5C: last one %.3f s after the KL15 off edge (model)\n", (last - kl_last_off) / 1e8);
    if (nddef) {
        unsigned nreal = 0, both = 0, ours_only = 0;
        uint64_t rlast = 0;
        for (size_t j = 0; j < nddef; j++) {
            if (ddef[j].id != 0x5C) continue;
            nreal++; rlast = ddef[j].t;
        }
        for (unsigned k = 0; k < ntx; k++) {
            if (txl[k].id != 0x5C) continue;
            bool m = false;
            for (size_t j = dde_lb(txl[k].txen > 400000 ? txl[k].txen - 400000 : 0); j < nddef && ddef[j].t < txl[k].txen + 400000; j++)
                if (ddef[j].id == 0x5C && ddef[j].cycle == txl[k].cycle) m = true;
            if (m) both++; else ours_only++;
        }
        printf("  NM 0x5C vs the real DDE: real %u (last %.3f s), ours %u, same cycle %u, ours only %u, real only %u\n",
               nreal, rlast / 1e8, n5c, both, ours_only, nreal - both);
    }
    return extra + badcyc + (missing > expected / 50 + 2 ? missing : 0);
}

// dyn-unsure: windows of cycles with something the tracker cannot explain before 0xE6
typedef struct { uint64_t from, to; const char *what; unsigned reason; } uwin_t;
static const uwin_t uwin[3] = {
    {200000000ull, 600000000ull, "frame start without header just before 0xE6", DDE_SKIP_MARK},
    {800000000ull, 1200000000ull, "18-byte frame (unknown length) before 0xE6", DDE_SKIP_LEN},
    {1400000000ull, 1800000000ull, "header without its own mark before 0xE6", DDE_SKIP_NOANCHOR},
};
static unsigned n_inj[3];
static struct { uint64_t cs; unsigned cyc; } inj_cyc[3][512];
static void setup_unsure(void)
{
    for (int w = 0; w < 3; w++) { period_exempt[w][0] = uwin[w].from; period_exempt[w][1] = uwin[w].to; }
    unsigned last_cyc = 99;
    for (size_t i = 0; i < ncar; i++) {
        int ri = dde_core_ref_index(car[i].id);   // first reference of each cycle gives its start
        if (ri < 0 || car[i].cycle == last_cyc) continue;
        unsigned cyc = car[i].cycle;
        last_cyc = cyc;
        if ((cyc & 7u) != dde_dyn_pdu[0].cyc_mod8) continue;
        double cs = (double)car[i].t - dde_emu_ref_off_q8[ri] / 256.0;
        // the last dynamic car frame before 0xE6 in this cycle and 0xE6's minislot
        double tp = -1; unsigned P = 0, lp = 0; size_t ip = 0; bool has_e5 = false;
        for (size_t j = i; j < ncar && (double)car[j].t < cs + CYC_T; j++) {
            if (car[j].cycle != cyc) continue;
            if (car[j].id == 0xE5) has_e5 = true;
            if (car[j].id >= DDE_DYN_FIRST_ID && car[j].id < 0xE6) { tp = (double)car[j].t; P = car[j].id; lp = car[j].len; ip = j; }
        }
        if (k_of(lp) < 0 && tp >= 0) continue;
        long m = tp < 0 ? 0xE6 - DDE_DYN_FIRST_ID : lround(ms_at(tp, cs)) + k_of(lp) + (long)(0xE6 - P - 1);
        double ap = cs + ((double)DDE_DYN_BASE_Q8 + (double)m * DDE_DYN_MS_Q8) / 256.0;
        for (int w = 0; w < 3; w++) {
            if (car[i].t < uwin[w].from || car[i].t >= uwin[w].to) continue;
            if (w == 0) inject((uint64_t)(ap - 550), 0x3FF, cyc, 0, EVF_MARKONLY);   // 5.5 us before our TXEN
            else if (w == 1) { if (has_e5) continue; inject((uint64_t)(ap - MS_T), 0xE5, cyc, 18, 0); }   // v1d: 20 B is now a known length (7 minislots)
            else if (tp >= 0) car[ip].hw = 0;   // the last frame before 0xE6 loses its mark
            else { if (has_e5) continue; inject((uint64_t)(ap - MS_T), 0xE5, cyc, 8, EVF_NOMARK); }
            if (n_inj[w] < 512) { inj_cyc[w][n_inj[w]].cs = (uint64_t)cs; inj_cyc[w][n_inj[w]].cyc = cyc; }
            n_inj[w]++;
        }
    }
}
static void check_unsure(void)
{
    dde_emu_stats_t s = st();
    for (int w = 0; w < 3; w++) {
        unsigned in = 0; uint64_t first_after = 0;
        for (unsigned k = 0; k < ntx; k++) {
            if (txl[k].id != 0xE6) continue;
            bool injected = false;
            for (unsigned j = 0; j < n_inj[w] && j < 512; j++)
                if (inj_cyc[w][j].cyc == txl[k].cycle && txl[k].txen > inj_cyc[w][j].cs && txl[k].txen < inj_cyc[w][j].cs + (uint64_t)CYC_T) injected = true;
            if (injected) {
                in++;
                if (getenv("DYN_TRACE")) {
                    double cs = cs_of(txl[k].r07);
                    printf("    sent inside: cycle %u at %.4f s m %ld;", txl[k].cycle, txl[k].txen / 1e8, lround(ms_at((double)txl[k].echo, cs)));
                    for (unsigned j = 0; j < ninj; j++) if (inj[j].cycle == txl[k].cycle && fabs((double)inj[j].t - (double)txl[k].txen) < 400000)
                        printf(" inj 0x%X at txen%+.2f us", inj[j].id, ((double)inj[j].t - (double)txl[k].txen) / 100.0);
                    for (size_t j = car_lb((uint64_t)cs); j < ncar && (double)car[j].t < cs + CYC_T; j++)
                        if (car[j].cycle == txl[k].cycle && car[j].id >= 0x5C && car[j].id < 0x100) printf(" [0x%X m%.2f L%u hw%u]", car[j].id, ms_at((double)car[j].t, cs), car[j].len, car[j].hw);
                    printf("\n");
                }
            }
            if (txl[k].txen >= uwin[w].to && !first_after) first_after = txl[k].txen;
        }
        double after_cyc = first_after ? (double)(first_after - uwin[w].to) / CYC_T : -1;
        printf("  unsure %d (%s): %u cycles injected, 0xE6 sent in one of them %u, first after the window +%.0f cycles, reason count %u\n",
               w + 1, uwin[w].what, n_inj[w], in, after_cyc, s.dyn_skip_reason[uwin[w].reason - 1]);
        CHECK(n_inj[w] > 20, "unsure %d: too few injections", w + 1);
        CHECK(in == 0, "unsure %d: 0xE6 sent %u times while unsure", w + 1, in);
        CHECK(first_after && after_cyc <= 20, "unsure %d: 0xE6 not resumed after the window", w + 1);
        CHECK(s.dyn_skip_reason[uwin[w].reason - 1] > 0, "unsure %d: reason not counted", w + 1);
    }
}
#else
static void shift_after_ours(uint64_t txen, unsigned id, unsigned cyc, unsigned plen) { (void)txen; (void)id; (void)cyc; (void)plen; }
#endif

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s VECTORS [run1|run1-v1|foreign|stuck|noecho|lateecho]\n", argv[0]); return 2; }
    const char *sc = argc > 2 ? argv[2] : "run1";
    load(argv[1]);
    uint64_t tend = car[ncar - 1].t + 100000;   // +1 ms: late echo headers still arrive
    size_t nplay = ncar;
    bool longrun = strcmp(sc, "foreign") == 0 || strcmp(sc, "stuck") == 0 || strcmp(sc, "noecho") == 0;
    if (longrun) nplay = extend(strcmp(sc, "foreign") == 0 ? 40 : 25, &tend);
    cfeed = 0; nfeed = nplay;   // the captured frames, fed in time order
    shift_on = true;            // (v1c) car frames with a higher ID than our dynamic frame move back
    t_capture_end = car[nplay - 1].t;
    if (strcmp(sc, "lateecho") == 0) echo_hdr_lat = 40000;   // 400 us
    bool full = strcmp(sc, "run1-full") == 0;
    bool spike = strstr(sc, "-spike") != NULL, bigspike = strstr(sc, "-bigspike") != NULL;
    if (spike) spike_us = 4;
    if (bigspike) spike_us = 8;

    set_now(0);
    dde_emu_init_core1();
    after_callback();
    { ev_t e = {0}; e.t = 10; e.type = EV_HOST; push(e); }   // logger enabled TX before the bus woke

    printf("scenario %s: %zu car frames, %.3f s\n", sc, ncar, (double)tend / 1e8);
#ifdef DDE_EMU_STATS_V2_SIZE
    if (strncmp(sc, "dyn-", 4) == 0) {
        bool r5 = strcmp(sc, "dyn-run5") == 0, unsure = strcmp(sc, "dyn-unsure") == 0, mitm = strcmp(sc, "dyn-mitm") == 0;
        if (mitm) bus_tol_us = 1.2;   // SavvyCAN timestamps: 1 us
        build_kl15_model();
        use_nomark = true;
        shift_on = !r5;   // run 5: the car frames are already where the real DDE's frames put them
        for (size_t i = 0; i < ncar; i++) if (car[i].id == 0x103) { t_ign = car[i].t; break; }
        if (r5) {
            if (argc < 4) { fprintf(stderr, "dyn-run5 needs the DDE frame file\n"); return 2; }
            load_dde(argv[3]);
            dde_emu_test_dyn_pick = pick_real;
        } else if (argc >= 4) load_dde(argv[3]);   // dyn-mitm: the real DDE's frames, for the 0x5C comparison only
        if (unsure) setup_unsure();
        run_until(tend);
        print_stats("end");
        dde_emu_stats_t s = st();
        printf("  ign: first 0x103 at %.3f s\n", t_ign / 1e8);
        dynchk_t d = check_dyn(r5);
        unsigned bad = print_dyn("end", &d);
        if (mitm) {
            // lossy vehicle side (capture gaps, 1 us stamps, missing frames): placement and
            // period results are informational; KL15 gating, NM, faults and foreign must hold
            printf("  (dyn-mitm: placement/period counts above are informational - lossy capture)\n");
            CHECK(d.e85_early == 0 && d.ph_bad == 0 && d.ctr_bad <= 8 && n_fifo_over == 0 && n_underrun == 0,
                  "KL15-gated %u, 0x142 phase %u, counter %u", d.e85_early, d.ph_bad, d.ctr_bad);
            CHECK(check_nm() == 0, "NM 0x5C checks");
            CHECK(s.fault == 0 && s.foreign == 0 && s.topup_timeouts == 0 && s.nm_ceiling_stops == 0,
                  "fault %u foreign %u", s.fault, (unsigned)s.foreign);
            printf("%s: %s\n", sc, fails ? "FAILED" : "ok");
            return fails ? 1 : 0;
        }
        CHECK(bad == 0, "dynamic frame checks: %u problems", bad);
#ifdef DDE_EMU_STATS_V3_SIZE
        CHECK(check_nm() == 0, "NM 0x5C checks");
        CHECK(s.topup_timeouts == 0 && s.nm_ceiling_stops == 0, "topup timeouts %u, ceiling stops %u", s.topup_timeouts, s.nm_ceiling_stops);
#endif
        CHECK(s.fault == 0 && s.foreign == 0 && s.echo_miss == 0 && n_truncated == 0 && n_fire_without_frame == 0,
              "fault %u foreign %u miss %u truncated %u", s.fault, (unsigned)s.foreign, (unsigned)s.echo_miss, n_truncated);
        CHECK(check_bus(0, tend + 1, NULL) == 0, "static frames off their slots");
        CHECK(s.sent[0] > 1000, "statics not sent (%u)", (unsigned)s.sent[0]);
        unsigned nfb7 = 0;
        for (unsigned k = 0; k < ntx; k++) nfb7 += (txl[k].plen & 3u) == 2u;
        CHECK((unsigned)s.dyn_parks + 1u >= nfb7 && (unsigned)s.dyn_parks <= nfb7, "parks %u vs %u 14-byte frames", (unsigned)s.dyn_parks, nfb7);
        if (!r5) {
            CHECK(check_preign_static() == 0, "static pre-ign content");
            // every PDU sent about as often as its period allows (bus time after the first lock)
            double cyc = (double)(t_capture_end - car[0].t) / CYC_T;
            for (int p = 0; p < DDE_DYN_N_PDU; p++) {
                double avail = cyc;
                if (dde_dyn_pdu[p].ign_only) {   // v1d: KL15 on-time of the model, minus the start delay per on edge
                    avail = 0;
                    for (unsigned i = 0; i < nks; i++) {
                        if (!kseg[i].v) continue;
                        uint64_t e = i + 1 < nks ? kseg[i + 1].t : t_capture_end;
                        double a = (double)(e - kseg[i].t) / CYC_T - dde_dyn_pdu[p].start_cyc;
                        if (a > 0) avail += a;
                    }
                }
                double want = avail > 0 ? avail / dde_dyn_pdu[p].period + 1 : 0;
                unsigned got = d.per_pdu[p];
                printf("  %s: sent %u, period allows %.1f\n", dde_dyn_pdu[p].name, got, want);
                if (!unsure && want >= 3) CHECK(got + 2 >= (unsigned)(want * 0.9), "%s sent %u of ~%.0f", dde_dyn_pdu[p].name, got, want);
                CHECK(got <= (unsigned)want + 2, "%s sent too often", dde_dyn_pdu[p].name);
            }
        } else {
            compare_real();
        }
        if (unsure) check_unsure();
        printf("%s: %s\n", sc, fails ? "FAILED" : "ok");
        return fails ? 1 : 0;
    }
#endif
    if (strncmp(sc, "run1", 4) == 0 || strcmp(sc, "lateecho") == 0) {
        run_until(tend);
        print_stats("end");
        dde_emu_stats_t s = st();
        // first transmitted cycle
        unsigned first_cyc = ntx ? txl[0].cycle : 99;
        printf("  first TX in cycle %u (car test 1: cycle 52)\n", first_cyc);
        if (strncmp(sc, "run1-v1", 7) == 0) {
            // v1 must show the car-test-1 failure: 0x3D counted as sent, never on the bus, FAULT 1
            unsigned n3d = 0;
            for (unsigned k = 0; k < ntx; k++) n3d += txl[k].id == 0x3D;
            printf("  v1: 0x3D on the bus %u times, counted %u\n", n3d, (unsigned)s.sent[3]);
            CHECK(s.fault == DDE_EMU_FAULT_TX_STUCK, "v1 replay: expected FAULT 1, got %u", s.fault);
            CHECK(s.sent[3] >= 1 && n3d == 0, "v1 replay: expected 0x3D counted but never sent");
            CHECK(ntx <= 8, "v1 replay: expected TX to stop after the fault (%u frames)", ntx);
            if (spike) {   // exactly the car-test-1 console line
                CHECK(s.sent[0] == 2 && s.sent[1] == 2 && s.sent[2] == 2 && s.sent[3] == 1 && s.late == 1 &&
                      s.echo_ok == 6 && s.echo_miss == 0 && s.foreign == 0 && s.rearms == 1 && s.lock_losses == 0,
                      "v1 + spike: counters differ from the car test 1 console");
            }
        } else if (spike) {
            // v1b: precheck 1.2 us (v1 2.5 us) absorbs the 4 us spike, 0x3D goes out
            CHECK(s.fault == 0 && s.late == 0 && s.sent[3] == s.sent[0] && s.sent[0] > 150, "v1b + spike: fault %u late %u",
                  s.fault, (unsigned)s.late);
        } else if (bigspike) {
            // v1b, 8 us spike (alarm after the TXEN target): one late with its reason, no fault, TX continues
            CHECK(s.fault == 0 && s.late == 1, "v1b + bigspike: fault %u late %u", s.fault, (unsigned)s.late);
#ifdef DDE_EMU_STATS_V1_SIZE
            CHECK(s.late_reason[DDE_EMU_LATE_SHORT - 1] == 1 && s.last_late_slot == 0x3D, "late reason not recorded");
#endif
            CHECK(s.sent[3] + 1 == s.sent[0] && s.sent[0] > 150, "0x3D not sent every other cycle");
        } else {
            CHECK(first_cyc == 52, "first TX cycle %u, car test 1 started in cycle 52", first_cyc);
            CHECK(s.fault == 0, "fault %u", s.fault);
            uint32_t n = s.sent[0];
            CHECK(n > 150, "too few cycles sent (%u)", (unsigned)n);
            if (full) {   // whole capture: a cycle can lack the references before 0x26..0x29 (bus start/stop)
                CHECK(s.sent[1] <= s.sent[3] && s.sent[2] <= s.sent[3] && s.sent[0] <= s.sent[3], "0x3D skipped");
            } else {
                CHECK(s.sent[1] == n && s.sent[2] == n && s.sent[3] == n, "slots not all sent every cycle");
            }
            CHECK(s.late == 0, "late %u", (unsigned)s.late);
            uint32_t tot = s.sent[0] + s.sent[1] + s.sent[2] + s.sent[3];
#ifdef DDE_EMU_STATS_V2_SIZE
#ifdef DDE_EMU_STATS_V3_SIZE
            for (int p = 0; p < DDE_DYN_V3_N_PDU; p++) tot += s.dyn_sent[p];   // v1c dynamic frames (0 if built off)
            for (int p = 0; p < 3; p++) tot += s.dyn_sent2[p];                 // v1d 0x10A / 0x142 / 0x68
            tot += s.sent[4];                                                  // v1d 0x5C
#else
            for (int p = 0; p < DDE_DYN_N_PDU; p++) tot += s.dyn_sent[p];   // v1c dynamic frames (0 if built off)
#endif
#endif
            CHECK(s.echo_ok == tot && s.echo_miss == 0 && s.foreign == 0, "echo bookkeeping");
            CHECK(ntx == tot && n_truncated == 0 && n_fire_without_frame == 0, "bus frames %u truncated %u", ntx, n_truncated);
            CHECK(check_bus(0, tend + 1, NULL) == 0, "frames off their slots");
            // no collision with any car frame: none of our frames overlaps another mark
            unsigned coll = 0;
            size_t ci = 0;
            for (unsigned k = 0; k < ntx; k++) {   // both lists are in time order
                while (ci < ncar && car[ci].t + 100 < txl[k].txen) ci++;
                for (size_t i = ci; i < ncar && car[i].t <= txl[k].end + 100; i++) coll++;
            }
            CHECK(coll == 0, "%u car frames overlap ours", coll);
        }
    }
#ifdef DDE_EMU_STATS_V1_SIZE   // v2 firmware only (fault policy + first-fault record)
    else if (strcmp(sc, "foreign") == 0) {
        // another DDE: one 0x3D at its slot in a cycle where we are already active, 3 times 3 s apart
        uint64_t t0 = t_capture_end;
        unsigned clears_seen = 0;
        for (int k = 0; k < 3; k++) {
            uint64_t at = t0 + (uint64_t)k * 300000000ull + 50000000ull;
            run_until(at);
            dde_emu_stats_t s = st();
            CHECK(s.fault == 0 && s.state == DDE_EMU_ST_ACTIVE, "before foreign #%d: state %u fault %u", k + 1, s.state, s.fault);
            // a 0x3D header we did not send (cycle 0, 2 ms from now, not in our slot)
            push_car(T + 200000, 0x3D, 0);
            run_until(T + 300000);
            s = st();
            CHECK(s.fault == DDE_EMU_FAULT_FOREIGN, "foreign #%d: fault %u", k + 1, s.fault);
            CHECK(s.ff_code == DDE_EMU_FAULT_FOREIGN && s.ff_slot == 0x3D, "first fault record %u/0x%02X", s.ff_code, s.ff_slot);
            run_until(T + 210000000ull);   // 2.1 s
            s = st();
            if (k < 2) {
                CHECK(s.fault == 0 && s.auto_clears == (unsigned)(k + 1), "foreign #%d: no auto re-arm (fault %u clears %u)", k + 1, s.fault, s.auto_clears);
                clears_seen = s.auto_clears;
            } else {
                CHECK(s.fault == DDE_EMU_FAULT_FOREIGN && s.latched == 1 && s.fault_count == 3, "3rd foreign: not latched");
            }
        }
        run_until(tend);
        print_stats("end");
        dde_emu_stats_t s = st();
        CHECK(s.fault == DDE_EMU_FAULT_FOREIGN && s.latched && s.auto_clears == 2 && clears_seen == 2, "end: latched foreign");
        dde_emu_host_set(0);   // host clear
        run_until(T + 200000);
        s = st();
        CHECK(s.fault == 0 && !s.latched && s.ff_code == 0, "host clear did not reset the latch");
    } else if (strcmp(sc, "stuck") == 0) {
        uint64_t t0 = t_capture_end;
        run_until(t0);
        dde_emu_stats_t s = st();
        CHECK(s.state == DDE_EMU_ST_ACTIVE && s.fault == 0, "not active before the hang");
        pio.hang_next = true;
        run_until(T + 1000000);   // 10 ms
        s = st();
        CHECK(s.fault == DDE_EMU_FAULT_TX_STUCK && s.ff_code == DDE_EMU_FAULT_TX_STUCK, "hang: fault %u", s.fault);
        CHECK(pin_fn[TXEN_FR_1_PIN] == GPIO_FUNC_SIO && pin_fn[TXD_FR_1_PIN] == GPIO_FUNC_SIO, "pins not back to SIO");
        printf("  hang: FAULT 1 slot 0x%02X detail %d us\n", s.ff_slot, (int)s.ff_detail);
        unsigned ntx_fault = ntx;
        run_until(T + 150000000ull);   // 1.5 s: still off
        CHECK(st().fault == DDE_EMU_FAULT_TX_STUCK && ntx == ntx_fault, "transmitted during the fault");
        run_until(T + 100000000ull);   // +1 s: auto re-armed, 8 quiet cycles, TX again
        s = st();
        CHECK(s.fault == 0 && s.auto_clears == 1 && s.state == DDE_EMU_ST_ACTIVE, "no auto re-arm after FAULT 1");
        CHECK(pin_fn[TXEN_FR_1_PIN] == GPIO_FUNC_PIO0 && pin_fn[TXD_FR_1_PIN] == GPIO_FUNC_PIO0, "pins not back to PIO");
        uint32_t sent_before = s.sent[3];
        run_until(tend);
        print_stats("end");
        s = st();
        // +1 when the hung frame was a 0x26 (v1d: can be a dynamic one, e.g. 0x142 -> ff_slot 0x42)
        CHECK(s.fault == 0 && s.sent[3] > sent_before + 100 && s.sent[0] >= s.sent[3] && s.sent[0] <= s.sent[3] + 1, "TX did not resume");
        CHECK(n_fire_without_frame == 0, "fire without frame");
    } else if (strcmp(sc, "noecho") == 0) {
        uint64_t t0 = t_capture_end;
        run_until(t0);
        // lose every echo of one cycle (4 misses, 1 cycle): no fault
        drop_echo_seen_cycles = 0; drop_echo_cycles_from = 1; drop_echo_cycles_n = 1;
        run_until(T + 5000000);   // 50 ms
        dde_emu_stats_t s = st();
        // 4 statics, + the v1d 0x5C / dynamic frames of that cycle when it has any
        CHECK(s.fault == 0 && s.echo_miss >= 4 && s.echo_miss <= 7, "1 cycle lost: fault %u miss %u", s.fault, (unsigned)s.echo_miss);
        // lose 3 cycles: 8 misses over 2 cycles -> FAULT 3
        drop_echo_seen_cycles = 0; drop_echo_cycles_from = 1; drop_echo_cycles_n = 3;
        run_until(T + 5000000);
        s = st();
        CHECK(s.fault == DDE_EMU_FAULT_NO_ECHO && s.ff_code == DDE_EMU_FAULT_NO_ECHO && s.ff_detail >= 8, "3 cycles lost: fault %u", s.fault);
        run_until(tend);
        print_stats("end");
        s = st();
        CHECK(s.fault == 0 && s.auto_clears == 1 && s.state == DDE_EMU_ST_ACTIVE, "no re-arm after FAULT 3");
    }
#endif
    printf("%s: %s\n", sc, fails ? "FAILED" : "ok");
    return fails ? 1 : 0;
}
