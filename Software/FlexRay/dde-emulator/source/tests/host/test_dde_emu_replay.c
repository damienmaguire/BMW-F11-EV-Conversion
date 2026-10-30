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
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "dde_emu.h"
#include "dde_emu_tables.h"
#include "flexray_ts.h"
#include "flexray_pins.h"

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

enum { EV_CAR, EV_HDR, EV_ALARM_MATCH, EV_ALARM_FORCE, EV_TXEN, EV_ECHO, EV_TXEND, EV_HOST };
typedef struct { uint64_t t; uint32_t seq; int type; uint32_t gen; uint32_t pos; uint8_t h[5]; uint8_t ours; } ev_t;
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
    bool loaded, sending, fire_busy, enabled[2];
    uint32_t gen;
    uint64_t fire_t;
    bool hang_next;          // scenario stuck: the next frame never finishes
    bool hung;
} pio;
static unsigned n_truncated, n_fire_without_frame;

void sim_pio_sm_reset(PIO p, uint sm)
{
    (void)p;
    if (sm == 0) {
        if (pio.sending) n_truncated++;
        pio.nw = 0; pio.loaded = false; pio.sending = false; pio.hung = false;
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
    if (sm == 0) return (pio.loaded || pio.sending || pio.hung) ? 3u : 0u;
    return pio.fire_busy ? 22u : 21u;   // fire program loaded after the 21-instruction TX program
}
bool sim_pio_sm_tx_fifo_empty(PIO p, uint sm) { (void)p; (void)sm; return true; }  // SM pulls at once

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
        if (pio.nw < 16) pio.words[pio.nw++] = v;
        pio.loaded = true;
        return;
    }
    // fire SM: TXEN falls r + K ticks after the counter read (= now in the sim)
    if (!pio.loaded || pio.nw < 3) { n_fire_without_frame++; return; }
    pio.fire_busy = true;
    memcpy(pio.frame, pio.words, sizeof pio.words);
    pio.nw = 0;   // the SM has pulled the whole frame
    uint64_t tf = T + v + K_TRUE;
    pio.fire_t = tf;
    ev_t e = {0};
    e.gen = pio.gen;
    e.t = tf; e.type = EV_TXEN; push(e);
    e.t = tf + frame_bits() * 10u; e.type = EV_TXEND; push(e);
}

// flexray_ts: latest mark per side
static flexray_ts_t ts_last[2]; static uint32_t ts_pos[2], ts_seq[2];
static void ts_mark(int side, uint32_t pos)
{
    ts_last[side].us = time_us_32();
    ts_last[side].fine = (uint16_t)(T + FINE_OFF);
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
typedef struct { uint64_t t; unsigned id, cycle; } car_t;
static car_t *car; static size_t ncar;
static uint32_t pos_ctr;

#define MAXTX 400000
static struct { uint64_t txen, echo, end; int64_t r07; unsigned id, cycle; } txl[MAXTX];
static unsigned ntx;
static int64_t ref07_tick[64];   // per cycle number: latest reference mark, moved to 0x07's offset (198 us)
static unsigned drop_echo_cycles_from = 999, drop_echo_cycles_n;  // scenario noecho
static unsigned drop_echo_seen_cycles;
static uint64_t echo_hdr_lat = HDR_LAT;
static unsigned last_car_cycle = 99;
static unsigned spike_us; static bool spiked;

static void handle(ev_t e)
{
    set_now(e.t);
    switch (e.type) {
    case EV_CAR: {
        uint32_t pos = (pos_ctr += 64) & RING_MASK;
        ts_mark(0, pos);
        ev_t h = e; h.type = EV_HDR; h.t = e.t + HDR_LAT; h.pos = pos; push(h);
        unsigned id = ((unsigned)(e.h[0] & 7u) << 8) | e.h[1], cyc = e.h[4] & 63u;
        int ri = dde_core_ref_index(id);   // cycle start estimate from any reference of this cycle
        if (ri >= 0) ref07_tick[cyc] = (int64_t)e.t - (int64_t)(dde_emu_ref_off_q8[ri] - dde_emu_ref_off_q8[0]) / 256;
        if (cyc != last_car_cycle) {
            last_car_cycle = cyc;
            if (drop_echo_cycles_from != 999 && drop_echo_seen_cycles <= drop_echo_cycles_from + drop_echo_cycles_n) drop_echo_seen_cycles++;
        }
        break; }
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
        ts_mark(1, 0);
        const uint32_t *w = pio.frame;
        uint8_t hb[5] = {(uint8_t)(w[2] >> 24), (uint8_t)(w[2] >> 16), (uint8_t)(w[2] >> 8), (uint8_t)w[2],
                         (uint8_t)(w[3] >> 24)};
        unsigned id = ((unsigned)(hb[0] & 7u) << 8) | hb[1], cyc = hb[4] & 63u;
        if (ntx < MAXTX) { txl[ntx].txen = e.t; txl[ntx].end = e.t + frame_bits() * 10u; txl[ntx].id = id; txl[ntx].cycle = cyc; txl[ntx].echo = 0; txl[ntx].r07 = ref07_tick[cyc]; ntx++; }
        bool drop = drop_echo_cycles_from != 999 && drop_echo_seen_cycles > drop_echo_cycles_from &&
                    drop_echo_seen_cycles <= drop_echo_cycles_from + drop_echo_cycles_n;
        if (!drop) {
            ev_t x = {0}; x.t = e.t + LOOP_TRUE; x.type = EV_ECHO; x.gen = pio.gen; x.pos = ntx; memcpy(x.h, hb, 5); push(x);
        }
        if (pio.hang_next) { pio.hang_next = false; pio.hung = true; }
        break; }
    case EV_ECHO: {
        uint32_t pos = (pos_ctr += 64) & RING_MASK;
        ts_mark(0, pos);
        if (e.pos >= 1 && e.pos <= MAXTX) txl[e.pos - 1].echo = e.t;
        ev_t h = e; h.type = EV_HDR; h.t = e.t + echo_hdr_lat; h.pos = pos; h.ours = 1; push(h);
        break; }
    case EV_TXEND:
        if (e.gen != pio.gen) return;
        pio.sending = false;   // a hung SM keeps pio.hung (not idle) until reset
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
        unsigned long long t; unsigned id, cyc, len, fl;
        if (sscanf(line, "%llu %u %u %u %u", &t, &id, &cyc, &len, &fl) != 5) continue;
        if (ncar == cap) { cap = cap ? cap * 2 : 8192; car = realloc(car, cap * sizeof(car_t)); }
        car[ncar].t = t + 100000ull;   // 1 ms margin before the first frame
        car[ncar].id = id; car[ncar].cycle = cyc; ncar++;
    }
    fclose(f);
}

static void push_car(uint64_t t, unsigned id, unsigned cyc)
{
    ev_t e = {0}; e.t = t; e.type = EV_CAR;
    dde_core_header(0, id, 8, cyc, e.h);
    push(e);
}

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
        for (size_t i = (size_t)a; i < (size_t)b; i++) push_car(car[i].t + period * k, car[i].id, car[i].cycle);
    *tend = car[a].t + period * (n + 1);
    return (size_t)b;   // original frames [0, b) are replayed as captured
}

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void run_until(uint64_t tend)
{
    while (nheap && heap[0].t <= tend) handle(pop());
}

static dde_emu_stats_t st(void) { dde_emu_stats_t s; dde_emu_fill_stats(&s); return s; }

// Timing / completeness of what went on the bus from cycle index c_from (absolute tick) on.
static unsigned check_bus(uint64_t from, uint64_t to, unsigned *cycles_out)
{
    static const unsigned ids[4] = {0x26, 0x28, 0x29, 0x3D};
    double maxerr = 0;
    unsigned bad = 0, cyc_seen = 0;
    // count cycles (0x07 marks) in [from, to)
    for (unsigned k = 0; k < ntx; k++) {
        if (txl[k].txen < from || txl[k].txen >= to) continue;
        int ti = -1;
        for (int j = 0; j < 4; j++) if (ids[j] == txl[k].id) ti = j;
        if (ti < 0) { bad++; continue; }
        int64_t r07 = txl[k].r07;   // this cycle's 0x07 mark
        double cs = (double)r07 - dde_emu_ref_off_q8[0] / 256.0;
        double err = ((double)txl[k].echo - cs - dde_emu_static_off_q8[ti] / 256.0) / 100.0;   // us
        if (!txl[k].echo) { bad++; continue; }
        if (err < 0) err = -err;
        if (err > maxerr) maxerr = err;
        if (err > 0.3) { if (bad < 5) printf("    off: id 0x%02X cycle %u err %.3f r07=%lld echo=%llu\n", txl[k].id, txl[k].cycle, err, (long long)r07, (unsigned long long)txl[k].echo); bad++; }
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
#endif
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s VECTORS [run1|run1-v1|foreign|stuck|noecho|lateecho]\n", argv[0]); return 2; }
    const char *sc = argc > 2 ? argv[2] : "run1";
    load(argv[1]);
    uint64_t tend = car[ncar - 1].t + 100000;   // +1 ms: late echo headers still arrive
    size_t nplay = ncar;
    bool longrun = strcmp(sc, "foreign") == 0 || strcmp(sc, "stuck") == 0 || strcmp(sc, "noecho") == 0;
    if (longrun) nplay = extend(strcmp(sc, "foreign") == 0 ? 40 : 25, &tend);
    for (size_t i = 0; i < nplay; i++) push_car(car[i].t, car[i].id, car[i].cycle);
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
        CHECK(s.fault == 0 && s.sent[3] > sent_before + 100 && s.sent[0] == s.sent[3] + 1, "TX did not resume");   // +1: the hung 0x26
        CHECK(n_fire_without_frame == 0, "fire without frame");
    } else if (strcmp(sc, "noecho") == 0) {
        uint64_t t0 = t_capture_end;
        run_until(t0);
        // lose every echo of one cycle (4 misses, 1 cycle): no fault
        drop_echo_seen_cycles = 0; drop_echo_cycles_from = 1; drop_echo_cycles_n = 1;
        run_until(T + 5000000);   // 50 ms
        dde_emu_stats_t s = st();
        CHECK(s.fault == 0 && s.echo_miss == 4, "1 cycle lost: fault %u miss %u", s.fault, (unsigned)s.echo_miss);
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
