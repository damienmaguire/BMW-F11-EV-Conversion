// v1g.2 host simulation: how often does the PT-CAN TX code talk to the CAN chip?
//
// In plain words
// --------------
// This links the REAL src/ptcan_emu.c (the code that hands our frames to the CAN chip
// and reads back the chip's "sent" list, the TEF) with a pretend CAN chip and a
// pretend clock. Every conversation with the chip over the SPI wires costs main-loop
// time (2 us per byte at 4 MHz), so the pretend clock moves on by that much. Then it
// counts: how much time went into reading the TEF, how many main-loop passes there
// were per second (each pass is a chance to move FlexRay frames to USB), and whether
// every frame we sent was reported to the laptop exactly once with the exact time.
//
//   test_ptcan_tef SCENARIO [seconds]
//     log-on    host asked for own-TX records (logger v1f+): every sent frame logged
//               once, exact SOF time, nothing lost, TEF looked at <= 1000 times/s
//     log-off   host did not ask (old logger): the TEF is never read
//     toggle    off for 3 s (TEF fills up and overflows), then on: old entries thrown
//               away quietly, everything after that logged, nothing counted lost
//     noack     nobody acknowledges our frames for 2 s (chip gives up): no endless
//               polling afterwards
//   Add "report" as 3rd argument to only print the numbers (used for the old code).
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/time.h"
#include "ptcan_emu.h"
#include "panda_usb.h"
#include "dde_emu.h"
#include "console.h"

uint64_t sim_now_us;

// ---- pretend CAN chip --------------------------------------------------------------
#define US_PER_BYTE 2u     // 4 MHz SPI
#define SPI_CALL_US 2u     // chip select guards + call overhead
#define FRAME_US 250u      // one 8-byte classic frame at 500 kbit/s (with stuffing), about
static struct { uint32_t seq; uint16_t id; uint8_t dlc; } txq[16];
static int txq_n;
static uint64_t bus_free_at;     // when the frame now on the wire ends
static struct { uint32_t seq, id, ts; uint8_t dlc; } tef[16];
static int tef_n;
static bool tef_ovf, acks = true;
static uint32_t seq_next;
static bool normal;
// what the code under test did
static uint64_t spi_us_tef, spi_us_all;
static uint32_t tef_status_reads, tef_looks, frames_sent, frames_given_up;
static uint64_t cur_pass, last_read_pass = ~0ull;   // a "look" = TEF reads within one main-loop pass
static struct { uint32_t seq; uint32_t sof; uint16_t id; } sent_log[200000];

static void spi_cost(unsigned bytes, bool is_tef)
{
    unsigned us = bytes * US_PER_BYTE + SPI_CALL_US;
    sim_now_us += us;
    spi_us_all += us;
    if (is_tef) spi_us_tef += us;
}

// Let the pretend bus send whatever is due up to 'now'.
static void chip_run(uint64_t now)
{
    while (txq_n > 0) {
        uint64_t start = bus_free_at > 0 ? bus_free_at : now;
        if (start > now) break;
        if (start + FRAME_US > now) break;   // still on the wire
        if (!acks) {                         // 3 attempts, then the chip gives up
            frames_given_up++;
        } else {
            frames_sent++;
            if (frames_sent <= sizeof sent_log / sizeof sent_log[0]) {
                sent_log[frames_sent - 1].seq = txq[0].seq;
                sent_log[frames_sent - 1].sof = (uint32_t)start;
                sent_log[frames_sent - 1].id = txq[0].id;
            }
            if (tef_n < 16) {
                tef[tef_n].seq = txq[0].seq; tef[tef_n].id = txq[0].id;
                tef[tef_n].ts = (uint32_t)start; tef[tef_n].dlc = txq[0].dlc;
                tef_n++;
            } else tef_ovf = true;           // full: the note is lost, the frame was sent
        }
        memmove(&txq[0], &txq[1], sizeof txq[0] * (size_t)(txq_n - 1));
        txq_n--;
        bus_free_at = start + FRAME_US * (acks ? 1u : 3u);
        if (txq_n == 0) bus_free_at = 0;
    }
}

mcp_status_t mcp251863_set_normal(bool n)
{
    normal = n;
    txq_n = 0; tef_n = 0; tef_ovf = false; bus_free_at = 0;   // Configuration mode resets both
    return MCP_OK;
}
uint32_t mcp251863_next_seq(void) { return seq_next & 0x7FFFFFu; }
mcp_status_t mcp251863_send_std(uint16_t id, const uint8_t *d, uint8_t dlc)
{
    (void)d;
    spi_cost(10 + 18 + 3, false);           // FIFO status/UA, object, UINC|TXREQ
    chip_run(sim_now_us);
    if (txq_n >= 16) return MCP_ERR_TX_FULL;
    txq[txq_n].seq = seq_next & 0x7FFFFFu; txq[txq_n].id = id; txq[txq_n].dlc = dlc;
    if (txq_n == 0 && bus_free_at == 0) bus_free_at = sim_now_us;
    txq_n++;
    seq_next++;
    return MCP_OK;
}
int mcp251863_tef_read(mcp_tef_t *e, bool *overflow)
{
    chip_run(sim_now_us);
    spi_cost(10, true);
    tef_status_reads++;
    if (last_read_pass != cur_pass) { tef_looks++; last_read_pass = cur_pass; }
    if (tef_ovf) { if (overflow) *overflow = true; tef_ovf = false; spi_cost(3, true); }
    if (tef_n == 0) return 0;
    spi_cost(14, true);
    e->id = tef[0].id; e->seq = tef[0].seq; e->dlc = tef[0].dlc; e->ts = tef[0].ts;
    memmove(&tef[0], &tef[1], sizeof tef[0] * (size_t)(tef_n - 1));
    tef_n--;
    spi_cost(3, true);
    return 1;
}
uint32_t mcp251863_read_tbc(void) { spi_cost(6, true); return (uint32_t)sim_now_us; }
mcp_status_t mcp251863_read_tx_state_fifo(uint8_t fifo, mcp_tx_state_t *st)
{
    (void)fifo;
    spi_cost(30, false);
    memset(st, 0, sizeof *st);
    st->opmod = normal ? 6 : 3;
    return MCP_OK;
}
void mcp251863_clear_tx_attempt_flag_fifo(uint8_t fifo) { (void)fifo; }
const char *mcp251863_strerror(mcp_status_t s) { (void)s; return "?"; }

// ---- pretend USB / engine / console -------------------------------------------------
static bool host_wants_tx;
static uint32_t rec_pushed, rec_bad_time, rec_unknown;
static uint32_t rec_seen_idx;
bool panda_can_tx_records_enabled(void) { return host_wants_tx; }
bool panda_can_record_push(const pico_can_record_t *r)
{
    rec_pushed++;
    // records arrive in send order: match against the pretend bus log
    while (rec_seen_idx < frames_sent && sent_log[rec_seen_idx].id != r->id) rec_seen_idx++;
    if (rec_seen_idx >= frames_sent) { rec_unknown++; return true; }
    if (sent_log[rec_seen_idx].sof != r->time_us) rec_bad_time++;
    rec_seen_idx++;
    return true;
}
void dde_engine_on_can(uint16_t id, const uint8_t *d, uint8_t len) { (void)id; (void)d; (void)len; }
bool dde_engine_snapshot(eng_out_t *out) { (void)out; return false; }
uint32_t console_dropped(void) { return 0; }

// ---- main loop model ----------------------------------------------------------------
#define OTHER_WORK_US 10u   // USB task + CAN RX service + FlexRay hand-off per pass, about

typedef struct { uint64_t passes; uint64_t spi_tef, spi_all; uint32_t looks; } window_t;

static void feed_rx(uint64_t t)
{
    mcp_rx_frame_t f;
    memset(&f, 0, sizeof f);
    f.len = f.dlc = 8;
    if (t % 10000 == 0) { f.id = 0x1A1; ptcan_emu_on_rx(&f, (uint32_t)t); }
    if (t % 100000 == 0) { f.id = 0x12F; f.data[2] = 0xA; ptcan_emu_on_rx(&f, (uint32_t)t); }
    if (t % 640000 == 0) { f.id = 0x510; ptcan_emu_on_rx(&f, (uint32_t)t); }
}

// Run the main loop from now until 'until'; returns passes.
static uint64_t run(uint64_t until)
{
    static uint64_t next_rx_ms, next_tick, next_keep;
    uint64_t passes = 0;
    while (sim_now_us < until) {
        // bus traffic from the car, once per ms granularity
        while (next_rx_ms * 1000u <= sim_now_us) { feed_rx(next_rx_ms * 1000u); next_rx_ms++; }
        if (sim_now_us >= next_keep) { ptcan_emu_host_set(0x7FFFFu); next_keep = sim_now_us + 2000000u; }
        if (!ptcan_emu_poll()) { printf("poll failed\n"); exit(1); }
        if (sim_now_us >= next_tick) { ptcan_emu_tick(); next_tick = sim_now_us + 50000u; }
        chip_run(sim_now_us);
        sim_now_us += OTHER_WORK_US;
        passes++;
        cur_pass++;
    }
    return passes;
}

int main(int argc, char **argv)
{
    const char *sc = argc > 1 ? argv[1] : "log-on";
    double secs = argc > 2 ? atof(argv[2]) : 20.0;
    bool report_only = argc > 3 && strcmp(argv[3], "report") == 0;
    int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
    // keep the code's own console prints out of the way
    if (!freopen("/dev/null", "w", stdout)) return 2;
    FILE *out = stderr;

    sim_now_us = 1000000;
    ptcan_emu_on_can_init(true);
    host_wants_tx = strcmp(sc, "log-off") != 0 && strcmp(sc, "toggle") != 0;
    uint64_t t0 = sim_now_us, end = t0 + (uint64_t)(secs * 1e6);
    uint64_t passes = 0;
    if (!strcmp(sc, "toggle")) {
        passes += run(t0 + 3000000u);
        host_wants_tx = true;
        rec_seen_idx = frames_sent;          // frames sent before logging was on are not expected
        passes += run(end);
    } else if (!strcmp(sc, "noack")) {
        passes += run(t0 + 2000000u);
        acks = false;
        passes += run(t0 + 4000000u);
        acks = true;
        uint32_t reads_before = tef_looks;
        uint64_t p2 = run(end);
        passes += p2;
        double s2 = (double)(end - (t0 + 4000000u)) / 1e6;
        fprintf(out, "  after the no-ACK phase: %.0f TEF looks/s\n", (tef_looks - reads_before) / s2);
    } else {
        passes = run(end);
    }
    double s = (double)(sim_now_us - t0) / 1e6;
    uint8_t buf[256];
    ptcan_stats_t st;
    memset(&st, 0, sizeof st);
    uint16_t n = ptcan_emu_get_stats(buf, sizeof buf);
    memcpy(&st, buf, n < sizeof st ? n : sizeof st);
    fprintf(out, "%s %.0f s: %" PRIu32 " frames sent (%.0f/s), main-loop passes %.0f/s, TEF SPI %.2f%% of the time "
            "(%.0f looks/s, %.0f status reads/s), all SPI %.2f%%, records %" PRIu32 " (bad time %" PRIu32 ", unknown %" PRIu32
            "), tx_logged %" PRIu32 " tx_log_lost %" PRIu32 "\n",
            sc, s, frames_sent, frames_sent / s, passes / s, 100.0 * (double)spi_us_tef / (s * 1e6),
            tef_looks / s, tef_status_reads / s, 100.0 * (double)spi_us_all / (s * 1e6), rec_pushed, rec_bad_time, rec_unknown,
            st.tx_logged, st.tx_log_lost);
    if (!report_only) {
        CHECK(frames_sent > 200 * s, "too few frames sent (%u)", frames_sent);
        if (!strcmp(sc, "log-off")) {
            CHECK(tef_status_reads == 0, "TEF read %u times with logging off", tef_status_reads);
            CHECK(rec_pushed == 0, "records pushed with logging off");
        } else {
            CHECK(tef_looks <= 1000 * s + 20, "TEF looked at %.0f times/s (> 1000)", tef_looks / s);
            CHECK(rec_bad_time == 0 && rec_unknown == 0, "records with wrong time / unknown frames");
            CHECK(st.tx_log_lost == 0, "tx_log_lost %u", st.tx_log_lost);
            if (strcmp(sc, "toggle") != 0)
                CHECK(st.tx_logged + 16 >= frames_sent, "logged %u of %u sent", st.tx_logged, frames_sent);
            else
                CHECK(st.tx_logged > 200 * (s - 3.5), "logged only %u after logging went on", st.tx_logged);
        }
        CHECK(100.0 * (double)spi_us_tef / (s * 1e6) < 3.0, "TEF SPI time too high");
        fprintf(out, "%s: %s\n", sc, fails ? "FAIL" : "PASS");
    }
    return fails ? 1 : 0;
}
