// PT-CAN DDE/EGS emulation TX glue, see ptcan_emu.h and ptcan_emu_core.h.
//
// In plain words
// --------------
// This is the part that switches the V2 board's CAN chip (MCP251863) between
// "listen only" (it can hear the car but cannot send, acknowledge or disturb anything)
// and "normal" (it can send), and hands the frames chosen by ptcan_emu_core.c to it.
// Listen only is the default after power-up. It changes to normal ONLY while the
// laptop logger is running with --can-tx / --can-tx-ids and keeps saying so (every
// 2 s; 20 s without that = off). Any problem drops it straight back to listen only
// and it stays there until the logger is restarted.
//
// Safety model
// - The MCP251863 boots in Listen Only (v2_cantest.c). Only while the host has CAN TX
//   enabled (and the keepalive is fresh, and no fault is latched) is it switched to
//   Normal CAN 2.0. In Normal mode it ACKs frames of other nodes; it cannot start bus
//   traffic by itself except by transmitting, and transmitting is gated by the core
//   on PT-CAN traffic seen in the last 200 ms.
// - Disable, keepalive timeout, bus-off, TX error passive, a foreign sender of one of
//   our IDs, or a mode error -> back to Listen Only at once (TX FIFO aborted by the
//   Configuration-mode transition). Faults stay latched until the host sends mask 0.
// - The forbidden-ID list is checked again right before each frame is queued.
#include "ptcan_emu.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"
#include "pico/time.h"
#include "ptcan_emu_core.h"
#include "dde_emu.h"
#include "console.h"
#include "panda_usb.h"
#include "pico_can_record.h"
#include "board_pico_flexray_v2.h"

// v1f: frames handed to the controller per main-loop pass. 16 = the TX FIFO depth, so
// after a hiccup every overdue frame can be queued at once (v1e: 4).
#define PTCAN_BURST 16

// v1f: own-TX logging. Every queued frame is remembered here by its controller sequence
// number; when the controller reports it sent (TX event FIFO, with the time of the start
// of frame) the frame is pushed to the host as a CAN record flagged TX.
#define TXLOG_N 64
static struct {
    uint32_t seq;        // the chip's number for this frame
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool used;           // handed to the chip, not reported back yet
    uint64_t queued_us;  // v1g.2: when it was handed to the chip
} txlog[TXLOG_N];
static uint32_t tx_logged, tx_log_lost;
static uint32_t loop_gap_max_us, loop_gaps_20ms, burst_max;
static uint64_t last_pass_us;

// v1g.2: own-TX logging - when to read the chip's "sent" list (explained at tef_service())
#define TEF_MIN_INTERVAL_US 1000u    // look at most once per millisecond
#define TEF_GIVE_UP_US 50000u        // a frame not reported 50 ms after it was queued never will
                                     // be (e.g. the chip gave up because nobody acknowledged it)
static uint32_t tef_unconfirmed;     // frames handed to the chip that it has not reported yet
static uint64_t tef_last_look_us;    // when we last looked at the list
static bool tef_logging;             // did the laptop want our own frames at the last pass?
static uint32_t tef_looks;           // how often we looked (shown in the 5 s status line)


_Static_assert(sizeof(ptcan_stats_t) == 214, "ptcan_stats_t layout (v2)");
_Static_assert(offsetof(ptcan_stats_t, loop_gap_max_us) == 178, "v2 tail starts after the v1 layout");
_Static_assert(PTE_N_FRAMES == 19, "ptcan_stats_t has 19 slots");

static pte_t core;
static bool can_ok;            // controller initialised
static bool normal_mode;       // controller currently in Normal mode (we put it there)
static bool keepalive_expired;
static uint32_t host_last_ms;
static volatile bool host_pending;
static volatile uint32_t host_mask;
static uint32_t fifo_full, tx_fail, mode_switches;
static mcp_tx_state_t last_state;
static repeating_timer_t ms_timer;
static bool ms_timer_on;
static pte_state_t last_logged_state = PTE_ST_OFF;

static uint64_t to64(uint32_t t32)
{
    uint64_t now64 = time_us_64();
    uint32_t now32 = (uint32_t)now64;
    return now64 - (uint32_t)(now32 - t32);
}

static bool ms_cb(repeating_timer_t *t)
{
    (void)t;
    __sev(); // wake the main loop so frames go out on time (main loop sleeps in __wfe)
    return true;
}

static bool set_mode(bool normal)
{
    mcp_status_t r = mcp251863_set_normal(normal);
    mode_switches++;
    // Every mode change passes through Configuration mode, which empties the chip's
    // send queue and its "sent" list: nothing is outstanding any more.
    tef_unconfirmed = 0;
    if (r == MCP_OK) {
        normal_mode = normal;
        printf("[CANTX] controller -> %s\n", normal ? "NORMAL (TX enabled, ACKs frames)" : "LISTEN-ONLY");
        return true;
    }
    printf("[CANTX] mode change to %s failed: %s\n", normal ? "NORMAL" : "LISTEN-ONLY", mcp251863_strerror(r));
    if (normal) {
        pte_fault(&core, PTE_FAULT_MODE, 0);
        (void)mcp251863_set_normal(false);
        normal_mode = false;
    }
    return r != MCP_ERR_SPI;
}

void ptcan_emu_on_can_init(bool ok)
{
    can_ok = ok;
    normal_mode = false; // init always leaves the controller in Listen Only
    if (!ms_timer_on) ms_timer_on = add_repeating_timer_ms(-1, ms_cb, NULL, &ms_timer);
    static bool inited;
    if (!inited) { pte_init(&core); inited = true; }
}

void ptcan_emu_on_rx(const mcp_rx_frame_t *f, uint32_t t_us32)
{
    // Extended / remote frames only count as bus traffic.
    if (f->ide || f->rtr || f->fdf) pte_on_rx(&core, 0, NULL, 0, to64(t_us32));
    else {
        pte_on_rx(&core, (uint16_t)f->id, f->data, f->len, to64(t_us32));
        dde_engine_on_can((uint16_t)f->id, f->data, f->len);   // v1g: ST_KL / brake for the fake start
    }
}

void ptcan_emu_host_set(uint32_t mask)
{
    host_mask = mask;
    host_pending = true;
    __sev();
}

uint8_t ptcan_emu_fault(void) { return (uint8_t)core.fault; }

static void handle_host(void)
{
    if (!host_pending) return;
    host_pending = false;
    uint32_t m = host_mask;
    host_last_ms = to_ms_since_boot(get_absolute_time());
    if (m == 0) {
        if (core.enabled || core.fault) printf("[CANTX] disabled by host%s\n", core.fault ? " (fault cleared)" : "");
        pte_set_mask(&core, 0);
        keepalive_expired = false;
        return;
    }
    if (m != core.mask) {
        uint32_t used = pte_set_mask(&core, m);
        printf("[CANTX] host mask 0x%05lx -> in use 0x%05lx\n", (unsigned long)m, (unsigned long)used);
    }
}

// ---------------------------------------------------------------------------------
// Own-TX logging: reading the chip's "sent" list (TX event FIFO, TEF)
// ---------------------------------------------------------------------------------
// In plain words
// --------------
// After the CAN chip has sent one of our frames (and another node acknowledged it), it
// writes a short note into a small list inside the chip: which frame, and the exact
// time it went out. Reading that list is how the laptop gets our own frames in its log
// (CSV dir=Tx), with the same time base as the frames we hear.
//
// Every look at that list is a conversation with the chip over the SPI wires, and the
// main loop waits for it (about 25 us for "anything new?", about 60 us per note read).
// v1f/v1g looked on EVERY pass of the main loop - many thousand times a second, almost
// always to hear "nothing new". That ate the time the main loop needs to hand FlexRay
// frames to USB, and in run 7 frames were dropped before they reached the laptop
// (usb_fifo_dropped). v1g.2 only looks when it is worth it:
//   1. Only if the laptop asked for our own frames (logger v1f or newer). If it did not,
//      the list is not read at all. When the list is full the chip simply forgets the
//      newest notes (it still SENDS every frame - Microchip reference manual DS20005678B,
//      section 8, figure 8-27: "the ID is discarded, and TEFOVIF is set").
//   2. Only if we handed the chip a frame that it has not reported back yet.
//   3. At most once per millisecond. The list holds 16 notes. We send about 280 frames
//      a second (usually 1-4 at a time); even the largest burst (16 frames after a
//      hiccup) takes the bus about 4 ms to send, so the list does not fill up in between.
// The time stamps are taken by the chip when the frame starts on the wire, so looking
// up to 1 ms later does not make them any less exact.
// (settings and counters: see "Own-TX logging" near the top of this file)

// Read up to max notes from the chip's list and turn them into TX records for the laptop.
// Returns how many notes were read (0 = list empty), or -1 if the SPI talk failed.
// count_losses = false is used once when logging starts, to throw old notes away quietly.
static int drain_tef(int max, bool count_losses)
{
    bool have_ref = false;
    uint32_t tbc_ref = 0, us_ref = 0;
    int got = 0;
    for (int n = 0; n < max; n++) {
        mcp_tef_t e;
        bool ovf = false;
        int r = mcp251863_tef_read(&e, &ovf);
        if (ovf && count_losses) tx_log_lost++;      // the list was full: some notes are gone
        if (r < 0) return -1;
        if (r == 0) break;                           // list empty
        got++;
        unsigned k = e.seq % TXLOG_N;
        if (!txlog[k].used || txlog[k].seq != e.seq || txlog[k].id != e.id) {
            if (count_losses) tx_log_lost++;         // a note we have no copy of the frame for
            continue;
        }
        txlog[k].used = false;
        if (!count_losses) continue;
        // The chip counts time with its own clock: read it once and convert to pico time.
        if (!have_ref) { tbc_ref = mcp251863_read_tbc(); us_ref = time_us_32(); have_ref = true; }
        pico_can_record_t rec;
        rec.bus = V2_CAN_USB_BUS;
        rec.flags = PICO_CAN_FLAG_TX;
        rec.dlc = txlog[k].dlc;
        rec.len = txlog[k].dlc;
        rec.id = e.id;
        rec.time_us = us_ref + (uint32_t)(int32_t)(e.ts - tbc_ref);
        memcpy(rec.data, txlog[k].data, rec.len);
        if (panda_can_record_push(&rec)) tx_logged++;
        else tx_log_lost++;
    }
    return got;
}

// Called once per main-loop pass while the controller is in Normal mode. Usually
// returns at once without touching the SPI wires. false = SPI failure.
static bool tef_service(uint64_t now)
{
    // 1. Laptop did not ask for our own frames: never read the list.
    if (!panda_can_tx_records_enabled()) {
        tef_logging = false;
        return true;
    }
    if (!tef_logging) {
        // Logging was just switched on. The list may still hold old notes (or be full)
        // from before: throw them away once, without counting them as lost.
        tef_logging = true;
        for (unsigned k = 0; k < TXLOG_N; k++) txlog[k].used = false;
        if (drain_tef(16, false) < 0) return false;
        tef_unconfirmed = 0;
        tef_last_look_us = now;
        return true;
    }
    // 2. Nothing sent that we have not heard back about: nothing to read.
    if (tef_unconfirmed == 0) return true;
    // 3. Looked less than 1 ms ago: wait.
    if (now - tef_last_look_us < TEF_MIN_INTERVAL_US) return true;
    tef_last_look_us = now;
    tef_looks++;
    // Read exactly as many notes as we expect (max 16 = list size), so the usual case
    // costs no extra "anything else?" question.
    int want = tef_unconfirmed > 16u ? 16 : (int)tef_unconfirmed;
    int got = drain_tef(want, true);
    if (got < 0) return false;
    tef_unconfirmed = (uint32_t)got >= tef_unconfirmed ? 0u : tef_unconfirmed - (uint32_t)got;
    if (got < want) {
        // The list ran dry before we got all the reports we expected. Count again what
        // is really still outstanding: frames queued in the last 50 ms. Older ones were
        // never sent (the chip gave up after its retries) and will never be reported.
        uint32_t still = 0;
        for (unsigned k = 0; k < TXLOG_N; k++) {
            if (!txlog[k].used) continue;
            if (now - txlog[k].queued_us > TEF_GIVE_UP_US) txlog[k].used = false;
            else still++;
        }
        tef_unconfirmed = still;
    }
    return true;
}

bool ptcan_emu_poll(void)
{
    if (!can_ok) return true;
    handle_host();
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (core.enabled && (uint32_t)(now_ms - host_last_ms) > PTCAN_EMU_KEEPALIVE_MS) {
        printf("[CANTX] host keepalive lost (> %u ms): TX off\n", (unsigned)PTCAN_EMU_KEEPALIVE_MS);
        pte_set_mask(&core, 0);
        keepalive_expired = true;
    }
    bool want_normal = core.enabled && core.fault == PTE_FAULT_NONE;
    if (want_normal != normal_mode) {
        if (!want_normal && core.fault)
            printf("[CANTX] FAULT %s (ID 0x%03x): TX stopped, back to Listen Only\n", pte_fault_name(core.fault),
                   core.fault_id);
        if (!set_mode(want_normal)) return false;
        want_normal = normal_mode;
    }
    uint64_t now = time_us_64();
    pte_state_t s = pte_state(&core, now);
    if (s != last_logged_state) {
        printf("[CANTX] %s -> %s\n", pte_state_name(last_logged_state), pte_state_name(s));
        last_logged_state = s;
    }
    if (!normal_mode) { last_pass_us = 0; return true; }
    // v1f: how regularly does the main loop get here? (run 6: 120-160 ms stalls every 5 s)
    if (last_pass_us && now > last_pass_us) {
        uint64_t gap = now - last_pass_us;
        if (gap > loop_gap_max_us) loop_gap_max_us = gap > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)gap;
        if (gap > 20000u) loop_gaps_20ms++;
    }
    last_pass_us = now;
    if (!tef_service(now)) return false;   // v1g.2: cheap; usually no SPI at all
    int n = 0;
    // v1g: fresh fake-engine snapshot from core 1 (NULL when off / FlexRay not stepping)
    static eng_out_t eng_snap;
    core.eng = dde_engine_snapshot(&eng_snap) ? &eng_snap : NULL;
    for (; n < PTCAN_BURST; n++) {
        pte_tx_t tx;
        if (!pte_next(&core, now, &tx)) break;
        if (pte_id_forbidden(tx.id)) { // belt and braces: the core never schedules these
            core.refused++;
            pte_fault(&core, PTE_FAULT_MODE, tx.id);
            break;
        }
        uint32_t seq = mcp251863_next_seq();
        mcp_status_t r = mcp251863_send_std(tx.id, tx.data, tx.dlc);
        if (r == MCP_ERR_TX_FULL) { fifo_full++; break; }
        if (r != MCP_OK) return false;
        unsigned k = seq % TXLOG_N;
        txlog[k].seq = seq;
        txlog[k].id = tx.id;
        txlog[k].dlc = tx.dlc;
        memcpy(txlog[k].data, tx.data, 8);
        txlog[k].used = true;
        txlog[k].queued_us = now;
        tef_unconfirmed++;        // the chip will report this one back once it is sent
        pte_commit(&core, &tx, now);
    }
    if ((uint32_t)n > burst_max) burst_max = (uint32_t)n;
    return true;
}

bool ptcan_emu_tick(void)
{
    if (!can_ok || !normal_mode) return true;
    if (mcp251863_read_tx_state_fifo(MCP_FIFO_CLASSIC, &last_state) != MCP_OK) {
        pte_fault(&core, PTE_FAULT_SPI, 0);
        return false;
    }
    if (last_state.tx_attempts_exhausted) {
        tx_fail++;
        mcp251863_clear_tx_attempt_flag_fifo(MCP_FIFO_CLASSIC);
    }
    if (last_state.bus_off) pte_fault(&core, PTE_FAULT_BUS_OFF, 0);
    else if (last_state.tx_passive) pte_fault(&core, PTE_FAULT_TX_PASSIVE, 0);
    if (core.fault != PTE_FAULT_NONE) {
        printf("[CANTX] FAULT %s (TEC=%u REC=%u): TX stopped, back to Listen Only\n",
               pte_fault_name(core.fault), last_state.tec, last_state.rec);
        return set_mode(false);
    }
    return true;
}

static int32_t ms_since(bool valid, uint64_t t, uint64_t now)
{
    if (!valid || now < t) return -1;
    uint64_t d = (now - t) / 1000u;
    return d > 0x7FFFFFFF ? 0x7FFFFFFF : (int32_t)d;
}

uint16_t ptcan_emu_get_stats(uint8_t *buf, uint16_t max)
{
    ptcan_stats_t s;
    memset(&s, 0, sizeof s);
    uint64_t now = time_us_64();
    s.version = PTCAN_STATS_VERSION;
    s.state = (uint8_t)pte_state(&core, now);
    s.fault = (uint8_t)core.fault;
    s.flags = (uint8_t)((core.enabled ? 1 : 0) | (normal_mode ? 2 : 0) | (core.kl15 ? 4 : 0) |
                        (core.kl15_ever ? 8 : 0) | (last_state.bus_off ? 16 : 0) | (last_state.tx_passive ? 32 : 0) |
                        (last_state.rx_passive ? 64 : 0) | (keepalive_expired ? 128 : 0));
    s.mask = core.mask;
    s.fault_id = core.fault_id;
    s.st_kl = core.st_kl;
    s.n_frames = PTE_N_FRAMES;
    s.tec = last_state.tec;
    s.rec = last_state.rec;
    s.opmod = last_state.opmod;
    s.rx_frames = core.rx_frames;
    s.n12f = core.n12f;
    s.wakes = core.wakes;
    s.ms_since_wake = ms_since(core.have_rx, core.wake_us, now);
    s.ms_since_rx = ms_since(core.have_rx, core.last_rx_us, now);
    s.ms_since_on = ms_since(core.kl15_on_edges != 0, core.on_us, now);
    s.ms_since_off = ms_since(core.kl15_off_edges != 0, core.off_us, now);
    s.ms_since_foreign_nm = ms_since(core.last_fnm_us != 0, core.last_fnm_us, now);
    s.tx_total = core.tx_total;
    s.tx_fifo_full = fifo_full;
    s.tx_fail = tx_fail;
    s.mode_switches = mode_switches;
    for (int i = 0; i < PTE_N_FRAMES; i++) {
        s.id[i] = pte_defs[i].id;
        s.tx[i] = core.f[i].tx;
    }
    s.loop_gap_max_us = loop_gap_max_us;
    s.loop_gaps_20ms = loop_gaps_20ms;
    s.late_max_us = core.late_max_us;
    s.late_sends = core.late_sends;
    s.resyncs = core.resyncs;
    s.tx_logged = tx_logged;
    s.tx_log_lost = tx_log_lost;
    s.console_dropped = console_dropped();
    s.burst_max = (uint16_t)burst_max;
    // v1 hosts ask for 178 bytes and get the v1 layout (with version = 2; they only check >= 1)
    uint16_t n = sizeof s < max ? (uint16_t)sizeof s : max;
    memcpy(buf, &s, n);
    return n;
}

void ptcan_emu_print_stats(void)
{
    uint64_t now = time_us_64();
    printf("[CANTX] %s%s%s mask 0x%05lx | KL15 %s (ST_KL %x) | tx %lu fifo-full %lu fail %lu | TEC %u REC %u%s | "
           "%s\n",
           pte_state_name(pte_state(&core, now)), core.fault ? " " : "", core.fault ? pte_fault_name(core.fault) : "",
           (unsigned long)core.mask, core.kl15 ? "on" : "off", core.st_kl, (unsigned long)core.tx_total,
           (unsigned long)fifo_full, (unsigned long)tx_fail, last_state.tec, last_state.rec,
           last_state.bus_off ? " BUS-OFF" : (last_state.tx_passive ? " TX-PASSIVE" : ""),
           normal_mode ? "Normal" : "Listen Only");
    if (!core.tx_total) return;
    printf("[CANTX] loop gap max %lu us (>20 ms: %lu) | late %lu (max %lu us) resync %lu | burst max %lu | "
           "TX logged %lu lost %lu (TEF looks %lu) | console dropped %lu\n",
           (unsigned long)loop_gap_max_us, (unsigned long)loop_gaps_20ms, (unsigned long)core.late_sends,
           (unsigned long)core.late_max_us, (unsigned long)core.resyncs, (unsigned long)burst_max,
           (unsigned long)tx_logged, (unsigned long)tx_log_lost, (unsigned long)tef_looks,
           (unsigned long)console_dropped());
    printf("[CANTX]");
    for (int i = 0; i < PTE_N_FRAMES; i++)
        if (core.f[i].tx) printf(" %03x:%lu", pte_defs[i].id, (unsigned long)core.f[i].tx);
    printf("\n");
}

void ptcan_emu_fill_health_tx(uint32_t *tx_cnt, uint32_t *err_cnt)
{
    *tx_cnt = core.tx_total;
    *err_cnt += tx_fail;
}
