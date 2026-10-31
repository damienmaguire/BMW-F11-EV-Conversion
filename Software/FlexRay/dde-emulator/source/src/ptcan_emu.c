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

#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"
#include "pico/time.h"
#include "ptcan_emu_core.h"

_Static_assert(sizeof(ptcan_stats_t) == 178, "ptcan_stats_t layout");
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
    else pte_on_rx(&core, (uint16_t)f->id, f->data, f->len, to64(t_us32));
}

void ptcan_emu_host_set(uint32_t mask)
{
    host_mask = mask;
    host_pending = true;
    __sev();
}

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
    if (!normal_mode) return true;
    for (int n = 0; n < 4; n++) {
        pte_tx_t tx;
        if (!pte_next(&core, now, &tx)) break;
        if (pte_id_forbidden(tx.id)) { // belt and braces: the core never schedules these
            core.refused++;
            pte_fault(&core, PTE_FAULT_MODE, tx.id);
            break;
        }
        mcp_status_t r = mcp251863_send_std(tx.id, tx.data, tx.dlc);
        if (r == MCP_ERR_TX_FULL) { fifo_full++; break; }
        if (r != MCP_OK) return false;
        pte_commit(&core, &tx, now);
    }
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
