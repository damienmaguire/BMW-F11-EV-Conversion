// V2 board: status LEDs and the MCP251863 CAN channel.
//   - RX (default): Listen Only, accept-all filter, nINT-driven drain on core0,
//     frames go to the USB bulk-IN stream as CAN records on SavvyCAN bus 2.
//   - Test TX (V2_CAN_TEST_TX=1, Normal mode only): 0x123 classic, 0x124 FD.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "board_pico_flexray_v2.h"
#include "mcp251863.h"
#include "panda_usb.h"
#include "pico_can_record.h"
#include "v2_cantest.h"
#if PICO_FLEXRAY_DDE_EMU
#include "dde_standalone.h"
#endif
#if EMU_CAN_TX
#include "ptcan_emu.h"
#endif

static const mcp251863_cfg_t can_cfg = {
    .spi = V2_CAN_SPI,
    .pin_miso = V2_CAN_SPI_MISO_PIN,
    .pin_mosi = V2_CAN_SPI_MOSI_PIN,
    .pin_sck = V2_CAN_SPI_SCK_PIN,
    .pin_cs = V2_CAN_SPI_CS_PIN,
    .pin_int = V2_CAN_INT_PIN,
    .pin_stby = V2_CAN_STBY_PIN,
    .spi_hz = V2_CAN_SPI_HZ,
    .osc_hz = V2_CAN_OSC_HZ,
    .bitrate = V2_CAN_NOMINAL_BITRATE,
    .data_bitrate = V2_CAN_FD_DATA_BITRATE,
    .listen_only = (V2_CAN_MODE == V2_CAN_MODE_LISTEN_ONLY),
    .tx_fifo_depth = V2_CAN_TX_FIFO_DEPTH,
    .tef_depth = EMU_CAN_TX ? 16 : 0, // v1f: our own TX frames, time stamped, for the logger
};

#define FD_ENABLED (V2_CAN_FD_DATA_BITRATE != 0)
#define LISTEN_ONLY (V2_CAN_MODE == V2_CAN_MODE_LISTEN_ONLY)

static repeating_timer_t tick_timer;
static volatile uint32_t tick_count;
static volatile bool tick_due;
static volatile uint32_t nint_edges;

static mcp_status_t init_status = MCP_ERR_SPI;
static bool can_ready;
static uint32_t ticks_since_init_try;

static struct {
    uint32_t init_attempts;
    uint32_t rx_frames, rx_std, rx_ext, rx_rtr, rx_fd;
    uint32_t rx_overflows;    // RXOVIF events (>= 1 frame lost in the MCP each)
    uint32_t rx_spi_errors;
    uint32_t max_batch;
    uint32_t last_id;
    bool rx_activity;         // frame (or ACKed test TX) since last tick
    absolute_time_t led2_hold_until;
    bool led2_state;
    mcp_tx_state_t last;      // last controller error state snapshot
} st;

static inline void led1(bool on)
{
    gpio_put(V2_LED1_PIN, on ? V2_LED_ON : V2_LED_OFF);
    gpio_put(V2_PICO_ONBOARD_LED_PIN, on);
}
static inline void led2(bool on) { st.led2_state = on; gpio_put(V2_LED2_PIN, on ? V2_LED_ON : V2_LED_OFF); }
static void led2_hold(uint32_t ms) { st.led2_hold_until = make_timeout_time_ms(ms); led2(true); }

void v2_leds_init(void)
{
    const uint pins[] = {V2_LED1_PIN, V2_LED2_PIN, V2_PICO_ONBOARD_LED_PIN};
    for (unsigned i = 0; i < 3; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 0);
    }
}

// nINT (GP20, active low) falling edge: only wakes the core0 main loop out of
// __wfe(); all SPI work happens in v2_cantest_poll(), which also checks the
// pin level, so a missed edge cannot stall RX.
static void nint_irq(uint gpio, uint32_t events)
{
    (void)gpio; (void)events;
    nint_edges++;
    __sev();
}

#if V2_CAN_TEST_TX
typedef struct {
    const char *name;
    uint8_t fifo;
    bool in_flight, was_failing;
    uint8_t counter;
    uint32_t queued, ok, fail, fifo_full;
} stream_t;
static stream_t s_classic = {.name = "0x123 classic", .fifo = MCP_FIFO_CLASSIC};
static stream_t s_fd = {.name = "0x124 FD+BRS", .fifo = MCP_FIFO_FD};
#endif

void v2_can_init(void)
{
    mcp_bittiming_t bt = {0};
    mcp_data_bittiming_t dbt = {0};
    uint8_t devid = 0;
    st.init_attempts++;
    init_status = mcp251863_init(&can_cfg, &bt, &dbt, &devid);
    can_ready = (init_status == MCP_OK);
    ticks_since_init_try = 0;
#if EMU_CAN_TX
    ptcan_emu_on_can_init(can_ready); // the controller is back in Listen Only after init
#endif
#if V2_CAN_TEST_TX
    s_classic.in_flight = s_fd.in_flight = false;
#endif
    if (st.init_attempts == 1 || can_ready) {
        printf("[CAN] MCP251863 on spi0 SCK=%d MOSI=%d MISO=%d CS=%d INT=%d STBY=%d, SPI %lu Hz\n",
               V2_CAN_SPI_SCK_PIN, V2_CAN_SPI_MOSI_PIN, V2_CAN_SPI_MISO_PIN, V2_CAN_SPI_CS_PIN,
               V2_CAN_INT_PIN, V2_CAN_STBY_PIN, (unsigned long)V2_CAN_SPI_HZ);
        printf("[CAN] osc=%lu Hz nominal=%lu bit/s: BRP=%lu TQ/bit=%lu TSEG1=%lu TSEG2=%lu SJW=%lu SP=%lu.%lu%%\n",
               (unsigned long)V2_CAN_OSC_HZ, (unsigned long)V2_CAN_NOMINAL_BITRATE,
               (unsigned long)bt.brp, (unsigned long)bt.tq_per_bit, (unsigned long)bt.tseg1,
               (unsigned long)bt.tseg2, (unsigned long)bt.sjw,
               (unsigned long)(bt.sample_point_permille / 10), (unsigned long)(bt.sample_point_permille % 10));
        if (FD_ENABLED)
            printf("[CAN] FD data=%lu bit/s: DBTCFG=0x%08lx TDC=0x%08lx\n", (unsigned long)V2_CAN_FD_DATA_BITRATE,
                   (unsigned long)dbt.dbtcfg, (unsigned long)dbt.tdc);
    }
    if (can_ready) {
        static bool irq_done = false;
        if (!irq_done) {
            // Runs on core0, so the GPIO IRQ is core0's; core1 only uses PIO IRQs.
            gpio_set_irq_enabled_with_callback(V2_CAN_INT_PIN, GPIO_IRQ_EDGE_FALL, true, nint_irq);
            irq_done = true;
        }
        printf("[CAN] init OK (DEVID=0x%02x): %s, %s, RX FIFO1 %d x %d B accept-all -> USB bus %d%s\n",
               devid, LISTEN_ONLY ? (EMU_CAN_TX ? "LISTEN-ONLY until the host enables PT-CAN TX (--can-tx)"
                                                : "LISTEN-ONLY (no ACK, no error frames, no TX)")
                                  : "NORMAL mode (ACKs!)",
               FD_ENABLED ? "CAN FD" : "classic CAN 2.0",
               FD_ENABLED ? MCP_RX_DEPTH_FD : MCP_RX_DEPTH_CLASSIC, FD_ENABLED ? 64 : 8, V2_CAN_USB_BUS,
               V2_CAN_TEST_TX ? ", test TX 0x123 on" : "");
        led2(false);
    } else {
        printf("[CAN] init FAILED (attempt %lu): %s - retrying every 5 s\n",
               (unsigned long)st.init_attempts, mcp251863_strerror(init_status));
        led2(true); // solid = failure
    }
}

static bool tick_cb(repeating_timer_t *t)
{
    (void)t;
    uint32_t n = ++tick_count;
    const uint32_t half_s = 500 / V2_CAN_TICK_MS;
#if PICO_FLEXRAY_DDE_EMU && EMU_STANDALONE
    // v1h.1 standalone: LED1 says the emulator state (flash every 2 s = waiting for the
    // car, slow blink = emulating, fast blink = fault); see dde_standalone.h
    (void)half_s;
    led1(dde_standalone_led(dde_standalone_led_state(), n * V2_CAN_TICK_MS));
#else
    if ((n % half_s) == 0) led1(((n / half_s) & 1) != 0); // 1 Hz heartbeat
#endif
    tick_due = true;
    __sev();
    return true;
}

void v2_tick_start(void)
{
    add_repeating_timer_ms(-V2_CAN_TICK_MS, tick_cb, NULL, &tick_timer);
}

// ---- RX ----------------------------------------------------------------------
static void can_rx_service(void)
{
    // Bounded: at most V2_CAN_RX_MAX_PER_POLL frames (~65 us of SPI each at
    // 4 MHz) per main-loop pass, so FlexRay notification handling and
    // tud_task() keep running even on a fully loaded CAN bus.
    if (gpio_get(V2_CAN_INT_PIN)) return; // nINT high: nothing pending

    bool have_ref = false;
    uint32_t tbc_ref = 0, us_ref = 0;
    uint32_t n = 0;
    for (; n < V2_CAN_RX_MAX_PER_POLL; n++) {
        mcp_rx_frame_t f;
        bool ovf = false;
        int r = mcp251863_rx_read(&f, &ovf);
        if (ovf) {
            st.rx_overflows++;
            led2_hold(V2_CAN_OVERFLOW_LED_MS);
        }
        if (r < 0) {
            st.rx_spi_errors++;
            printf("[CAN] SPI read failed during RX, re-initialising\n");
            can_ready = false;
            led2(true);
            return;
        }
        if (r == 0) break;
        if (!have_ref) {
            // Map the controller's 1 us time base onto time_us_32() once per
            // batch: frame time = now - (TBC now - RXMSGTS).
            tbc_ref = mcp251863_read_tbc();
            us_ref = time_us_32();
            have_ref = true;
        }
        pico_can_record_t rec;
        rec.bus = V2_CAN_USB_BUS;
        rec.flags = (uint8_t)((f.ide ? PICO_CAN_FLAG_IDE : 0) | (f.rtr ? PICO_CAN_FLAG_RTR : 0) |
                              (f.fdf ? PICO_CAN_FLAG_FDF : 0) | (f.brs ? PICO_CAN_FLAG_BRS : 0) |
                              (f.esi ? PICO_CAN_FLAG_ESI : 0));
        uint8_t full = mcp251863_dlc_to_len(f.dlc);
        if (!f.fdf && full > 8) full = 8;
        if (f.rtr && !f.fdf) full = 0;
        if (f.len < full) rec.flags |= PICO_CAN_FLAG_TRUNCATED;
        rec.dlc = f.dlc;
        rec.len = f.len;
        rec.id = f.id;
        rec.time_us = us_ref + (uint32_t)(int32_t)(f.ts - tbc_ref); // signed: frames newer than tbc_ref
        memcpy(rec.data, f.data, f.len);

        st.rx_frames++;
        if (f.ide) st.rx_ext++; else st.rx_std++;
        if (f.rtr) st.rx_rtr++;
        if (f.fdf) st.rx_fd++;
        st.last_id = f.id;
        st.rx_activity = true;
#if EMU_CAN_TX
        ptcan_emu_on_rx(&f, rec.time_us);
#endif
        (void)panda_can_record_push(&rec); // never blocks; drops are counted in panda_usb.c
    }
    if (n > st.max_batch) st.max_batch = n;
    // More frames left (nINT still low): make the next __wfe() return at once.
    if (!gpio_get(V2_CAN_INT_PIN)) __sev();
}

#if V2_CAN_TEST_TX
static void on_tx_result(stream_t *s, bool ok)
{
    if (ok) {
        s->ok++;
        if (s->was_failing) { printf("[CAN] %s TX recovered (ACK seen)\n", s->name); s->was_failing = false; }
        st.rx_activity = true;
    } else {
        s->fail++;
        if (!s->was_failing) {
            printf("[CAN] %s TX failed: TEC=%u REC=%u%s%s BDIAG1=0x%08lx (no ACK? 2nd node + termination)\n",
                   s->name, st.last.tec, st.last.rec, st.last.tx_passive ? " TX-PASSIVE" : "",
                   st.last.bus_off ? " BUS-OFF" : "", (unsigned long)st.last.bdiag1);
            s->was_failing = true;
        }
        led2_hold(V2_CAN_OVERFLOW_LED_MS);
    }
}

static bool service_stream(stream_t *s)
{
    if (mcp251863_read_tx_state_fifo(s->fifo, &st.last) != MCP_OK) return false;
    if (s->in_flight) {
        on_tx_result(s, !st.last.txreq_pending && !st.last.tx_attempts_exhausted);
        if (st.last.tx_attempts_exhausted) mcp251863_clear_tx_attempt_flag_fifo(s->fifo);
        s->in_flight = false;
    }
    mcp_status_t r;
    if (s->fifo == MCP_FIFO_CLASSIC) {
        uint8_t data[8] = {s->counter, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE};
        r = mcp251863_send_std(V2_CAN_TEST_ID, data, 8);
    } else {
        uint8_t data[V2_CAN_FD_TEST_LEN];
        for (unsigned i = 0; i < sizeof data; i++) data[i] = (uint8_t)i;
        data[0] = s->counter;
        r = mcp251863_send_fd_std(V2_CAN_FD_TEST_ID, data, sizeof data, true);
    }
    if (r == MCP_OK) { s->counter++; s->queued++; s->in_flight = true; }
    else { s->fifo_full++; on_tx_result(s, false); }
    return true;
}
#endif

static void housekeeping_tick(uint32_t n)
{
    // LED2: one toggle per 50 ms tick with RX activity -> visible <= 10 Hz
    // flicker however busy the bus is; solid while a hold (overflow / TX
    // failure) is active.
    bool holding = !time_reached(st.led2_hold_until);
    if (holding) led2(true);
    else if (st.rx_activity) led2(!st.led2_state);
    st.rx_activity = false;

    // Error counters for status / panda health (cheap: once per second).
    if ((n % (1000 / V2_CAN_TICK_MS)) == 0) {
        if (mcp251863_read_tx_state_fifo(MCP_FIFO_RX, &st.last) != MCP_OK) {
            printf("[CAN] SPI read failed, re-initialising\n");
            can_ready = false;
            led2(true);
            return;
        }
    }
#if EMU_CAN_TX
    if (!ptcan_emu_tick()) { can_ready = false; led2(true); return; }
#endif
#if V2_CAN_TEST_TX
    stream_t *s = (n & 1) ? &s_fd : &s_classic;
    if (s == &s_fd && !FD_ENABLED) return;
    if (!service_stream(s)) { can_ready = false; led2(true); }
#endif
}

void v2_cantest_poll(void)
{
    if (can_ready) can_rx_service();
#if EMU_CAN_TX
    if (can_ready && !ptcan_emu_poll()) { can_ready = false; led2(true); }
#endif

    if (!tick_due) return;
    tick_due = false;
    uint32_t n = tick_count;
    if (!can_ready) {
        if (++ticks_since_init_try >= 5000 / V2_CAN_TICK_MS) v2_can_init();
        return;
    }
    housekeeping_tick(n);
}

void v2_cantest_print_status(void)
{
    if (!can_ready) {
        printf("[CAN] not ready: %s (init attempts %lu)\n", mcp251863_strerror(init_status),
               (unsigned long)st.init_attempts);
        return;
    }
    panda_can_q_stats_t q;
    panda_can_get_q_stats(&q);
    printf("[CAN] RX %lu (std %lu ext %lu rtr %lu fd %lu) last ID 0x%lx | MCP overflows %lu | USB %s: sent %lu "
           "queued %lu dropped %lu not-streamed %lu depth %u | max batch %lu nINT edges %lu | TEC=%u REC=%u%s opmod=%u\n",
           (unsigned long)st.rx_frames, (unsigned long)st.rx_std, (unsigned long)st.rx_ext,
           (unsigned long)st.rx_rtr, (unsigned long)st.rx_fd, (unsigned long)st.last_id,
           (unsigned long)st.rx_overflows, q.enabled ? "on" : "off (host not opted in)", (unsigned long)q.sent,
           (unsigned long)q.queued, (unsigned long)q.dropped, (unsigned long)q.not_streamed, q.depth,
           (unsigned long)st.max_batch, (unsigned long)nint_edges, st.last.tec, st.last.rec,
           st.last.bus_off ? " BUSOFF" : (st.last.rx_passive ? " RXPASSIVE" : ""), st.last.opmod);
#if EMU_CAN_TX
    ptcan_emu_print_stats();
#endif
#if V2_CAN_TEST_TX
    printf("[CAN] test TX 0x123: queued=%lu ok=%lu fail=%lu | 0x124: queued=%lu ok=%lu fail=%lu\n",
           (unsigned long)s_classic.queued, (unsigned long)s_classic.ok, (unsigned long)s_classic.fail,
           (unsigned long)s_fd.queued, (unsigned long)s_fd.ok, (unsigned long)s_fd.fail);
#endif
}

void v2_can_fill_health(struct can_health_t *h)
{
    // panda health reports speeds in kbit/s (bus_config speed / 10 in panda fw)
    h->bus_off = st.last.bus_off;
    h->error_warning = st.last.tx_warn || st.last.rx_warn;
    h->error_passive = st.last.tx_passive || st.last.rx_passive;
    h->receive_error_cnt = st.last.rec;
    h->transmit_error_cnt = st.last.tec;
    h->total_rx_cnt = st.rx_frames;
    h->total_rx_lost_cnt = st.rx_overflows; // overflow events (each >= 1 frame)
#if EMU_CAN_TX
    {
        uint32_t txc = 0, errc = h->total_error_cnt;
        ptcan_emu_fill_health_tx(&txc, &errc);
        h->total_tx_cnt = txc;
        h->total_error_cnt = errc;
    }
#endif
#if V2_CAN_TEST_TX
    h->total_tx_cnt = s_classic.ok + s_fd.ok;
    h->total_error_cnt = s_classic.fail + s_fd.fail;
#endif
    h->can_speed = can_ready ? (uint16_t)(V2_CAN_NOMINAL_BITRATE / 1000) : 0;
    h->can_data_speed = (can_ready && FD_ENABLED) ? (uint16_t)(V2_CAN_FD_DATA_BITRATE / 1000) : 0;
    h->canfd_enabled = can_ready && FD_ENABLED;
    h->brs_enabled = can_ready && FD_ENABLED;
    h->canfd_non_iso = 0;
}
