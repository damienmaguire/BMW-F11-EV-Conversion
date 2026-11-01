#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "mcp251863.h"

// SPI instructions (4-bit command + 12-bit address)
#define CMD_RESET 0x0
#define CMD_WRITE 0x2
#define CMD_READ  0x3

// Registers (MCP251863 DS20006624B, tables 4-1 / 4-2)
#define REG_C1CON      0x000
#define REG_C1NBTCFG   0x004
#define REG_C1DBTCFG   0x008
#define REG_C1TDC      0x00C
#define REG_C1TBC      0x010
#define REG_C1TSCON    0x014
#define REG_C1INT      0x01C
#define REG_C1FLTCON0  0x1D0
#define REG_C1FLTOBJ0  0x1F0
#define REG_C1MASK0    0x1F4
#define REG_C1TREC     0x034
#define REG_C1BDIAG1   0x03C
#define REG_C1TEFCON   0x040
#define REG_C1TEFSTA   0x044
#define REG_C1TEFUA    0x048
#define REG_C1FIFOCON1 0x05C
#define REG_C1FIFOSTA1 0x060
#define REG_C1FIFOUA1  0x064
#define REG_OSC        0xE00
#define REG_IOCON      0xE04
#define REG_DEVID      0xE14
#define RAM_START      0x400

#define CON_RTXAT  (1u << 16)
#define CON_STEF   (1u << 19)
#define CON_TXQEN  (1u << 20)
#define CON_OPMOD_SHIFT 21
#define CON_REQOP_SHIFT 24
#define OPMOD_CAN20  6
#define OPMOD_CANFD  0
#define CON_ISOCRCEN (1u << 5)
#define CON_BRSDIS   (1u << 12)
#define OPMOD_CONFIG 4
#define OPMOD_LISTEN 3

#define OSC_OSCRDY  (1u << 10)
#define INT_RXIE    (1u << 17)
#define INT_RXOVIE  (1u << 27)
#define TSCON_TBCEN (1u << 16)

#define FIFOCON_TFNRFNIE  (1u << 0)
#define FIFOCON_RXOVIE    (1u << 3)
#define FIFOCON_RXTSEN    (1u << 5)
#define FIFOCON_TXEN      (1u << 7)
#define FIFOCON_FSIZE(n)  ((uint32_t)((n) - 1) << 24)
#define FIFOCON_UINC      (1u << 8)
#define FIFOCON_TXREQ     (1u << 9)
#define FIFOCON_TXAT_3    (1u << 21)
#define FIFOCON_PLSIZE_64 (7u << 29)
#define FIFO_CON(n) ((uint16_t)(REG_C1FIFOCON1 + 12 * ((n) - 1)))
#define FIFO_STA(n) ((uint16_t)(REG_C1FIFOSTA1 + 12 * ((n) - 1)))
#define FIFO_UA(n)  ((uint16_t)(REG_C1FIFOUA1 + 12 * ((n) - 1)))
#define FIFOSTA_TFNRFNIF  (1u << 0)
#define FIFOSTA_RXOVIF    (1u << 3)
#define FIFOSTA_TXATIF    (1u << 4)
#define FIFOSTA_TXERR     (1u << 5)
#define FIFOSTA_TXLARB    (1u << 6)
#define FIFOSTA_TXABT     (1u << 7)

static const mcp251863_cfg_t *g_cfg;
static uint32_t g_tx_seq;
static bool g_tef_on;
static uint8_t g_rx_plsize_bytes = 8;

// nCS setup/hold: >= TSCK/2 before first SCK, >= TSCK high between frames
// (250 ns at 4 MHz). 50 cycles = 500 ns at the 100 MHz system clock.
#define CS_GUARD() busy_wait_at_least_cycles(50)
static inline void cs_low(void)  { CS_GUARD(); gpio_put(g_cfg->pin_cs, 0); CS_GUARD(); }
static inline void cs_high(void) { CS_GUARD(); gpio_put(g_cfg->pin_cs, 1); CS_GUARD(); }

static void hdr(uint8_t *h, uint8_t cmd, uint16_t addr)
{
    h[0] = (uint8_t)((cmd << 4) | ((addr >> 8) & 0x0F));
    h[1] = (uint8_t)(addr & 0xFF);
}

static void mcp_reset_cmd(void)
{
    uint8_t h[2] = {0, 0};
    cs_low();
    spi_write_blocking(g_cfg->spi, h, 2);
    cs_high();
}

static void mcp_read(uint16_t addr, uint8_t *buf, size_t len)
{
    uint8_t h[2];
    hdr(h, CMD_READ, addr);
    cs_low();
    spi_write_blocking(g_cfg->spi, h, 2);
    spi_read_blocking(g_cfg->spi, 0x00, buf, len);
    cs_high();
}

static void mcp_write(uint16_t addr, const uint8_t *buf, size_t len)
{
    uint8_t h[2];
    hdr(h, CMD_WRITE, addr);
    cs_low();
    spi_write_blocking(g_cfg->spi, h, 2);
    spi_write_blocking(g_cfg->spi, buf, len);
    cs_high();
}

static uint32_t rd32(uint16_t addr)
{
    uint8_t b[4];
    mcp_read(addr, b, 4);
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static void wr32(uint16_t addr, uint32_t v)
{
    uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    mcp_write(addr, b, 4);
}

static void wr8(uint16_t addr, uint8_t v) { mcp_write(addr, &v, 1); }

static bool wait_opmod(uint8_t mode, uint32_t timeout_ms)
{
    absolute_time_t until = make_timeout_time_ms(timeout_ms);
    do {
        uint8_t b2;
        mcp_read(REG_C1CON + 2, &b2, 1);
        if (((b2 >> 5) & 7) == mode) return true;
        sleep_us(100);
    } while (!time_reached(until));
    return false;
}

const char *mcp251863_strerror(mcp_status_t s)
{
    switch (s) {
    case MCP_OK: return "ok";
    case MCP_ERR_SPI: return "SPI check failed (no response / wrong reset values)";
    case MCP_ERR_OSC: return "oscillator not ready (OSC.OSCRDY never set)";
    case MCP_ERR_MODE: return "mode change refused (bus stuck dominant?)";
    case MCP_ERR_BITTIMING: return "no bit timing for oscillator/bitrate";
    case MCP_ERR_TX_FULL: return "TX FIFO full (previous frame not sent)";
    }
    return "?";
}

mcp_status_t mcp251863_init(const mcp251863_cfg_t *cfg, mcp_bittiming_t *bt_out,
                            mcp_data_bittiming_t *dbt_out, uint8_t *devid_out)
{
    g_cfg = cfg;
    static bool io_done = false;
    if (!io_done) {
        // Transceiver in standby until the controller is configured.
        gpio_init(cfg->pin_stby);
        gpio_set_dir(cfg->pin_stby, GPIO_OUT);
        gpio_put(cfg->pin_stby, 1);

        gpio_init(cfg->pin_int);
        gpio_set_dir(cfg->pin_int, GPIO_IN);
        gpio_pull_up(cfg->pin_int);

        gpio_init(cfg->pin_cs);
        gpio_set_dir(cfg->pin_cs, GPIO_OUT);
        gpio_put(cfg->pin_cs, 1);

        spi_init(cfg->spi, cfg->spi_hz);
        spi_set_format(cfg->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        gpio_set_function(cfg->pin_sck, GPIO_FUNC_SPI);
        gpio_set_function(cfg->pin_mosi, GPIO_FUNC_SPI);
        gpio_set_function(cfg->pin_miso, GPIO_FUNC_SPI);
        gpio_pull_up(cfg->pin_miso); // keep MISO defined if the chip is absent
        io_done = true;
    }
    gpio_put(cfg->pin_stby, 1);

    mcp_bittiming_t bt;
    if (!mcp251863_calc_bittiming(cfg->osc_hz, cfg->bitrate, &bt)) return MCP_ERR_BITTIMING;
    if (bt_out) *bt_out = bt;
    const bool fd = cfg->data_bitrate != 0;
    mcp_data_bittiming_t dbt = {0};
    if (fd && !mcp251863_calc_data_bittiming(cfg->osc_hz, cfg->data_bitrate, &dbt)) return MCP_ERR_BITTIMING;
    if (dbt_out) *dbt_out = dbt;

    mcp_reset_cmd();
    sleep_ms(3); // oscillator stabilisation, TOSCSTAB max 3 ms

    // Wait for the crystal.
    absolute_time_t until = make_timeout_time_ms(20);
    uint32_t osc;
    do {
        osc = rd32(REG_OSC);
        if (osc & OSC_OSCRDY) break;
        sleep_us(200);
    } while (!time_reached(until));

    // Sanity check SPI: after RESET, CON must report configuration mode, and a
    // write/readback of NBTCFG must round-trip.
    uint32_t con = rd32(REG_C1CON);
    if (((con >> CON_OPMOD_SHIFT) & 7) != OPMOD_CONFIG || con == 0xFFFFFFFFu) return MCP_ERR_SPI;
    uint32_t nbt = ((bt.brp - 1) << 24) | ((bt.tseg1 - 1) << 16) | ((bt.tseg2 - 1) << 8) | (bt.sjw - 1);
    wr32(REG_C1NBTCFG, nbt);
    if (rd32(REG_C1NBTCFG) != nbt) return MCP_ERR_SPI;
    if (!(osc & OSC_OSCRDY)) return MCP_ERR_OSC;

    if (fd) {
        wr32(REG_C1DBTCFG, dbt.dbtcfg);
        wr32(REG_C1TDC, dbt.tdc);
        if (rd32(REG_C1DBTCFG) != dbt.dbtcfg) return MCP_ERR_SPI;
    }

    if (devid_out) mcp_read(REG_DEVID, devid_out, 1);

    // nINT0/nINT1 are unconnected: make them driven-low GPIO outputs instead of
    // floating inputs. IOCON must be written byte-wise (datasheet note).
    wr8(REG_IOCON + 0, 0x00); // TRIS0=TRIS1=0 (output), XSTBYEN=0; LATx default 0

    // CON: no TX event FIFO, no TX queue -> FIFO1 sits at the start of RAM.
    // Restricted retransmissions so an un-ACKed test frame is dropped after 3
    // retries instead of retrying forever. ISO CRC, BRS allowed.
    con &= ~(CON_STEF | CON_TXQEN | CON_BRSDIS);
    if (cfg->tef_depth) con |= CON_STEF;  // v1f: TX event FIFO (our own TX, time stamped)
    con |= CON_RTXAT | CON_ISOCRCEN;
    con = (con & ~(7u << CON_REQOP_SHIFT)) | ((uint32_t)OPMOD_CONFIG << CON_REQOP_SHIFT);
    wr32(REG_C1CON, con);

    // Time base: 1 us per tick (TBCPRE = osc/1 MHz - 1), stamp at SOF (TSEOF=0).
    uint32_t pre = cfg->osc_hz / 1000000u;
    if (pre < 1) pre = 1;
    wr32(REG_C1TSCON, TSCON_TBCEN | ((pre - 1) & 0x3FF));

    // TEF (v1f, optional): tef_depth objects of 12 B (ID, flags+SEQ, SOF time stamp).
    // No interrupt; polled by the PT-CAN emulation.
    g_tef_on = cfg->tef_depth != 0;
    if (g_tef_on) {
        uint32_t d = cfg->tef_depth > 32 ? 32u : cfg->tef_depth;
        wr32(REG_C1TEFCON, (1u << 5) | ((d - 1u) << 24)); // TEFTSEN, FSIZE
    }

    // FIFO1: receive, time stamped, interrupt on not-empty and on overflow.
    g_rx_plsize_bytes = fd ? 64 : 8;
    uint32_t rxcon = FIFOCON_TFNRFNIE | FIFOCON_RXOVIE | FIFOCON_RXTSEN |
                     FIFOCON_FSIZE(fd ? MCP_RX_DEPTH_FD : MCP_RX_DEPTH_CLASSIC) |
                     (fd ? FIFOCON_PLSIZE_64 : 0);
    wr32(FIFO_CON(MCP_FIFO_RX), rxcon);
    // FIFO2: transmit, tx_fifo_depth messages (default 1), 8 byte payload, 3 retransmissions.
    // FIFO3: transmit, 1 message deep, 64 byte payload, 3 retransmissions.
    uint32_t txd = cfg->tx_fifo_depth ? cfg->tx_fifo_depth : 1u;
    if (txd > 32) txd = 32;
    wr32(FIFO_CON(MCP_FIFO_CLASSIC), FIFOCON_TXEN | FIFOCON_TXAT_3 | FIFOCON_FSIZE(txd));
    wr32(FIFO_CON(MCP_FIFO_FD), FIFOCON_PLSIZE_64 | FIFOCON_TXEN | FIFOCON_TXAT_3);

    // Filter 0 -> FIFO1, mask 0 with MIDE=0: accepts every standard and
    // extended ID. Object/mask first, then enable (filter is off after reset).
    wr32(REG_C1FLTOBJ0, 0);
    wr32(REG_C1MASK0, 0);
    wr8(REG_C1FLTCON0, (uint8_t)(0x80 | MCP_FIFO_RX)); // FLTEN0 | F0BP = 1

    // nINT (active low) = RX FIFO not empty or RX overflow; all else off.
    wr32(REG_C1INT, INT_RXIE | INT_RXOVIE);

    // Transceiver to normal mode (needed to receive; in Listen Only the
    // controller keeps TXD recessive), then the controller mode.
    gpio_put(cfg->pin_stby, 0);
    sleep_us(50);
#ifdef MCP251863_RX_ONLY
    // RX-only build: the controller is only ever put into Listen Only (no ACK,
    // no error frames, no TX). Anything else is refused before leaving config mode.
    if (!cfg->listen_only) return MCP_ERR_MODE;
#endif
    const uint8_t mode = cfg->listen_only ? OPMOD_LISTEN : (fd ? OPMOD_CANFD : OPMOD_CAN20);
    wr8(REG_C1CON + 3, mode); // REQOP, ABAT=0, TXBWS=0
    if (!wait_opmod(mode, 20)) return MCP_ERR_MODE;
    g_tx_seq = 0;
    return MCP_OK;
}

#ifndef MCP251863_RX_ONLY
static mcp_status_t send_obj(uint8_t fifo, uint16_t id, const uint8_t *data, uint8_t len, bool fdf, bool brs)
{
    uint8_t dlc = mcp251863_len_to_dlc(len);
    if (!fdf && dlc > 8) dlc = 8;
    uint8_t wire_len = mcp251863_dlc_to_len(dlc);

    uint8_t sta;
    mcp_read(FIFO_STA(fifo), &sta, 1);
    if (!(sta & FIFOSTA_TFNRFNIF)) return MCP_ERR_TX_FULL;

    uint32_t ua = rd32(FIFO_UA(fifo));
    uint16_t addr = (uint16_t)(RAM_START + (ua & 0xFFF));

    uint8_t obj[8 + 64] = {0};
    uint32_t t0 = id & 0x7FFu;                                          // SID, SID11=0
    uint32_t t1 = ((g_tx_seq++ & 0x7FFFFFu) << 9) | (fdf ? (1u << 7) : 0) |
                  ((fdf && brs) ? (1u << 6) : 0) | (dlc & 0xF);          // IDE=0 RTR=0 ESI=0
    for (int i = 0; i < 4; i++) { obj[i] = (uint8_t)(t0 >> (8 * i)); obj[4 + i] = (uint8_t)(t1 >> (8 * i)); }
    memcpy(&obj[8], data, len < wire_len ? len : wire_len); // pad (if any) stays 0
    size_t n = 8 + (((size_t)wire_len + 3u) & ~3u);           // RAM writes: whole 32-bit words
    mcp_write(addr, obj, n);

    wr8((uint16_t)(FIFO_CON(fifo) + 1), (uint8_t)((FIFOCON_UINC | FIFOCON_TXREQ) >> 8));
    return MCP_OK;
}

mcp_status_t mcp251863_send_std(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > 8) dlc = 8;
    return send_obj(MCP_FIFO_CLASSIC, id, data, dlc, false, false);
}

mcp_status_t mcp251863_send_fd_std(uint16_t id, const uint8_t *data, uint8_t len, bool brs)
{
    if (len > 64) len = 64;
    return send_obj(MCP_FIFO_FD, id, data, len, true, brs);
}

uint32_t mcp251863_next_seq(void) { return g_tx_seq & 0x7FFFFFu; }

int mcp251863_tef_read(mcp_tef_t *e, bool *overflow)
{
    if (!g_tef_on || !g_cfg) return 0;
    uint8_t b[8];
    mcp_read(REG_C1TEFSTA, b, 8); // C1TEFSTA + C1TEFUA in one transfer
    uint32_t ua = b[4] | (b[5] << 8) | ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
    if (ua == 0xFFFFFFFFu) return MCP_ERR_SPI;
    if (b[0] & (1u << 3)) {                  // TEFOVIF
        if (overflow) *overflow = true;
        wr8(REG_C1TEFSTA, 0x00);             // clear (the other bits are read-only)
    }
    if (!(b[0] & 1u)) return 0;              // TEFNEIF: not empty
    uint8_t o[12];
    mcp_read((uint16_t)(RAM_START + (ua & 0xFFF)), o, sizeof o);
    uint32_t te0 = o[0] | (o[1] << 8) | ((uint32_t)o[2] << 16) | ((uint32_t)o[3] << 24);
    uint32_t te1 = o[4] | (o[5] << 8) | ((uint32_t)o[6] << 16) | ((uint32_t)o[7] << 24);
    e->id = te0 & 0x7FFu;
    e->dlc = (uint8_t)(te1 & 0xF);
    e->seq = te1 >> 9;
    e->ts = o[8] | (o[9] << 8) | ((uint32_t)o[10] << 16) | ((uint32_t)o[11] << 24);
    wr8((uint16_t)(REG_C1TEFCON + 1), 0x01); // UINC: release the event
    return 1;
}

mcp_status_t mcp251863_set_normal(bool normal)
{
    if (!g_cfg) return MCP_ERR_SPI;
    // Normal <-> Listen Only goes through Configuration mode (always allowed; the
    // controller enters it once the bus is idle). REQOP byte: ABAT=0, TXBWS=0.
    wr8(REG_C1CON + 3, OPMOD_CONFIG);
    if (!wait_opmod(OPMOD_CONFIG, 20)) return MCP_ERR_MODE;
    const uint8_t mode = normal ? (g_cfg->data_bitrate ? OPMOD_CANFD : OPMOD_CAN20) : OPMOD_LISTEN;
    wr8(REG_C1CON + 3, mode);
    if (!wait_opmod(mode, 20)) return MCP_ERR_MODE;
    return MCP_OK;
}
#endif // MCP251863_RX_ONLY

mcp_status_t mcp251863_read_tx_state_fifo(uint8_t fifo, mcp_tx_state_t *st)
{
    uint8_t b[8];
    mcp_read(FIFO_CON(fifo), b, 8); // FIFOCONm + FIFOSTAm
    uint32_t con = b[0] | (b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    uint8_t sta = b[4];
    st->txreq_pending = (con & FIFOCON_TXREQ) != 0;
    st->tx_attempts_exhausted = (sta & FIFOSTA_TXATIF) != 0;
    st->tx_err = (sta & FIFOSTA_TXERR) != 0;
    st->tx_lost_arb = (sta & FIFOSTA_TXLARB) != 0;
    st->tx_aborted = (sta & FIFOSTA_TXABT) != 0;
    uint32_t trec = rd32(REG_C1TREC);
    st->rec = (uint8_t)trec;
    st->tec = (uint8_t)(trec >> 8);
    st->tx_warn = (trec >> 18) & 1;
    st->rx_warn = (trec >> 17) & 1;
    st->rx_passive = (trec >> 19) & 1;
    st->tx_passive = (trec >> 20) & 1;
    st->bus_off = (trec >> 21) & 1;
    st->bdiag1 = rd32(REG_C1BDIAG1);
    uint8_t c2;
    mcp_read(REG_C1CON + 2, &c2, 1);
    st->opmod = (c2 >> 5) & 7;
    return (con == 0xFFFFFFFFu) ? MCP_ERR_SPI : MCP_OK;
}

mcp_status_t mcp251863_read_tx_state(mcp_tx_state_t *st) { return mcp251863_read_tx_state_fifo(MCP_FIFO_CLASSIC, st); }

void mcp251863_clear_tx_attempt_flag_fifo(uint8_t fifo)
{
    // Byte 0 of FIFOSTA: TXATIF/RXOVIF are clear-on-write-0, the rest read-only.
    wr8(FIFO_STA(fifo), 0x00);
}

void mcp251863_clear_tx_attempt_flag(void) { mcp251863_clear_tx_attempt_flag_fifo(MCP_FIFO_CLASSIC); }

int mcp251863_rx_read(mcp_rx_frame_t *f, bool *overflow)
{
    uint8_t b[8];
    mcp_read(FIFO_STA(MCP_FIFO_RX), b, 8); // FIFOSTA1 + FIFOUA1 in one transfer
    uint32_t ua = b[4] | (b[5] << 8) | ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
    if (ua == 0xFFFFFFFFu) return MCP_ERR_SPI;
    if (b[0] & FIFOSTA_RXOVIF) {
        if (overflow) *overflow = true;
        wr8(FIFO_STA(MCP_FIFO_RX), 0x00); // clears RXOVIF (other bits read-only in an RX FIFO)
    }
    if (!(b[0] & FIFOSTA_TFNRFNIF)) return 0;

    uint16_t addr = (uint16_t)(RAM_START + (ua & 0xFFF));
    uint8_t o[12 + 8];
    mcp_read(addr, o, sizeof o);  // R0 ID, R1 flags, R2 timestamp, first 8 data bytes
    uint32_t r0 = o[0] | (o[1] << 8) | ((uint32_t)o[2] << 16) | ((uint32_t)o[3] << 24);
    uint8_t r1 = o[4];
    f->ide = (r1 >> 4) & 1;
    f->fdf = (r1 >> 7) & 1;
    f->rtr = !f->fdf && ((r1 >> 5) & 1);  // FD frames have no remote frames (the bit is RRS there)
    f->brs = f->fdf && ((r1 >> 6) & 1);
    f->esi = f->fdf && (o[5] & 1);
    f->dlc = r1 & 0xF;
    f->id = f->ide ? (((r0 & 0x7FFu) << 18) | ((r0 >> 11) & 0x3FFFFu)) : (r0 & 0x7FFu);
    f->ts = o[8] | (o[9] << 8) | ((uint32_t)o[10] << 16) | ((uint32_t)o[11] << 24);
    uint8_t len = mcp251863_dlc_to_len(f->dlc);
    if (!f->fdf && len > 8) len = 8;      // classic DLC 9..15 means 8 bytes
    if (f->rtr && !f->fdf) len = 0;        // remote frame: DLC only, no data
    if (len > g_rx_plsize_bytes) len = g_rx_plsize_bytes; // truncated by the FIFO payload size
    f->len = len;
    memcpy(f->data, &o[12], len < 8 ? len : 8);
    if (len > 8) mcp_read((uint16_t)(addr + 20), &f->data[8], (size_t)(len - 8)); // FD only

    wr8((uint16_t)(FIFO_CON(MCP_FIFO_RX) + 1), (uint8_t)(FIFOCON_UINC >> 8)); // release the object
    return 1;
}

uint32_t mcp251863_read_tbc(void) { return rd32(REG_C1TBC); }
