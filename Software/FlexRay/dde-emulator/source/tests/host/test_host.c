// Host-side checks: bit timing math, the driver's SPI byte stream against a
// small MCP251863 register/RAM model (incl. an RX FIFO), and the USB CAN record
// encoding (src/pico_can_record.h).
#include <stdio.h>
#include <string.h>
#include "mcp251863.h"
#include "pico_can_record.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

/* ---- fake chip ---- */
static uint8_t mem[0x1000];
static uint32_t r32(uint16_t a) { uint32_t v; memcpy(&v, &mem[a], 4); return v; }
static void w32(uint16_t a, uint32_t v) { memcpy(&mem[a], &v, 4); }
static int cs_active, nbytes; static uint8_t cmd; static uint16_t addr;
static uint64_t now_us; static int chip_absent;
absolute_time_t make_timeout_time_ms(uint32_t ms) { return now_us + ms * 1000ull; }
bool time_reached(absolute_time_t t) { return now_us >= t; }
void sleep_ms(uint32_t ms) { now_us += ms * 1000ull; }
void sleep_us(uint64_t us) { now_us += us; }
void gpio_init(uint p) { (void)p; } void gpio_set_dir(uint p, bool o) { (void)p; (void)o; }
void gpio_pull_up(uint p) { (void)p; } void gpio_set_function(uint p, int f) { (void)p; (void)f; }
static int stby_level = -1;

#define FCON(m) (0x05C + 12 * ((m) - 1))
#define FSTA(m) (FCON(m) + 4)
#define FUA(m)  (FCON(m) + 8)

// RX FIFO1 model: frames queued by the test, written into RAM at the user address
typedef struct { uint32_t r0, r1, ts; uint8_t data[64]; } rxobj_t;
static rxobj_t rxq[64]; static int rxq_n, rxq_head;
static uint32_t ram_used, fifo_base[32], fifo_objsz[32], fifo_depth[32], rx_tail_idx;

static void chip_reset(void) {
    memset(mem, 0, sizeof mem);
    w32(0x000, 0x04980760);                                // reset value, config mode
    w32(0xE00, 0x00000460);                                // OSCRDY
    for (int m = 1; m <= 31; m++) w32(FCON(m), 0x00600000); // FIFOCONm reset value
    w32(FSTA(2), 1); w32(FSTA(3), 1);                      // TX FIFOs not full
    w32(0x00C, 0x00021000);                                // TDC reset value
    mem[0xE14] = 0x14;
    rxq_n = rxq_head = 0; rx_tail_idx = 0;
}
static void layout_ram(void) {
    static const uint8_t pl[8] = {8, 12, 16, 20, 24, 32, 48, 64};
    uint32_t off = 0;
    for (int m = 1; m <= 31; m++) {
        uint32_t con = r32(FCON(m));
        bool tx = con & (1u << 7), ts = con & (1u << 5);
        fifo_base[m] = off;
        fifo_objsz[m] = pl[con >> 29] + ((tx || !ts) ? 8u : 12u);
        fifo_depth[m] = ((con >> 24) & 0x1F) + 1;
        w32(FUA(m), off);
        off += fifo_objsz[m] * fifo_depth[m];
    }
    ram_used = off;
}
static void rx_present_head(void) {
    // put the head frame at FIFO1's current user address
    if (rxq_head < rxq_n) {
        uint16_t a = (uint16_t)(0x400 + r32(FUA(1)));
        rxobj_t *o = &rxq[rxq_head];
        w32(a, o->r0); w32(a + 4, o->r1); w32(a + 8, o->ts);
        memcpy(&mem[a + 12], o->data, fifo_objsz[1] - 12);
        mem[FSTA(1)] |= 1;   // TFNRFNIF
    } else mem[FSTA(1)] &= ~1;
}
static void rx_inject(uint32_t r0, uint32_t r1, uint32_t ts, const uint8_t *d, int n) {
    rxobj_t *o = &rxq[rxq_n++];
    memset(o, 0, sizeof *o); o->r0 = r0; o->r1 = r1; o->ts = ts; memcpy(o->data, d, n);
    if (rxq_n - rxq_head == 1) rx_present_head();
}
static void chip_after_write(uint16_t a) {
    if (a == 0x003) {
        if (((mem[0x002] >> 5) & 7) == 4 && (mem[0x003] & 7) != 4) layout_ram();
        mem[0x002] = (uint8_t)((mem[0x002] & 0x1F) | ((mem[0x003] & 7) << 5)); // OPMOD follows REQOP
    }
    if (a == FCON(1) + 1 && (mem[a] & 0x01)) {           // RX UINC: pop, advance UA
        mem[a] &= ~0x01;
        if (rxq_head < rxq_n) rxq_head++;
        rx_tail_idx = (rx_tail_idx + 1) % fifo_depth[1];
        w32(FUA(1), fifo_base[1] + rx_tail_idx * fifo_objsz[1]);
        rx_present_head();
    }
    for (int m = 2; m <= 3; m++)
        if (a == FCON(m) + 1 && (mem[a] & 0x02)) mem[a] &= ~0x01; // TX UINC self clears; TXREQ stays
}
static uint16_t last_wr_addr; static int last_wr_len;
void gpio_put(uint p, bool v) {
    if (p == 21) stby_level = v;
    if (p != 17) return;
    if (!v) { cs_active = 1; nbytes = 0; }
    else { if (cs_active && nbytes == 2 && cmd == 0) chip_reset(); cs_active = 0; }
}
static uint8_t xfer(uint8_t out) {
    uint8_t in = 0;
    if (nbytes == 0) { cmd = out >> 4; addr = (uint16_t)((out & 0xF) << 8); }
    else if (nbytes == 1) { addr |= out; }
    else {
        if (cmd == 3) in = chip_absent ? 0xFF : mem[addr & 0xFFF];
        else if (cmd == 2) {
            if (addr >= 0x400 && addr < 0xC00) { if (nbytes == 2) last_wr_addr = addr; last_wr_len = nbytes - 1; }
            mem[addr & 0xFFF] = out; chip_after_write(addr);
        }
        addr++;
    }
    nbytes++;
    return in;
}
uint spi_init(spi_inst_t *s, uint hz) { (void)s; return hz; }
void spi_set_format(spi_inst_t *s, uint b, int c, int p, int o) { (void)s; (void)b; (void)c; (void)p; (void)o; }
int spi_write_blocking(spi_inst_t *s, const uint8_t *src, size_t n) { (void)s; for (size_t i = 0; i < n; i++) xfer(src[i]); return (int)n; }
int spi_read_blocking(spi_inst_t *s, uint8_t tx, uint8_t *dst, size_t n) { (void)s; for (size_t i = 0; i < n; i++) dst[i] = xfer(tx); return (int)n; }

static void dump_cfg(const char *what, mcp_status_t s) {
    printf("%s -> %d (%s) CON=0x%08x OPMOD=%u NBTCFG=0x%08x DBTCFG=0x%08x TSCON=0x%08x INT=0x%08x\n"
           "    FIFOCON1=0x%08x FIFOCON2=0x%08x FIFOCON3=0x%08x UA1=0x%03x UA2=0x%03x UA3=0x%03x "
           "FLTCON0=0x%02x FLTOBJ0=0x%08x MASK0=0x%08x RAM %u/2048 STBY=%d\n",
           what, s, mcp251863_strerror(s), r32(0x000), (mem[2] >> 5) & 7, r32(0x004), r32(0x008), r32(0x014),
           r32(0x01C), r32(FCON(1)), r32(FCON(2)), r32(FCON(3)), r32(FUA(1)), r32(FUA(2)), r32(FUA(3)),
           mem[0x1D0], r32(0x1F0), r32(0x1F4), ram_used, stby_level);
}

int main(void)
{
    /* ---- nominal bit timing ---- */
    struct { uint32_t osc, br; } cases[] = {{16000000,500000},{20000000,500000},{40000000,500000},{16000000,250000},{16000000,1000000},{40000000,1000000}};
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        mcp_bittiming_t bt;
        bool ok = mcp251863_calc_bittiming(cases[i].osc, cases[i].br, &bt);
        CHECK(ok);
        uint32_t total = 1 + bt.tseg1 + bt.tseg2;
        printf("osc=%8lu br=%7lu -> BRP=%lu TQ=%lu TSEG1=%lu TSEG2=%lu SJW=%lu SP=%.2f%% NBTCFG=0x%08lx\n",
               (unsigned long)cases[i].osc, (unsigned long)cases[i].br, (unsigned long)bt.brp, (unsigned long)total,
               (unsigned long)bt.tseg1, (unsigned long)bt.tseg2, (unsigned long)bt.sjw, 100.0 * (1 + bt.tseg1) / total,
               (unsigned long)(((bt.brp - 1) << 24) | ((bt.tseg1 - 1) << 16) | ((bt.tseg2 - 1) << 8) | (bt.sjw - 1)));
        CHECK(total == bt.tq_per_bit);
        CHECK((double)cases[i].osc / (bt.brp * total) == (double)cases[i].br);
        double sp = (double)(1 + bt.tseg1) / total;
        CHECK(sp >= 0.75 && sp <= 0.875);
    }
    mcp_bittiming_t bt; mcp251863_calc_bittiming(16000000, 500000, &bt);
    CHECK(bt.brp == 1 && bt.tseg1 == 25 && bt.tseg2 == 6 && bt.sjw == 6);

    /* ---- FD data timing table (kept for the FD define) ---- */
    struct { uint32_t osc, dbr, dbtcfg, tdc; } fdc[] = {
        {16000000, 2000000, 0x00040101u, 0x00020500u}, {16000000, 1000000, 0x000A0303u, 0x00020B00u},
        {16000000, 4000000, 0x00010000u, 0x00020200u}, {40000000, 1000000, 0x001E0707u, 0x00021F00u},
        {40000000, 2000000, 0x000E0303u, 0x00020F00u}, {40000000, 4000000, 0x00060101u, 0x00020700u},
    };
    mcp_data_bittiming_t dbt;
    for (unsigned i = 0; i < sizeof fdc / sizeof fdc[0]; i++) {
        CHECK(mcp251863_calc_data_bittiming(fdc[i].osc, fdc[i].dbr, &dbt));
        CHECK(dbt.dbtcfg == fdc[i].dbtcfg && dbt.tdc == fdc[i].tdc);
    }

    mcp251863_cfg_t cfg = {0};
    cfg.pin_cs = 17; cfg.pin_stby = 21; cfg.pin_int = 20; cfg.osc_hz = 16000000; cfg.bitrate = 500000; cfg.spi_hz = 4000000;
    uint8_t devid = 0;
    mcp_status_t s;

    /* ---- default build: classic 500k, Listen Only, RX FIFO1 ---- */
    cfg.listen_only = true; cfg.data_bitrate = 0;
    chip_reset();
    s = mcp251863_init(&cfg, &bt, NULL, &devid);
    dump_cfg("init classic listen-only", s);
    CHECK(s == MCP_OK && devid == 0x14 && stby_level == 0);
    CHECK(r32(0x004) == 0x00180505u);                     // 500k, 81.25 %
    CHECK(((mem[2] >> 5) & 7) == 3);                      // OPMOD = Listen Only
    CHECK(r32(0x014) == 0x0001000Fu);                     // TBCEN, TBCPRE 15 -> 1 us
    CHECK(r32(FCON(1)) == 0x1F000029u);                   // RX: FSIZE 32, PLSIZE 8, RXTSEN, RXOVIE, TFNRFNIE
    CHECK(r32(FCON(2)) == 0x00200080u);                   // classic TX
    CHECK(r32(FCON(3)) == 0xE0200080u);                   // FD TX
    CHECK(mem[0x1D0] == 0x81 && r32(0x1F0) == 0 && r32(0x1F4) == 0); // filter0 -> FIFO1, accept all std+ext
    CHECK(r32(0x01C) == 0x08020000u);                     // RXOVIE | RXIE
    CHECK(r32(FUA(1)) == 0x000 && r32(FUA(2)) == 0x280 && r32(FUA(3)) == 0x290);
    CHECK(ram_used == 32 * 20 + 16 + 72 + 28 * 16 && ram_used <= 2048);

    /* RX decode */
    mcp_rx_frame_t f; bool ovf = false;
    CHECK(mcp251863_rx_read(&f, &ovf) == 0 && !ovf);      // empty
    uint8_t d8[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    rx_inject(0x123, 8, 1000, d8, 8);                                         // std, DLC 8
    uint32_t eid = 0x1ABCDE12u;                                               // 29-bit
    rx_inject(((eid >> 18) & 0x7FF) | ((eid & 0x3FFFF) << 11), (1u << 4) | 3, 2000, d8, 8); // ext, DLC 3
    rx_inject(0x7FF, (1u << 5) | 4, 3000, d8, 8);                             // RTR, DLC 4
    rx_inject(0x010, 12, 4000, d8, 8);                                        // classic DLC 12 -> 8 bytes
    CHECK(mcp251863_rx_read(&f, &ovf) == 1);
    printf("RX1 id=0x%lx ide=%d rtr=%d dlc=%u len=%u ts=%lu d0=%02x d7=%02x\n", (unsigned long)f.id, f.ide, f.rtr,
           f.dlc, f.len, (unsigned long)f.ts, f.data[0], f.data[7]);
    CHECK(f.id == 0x123 && !f.ide && !f.rtr && !f.fdf && f.dlc == 8 && f.len == 8 && f.ts == 1000 && !memcmp(f.data, d8, 8));
    CHECK(r32(FUA(1)) == 20);                              // UINC advanced one 20 B object
    CHECK(mcp251863_rx_read(&f, &ovf) == 1);
    printf("RX2 id=0x%lx ide=%d rtr=%d dlc=%u len=%u ts=%lu\n", (unsigned long)f.id, f.ide, f.rtr, f.dlc, f.len, (unsigned long)f.ts);
    CHECK(f.id == eid && f.ide && f.dlc == 3 && f.len == 3 && f.ts == 2000);
    CHECK(mcp251863_rx_read(&f, &ovf) == 1);
    CHECK(f.id == 0x7FF && f.rtr && f.dlc == 4 && f.len == 0);
    CHECK(mcp251863_rx_read(&f, &ovf) == 1);
    CHECK(f.dlc == 12 && f.len == 8);
    CHECK(mcp251863_rx_read(&f, &ovf) == 0);
    // overflow flag reported once and cleared
    mem[FSTA(1)] |= 0x08;
    rx_inject(0x321, 1, 5000, d8, 1);
    CHECK(mcp251863_rx_read(&f, &ovf) == 1 && ovf && !(mem[FSTA(1)] & 0x08));
    ovf = false;
    CHECK(mcp251863_rx_read(&f, &ovf) == 0 && !ovf);
    // TBC readback
    w32(0x010, 0xDEADBEEF);
    CHECK(mcp251863_read_tbc() == 0xDEADBEEF);

    /* ---- record encoding ---- */
    pico_can_record_t rec = {.bus = 2, .flags = 0, .dlc = 8, .len = 8, .id = 0x123, .time_us = 0x04030201};
    memcpy(rec.data, d8, 8);
    uint8_t wire[PICO_CAN_REC_MAX_WIRE];
    size_t n = pico_can_record_encode(&rec, wire);
    const uint8_t exp_std[] = {0x14, 0x80, 0x43, 2, 0x00, 8, 0x23, 0x01, 0, 0, 0x01, 0x02, 0x03, 0x04,
                               0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    printf("record std:"); for (size_t i = 0; i < n; i++) printf(" %02x", wire[i]); printf("\n");
    CHECK(n == sizeof exp_std && !memcmp(wire, exp_std, n));
    pico_can_record_t rx = {.bus = 2, .flags = PICO_CAN_FLAG_IDE | PICO_CAN_FLAG_RTR, .dlc = 2, .len = 0, .id = eid, .time_us = 7};
    n = pico_can_record_encode(&rx, wire);
    const uint8_t exp_ext[] = {0x0C, 0x80, 0x43, 2, 0x03, 2, 0x12, 0xDE, 0xBC, 0x1A, 7, 0, 0, 0};
    printf("record ext rtr:"); for (size_t i = 0; i < n; i++) printf(" %02x", wire[i]); printf("\n");
    CHECK(n == sizeof exp_ext && !memcmp(wire, exp_ext, n));

    /* ---- Normal mode classic (bench ACK build) + test TX ---- */
    cfg.listen_only = false;
    chip_reset();
    s = mcp251863_init(&cfg, &bt, NULL, &devid);
    dump_cfg("init classic normal", s);
    CHECK(s == MCP_OK && ((mem[2] >> 5) & 7) == 6);       // OPMOD = CAN 2.0
    s = mcp251863_send_std(0x123, d8, 8);
    CHECK(s == MCP_OK && last_wr_addr == 0x680 && last_wr_len == 16);
    CHECK(r32(0x680) == 0x123 && (r32(0x684) & 0x1FF) == 8 && !memcmp(&mem[0x688], d8, 8));
    CHECK(r32(FCON(2)) & (1u << 9));

    /* ---- FD normal build (V2_CAN_FD_DATA_BITRATE 2000000) ---- */
    cfg.data_bitrate = 2000000;
    chip_reset();
    s = mcp251863_init(&cfg, &bt, &dbt, &devid);
    dump_cfg("init FD normal", s);
    CHECK(s == MCP_OK && ((mem[2] >> 5) & 7) == 0 && r32(0x008) == 0x00040101u && r32(0x00C) == 0x00020500u);
    CHECK(r32(FCON(1)) == 0xEF000029u);                   // RX 16 x 64 B
    CHECK(ram_used == 16 * 76 + 16 + 72 + 28 * 16 && ram_used <= 2048);
    uint8_t fd[64]; for (int i = 0; i < 64; i++) fd[i] = (uint8_t)i;
    s = mcp251863_send_fd_std(0x124, fd, 64, true);
    CHECK(s == MCP_OK && last_wr_len == 72 && (r32(0x400 + r32(FUA(3)) + 4) & 0x1FF) == ((1u << 7) | (1u << 6) | 15));
    rx_inject(0x124, (1u << 7) | (1u << 6) | 15, 9, fd, 64);  // FD RX, 64 bytes
    CHECK(mcp251863_rx_read(&f, &ovf) == 1 && f.fdf && f.brs && f.len == 64 && !memcmp(f.data, fd, 64));
    /* FD frame seen by a classic-config RX FIFO (8 B payload): truncated to 8 */
    cfg.data_bitrate = 0; cfg.listen_only = true;
    chip_reset();
    s = mcp251863_init(&cfg, &bt, NULL, &devid);
    rx_inject(0x124, (1u << 7) | 15, 9, fd, 8);
    CHECK(mcp251863_rx_read(&f, &ovf) == 1 && f.fdf && f.dlc == 15 && f.len == 8);

    /* ---- missing chip ---- */
    chip_absent = 1;
    s = mcp251863_init(&cfg, &bt, NULL, &devid);
    printf("init with no chip (MISO=0xFF) -> %d (%s)\n", s, mcp251863_strerror(s));
    CHECK(s == MCP_ERR_SPI);
    CHECK(mcp251863_rx_read(&f, &ovf) == MCP_ERR_SPI);
    chip_absent = 0;
    printf("%s (%d failures)\n", fails ? "FAILED" : "ALL CHECKS PASSED", fails);
    return fails != 0;
}
