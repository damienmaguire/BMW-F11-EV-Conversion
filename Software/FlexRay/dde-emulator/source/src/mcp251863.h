// Minimal SPI driver for the Microchip MCP251863 (MCP2518FD CAN FD controller
// + ATA6563 transceiver in one package). RX via FIFO1 (accept-all filter,
// time stamped, nINT on "FIFO not empty" / overflow); test TX via FIFO2/FIFO3.
#ifndef MCP251863_H
#define MCP251863_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/spi.h"

typedef enum {
    MCP_OK = 0,
    MCP_ERR_SPI = -1,        // SPI readback / reset-state check failed (no chip, wiring, power)
    MCP_ERR_OSC = -2,        // oscillator never became ready (crystal not running)
    MCP_ERR_MODE = -3,       // controller refused the requested operating mode
    MCP_ERR_BITTIMING = -4,  // no valid bit timing for the configured oscillator
    MCP_ERR_TX_FULL = -5,    // TX FIFO still holds the previous frame
} mcp_status_t;

typedef struct {
    uint32_t brp, tseg1, tseg2, sjw; // real values
    uint32_t tq_per_bit, sample_point_permille;
    uint32_t tdco, tdcmod;           // TDCO in SYSCLK periods; TDCMOD 2 = auto
    uint32_t dbtcfg, tdc;            // register images (C1DBTCFG, C1TDC)
} mcp_data_bittiming_t;

typedef struct {
    spi_inst_t *spi;
    uint pin_miso, pin_mosi, pin_sck, pin_cs, pin_int, pin_stby;
    uint32_t spi_hz;
    uint32_t osc_hz;
    uint32_t bitrate;
    uint32_t data_bitrate;  // CAN FD data phase (BRS); 0 = classic CAN 2.0 only mode
    bool listen_only;       // true: REQOP Listen Only (never ACKs / error-frames / transmits)
    uint8_t tx_fifo_depth;  // classic TX FIFO2 depth, 1..32 (0 = 1)
    uint8_t tef_depth;      // v1f: TX event FIFO depth 1..32 (0 = no TEF). With a TEF the
                            // controller notes every frame it SENT SUCCESSFULLY (ACKed) with
                            // the SOF time stamp, so the logger can show our own TX.
} mcp251863_cfg_t;

// FIFO layout (TXQ disabled; TEF only if cfg->tef_depth, 16 x 12 B = 192 B in the
// emulator build, in front of FIFO1 -> 1608 B total; otherwise FIFO1 starts at RAM 0x400):
//   FIFO1 RX, time stamped, 32 x 8 B payload  (32 x 20 B = 640 B)   classic build
//                           16 x 64 B payload (16 x 76 B = 1216 B)  FD build
//   FIFO2 TX classic, 1 x 8 B   (16 B)
//   FIFO3 TX CAN FD,  1 x 64 B  (72 B)
//   FIFO4..31 left at reset default (RX, 1 x 8 B, no TS = 28 x 16 B = 448 B)
//   total 1176 B (classic) / 1752 B (FD) of 2048 B
#define MCP_FIFO_RX      1
#define MCP_FIFO_CLASSIC 2
#define MCP_FIFO_FD      3
#define MCP_RX_DEPTH_CLASSIC 32
#define MCP_RX_DEPTH_FD      16

typedef struct {
    uint32_t id;       // 11 or 29 bit
    bool ide, rtr, fdf, brs, esi;
    uint8_t dlc;       // as received
    uint8_t len;       // data bytes stored in data[] (<= FIFO payload size)
    uint32_t ts;       // RXMSGTS: time base counter at SOF, 1 us/tick (TBCPRE)
    uint8_t data[64];
} mcp_rx_frame_t;

typedef struct {
    uint32_t brp, tseg1, tseg2, sjw; // real values (register value + 1)
    uint32_t tq_per_bit;
    uint32_t sample_point_permille;
} mcp_bittiming_t;

typedef struct {
    bool txreq_pending; // FIFO still trying to send
    bool tx_attempts_exhausted; // TXATIF: frame dropped after retries
    bool tx_err, tx_lost_arb, tx_aborted;
    uint8_t tec, rec;
    bool bus_off, tx_passive, rx_passive, tx_warn, rx_warn;
    uint32_t bdiag1;
    uint8_t opmod;
} mcp_tx_state_t;

// Pure function (host-testable): compute nominal bit timing for classic CAN.
bool mcp251863_calc_bittiming(uint32_t osc_hz, uint32_t bitrate, mcp_bittiming_t *bt);
bool mcp251863_calc_data_bittiming(uint32_t osc_hz, uint32_t dbitrate, mcp_data_bittiming_t *d);
uint8_t mcp251863_len_to_dlc(uint8_t len);
uint8_t mcp251863_dlc_to_len(uint8_t dlc);

mcp_status_t mcp251863_init(const mcp251863_cfg_t *cfg, mcp_bittiming_t *bt_out,
                            mcp_data_bittiming_t *dbt_out, uint8_t *devid_out);
#ifndef MCP251863_RX_ONLY // RX-only builds have no send functions at all
// Classic CAN 2.0 frame via FIFO1 (unchanged behaviour from the first test).
mcp_status_t mcp251863_send_std(uint16_t id, const uint8_t *data, uint8_t dlc);
// CAN FD frame (FDF=1, optional BRS) via FIFO2; len rounded up to a valid FD length.
mcp_status_t mcp251863_send_fd_std(uint16_t id, const uint8_t *data, uint8_t len, bool brs);
// Runtime mode change via Configuration mode (aborts anything still queued in the TX
// FIFOs): normal = Normal CAN 2.0 (CAN FD if a data bit rate is configured), else
// Listen Only. Blocks up to 2 x 20 ms waiting for the controller.
mcp_status_t mcp251863_set_normal(bool normal);
// Sequence number the NEXT queued frame will carry (23 bits; the TEF returns it).
uint32_t mcp251863_next_seq(void);
// One TX event (a frame we sent, ACKed by the bus). Returns 1 = event read, 0 = TEF
// empty or not configured, MCP_ERR_SPI on a bad read. *overflow is set if the TEF had
// overflowed (events lost) since the last call (TEFOVIF, cleared here).
typedef struct {
    uint32_t id;     // 11-bit ID (we only send standard frames)
    uint32_t seq;    // 23-bit sequence number given at queue time
    uint8_t dlc;
    uint32_t ts;     // time base counter at SOF (same 1 us clock as RXMSGTS)
} mcp_tef_t;
int mcp251863_tef_read(mcp_tef_t *e, bool *overflow);
#endif
mcp_status_t mcp251863_read_tx_state_fifo(uint8_t fifo, mcp_tx_state_t *st);
mcp_status_t mcp251863_read_tx_state(mcp_tx_state_t *st); // FIFO1
void mcp251863_clear_tx_attempt_flag_fifo(uint8_t fifo);
void mcp251863_clear_tx_attempt_flag(void); // FIFO1
// Reads one frame from the RX FIFO. Returns 1 = frame read, 0 = FIFO empty,
// MCP_ERR_SPI on a bad read. *overflow is set if the FIFO had overflowed since
// the last call (RXOVIF, cleared here).
int mcp251863_rx_read(mcp_rx_frame_t *f, bool *overflow);
uint32_t mcp251863_read_tbc(void);  // C1TBC, same 1 us time base as RXMSGTS
const char *mcp251863_strerror(mcp_status_t s);

#endif
