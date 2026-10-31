// PT-CAN DDE/EGS emulation TX glue (V2 board, DDE emulator build with EMU_CAN_TX=1).
// Scheduling / content / gating live in ptcan_emu_core.c (host-testable); this file
// owns the MCP251863 mode (Listen Only <-> Normal), the TX FIFO and the host interface.
#ifndef PTCAN_EMU_H
#define PTCAN_EMU_H

#include <stdbool.h>
#include <stdint.h>
#include "mcp251863.h"

#define PICOFLEX_SET_CANTX 0xa5u  // vendor OUT: wValue = ID mask bits 0..15, wIndex = bits 16..31; 0 = off + clear fault
#define PICOFLEX_GET_CANTX 0xa6u  // vendor IN: ptcan_stats_t (178 bytes)
#define PTCAN_EMU_KEEPALIVE_MS 20000u  // host must repeat SET_CANTX within this, else TX off

// Stats for vendor request 0xa6 (GET_CANTX), little endian, packed. Python:
// struct '<BBBBIHBBBBBBIIIiiiiiIIII19H19I'
#define PTCAN_STATS_VERSION 1
typedef struct __attribute__((packed)) {
    uint8_t version;        // PTCAN_STATS_VERSION
    uint8_t state;          // pte_state_t
    uint8_t fault;          // pte_fault_t
    uint8_t flags;          // b0 enabled, b1 controller in Normal mode, b2 KL15, b3 KL15 seen this wake,
                            // b4 bus-off, b5 TX error passive, b6 RX error passive, b7 keepalive timed out
    uint32_t mask;          // ID selection in use
    uint16_t fault_id;      // ID that caused FOREIGN
    uint8_t st_kl;          // last 0x12F ST_KL nibble
    uint8_t n_frames;       // PTE_N_FRAMES
    uint8_t tec, rec, opmod, reserved;
    uint32_t rx_frames, n12f, wakes;
    int32_t ms_since_wake, ms_since_rx, ms_since_on, ms_since_off, ms_since_foreign_nm; // -1 = n/a
    uint32_t tx_total, tx_fifo_full, tx_fail, mode_switches;
    uint16_t id[19];
    uint32_t tx[19];
} ptcan_stats_t;

void ptcan_emu_on_can_init(bool ok);
void ptcan_emu_on_rx(const mcp_rx_frame_t *f, uint32_t t_us32);
bool ptcan_emu_poll(void);   // false = SPI failure (caller re-inits the controller)
bool ptcan_emu_tick(void);   // 50 ms housekeeping; false = SPI failure
void ptcan_emu_print_stats(void);
void ptcan_emu_fill_health_tx(uint32_t *tx_cnt, uint32_t *err_cnt);

// Host interface (USB control requests, core 0). mask 0 = TX off + clear latched fault.
void ptcan_emu_host_set(uint32_t mask);
uint16_t ptcan_emu_get_stats(uint8_t *buf, uint16_t max);

#endif
