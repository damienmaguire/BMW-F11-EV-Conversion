// Host stand-in for the pioasm output of src/dde_emu_tx.pio (offsets only).
#ifndef DDE_EMU_TX_PIO_H_STUB
#define DDE_EMU_TX_PIO_H_STUB
#include "hardware/pio.h"
#define dde_emu_tx_offset_idle 0u
static const uint16_t sim_tx_instr[21] = {0}, sim_fire_instr[3] = {0};
static const pio_program_t dde_emu_tx_program = {sim_tx_instr, 21, -1};
static const pio_program_t dde_emu_fire_program = {sim_fire_instr, 3, -1};
static inline pio_sm_config dde_emu_tx_program_get_default_config(uint off) { (void)off; return pio_get_default_sm_config(); }
static inline pio_sm_config dde_emu_fire_program_get_default_config(uint off) { (void)off; return pio_get_default_sm_config(); }
#endif
