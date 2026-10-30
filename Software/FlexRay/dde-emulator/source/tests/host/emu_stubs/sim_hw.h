// Host stand-ins for the Pico SDK pieces src/dde_emu.c uses. The replay test
// (test_dde_emu_replay.c) implements the sim_* functions: a 10 ns tick clock,
// TIMER0 alarm, PWM fine counter, the PIO TX + fire state machines and the
// flexray_ts side-0/side-1 mark rings.
#ifndef SIM_HW_H
#define SIM_HW_H
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef unsigned int uint;
#define __time_critical_func(f) f
#define __not_in_flash_func(f) f

uint32_t time_us_32(void);
// busy-wait step inside an ISR (dde_emu.c DDE_EMU_SPIN_HOOK): the sim advances the
// clock by a few ticks and delivers the frame-start marks that happen meanwhile.
void sim_spin(void);
#define DDE_EMU_SPIN_HOOK() sim_spin()
static inline void tight_loop_contents(void) {}
#define DDE_EMU_TEST_HOOKS 1
static inline void hw_set_bits(volatile uint32_t *a, uint32_t m) { *a |= m; }
static inline void hw_clear_bits(volatile uint32_t *a, uint32_t m) { *a &= ~m; }

// gpio
#define GPIO_OUT 1
enum { GPIO_FUNC_SIO = 5, GPIO_FUNC_PIO0 = 6 };
void sim_gpio_put(uint pin, bool v);
void sim_gpio_set_function(uint pin, int fn);
static inline void gpio_put(uint pin, bool v) { sim_gpio_put(pin, v); }
static inline void gpio_set_dir(uint pin, int out) { (void)pin; (void)out; }
static inline void gpio_set_function(uint pin, int fn) { sim_gpio_set_function(pin, fn); }
static inline void gpio_init(uint pin) { sim_gpio_set_function(pin, GPIO_FUNC_SIO); }

// irq / sync
static inline void irq_set_exclusive_handler(uint n, void (*h)(void)) { extern void (*sim_alarm_handler)(void); (void)n; sim_alarm_handler = h; }
static inline void irq_set_priority(uint n, uint p) { (void)n; (void)p; }
static inline void irq_set_enabled(uint n, bool e) { (void)n; (void)e; }
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t s) { (void)s; }

// timer
typedef struct { volatile uint32_t alarm[4]; volatile uint32_t intr, inte, intf; } sim_timer_hw_t;
extern sim_timer_hw_t sim_timer;
#define timer_hw (&sim_timer)
static inline int hardware_alarm_claim_unused(bool req) { (void)req; return 0; }
static inline uint timer_hardware_alarm_get_irq_num(sim_timer_hw_t *t, uint n) { (void)t; return n; }

// pwm (fine counter)
typedef struct { volatile uint32_t csr, div, ctr, cc, top; } sim_pwm_slice_t;
typedef struct { sim_pwm_slice_t slice[12]; } sim_pwm_hw_t;
extern sim_pwm_hw_t sim_pwm;
#define pwm_hw (&sim_pwm)

// pio
typedef struct { int id; } sim_pio_t;
typedef sim_pio_t *PIO;
extern sim_pio_t sim_pio0;
#define pio0 (&sim_pio0)
typedef struct { uint32_t dummy; } pio_sm_config;
typedef struct { const uint16_t *instructions; uint8_t length; int8_t origin; } pio_program_t;
enum pio_src_dest { pio_pins = 0 };
enum pio_fifo_join { PIO_FIFO_JOIN_NONE = 0, PIO_FIFO_JOIN_TX = 1 };
uint sim_pio_add_program(PIO pio, const pio_program_t *p);
void sim_pio_sm_put(PIO pio, uint sm, uint32_t v);
uint sim_pio_sm_get_pc(PIO pio, uint sm);
bool sim_pio_sm_tx_fifo_empty(PIO pio, uint sm);
void sim_pio_sm_set_enabled(PIO pio, uint sm, bool en);
void sim_pio_sm_reset(PIO pio, uint sm);   // clear fifos / restart / jmp idle
static inline uint pio_add_program(PIO pio, const pio_program_t *p) { return sim_pio_add_program(pio, p); }
static inline int pio_claim_unused_sm(PIO pio, bool r) { static int n; (void)pio; (void)r; return n++ & 3; }
static inline pio_sm_config pio_get_default_sm_config(void) { pio_sm_config c = {0}; return c; }
static inline void sm_config_set_set_pins(pio_sm_config *c, uint b, uint n) { (void)c; (void)b; (void)n; }
static inline void sm_config_set_out_pins(pio_sm_config *c, uint b, uint n) { (void)c; (void)b; (void)n; }
static inline void sm_config_set_out_shift(pio_sm_config *c, bool r, bool a, uint t) { (void)c; (void)r; (void)a; (void)t; }
static inline void sm_config_set_fifo_join(pio_sm_config *c, enum pio_fifo_join j) { (void)c; (void)j; }
static inline void sm_config_set_clkdiv_int_frac8(pio_sm_config *c, uint i, uint8_t f) { (void)c; (void)i; (void)f; }
static inline void pio_sm_init(PIO pio, uint sm, uint off, const pio_sm_config *c) { (void)off; (void)c; sim_pio_sm_reset(pio, sm); }
static inline void pio_sm_set_pins_with_mask(PIO pio, uint sm, uint32_t v, uint32_t m) { (void)pio; (void)sm; (void)v; (void)m; }
static inline void pio_sm_set_pindirs_with_mask(PIO pio, uint sm, uint32_t v, uint32_t m) { (void)pio; (void)sm; (void)v; (void)m; }
static inline void pio_gpio_init(PIO pio, uint pin) { (void)pio; sim_gpio_set_function(pin, GPIO_FUNC_PIO0); }
static inline void pio_interrupt_clear(PIO pio, uint n) { (void)pio; (void)n; }
static inline void pio_sm_set_enabled(PIO pio, uint sm, bool en) { sim_pio_sm_set_enabled(pio, sm, en); }
static inline uint8_t pio_sm_get_pc(PIO pio, uint sm) { return (uint8_t)sim_pio_sm_get_pc(pio, sm); }
static inline bool pio_sm_is_tx_fifo_empty(PIO pio, uint sm) { return sim_pio_sm_tx_fifo_empty(pio, sm); }
static inline void pio_sm_clear_fifos(PIO pio, uint sm) { sim_pio_sm_reset(pio, sm); }
static inline void pio_sm_restart(PIO pio, uint sm) { sim_pio_sm_reset(pio, sm); }
static inline void pio_sm_exec(PIO pio, uint sm, uint instr) { (void)pio; (void)sm; (void)instr; }
static inline uint pio_encode_set(enum pio_src_dest d, uint v) { (void)d; return v; }
static inline uint pio_encode_jmp(uint a) { return a; }
static inline void pio_sm_put(PIO pio, uint sm, uint32_t v) { sim_pio_sm_put(pio, sm, v); }
#endif
