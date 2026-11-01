// host test stub (test_ptcan_tef.c): simulated time, advanced by the test.
#pragma once
#include <stdbool.h>
#include <stdint.h>
extern uint64_t sim_now_us;
typedef uint64_t absolute_time_t;
typedef struct repeating_timer { int dummy; } repeating_timer_t;
typedef bool (*repeating_timer_callback_t)(repeating_timer_t *);
static inline uint64_t time_us_64(void) { return sim_now_us; }
static inline uint32_t time_us_32(void) { return (uint32_t)sim_now_us; }
static inline absolute_time_t get_absolute_time(void) { return sim_now_us; }
static inline uint32_t to_ms_since_boot(absolute_time_t t) { return (uint32_t)(t / 1000u); }
static inline bool add_repeating_timer_ms(int32_t ms, repeating_timer_callback_t cb, void *u, repeating_timer_t *t)
{ (void)ms; (void)cb; (void)u; (void)t; return true; }
