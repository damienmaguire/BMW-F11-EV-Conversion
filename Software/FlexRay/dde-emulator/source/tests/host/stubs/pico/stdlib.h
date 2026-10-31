#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef unsigned int uint;
typedef uint64_t absolute_time_t;
absolute_time_t make_timeout_time_ms(uint32_t ms);
bool time_reached(absolute_time_t t);
void sleep_ms(uint32_t ms);
void sleep_us(uint64_t us);
static inline void busy_wait_at_least_cycles(uint32_t c) { (void)c; }
