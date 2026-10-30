#pragma once
#include "pico/stdlib.h"
#define GPIO_OUT 1
#define GPIO_IN 0
#define GPIO_FUNC_SPI 1
void gpio_init(uint p); void gpio_set_dir(uint p, bool o); void gpio_put(uint p, bool v);
void gpio_pull_up(uint p); void gpio_set_function(uint p, int f);
