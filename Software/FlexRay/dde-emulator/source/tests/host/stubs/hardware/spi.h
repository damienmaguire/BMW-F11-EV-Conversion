#pragma once
#include "pico/stdlib.h"
typedef struct spi_inst spi_inst_t;
#define SPI_CPOL_0 0
#define SPI_CPHA_0 0
#define SPI_MSB_FIRST 1
uint spi_init(spi_inst_t *s, uint hz);
void spi_set_format(spi_inst_t *s, uint bits, int cpol, int cpha, int order);
int spi_write_blocking(spi_inst_t *s, const uint8_t *src, size_t len);
int spi_read_blocking(spi_inst_t *s, uint8_t tx, uint8_t *dst, size_t len);
