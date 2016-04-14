/* spi device driver
   (c) Andrzej Szombierski, 2016 */

#ifndef __SPI_H
#define __SPI_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pgm.h"

typedef struct spi_context_s {
	int spi_fd;
} spi_context_t;

typedef enum {
	SPI_OK = 0,
	SPI_SPI_ERROR,
	SPI_SWIM_ERROR
} spi_status_t;

bool spi_open(programmer_t *pgm);
void spi_close(programmer_t *pgm);
void spi_srst(programmer_t *pgm);
void spi_swim_srst(programmer_t *pgm);
int spi_swim_read_range(programmer_t *pgm, const stm8_device_t *device, unsigned char *buffer, unsigned int start, unsigned int length);
int spi_swim_write_range(programmer_t *pgm, const stm8_device_t *device, unsigned char *buffer, unsigned int start, unsigned int length, const memtype_t memtype);

#endif
