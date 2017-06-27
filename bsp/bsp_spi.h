#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include <stdint.h>

void spi_init(void);
void bsp_spi1_wr_rd(uint8_t *wr_data, uint8_t *rd_data, uint32_t size);

#endif // __BSP_SPI_H__