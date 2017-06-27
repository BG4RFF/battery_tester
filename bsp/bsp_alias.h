#ifndef _BSP_ALIAS_H_
#define _BSP_ALIAS_H_


/* -------------------------------------------------------------------------- */

inline void bsp_touch_wr_rd(uint8_t *wr_data, uint8_t *rd_data, uint32_t size) {
    bsp_spi1_wr_rd(wr_data, rd_data, size);
}

/* -------------------------------------------------------------------------- */

inline void bsp_sd_wr_rd(uint8_t *wr_data, uint8_t *rd_data, uint32_t size) {
    bsp_spi1_wr_rd(wr_data, rd_data, size);
}

/* -------------------------------------------------------------------------- */

inline void bsp_debug_write(const uint8_t *data, uint32_t size) {

    bsp_uart1_write(data, size);
}




#endif //_BSP_ALIAS_H_