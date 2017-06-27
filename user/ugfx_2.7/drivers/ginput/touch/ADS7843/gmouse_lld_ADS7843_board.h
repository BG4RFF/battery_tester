/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#ifndef _GINPUT_LLD_MOUSE_BOARD_H
#define _GINPUT_LLD_MOUSE_BOARD_H

#include "bsp.h"

// Resolution and Accuracy Settings
#define GMOUSE_ADS7843_PEN_CALIBRATE_ERROR		50*2
#define GMOUSE_ADS7843_PEN_CLICK_ERROR			12
#define GMOUSE_ADS7843_PEN_MOVE_ERROR			4
#define GMOUSE_ADS7843_FINGER_CALIBRATE_ERROR	50*2
#define GMOUSE_ADS7843_FINGER_CLICK_ERROR		18
#define GMOUSE_ADS7843_FINGER_MOVE_ERROR		14

// How much extra data to allocate at the end of the GMouse structure for the board's use
#define GMOUSE_ADS7843_BOARD_DATA_SIZE			0

static bool_t init_board(GMouse* m, unsigned driverinstance) {
    return TRUE;
}

static GFXINLINE bool_t getpin_pressed(GMouse* m) {
    return bsp_is_touch_irq_active();
}

static GFXINLINE void aquire_bus(GMouse* m) {

    bsp_cs_touch_set_low();
}

static GFXINLINE void release_bus(GMouse* m) {

    bsp_cs_touch_set_high();
}

static GFXINLINE uint16_t read_value(GMouse* m, uint16_t port) {

    uint8_t txbuf[3] = {0, 0, 0};
    uint8_t rxbuf[3] = {0, 0, 0};

    (void)m;

    txbuf[0] = (uint8_t)port;
    bsp_touch_wr_rd(txbuf, rxbuf, 3);

    return (rxbuf[0] << 5) | (rxbuf[1] >> 3);
}

#endif /* _GINPUT_LLD_MOUSE_BOARD_H */
