/**
  ******************************************************************************
  * @file    touch_tsc2046.h
  * @author  Krasutski Denis (Krasutski.Denis@gmail.com)
  * @version V1.0.0
  * @date    05-January-2015
  * @brief   This file contains the defines for work with tsc2046
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 </center></h2>
  ******************************************************************************
  */

#ifndef __TOUCH_TSC2046_H
#define __TOUCH_TSC2046_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

#ifdef TSC2046_LOG_ENABLE
#define TSC2046_LOG	 printf
#else
#define TSC2046_LOG(...)
#endif


/**
* @brief Control Byte
* The control byte (on DIN), as shown in Table 3, provides the
* start conversion, addressing, ADC resolution, configuration,
* and power-down of the TSC2046. Figure 9, Table 3 and
* Table 4 give detailed information regarding the order and
* description of these control bits within the control byte.
*/
#define TSC2046_REG_START_BIT			(0x80) /*<!-- The first bit, the S bit, must always be high and initiates the start of the control byte. The TSC2046 ignores inputs on the DIN pin until the start bit is detected. */
#define TSC2046_REG_CHAN0_BIT			(0x00) /*<!-- The next three bits (A2, A1, and A0) select the active input channel(s) of the input multiplexer (see Table 1, Table 2, and Figure 2), touch screen drivers, and the reference inputs. */
#define TSC2046_REG_CHAN1_BIT			(0x10)
#define TSC2046_REG_CHAN2_BIT			(0x20)
#define TSC2046_REG_CHAN3_BIT			(0x30)
#define TSC2046_REG_CHAN4_BIT			(0x40)
#define TSC2046_REG_CHAN5_BIT			(0x50)
#define TSC2046_REG_CHAN6_BIT			(0x60)
#define TSC2046_REG_CHAN7_BIT 		(0x70)
#define TSC2046_REG_MODE_12_BIT		(0x00) /*<!-- The mode bit sets the resolution of the ADC. With this bit low, the next conversion has 12 bits of resolution, whereas with this bit high, the next conversion has eight bits of resolution. */
#define TSC2046_REG_MODE_8_BIT		(0x08) /*<!-- The mode bit sets the resolution of the ADC. With this bit low, the next conversion has 12 bits of resolution, whereas with this bit high, the next conversion has eight bits of resolution. */
#define TSC2046_REG_SERDFR_S_BIT	(0x04) /*<!-- The SER/DFR bit controls the reference mode, either single-ended (high) or differential (low). The differential mode is also referred to as the ratiometric conversion mode and is preferred for X-Position, Y-Position, and Pressure-Touch measurements for optimum performance. */
#define TSC2046_REG_SERDFR_D_BIT	(0x00) /*<!-- The SER/DFR bit controls the reference mode, either single-ended (high) or differential (low). The differential mode is also referred to as the ratiometric conversion mode and is preferred for X-Position, Y-Position, and Pressure-Touch measurements for optimum performance. */
#define TSC2046_REG_PD_EN1_BIT		(0x00) /*<!-- Power-Down Between Conversions. When each conversion is finished, the converter enters a low-power mode. At the start of the next conversion, the device instantly powers up to full power. There is no need for additional delays to ensure full operation, and the very first conversion is valid. The Y? switch is on when in power-down */
#define TSC2046_REG_PD_DIS1_BIT		(0x01) /*<!-- Reference is off and ADC is on. */
#define TSC2046_REG_PD_EN2_BIT		(0x02) /*<!-- Reference is on and ADC is off. */
#define TSC2046_REG_PD_DIS2_BIT		(0x03) /*<!-- Device is always powered. Reference is on and ADC is on. */

#define TSC2046_REG_READ_DATA			(0x00)

//static uint16_t _tsc2046_ReadY(void);
//static uint16_t _tsc2046_ReadX(void);


uint32_t Touch_GetPos(uint32_t *x, uint32_t *y);
#ifdef __cplusplus
}
#endif

#endif


















