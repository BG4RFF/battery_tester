/**
  ******************************************************************************
  * @file    bsp.h
  * @author  DKrasutski@gmail.com
  * @version V1.0
  * @date    3/16/2017
  * @brief   This file contains all the functions prototypes for the HAL
  *          module driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 Denis Krasutski </center></h2>
  *
  ******************************************************************************
  */

#ifndef __BSP_H_H__
#define __BSP_H_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Exported types ------------------------------------------------------------*/

typedef enum bsp_relay_swith_e {
    RELAY_CHARGE,
    RELAY_DISCHARGE,

    RELAY_COUNT
}
bsp_relay_swith_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void bsp_switch_relay(const bsp_relay_swith_t select);
void bsp_init(void);
void bsp_debug_write(const uint8_t *data, uint32_t size);
void bsp_delay_ms(uint32_t ms);

#ifdef __cplusplus
 }
#endif /* __cplusplus */


#endif /* __BSP_H_H__ */

/* end: bsp.h ----- */
