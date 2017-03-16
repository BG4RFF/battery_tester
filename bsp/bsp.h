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

#ifdef __cplusplus
 }
#endif /* __cplusplus */


#endif /* __BSP_H_H__ */

/* end: bsp.h ----- */
