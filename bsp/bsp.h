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

typedef enum {
    VOLTAGE_SOURCE_VBAT = 0,
    VOLTAGE_SOURCE_VIN,
}bsp_voltage_source_t;



typedef enum {
    LCD_BACKLIGHT_DISABLE = 0,
    LCD_BACKLIGHT_ENABLE,
}bsp_backlight_control_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void bsp_switch_relay(const bsp_relay_swith_t select);
void bsp_init(void);
void bsp_debug_write(const uint8_t *data, uint32_t size);
void bsp_delay_ms(uint32_t ms);
uint32_t bsp_get_tick_ms(void);
void bsp_enable_irq(void);
void bsp_disable_irq(void);
bool bsp_is_charger_flag_high(void);
bool bsp_is_charger_swire_pin_high(void);
void bsp_toggle_charger_swire_pin(void);
void bsp_charger_timer_set_period(uint32_t us, void (*cb)(void));
void bsp_charger_timer_disable(void);
void bsp_register_charger_status_cb(void (*cb)(void));
void bsp_charger_en_set_low(void);
void bsp_charger_en_set_high(void);
uint32_t bsp_get_voltage(bsp_voltage_source_t source);
void bsp_lcd_write_reg(uint32_t reg, uint16_t data);
uint16_t bsp_lcd_read_reg(uint32_t reg);
void bsp_lcd_write_ram(uint16_t data);
void bsp_lcd_write_prepare(void);
void bsp_lcd_write_command(uint16_t data);
uint16_t bsp_lcd_read_reg(uint32_t reg);
void bsp_lcd_backlight_enable(bsp_backlight_control_t control);
#ifdef __cplusplus
 }
#endif /* __cplusplus */


#endif /* __BSP_H_H__ */

/* end: bsp.h ----- */
