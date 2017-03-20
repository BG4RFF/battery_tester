/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "logger/logger.h"
#include "charger/charger.h"
#include "lcd_driver/ili9320.h"
#include "touch_driver/touch_tsc2046.h"

/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

int main(void)
{

    bsp_init();

    bsp_lcd_backlight_enable(LCD_BACKLIGHT_ENABLE);
    ili9320_Initializtion();
    ili9320_Clear(Black);

    uint32_t x, y;

    if(bsp_is_button2_pressed()) {

        while(!bsp_is_button1_pressed()) {

            Touch_GetPos(&x, &y);
            float fx = 320-(float)x / 4096.f*2 * 320;
            float fy = (float)y / 4096.f*2 * 240;

            //INFO("%d %d\r\n", x, y);

            ili9320_SetPoint((uint32_t)fx, (uint32_t)fy + 1, Green);
            ili9320_SetPoint((uint32_t)fx, (uint32_t)fy - 1, Green);
            ili9320_SetPoint((uint32_t)fx + 1, (uint32_t)fy, Green);
            ili9320_SetPoint((uint32_t)fx - 1, (uint32_t)fy, Green);
            ili9320_SetPoint((uint32_t)fx, (uint32_t)fy, Green);
        }
    }

    bsp_led1_enable(LED_DISABLE);
    bsp_delay_ms(500);
    bsp_led2_enable(LED_DISABLE);
    bsp_delay_ms(500);
    bsp_led3_enable(LED_DISABLE);
    bsp_delay_ms(500);


    ili9320_Clear(Blue);

    INFO("\r\n-=Battery tester=-\r\n");

    bsp_switch_relay(RELAY_CHARGE);

    charger_enable(CHARGER_DISABLE);
    bsp_delay_ms(1000);
    charger_enable(CHARGER_ENABLE);


    charger_send_swp_message(SWPC_AUTO_RECHARGE_ON);
    bsp_delay_ms(100);

    charger_send_swp_message(SWPC_AUTO_RECHARGE_OFF);
    bsp_delay_ms(100);
    charger_send_swp_message(SWPC_VFLOAT_BAT_AGING_OFF);
    bsp_delay_ms(100);
    charger_send_swp_message(SWPC_INCREASE_150MV);
    bsp_delay_ms(100);
    /* Infinite loop */
    while (1) {
        /* test demo*/
        bsp_delay_ms(1000);

        INFO("Vbat = %d, Vin = %d ", bsp_get_voltage(VOLTAGE_SOURCE_VBAT), bsp_get_voltage(VOLTAGE_SOURCE_VIN));

        switch(charger_get_status()) {
        case CHARGER_STATUS_NOT_VALID_INPUT:
            INFO("CHARGER_STATUS_NOT_VALID_INPUT \r\n");
            break;
        case CHARGER_STATUS_VALID_INPUT:
            INFO("CHARGER_STATUS_VALID_INPUT \r\n");
            break;
        case CHARGER_STATUS_END_OF_CHARGING:
            INFO("charge 100% \r\n");
            break;
        case CHARGER_STATUS_CHARGING_PHASE:
            INFO("Charging ...\r\n");
            break;
        case CHARGER_STATUS_OVER_CHARGE_FAULT:
            INFO("CHARGER_STATUS_OVER_CHARGE_FAULT \r\n");
            break;
        case CHARGER_STATUS_CHARGING_TIMEOUT:
            INFO("CHARGER_STATUS_CHARGING_TIMEOUT \r\n");
            break;
        case CHARGER_STATUS_BAT_VOLTAGE_BELOW_VPRE_AFTER_FAST_CHARGE:
            INFO("CHARGER_STATUS_BAT_VOLTAGE_BELOW_VPRE_AFTER_FAST_CHARGE \r\n");
            break;
        case CHARGER_STATUS_CHARGING_THERMAL_LIMITATION:
            INFO("CHARGER_STATUS_CHARGING_THERMAL_LIMITATION \r\n");
            break;
        case CHARGER_STATUS_BATTERY_TEMPERATURE_FAULT:
            INFO("CHARGER_STATUS_BATTERY_TEMPERATURE_FAULT \r\n");
            break;
        default:
            INFO("Error \r\n");
            break;
        }
    }
}


/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
