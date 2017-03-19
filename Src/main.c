/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "logger/logger.h"
#include "charger/charger.h"
#include "../drivers/lcd_driver/ili9320.h"

/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

int main(void)
{

    bsp_init();

    bsp_lcd_backlight_enable(LCD_BACKLIGHT_ENABLE);
    ili9320_Initializtion();
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
        //charger_send_swp_message(SWPC_BATMS_ON);
        //bsp_delay_ms(1000);
        //charger_send_swp_message(SWPC_BATMS_OFF);

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
