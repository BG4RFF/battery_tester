/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "logger/logger.h"
#include "charger/charger.h"
#include "lcd_driver/ili9320.h"
#include "touch_driver/touch_tsc2046.h"
#include "mmc_sd/mmc_sd.h"
#include "fs_fat/fat_filelib.h"

/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//(uint32 sector, uint8 *buffer, uint32 sector_count);
static int _media_read(uint32 sector, uint8 *buffer, uint32 sector_count) {

    for(uint32_t i = 0; i <sector_count; i++) {
        SD_ReadSingleBlock(sector, &buffer[i*FAT_SECTOR_SIZE]);
    }

    return 1;
}

static int _media_write(uint32 sector, uint8 *buffer, uint32 sector_count) {
    for(uint32_t i = 0; i <sector_count; i++) {
        SD_WriteSingleBlock(sector, &buffer[i]);
    }
	return 1;
}

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

    if(bsp_is_button1_pressed()) {

        while (SD_Init() != 0) {
            INFO("SD Card Failed!\r\n");
            INFO("Please Check!\r\n");
            bsp_delay_ms(500);
        }

        bsp_delay_ms(50);
        INFO("SD Card Detected!\r\n");
        uint32_t sd_size = SD_GetCapacity();
        INFO("SD Card Size: %d  b\r\n", sd_size);

        fl_init();
        // Attach media access functions to library
        if (fl_attach_media(_media_read, _media_write) != FAT_INIT_OK) {
            printf("ERROR: Media attach failed\r\n");
            //return;
        }


		fl_listdirectory("/pict");

    }

    bsp_led1_enable(LED_DISABLE);
    bsp_delay_ms(500);
    bsp_led2_enable(LED_DISABLE);
    bsp_delay_ms(500);
    bsp_led3_enable(LED_DISABLE);
    bsp_delay_ms(500);


    ili9320_Clear(Blue);

    INFO("\r\n-=Battery tester=-\r\n");

    bsp_switch_relay(RELAY_DISCHARGE_OFF);

    charger_init();
    charger_enable(CHARGER_DISABLE);
    bsp_delay_ms(500);
    charger_enable(CHARGER_ENABLE);


    charger_send_swp_message(SWPC_AUTO_RECHARGE_ON);
    bsp_delay_ms(100);

    charger_send_swp_message(SWPC_AUTO_RECHARGE_OFF);
    bsp_delay_ms(100);
    charger_send_swp_message(SWPC_VFLOAT_BAT_AGING_OFF);
    bsp_delay_ms(100);
    INFO("VFLOAT = 4350\r\n");
    charger_send_swp_message(SWPC_INCREASE_150MV);
    bsp_delay_ms(100);

    bool is_discharge = false;
    uint32_t start_time = bsp_get_tick_ms();

    uint32_t time = bsp_get_tick_ms() / 1000 * 1000;
    /* Infinite loop */
    while (1) {

        while( (bsp_get_tick_ms() - time) < 1000);
        time = bsp_get_tick_ms();

        const uint32_t vbat = bsp_get_voltage(VOLTAGE_SOURCE_VBAT);
        const uint32_t vin = bsp_get_voltage(VOLTAGE_SOURCE_VIN);
        const uint32_t duration = (bsp_get_tick_ms() - start_time) / 1000;

        INFO("%d:%d:%d\t%d\tVbat\t%d\tVin\t%d\t", duration / 60 / 60 %24, duration / 60 %60, duration %60
             , duration, vbat, vin);

        if(is_discharge) {
            INFO("Discharge\r\n");

            if(vbat < 3000) {
                INFO("Discharge complete\r\n");
                INFO("Time %d s, Capacity %d Ah\r\n",  duration, duration * 10 /60/60);

                is_discharge = false;
                charger_enable(CHARGER_ENABLE);
                bsp_switch_relay(RELAY_DISCHARGE_OFF);
                start_time = bsp_get_tick_ms();
            }
        } else {

            switch(charger_get_status()) {
            case CHARGER_STATUS_NOT_VALID_INPUT:
                INFO("CHARGER_STATUS_NOT_VALID_INPUT \r\n");
                break;
            case CHARGER_STATUS_VALID_INPUT:
                INFO("CHARGER_STATUS_VALID_INPUT \r\n");
                break;
            case CHARGER_STATUS_END_OF_CHARGING:
                INFO("CHARGE COMPLETE\r\n");
                INFO("Duration %d S\r\n", duration);

                INFO("Start discharge\r\n");

                is_discharge = true;
                charger_enable(CHARGER_DISABLE);
                bsp_switch_relay(RELAY_DISCHARGE_ON);
                start_time = bsp_get_tick_ms();
                break;
            case CHARGER_STATUS_CHARGING_PHASE:
                INFO("CHARGING\r\n");
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

        if(bsp_is_button1_pressed()) {
            if(is_discharge) {
                INFO("Discharge stoped\r\n");
                INFO("Time %d s, Capacity %d Ah\r\n",  duration, duration * 10 /60/60);

                INFO("Start Charge\r\n");
                charger_enable(CHARGER_ENABLE);
                bsp_switch_relay(RELAY_DISCHARGE_OFF);
            } else {

                INFO("charge stoped, time %d\r\n", duration);

                INFO("Start discharge\r\n");
                charger_enable(CHARGER_DISABLE);
                bsp_switch_relay(RELAY_DISCHARGE_ON);
            }

            start_time = bsp_get_tick_ms();
            is_discharge = !is_discharge;
            bsp_delay_ms(800);
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
