/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "logger/logger.h"
#include "charger/charger.h"
#include "touch_driver/touch_tsc2046.h"
#include "mmc_sd/mmc_sd.h"
#include "fs_fat/fat_filelib.h"

#include "gfx.h"
#include "ui/gui.h"

#include "freertos.h"
#include "task.h"

/* Private variables ---------------------------------------------------------*/
static FL_FILE *_flog = NULL;
#define FNAME_SIZE      32
static char _log_file_name[FNAME_SIZE] = "/bat_test.txt";
/* Private function prototypes -----------------------------------------------*/

static int _media_read(uint32 sector, uint8 *buffer, uint32 sector_count) {

    for(uint32_t i = 0; i <sector_count; i++) {
        SD_ReadSingleBlock(sector, &buffer[i*FAT_SECTOR_SIZE]);
    }

    return 1;
}

/* -------------------------------------------------------------------------- */

static int _media_write(uint32 sector, uint8 *buffer, uint32 sector_count) {
    for(uint32_t i = 0; i <sector_count; i++) {
        SD_WriteSingleBlock(sector, &buffer[i]);
    }
	return 1;
}

/* -------------------------------------------------------------------------- */

static void _debug_file_log(const uint8_t *data, uint32_t size) {

    static uint32_t disable_recursive_from_fl = 0;

    bsp_debug_write(data, size);

    disable_recursive_from_fl++;

    if(_flog  != NULL && disable_recursive_from_fl == 1) {
        fl_fwrite(data, 1, size, _flog);
    }

    disable_recursive_from_fl--;
}

/* -------------------------------------------------------------------------- */

static void _start_discharge(void) {

    charger_enable(CHARGER_DISABLE);
    bsp_switch_relay(RELAY_DISCHARGE_ON);
}

/* -------------------------------------------------------------------------- */

static void _start_charge(void) {

    charger_enable(CHARGER_ENABLE);
    bsp_switch_relay(RELAY_DISCHARGE_OFF);
}

/* -------------------------------------------------------------------------- */

static void _open_log_file() {

    for(uint32_t i = 0; i < UINT32_MAX; i++) {

        snprintf(_log_file_name, FNAME_SIZE,  "/bat_test_%d.txt", i);

        _flog = fl_fopen(_log_file_name, "r");

        if(_flog == NULL) {
            fl_fclose(_flog);
            _flog = fl_fopen(_log_file_name, "w");

            INFO("Create file: %s\r\n", _log_file_name);

            break;
        }

        fl_fclose(_flog);
        _flog = NULL;
    }

    if(_flog) {
        logger_init(_debug_file_log);
    }
}

/* -------------------------------------------------------------------------- */

bool_t LoadMouseCalibration(unsigned instance, void *data, size_t sz) {

	return FALSE;
}

/* -------------------------------------------------------------------------- */

bool_t SaveMouseCalibration(unsigned instance, const void *data, size_t sz) {

	return TRUE;
}

/* -------------------------------------------------------------------------- */

void _startup(void *context) {

    gfxInit();

    guiCreate();

    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */

int main(void) {

    bsp_init();

    logger_init(bsp_debug_write);

    /* Create the task, storing the handle. */
     xTaskCreate(_startup, "STARTUP", 200, NULL, configMAX_PRIORITIES, NULL);

     /* todo relese systick handler */
     vTaskStartScheduler();

     /* Todo move logic to tasks */

    /* Check sd card */
    uint32_t timeout = 10;
    while (SD_Init() != 0 && timeout--) {
        INFO("SD Card Failed!\r\n");
        INFO("Please Check!\r\n");
        bsp_delay_ms(500);
    }


    if( timeout > 0 ) {
        INFO("SD Card Detected!\r\n");
        uint32_t sd_size = SD_GetCapacity();
        INFO("SD Card Size: %d  b\r\n", sd_size);

        fl_init();
        // Attach media access functions to library
        if (fl_attach_media(_media_read, _media_write) != FAT_INIT_OK) {
            printf("ERROR: Media attach failed\r\n");
            //return;
        }

        /* debug purposes */
        fl_listdirectory("/");

        _open_log_file();
    }

    bsp_led1_enable(LED_DISABLE);
    bsp_led2_enable(LED_DISABLE);
    bsp_led3_enable(LED_DISABLE);

    INFO("\r\n-=Battery tester=-\r\n");

    bsp_switch_relay(RELAY_DISCHARGE_OFF);

    charger_init();

    INFO("Set VFLOAT = 4350\r\n");
    charger_send_swp_message(SWPC_INCREASE_150MV);


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
                INFO("Discharge complete\r\nTime %d s, Capacity %d Ah\r\n",  duration, duration * 10 /60/60);

                is_discharge = false;
                start_time = bsp_get_tick_ms();
                _start_charge();
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
                INFO("Charge complete\r\nDuration %d S\r\n", duration);
                is_discharge = true;
                start_time = bsp_get_tick_ms();
                _start_discharge();
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

                INFO("Force stop, Discharge time %d s, Capacity %d Ah\r\nStart Charge\r\n",  duration, duration * 10 /60/60);
                _start_charge();
            } else {

                INFO("Forse stop, Charge time %d\r\nStart discharge\r\n", duration);
                _start_discharge();
            }

            start_time = bsp_get_tick_ms();
            is_discharge = !is_discharge;

            while(bsp_is_button1_pressed());
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
