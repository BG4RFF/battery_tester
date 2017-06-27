/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "bsp.h"
#include "logger/logger.h"
#include "charger/charger.h"
#include "mmc_sd/mmc_sd.h"
#include "fs_fat/fat_filelib.h"

#include "gfx.h"
#include "ui/gui.h"

#include "freertos.h"
#include "task.h"

/* Private variables ---------------------------------------------------------*/
static FL_FILE *_flog = NULL;
#define FNAME_SIZE      32
static char _log_file_name[FNAME_SIZE] = "/logs/bat_test_000.txt";
const static char _file_touch_bin[] = "/settings/touch.bin";
/* Private function prototypes -----------------------------------------------*/
SemaphoreHandle_t  semaphore_fs;
static void _fs_lock(void) {

    xSemaphoreTake(semaphore_fs, portMAX_DELAY);
}

static void _fs_unlock(void) {

    xSemaphoreGive(semaphore_fs);
}



/* -------------------------------------------------------------------------- */

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
        _fs_lock();
        fl_fwrite(data, 1, size, _flog);
        _fs_unlock();
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

    _fs_lock();

    for(uint32_t i = 0; i < UINT32_MAX; i++) {

        snprintf(_log_file_name, FNAME_SIZE,  "/bat_test_%d.txt", i);

        _flog = fl_fopen(_log_file_name, "r");

        if(_flog == NULL) {
           // fl_fclose(_flog);
            _flog = fl_fopen(_log_file_name, "w");

            if(_flog != NULL) {
                LOG("Create file: %s\r\n", _log_file_name);
            } else {
                LOG("Cant create file: %s\r\n", _log_file_name);
            }

            break;
        }

        fl_fclose(_flog);
        _flog = NULL;
    }

    if(_flog) {
        logger_init(_debug_file_log);
    }

    _fs_unlock();
}

/* -------------------------------------------------------------------------- */

bool_t LoadMouseCalibration(unsigned instance, void *data, size_t sz) {

    LOG("load touch settings\r\n");

    _fs_lock();

    FL_FILE *tch_bin = fl_fopen(_file_touch_bin, "r");
    if(tch_bin != NULL) {
        fl_fread(data, sizeof(uint8_t), sz, tch_bin);
        fl_fclose(tch_bin);
    }
    _fs_unlock();

	return (tch_bin != NULL)? TRUE : FALSE;
}

/* -------------------------------------------------------------------------- */

bool_t SaveMouseCalibration(unsigned instance, const void *data, size_t sz) {

    LOG("Save touch settings\r\n");

    _fs_lock();

    FL_FILE *tch_bin = fl_fopen(_file_touch_bin, "w");
    if(tch_bin != NULL) {
        fl_fwrite(data, sizeof(uint8_t), sz, tch_bin);
        fl_fclose(tch_bin);
    }
    _fs_unlock();

	return TRUE;
}

/* -------------------------------------------------------------------------- */

static void _logic (void *context) {

    char str[32];

    uint32_t vbat = 1000;

    for(;;) {
        vTaskDelay(1000);
        snprintf(str, sizeof(str), "VBAT=%4d", vbat++);
        guiSetVBATValue(str);
    }
}

/* -------------------------------------------------------------------------- */

static void _gxf_listener(void *context) {

    /* Check sd card */
    uint32_t timeout = 10;
    while (SD_Init() != 0 && timeout--) {
        LOG("SD Card Failed! Attempt %d\r\n", timeout);
        bsp_delay_ms(1000);
    }


    if( timeout > 0 ) {
        LOG("SD Card Detected!\r\n");
        uint32_t sd_size = SD_GetCapacity();
        LOG("SD Card Size: %d  b\r\n", sd_size);

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

    gfxInit();

    guiCreate();

    /* Create the task, storing the handle. */
    BaseType_t res = xTaskCreate(_logic, "logic", 150, NULL, configMAX_PRIORITIES-1, NULL);
    if(res == NULL) {
        res = 0;
    }

    for(;;) {
        guiEventLoop();
    }
}

/* -------------------------------------------------------------------------- */

int main(void) {

    bsp_init();

    bsp_rtos_init();

    semaphore_fs = xSemaphoreCreateBinary();
    vQueueAddToRegistry(semaphore_fs,"fs");

    xSemaphoreGive(semaphore_fs);
    logger_init(bsp_debug_write);

    /* Create the task, storing the handle. */
    xTaskCreate(_gxf_listener, "listener", 350, NULL, configMAX_PRIORITIES, NULL);

    LOG("Start Scheduler\r\n");
    /* todo relese systick handler */
    vTaskStartScheduler();

    /* Todo move logic to tasks */

    bsp_led1_enable(LED_DISABLE);
    bsp_led2_enable(LED_DISABLE);
    bsp_led3_enable(LED_DISABLE);

    LOG("\r\n-=Battery tester=-\r\n");

    bsp_switch_relay(RELAY_DISCHARGE_OFF);

    charger_init();

    LOG("Set VFLOAT = 4350\r\n");
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

        LOG("%d:%d:%d\t%d\tVbat\t%d\tVin\t%d\t", duration / 60 / 60 %24, duration / 60 %60, duration %60
             , duration, vbat, vin);

        if(is_discharge) {
            LOG("Discharge\r\n");

            if(vbat < 3000) {
                LOG("Discharge complete\r\nTime %d s, Capacity %d Ah\r\n",  duration, duration * 10 /60/60);

                is_discharge = false;
                start_time = bsp_get_tick_ms();
                _start_charge();
            }
        } else {

            switch(charger_get_status()) {
            case CHARGER_STATUS_NOT_VALID_INPUT:
                LOG("CHARGER_STATUS_NOT_VALID_INPUT \r\n");
                break;
            case CHARGER_STATUS_VALID_INPUT:
                LOG("CHARGER_STATUS_VALID_INPUT \r\n");
                break;
            case CHARGER_STATUS_END_OF_CHARGING:
                LOG("Charge complete\r\nDuration %d S\r\n", duration);
                is_discharge = true;
                start_time = bsp_get_tick_ms();
                _start_discharge();
                break;
            case CHARGER_STATUS_CHARGING_PHASE:
                LOG("CHARGING\r\n");
                break;
            case CHARGER_STATUS_OVER_CHARGE_FAULT:
                LOG("CHARGER_STATUS_OVER_CHARGE_FAULT \r\n");
                break;
            case CHARGER_STATUS_CHARGING_TIMEOUT:
                LOG("CHARGER_STATUS_CHARGING_TIMEOUT \r\n");
                break;
            case CHARGER_STATUS_BAT_VOLTAGE_BELOW_VPRE_AFTER_FAST_CHARGE:
                LOG("CHARGER_STATUS_BAT_VOLTAGE_BELOW_VPRE_AFTER_FAST_CHARGE \r\n");
                break;
            case CHARGER_STATUS_CHARGING_THERMAL_LIMITATION:
                LOG("CHARGER_STATUS_CHARGING_THERMAL_LIMITATION \r\n");
                break;
            case CHARGER_STATUS_BATTERY_TEMPERATURE_FAULT:
                LOG("CHARGER_STATUS_BATTERY_TEMPERATURE_FAULT \r\n");
                break;
            default:
                LOG("Error \r\n");
                break;
            }
        }

        if(bsp_is_button1_pressed()) {

            if(is_discharge) {

                LOG("Force stop, Discharge time %d s, Capacity %d Ah\r\nStart Charge\r\n",  duration, duration * 10 /60/60);
                _start_charge();
            } else {

                LOG("Forse stop, Charge time %d\r\nStart discharge\r\n", duration);
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
