
/* file charger.c created 11.11.16 by Roman_Rusak ---------------------------- */

#include "charger.h"
#include "bsp.h"

/* ===== SETTINGS =========================================================== */

#define CHG_FLG_MAX_PERIOD_MS           (250U*10U)

/* ===== CONST ============================================================== */

const uint32_t PULSE_TIMING = 105U;
const uint32_t START_BIT_TIMING = 355U;
const uint32_t STOP_BIT_TIMING = 600U;

/* ===== LOCAL VARIABLES ==================================================== */

/* Single wire programming interface step */
static swire_step_t _swpi_step = STEP_START_BIT;

static uint8_t _swp_code_pulse_count = 0U;
static uint32_t _last_chr_flg_rising_time_ms = 0U;
static uint8_t _last_chr_flg_frequency = 0U;

static uint8_t _charger_status_freq[] =
{
    [CHARGER_STATUS_NOT_VALID_INPUT] = 0U,
    [CHARGER_STATUS_VALID_INPUT] = 0U,
    [CHARGER_STATUS_END_OF_CHARGING]  = 4U,
    [CHARGER_STATUS_CHARGING_PHASE] = 6U,
    [CHARGER_STATUS_OVER_CHARGE_FAULT] = 8U,
    [CHARGER_STATUS_CHARGING_TIMEOUT] = 10U,
    [CHARGER_STATUS_BAT_VOLTAGE_BELOW_VPRE_AFTER_FAST_CHARGE] = 12U,
    [CHARGER_STATUS_CHARGING_THERMAL_LIMITATION] = 14U,
    [CHARGER_STATUS_BATTERY_TEMPERATURE_FAULT] = 16,
};

/* ===== LOCAL FUNCTIONS PROTOTYPES ========================================= */

static void _on_message_sent(void);
static void _timer_handler(void);
static void _status_pin_toggle_handler(void);
/* ===== GLOBAL FUNCTIONS =================================================== */

void charger_send_swp_message(uint8_t message) {

    bsp_disable_irq();

    _swp_code_pulse_count = message;
    _swpi_step = STEP_START_BIT;

    bsp_timer0_set_period(START_BIT_TIMING, _timer_handler);
    bsp_toggle_charger_swire_pin();

    bsp_enable_irq();
}

/* -------------------------------------------------------------------------- */
#include "logger/logger.h"
charger_status_t charger_get_status(void) {

    charger_status_t charger_status = CHARGER_STATUS_count;

    bsp_disable_irq();
    const uint32_t last_chr_flg_rising_time_ms = _last_chr_flg_rising_time_ms;
    const uint8_t last_chr_flg_frequency = _last_chr_flg_frequency;
    bsp_enable_irq();


    const bool is_illegal_frequency = (bsp_get_tick_ms() - last_chr_flg_rising_time_ms) > CHG_FLG_MAX_PERIOD_MS;

    if (is_illegal_frequency) {

        if (bsp_is_charger_flag_high()) {
            charger_status = CHARGER_STATUS_NOT_VALID_INPUT;
        } else {
            charger_status = CHARGER_STATUS_VALID_INPUT;
        }

    } else {

        INFO("Freaq = %d\r\n", last_chr_flg_frequency);
        for (charger_status_t status = CHARGER_STATUS_END_OF_CHARGING;  status < CHARGER_STATUS_count; ++status)
        {
            if (last_chr_flg_frequency == _charger_status_freq[status]) {
                charger_status = status;
                break;
            }
        }
    }

    return charger_status;

}

/* -------------------------------------------------------------------------- */

void charger_init(void) {

    bsp_register_charger_status_cb(_status_pin_toggle_handler);
}

/* -------------------------------------------------------------------------- */

/* ===== LOCAL FUNCTIONS ==================================================== */

static void _on_message_sent(void) {

    _swpi_step = STEP_STOP_BIT;
    bsp_timer0_disable();
}

/* -------------------------------------------------------------------------- */

static void _timer_handler(void) {

        if (bsp_is_charger_swire_pin_high()) {

            if (_swp_code_pulse_count == 0U) {
                _on_message_sent();
            } else {
                if (_swpi_step == STEP_START_BIT) {
                    _swpi_step = STEP_PULSE;
                    bsp_timer0_set_period(PULSE_TIMING, _timer_handler);
                } else {
                    _swp_code_pulse_count--;
                }
            }
        } else {

            if (_swp_code_pulse_count == 0U) {
                bsp_timer0_set_period(STOP_BIT_TIMING, _timer_handler);
            }
        }
        bsp_toggle_charger_swire_pin();
}

/* -------------------------------------------------------------------------- */

static void _status_pin_toggle_handler(void) {

    const uint32_t timestamp = bsp_get_tick_ms();

    float _last_chr_flg_frequency_f = (1000.0f / (timestamp - _last_chr_flg_rising_time_ms));

    _last_chr_flg_frequency = (uint8_t) (_last_chr_flg_frequency_f);
    _last_chr_flg_rising_time_ms = timestamp;
}

/* file charger.c created 11.11.16 by Roman_Rusak ---------------------------- */
