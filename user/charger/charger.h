
/* file charger.h created 11.11.16 by Roman_Rusak ---------------------------- */

/* ----- prevent recursive inclusion ---------------------------------------- */
#ifndef __CHARGER__
#define __CHARGER__

/* ----- cpp protection ----------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>

/* ===== TYPES ============================================================== */

typedef enum
{
    STEP_START_BIT,
    STEP_PULSE,
    STEP_STOP_BIT,

}swire_step_t;

/* -------------------------------------------------------------------------- */

typedef enum sw_program_codes_e {

    SWPC_SW1_OA_OFF = 1,
    SWPC_SW1_OA_ON,
    SWPC_SW1_OB_OFF,
    SWPC_SW1_OB_ON,
    SWPC_SW2_OA_OFF,
    SWPC_SW2_OA_ON,
    SWPC_SW2_OB_OFF,
    SWPC_SW2_OB_ON,

    /* Default */
    SWPC_BATMS_OFF,

    SWPC_BATMS_ON,

    SWPC_IEND_OFF,

    /* Default */
    SWPC_IEND_5_PER_IFAST,

    SWPC_IEND_2P5_IFAST,

    SWPC_IBATOCP_900MA,
    SWPC_IBATOCP_500MA,
    SWPC_IBATOCP_250MA,
    SWPC_IBATOCP_100MA,

    /* Default */
    SWPC_VFLOAT_BAT_AGING_OFF,

    SWPC_INCREASE_50MV,
    SWPC_INCREASE_100MV,
    SWPC_INCREASE_150MV,
    SWPC_INCREASE_200MV,

    SWPC_SHUTDOWN,

    /* Default */
    SWPC_AUTO_RECHARGE_OFF,
    SWPC_AURO_RECHARGE_ON,

    /* Default */
    SWPC_WATCHDOG_OFF,

    SWPC_WATCHDOG_ON,

    /* Default */
    SWPC_IFAST_AND_IPRE_50_PER_OFF,

    SWPC_IFAST_AND_IPRE_50_PER_ON,

    SWPC_last,


} sw_program_codes_t;

/* -------------------------------------------------------------------------- */

typedef enum charger_status_e {

    CHARGER_STATUS_NOT_VALID_INPUT = 0,
    CHARGER_STATUS_VALID_INPUT,
    CHARGER_STATUS_END_OF_CHARGING,
    CHARGER_STATUS_CHARGING_PHASE,
    CHARGER_STATUS_OVER_CHARGE_FAULT,
    CHARGER_STATUS_CHARGING_TIMEOUT,
    CHARGER_STATUS_BAT_VOLTAGE_BELOW_VPRE_AFTER_FAST_CHARGE,
    CHARGER_STATUS_CHARGING_THERMAL_LIMITATION,
    CHARGER_STATUS_BATTERY_TEMPERATURE_FAULT,
    CHARGER_STATUS_count,

} charger_status_t;

/* ===== GLOBAL FUNCTIONS PROTOTYPES ======================================== */

charger_status_t charger_get_status(void);
void charger_send_swp_message(uint8_t message);
void charger_irq_handler(void);
void init_charger_int(void);
void charger_tim_irq_handler(void);

/* ----- cpp protection ----------------------------------------------------- */
#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __CHARGER__ */

/* file charger.h created 11.11.16 by Roman_Rusak ---------------------------- */
