/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "logger/logger.h"
#include "charger/charger.h"

/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

int main(void)
{

    bsp_init();

    bsp_switch_relay(RELAY_CHARGE);
    charger_init();

  /* Infinite loop */
  while (1)
  {
      bsp_delay_ms(1000);

      switch(charger_get_status()) {
      case CHARGER_STATUS_NOT_VALID_INPUT:
          INFO("CHARGER_STATUS_NOT_VALID_INPUT \r\n");
          break;
      case CHARGER_STATUS_VALID_INPUT:
          INFO("CHARGER_STATUS_VALID_INPUT \r\n");
          break;
      case CHARGER_STATUS_END_OF_CHARGING:
          INFO("CHARGER_STATUS_END_OF_CHARGING \r\n");
          break;
      case CHARGER_STATUS_CHARGING_PHASE:
          INFO("CHARGER_STATUS_CHARGING_PHASE \r\n");
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
