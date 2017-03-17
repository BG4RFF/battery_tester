/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
/* Includes ------------------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

#define INT_TS_PIN                  GPIO_PIN_13
#define INT_TS_GPIO_PORT            GPIOC

#define DB00_PIN                    GPIO_PIN_0
#define DB00_GPIO_PORT              GPIOC
#define DB01_PIN                    GPIO_PIN_1
#define DB01_GPIO_PORT              GPIOC
#define DB02_PIN                    GPIO_PIN_2
#define DB02_GPIO_PORT              GPIOC
#define DB03_PIN                    GPIO_PIN_3
#define DB03_GPIO_PORT              GPIOC
#define DB04_PIN                    GPIO_PIN_4
#define DB04_GPIO_PORT              GPIOC
#define DB05_PIN                    GPIO_PIN_5
#define DB05_GPIO_PORT              GPIOC
#define DB06_PIN                    GPIO_PIN_6
#define DB06_GPIO_PORT              GPIOC
#define DB07_PIN                    GPIO_PIN_7
#define DB07_GPIO_PORT              GPIOC
#define DB08_PIN                    GPIO_PIN_8
#define DB08_GPIO_PORT              GPIOB
#define DB09_PIN                    GPIO_PIN_9
#define DB09_GPIO_PORT              GPIOB
#define DB10_PIN                    GPIO_PIN_10
#define DB10_GPIO_PORT              GPIOB
#define DB11_PIN                    GPIO_PIN_11
#define DB11_GPIO_PORT              GPIOB
#define DB12_PIN                    GPIO_PIN_12
#define DB12_GPIO_PORT              GPIOB
#define DB13_PIN                    GPIO_PIN_13
#define DB13_GPIO_PORT              GPIOB
#define DB14_PIN                    GPIO_PIN_14
#define DB14_GPIO_PORT              GPIOB
#define DB15_PIN                    GPIO_PIN_15
#define DB15_GPIO_PORT              GPIOB


#define BUTTON_1_PIN                GPIO_PIN_0
#define BUTTON_1_GPIO_PORT          GPIOA
#define BUTTON_2_PIN                GPIO_PIN_1
#define BUTTON_2_GPIO_PORT          GPIOA

#define LED1_PIN                    GPIO_PIN_2
#define LED1_GPIO_PORT              GPIOA
#define LED2_PIN                    GPIO_PIN_3
#define LED2_GPIO_PORT              GPIOA
#define LED3_PIN                    GPIO_PIN_2
#define LED3_GPIO_PORT              GPIOB

#define CS_TS_PIN                   GPIO_PIN_4
#define CS_TS_GPIO_PORT             GPIOA

#define ADC_VBAT_PIN                GPIO_PIN_0
#define ADC_VBAT_GPIO_PORT          GPIOB
#define ADC_VIN_PIN                 GPIO_PIN_1
#define ADC_VIN_GPIO_PORT           GPIOB


#define LCD_RS_PIN                  GPIO_PIN_8
#define LCD_RS_GPIO_PORT            GPIOC

#define CS_LCD_PIN                  GPIO_PIN_9
#define CS_LCD_GPIO_PORT            GPIOC

#define CHARGER_STATUS_PIN          GPIO_PIN_8
#define CHARGER_STATUS_PORT         GPIOA

#define CHARGER_SW_SEL_PIN          GPIO_PIN_11
#define CHARGER_SW_SEL_PORT         GPIOA

#define CHARGER_EN_PIN              GPIO_PIN_12
#define CHARGER_EN_PORT             GPIOA

#define LCD_WR_PIN                  GPIO_PIN_10
#define LCD_WR_GPIO_PORT            GPIOC

#define LCD_RD_PIN                  GPIO_PIN_11
#define LCD_RD_GPIO_PORT            GPIOC

#define LCD_BL_EN_PIN               GPIO_PIN_12
#define LCD_BL_EN_GPIO_PORT         GPIOC

#define RELAY_CONTROL_PIN           GPIO_PIN_3
#define RELAY_CONTROL_GPIO_PORT     GPIOB

#define CS_F_PIN                    GPIO_PIN_6
#define CS_F_GPIO_PORT              GPIOB

#define CS_SD_PIN                   GPIO_PIN_7
#define CS_SD_GPIO_PORT             GPIOB

#define UART1_TX_PIN                GPIO_PIN_9
#define UART1_TX_PORT               GPIOA
#define UART1_RX_PIN                GPIO_PIN_10
#define UART1_RX_PORT               GPIOA


/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
