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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define INT_TS_Pin GPIO_PIN_13
#define INT_TS_GPIO_Port GPIOC
#define DB00_Pin GPIO_PIN_0
#define DB00_GPIO_Port GPIOC
#define DB01_Pin GPIO_PIN_1
#define DB01_GPIO_Port GPIOC
#define DB02_Pin GPIO_PIN_2
#define DB02_GPIO_Port GPIOC
#define DB03_Pin GPIO_PIN_3
#define DB03_GPIO_Port GPIOC
#define BUTTON_1_Pin GPIO_PIN_0
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_2_Pin GPIO_PIN_1
#define BUTTON_2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOA
#define CS_TS_Pin GPIO_PIN_4
#define CS_TS_GPIO_Port GPIOA
#define DB04_Pin GPIO_PIN_4
#define DB04_GPIO_Port GPIOC
#define DB05_Pin GPIO_PIN_5
#define DB05_GPIO_Port GPIOC
#define ADC_VBAT_Pin GPIO_PIN_0
#define ADC_VBAT_GPIO_Port GPIOB
#define ADC_VIN_Pin GPIO_PIN_1
#define ADC_VIN_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOB
#define DB10_Pin GPIO_PIN_10
#define DB10_GPIO_Port GPIOB
#define DB11_Pin GPIO_PIN_11
#define DB11_GPIO_Port GPIOB
#define DB12_Pin GPIO_PIN_12
#define DB12_GPIO_Port GPIOB
#define DB13_Pin GPIO_PIN_13
#define DB13_GPIO_Port GPIOB
#define DB14_Pin GPIO_PIN_14
#define DB14_GPIO_Port GPIOB
#define DB15_Pin GPIO_PIN_15
#define DB15_GPIO_Port GPIOB
#define DB06_Pin GPIO_PIN_6
#define DB06_GPIO_Port GPIOC
#define DB07_Pin GPIO_PIN_7
#define DB07_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_8
#define LCD_RS_GPIO_Port GPIOC
#define CS_LCD_Pin GPIO_PIN_9
#define CS_LCD_GPIO_Port GPIOC
#define CHARGER_STATUS_Pin GPIO_PIN_8
#define CHARGER_STATUS_GPIO_Port GPIOA
#define CHARGER_SW_SEL_Pin GPIO_PIN_11
#define CHARGER_SW_SEL_GPIO_Port GPIOA
#define LCD_WR_Pin GPIO_PIN_10
#define LCD_WR_GPIO_Port GPIOC
#define LCD_RD_Pin GPIO_PIN_11
#define LCD_RD_GPIO_Port GPIOC
#define LCD_BL_EN_Pin GPIO_PIN_12
#define LCD_BL_EN_GPIO_Port GPIOC
#define RELAY_CONTROL_Pin GPIO_PIN_3
#define RELAY_CONTROL_GPIO_Port GPIOB
#define CS_F_Pin GPIO_PIN_6
#define CS_F_GPIO_Port GPIOB
#define CS_SD_Pin GPIO_PIN_7
#define CS_SD_GPIO_Port GPIOB
#define DB08_Pin GPIO_PIN_8
#define DB08_GPIO_Port GPIOB
#define DB09_Pin GPIO_PIN_9
#define DB09_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
