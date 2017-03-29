/**
  ******************************************************************************
  * @file    JoyHal.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This file contains all the Joystick functions whose
  *          implementation depends on the Controller used in your Design.
  *          You only need to change these functions implementations
  *          in order to reuse this code with other Controller 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ---------------------------------------------------------------------------*/
#include "JoyHal.h"
#include "graphicObjectTypes.h"


/** @addtogroup Embedded_GUI_Library
  * @{
  */

/** @defgroup JoyHal 
  * @brief JoyHal main functions
  * @{
  */
  
/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Joystick Hardware Parameters Structure */
JOY_HW_Parameters_TypeDef pJoyHwParam; 

/* BUTTON Hardware Parameters Structure */
BTN_HW_Parameters_TypeDef pBtnHwParam; 

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup JoyHal_Private_Functions
  * @{
  */

/**
  * @brief  Create and initialize a new Joystick Hw Parameter structure object
  * @param  None
  * @retval JOY_HW_Parameters_TypeDef*, The created Object pointer
  */
JOY_HW_Parameters_TypeDef* NewJoyHwParamObj (void)
{
  return &pJoyHwParam;
}

/**
  * @brief  Create and initialize a new Button Hw Parameter structure object
  * @param  None
  * @retval BTN_HW_Parameters_TypeDef*, The created Object pointer
  */
BTN_HW_Parameters_TypeDef* NewBtnHwParamObj (void)
{
  return &pBtnHwParam;
}

/**
  * @brief  Initializes the IO Expander registers.
  * @param  None
  * @retval - 0: if all initializations are OK.
  */
uint32_t GL_JOY_Interface_Init(void)
{

  return 0; /* Configuration is OK */
}

/**
  * @brief  Initializes the IO Expander for JoyStick operations.
  * @param  None
  * @retval - 0: if all initializations are OK.
  */
void GL_JoyStickConfig_IOExpander(void)
{

}

/**
  * @brief  Configures the GPIO ports pins concerned with joystick.
  * @param  None
  * @retval None
  */
void GL_JoyStickConfig_GPIO(void)
{
	
}

/**
  * @brief  Return the Joystick status.
  * @param  None
  * @retval uint32_t - The code of the Joystick key pressed
  */
uint32_t GL_JoyStickState(JOY_ReadMode mode)
{
  uint32_t val = GL_JOY_NONE;
	
    return  val;
}

/**
  * @brief  Return the Joystick status.
  * @param  None
  * @retval uint32_t - The code of the Joystick key pressed
  */
uint32_t GL_JoyStickStateIOEXP(void)
{
  {
    return (uint32_t)GL_JOY_NONE;
  }
}


/**
  * @brief  Return the Joystick status.
  * @param  None
  * @retval uint32_t - The code of the Joystick key pressed
  */
uint32_t GL_JoyStickStatePolling(void)
{

  return (uint32_t)GL_JOY_NONE;
}

/**
  * @brief  Configure user key button.
  * @param  none
  * @retval none
  */
void GL_ButtonInit(void)
{
	
}
/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to read.
  *         This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval uint8_t - The input port pin value.
  */
uint8_t GL_GPIO_ReadInputDataBit(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin)
{

  return 0;
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
