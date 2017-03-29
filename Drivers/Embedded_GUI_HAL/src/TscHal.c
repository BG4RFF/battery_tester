/**
  ******************************************************************************
  * @file    TscHal.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This file contains all the TouchScreen functions whose
  *          implementation depends on the TSC Controller used in your Design.
  *          You only need to change these functions implementations
  *          in order to reuse this code with other TSC Controller 
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
#include "TscHal.h"
#include "touchscreen.h"
#include "platform.h"
#include <string.h>
/** @addtogroup Embedded_GUI_Library
  * @{
  */

/** @defgroup TscHal 
  * @brief TscHal main functions
  * @{
  */ 

/* External variables --------------------------------------------------------*/
extern __IO uint32_t u32_TSXCoordinate;
extern __IO uint32_t u32_TSYCoordinate;

extern uint32_t TSC_Value_X;
extern uint32_t TSC_Value_Y;

extern __IO uint8_t touch_done;

/* Touchscreen Hardware Parameters Structure */
TSC_HW_Parameters_TypeDef pTscHwParam;

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define    _1_K                     ((uint8_t)1024)
#define TOUCH_DELAY     			0x8

     
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void GL_Delay(uint32_t nTime);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Create and initialize a new Touchscreen Hw Parameter structure object
  * @param  None
  * @retval TSC_HW_Parameters_TypeDef*, The created Object pointer
  */

TSC_HW_Parameters_TypeDef* NewTscHwParamObj (void)
{
  return &pTscHwParam;
}

/**
  * @brief  This function handles External lines interrupt request for Touchscreen.
  * @param  None
  * @retval None
  */
void GL_EXTI_TSC_IRQHandler( void )
{
#if TOUCH_SCREEN_CAPABILITY
	
	
#endif
}

/**
  * @brief  Init the TS interface
  * @param  None
  * @retval None
  */
void TSC_Init(void)
{
#if TOUCH_SCREEN_CAPABILITY
  /* Reset the STMPE811 using the serial communication interface */
	//TP_Init();
#endif
}

/**
  * @brief  Initializes the IO Expander registers.
  * @param  None
  * @retval - 0: if all initializations are OK.
  */
uint32_t GL_TSC_Interface_Init(void)
{
#if TOUCH_SCREEN_CAPABILITY

 
#endif

  return 0; /* Configuration is OK */
}


/**
  * @brief  Read the coordinate of the point touched and assign their
  *         value to the variables u32_TSXCoordinate and u32_TSYCoordinate
  * @param  None
  * @retval None
  */
void TSC_Read(void)
{
#if TOUCH_SCREEN_CAPABILITY

	if(Touch_GetPos(&TSC_Value_X,&TSC_Value_Y))
	{
		touch_done = 1;
      
		u32_TSXCoordinate = getDisplayCoordinateX( TSC_Value_X, TSC_Value_Y );
		u32_TSYCoordinate = getDisplayCoordinateY( TSC_Value_X, TSC_Value_Y );	
	}
#endif

}


/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
