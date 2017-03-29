/**
  ******************************************************************************
  * @file    graphicObject.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This file contains the methods to create the objects that
  *          can be printed on the LCD, and the TouchScreen calibration.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRAPHIC_OBJECT_H
#define __GRAPHIC_OBJECT_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "graphicObjectTypes.h"
#include "font.h"
/** @addtogroup Embedded_GUI_Library
  * @{
  */

/** @addtogroup graphicObject 
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup graphicObject_Exported_Constants
  * @{
  */
#define GL_NULL  0x00
/**
  * @}
  */

/** @defgroup graphicObject_Exported_variables
  * @{
  */
extern GL_Page_TypeDef* PagesList[];      /*!< Pointer to page list */
extern __IO uint8_t touch_done;           /*!< touchscreen event status */
//extern __IO uint8_t calibration_magic;     /*!< touchscreen calibration status */
extern __IO uint8_t joy_done;             /*!< joystick event status */

extern __IO uint16_t LCD_Height;               /*!< Screen  Height */
extern __IO uint16_t LCD_Width;                /*!< Screen  Width */
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup graphicObject_Exported_Macros
  * @{
  */
#define RADIO_BUTTON_ADD(RadioButtonGrpName, Label, Function) \
 RadioButtonGrpName->AddRadioOption( RadioButtonGrpName, Label, Function );

/**
  * @}
  */
  
/* Exported functions ------------------------------------------------------- */
/** @defgroup graphicObject_Exported_Functions
  * @{
  */
/*----- High layer function -----*/
                                    
GL_PageControls_TypeDef* NewButton (uint16_t ID, const uint8_t* label, 
		uint16_t Font, void (*pEventHandler)(void));
                                    
GL_PageControls_TypeDef* NewCheckbox (uint16_t ID, const uint8_t* label,
                                       void (*pEventHandler)(void));
                                       
GL_PageControls_TypeDef* NewSwitch (uint16_t ID, const uint8_t* label_1, const uint8_t* label_2,
                                    void (*pEventHandler)(void));

/* Label handler **************************************************************/ 
GL_PageControls_TypeDef* NewLabel (uint16_t ID, const uint8_t* label, GL_Direction direction,
	font_list_t Font, __IO uint32_t FontColour, __IO uint32_t BackgroundColour);

GL_ErrStatus Set_Label ( GL_Page_TypeDef* pPage, uint16_t ID, const uint8_t* label );

GL_ErrStatus Get_Label ( GL_Page_TypeDef* pPage, uint16_t ID, uint8_t* label );

GL_ErrStatus Set_Label_Colour( GL_Page_TypeDef* pPage, uint16_t ID,
	__IO uint32_t FontColour, __IO uint32_t BackgroundColour);

/* Slidebar handler ***********************************************************/                                   
GL_PageControls_TypeDef* NewSlidebar (uint16_t ID, const uint8_t* label,
                                      GL_Direction direction, void (*pEventHandler)(void));
                                      
uint8_t Get_SlidebarValue ( GL_Page_TypeDef* pPage, uint16_t ID );  
                                    
/* Icon handler ***************************************************************/                                     
GL_PageControls_TypeDef* NewIcon (uint16_t ID, const uint8_t* Image_PTR,
                                  uint16_t Width, uint8_t Height, 
                                  void (*pEventHandler)(void));
                                  
GL_ErrStatus SetIconImage(GL_Page_TypeDef* pPage, uint16_t ID,
                          const uint8_t* pImage, uint16_t Width, uint8_t Height);
/* IconBtn handler ***************************************************************/                                     
GL_PageControls_TypeDef* NewIconBtn (uint16_t ID,
														const uint8_t* Image_PTR,
														const uint8_t* Image_Press_PTR,
														uint16_t Width, 
														uint8_t Height, 
														void (*pEventHandler)(void));
/* IconBtnText handler ***************************************************************/ 														
GL_PageControls_TypeDef* NewIconBtnText (
			uint16_t ID, 
			const uint8_t* Image_PTR,
			const uint8_t* Image_Press_PTR,
			const uint8_t* pText, 
			const font_list_t Font, 
			const uint16_t FontColour, 
			uint16_t Width, 
			uint8_t Height, 
			void (*pEventHandler)(void));														
			
/* IconBtnPushPull handler ***************************************************************/   
GL_PageControls_TypeDef* NewIconBtnPushPullText (
			uint16_t ID, 
			const uint8_t* Image_PTR,
			const uint8_t* Image_Press_PTR,
			const uint8_t* pText, 
			const uint16_t PushPullState, 
			uint16_t Width, 
			uint8_t Height, 
			void (*pEventHandler)(void));
GL_ErrStatus SetIconPushPullState_Relased(GL_Page_TypeDef* pPage, uint16_t ID);
GL_ErrStatus SetIconPushPullState_Pressed(GL_Page_TypeDef* pPage, uint16_t ID);
/* Histogram handler **********************************************************/                                       
GL_PageControls_TypeDef* NewHistogram (uint16_t ID, const uint8_t* labelX,
                                       const uint8_t* labelY, int16_t data_points[],
                                       uint8_t n_points);

GL_ErrStatus SetHistogramPoints( GL_Page_TypeDef* pPage, uint16_t ID,
                                 int16_t data_points[], uint8_t n_points );

/* Graph handler **************************************************************/                                        
GL_PageControls_TypeDef* NewGraphChart (uint16_t ID, const uint8_t* labelX,
                                        const uint8_t* labelY, int16_t data_points[],
                                        uint32_t n_points, GL_bool Background );

GL_ErrStatus SetGraphChartPoints( GL_Page_TypeDef* pPage, uint16_t ID,
                                  int16_t data_points[], uint32_t n_points );
/* Radio Button handler *******************************************************/ 
GL_RadioButtonGrp_TypeDef* NewRadioButtonGrp (uint16_t ID);

GL_PageControls_TypeDef* AddRadioOption (GL_RadioButtonGrp_TypeDef* pThis, const uint8_t* label, void (*pEventHandler)(void));

/* ComboBox handler ***********************************************************/ 
GL_PageControls_TypeDef* NewComboBoxGrp (uint16_t ID);
GL_ErrStatus AddComboOption (GL_ComboBoxGrp_TypeDef* pThis, const uint8_t* label, void (*pEventHandler)(void));
uint8_t GetComboOptionActive(GL_Page_TypeDef* pPage, uint16_t ID);
GL_ErrStatus ResetComboOptionActive(GL_Page_TypeDef* pPage, uint16_t ID);
const uint8_t* GetComboOptionLabel(GL_Page_TypeDef* pPage, uint16_t ID);
GL_ErrStatus SetComboOptionLabel(GL_Page_TypeDef* pPage, uint16_t ID, const uint8_t* label);

/* Line handler **************************************************************/ 
GL_PageControls_TypeDef* NewLine (uint32_t ID, __IO uint32_t Len, 
																	GL_Direction direction, __IO uint16_t Colour);
																				
/* Rect handler **************************************************************/ 
GL_PageControls_TypeDef* NewRect (uint16_t ID, __IO uint16_t LenX, __IO uint16_t LenY, __IO uint16_t Colour);

/* Graphic Page handler *******************************************************/ 
GL_PageControls_TypeDef* NewProgressBar (uint16_t ID, __IO uint16_t Len, __IO uint16_t Value, GL_Direction direction, __IO uint16_t Colour);
GL_ErrStatus SetProgressBarValue(GL_Page_TypeDef* pPage, uint16_t ID, uint16_t Value);
/* Graphic Page handler *******************************************************/ 
GL_ErrStatus Create_PageObj (GL_Page_TypeDef* pThis, void (*TimerHandler)());
																				

GL_ErrStatus AddPageControlObj (uint32_t PosX, uint32_t PosY,
                                GL_PageControls_TypeDef* objPTR,
                                GL_Page_TypeDef* pagePTR);
                                
GL_ErrStatus DestroyPageControl ( GL_Page_TypeDef* pPage, uint16_t ID );

GL_ErrStatus DestroyPage (GL_Page_TypeDef *pThis);

GL_ErrStatus ShowPage(GL_Page_TypeDef* pThis, GL_bool bVal);

void RefreshPage(GL_Page_TypeDef* pThis);

void ChangePage(GL_Page_TypeDef* pPageOld, GL_Page_TypeDef* pPageNew);
/*****************************************************************/
GL_PageControls_TypeDef* NewCustomBtn (
			uint16_t ID, 
			const uint32_t Color,
			const uint32_t ColorPress,
			const uint8_t* pText, 
			const font_list_t Font, 
			const uint16_t FontColour, 
			uint16_t Width, 
			uint8_t Height, 
			void (*pEventHandler)(void));
			
GL_ErrStatus CustomBtn_SetText(
			GL_Page_TypeDef* pPage,
			uint16_t ID, 
			const uint8_t* pText, 
			const font_list_t Font, 
			const uint16_t FontColour);
/* Common GUI handler *********************************************************/
void GL_Cross(uint16_t Ypos, uint16_t Xpos); 

void GL_DrawButtonBMP(uint16_t maxX, uint16_t minX, uint16_t maxY,
                      uint16_t minY, uint8_t* ptrBitmap);
                      
GL_bool GetObjStatus(GL_Page_TypeDef* pThis, uint16_t ID);

GL_ErrStatus RefreshPageControl( GL_Page_TypeDef* pPage, uint16_t ID);

void Set_LCD_Resolution( uint16_t Lcd_Width, uint16_t Lcd_Height );

void NullFunc(void);

/* Input handler **************************************************************/
uint8_t CompareCoordinates(uint16_t u16_XMax, uint16_t u16_XMin,
                           uint16_t u16_YMax, uint16_t u16_YMin);
                           
uint8_t CompareJoyCoordinates(uint16_t u16_XMax, uint16_t u16_XMin,
                              uint16_t u16_YMax, uint16_t u16_YMin);
                              
void ProcessInputData(void);
void GL_SetBackLightDeep_Off(uint32_t deep);
void GL_SetBackLightTimeout_Off(uint32_t timeout);
/* Delay handler **************************************************************/
void SetPowerSaveOption(void);
void TimeOutCalculate(uint32_t spend_time);
void GL_Delay(uint32_t nCount);
void TimingDelay_Decrement(void);

	
#ifdef __cplusplus
}
#endif


#endif /*__GRAPHIC_OBJECT_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
