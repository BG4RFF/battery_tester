/**
  ******************************************************************************
  * @file    LcdHal.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This file contains all the LCD functions whose
  *          implementation depends on the LCD Type used in your Application.
  *          You only need to change these functions implementations
  *          in order to reuse this code with other LCD
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

/* Define to prevent recursive inclusion ---------------------------------------*/
#ifndef __LCD_HAL_H
#define __LCD_HAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32HAL.h"
#include "graphicObjectTypes.h"
#include "bsp.h"
//#include "lcd_driver/lcd_manager.h"

#include "font.h"
/** @addtogroup Embedded_GUI_Library
  * @{
  */

/** @addtogroup LcdHal
  * @{
  */

/** @defgroup LcdHal_Exported_Types
  * @{
  */

/**
  * @brief  LCD_Direction_TypeDef enumeration definition
  */
typedef enum
{ _0_degree = 0,
  _90_degree,
  _180_degree,
  _270_degree
}LCD_Direction_TypeDef;

/**
  * @brief  GL_FlagStatus, GL_ITStatus enumeration definition
  */
typedef enum {GL_RESET = 0, GL_SET = !GL_RESET} GL_FlagStatus, GL_ITStatus;


typedef uint32_t color_t;
/**
  * @}
  */

/** @defgroup LcdHal_Imported_Variables
  * @{
  */
//extern __IO color_t          GL_TextColor;
//extern __IO color_t          GL_BackColor;
/**
  * @}
  */
#define BLACK				0x0000
#define NAVY         0x000F
#define DGREEN       0x03E0
#define DCYAN        0x03EF
#define MAROON       0x7800
#define PURPLE       0x780F
#define OLIVE        0x7BE0
#define GREY         0xF7DE
#define LGRAY        0xC618
#define DGRAY        0x7BEF
#define BLUE         0x001F
#define GREEN        0x07E0
#define CYAN         0x07FF
#define RED          0xF800
#define MAGENTA      0xF81F
#define YELLOW       0xFFE0
#define WHITE        0xFFFF

/* LCD color */
#define GL_White              WHITE
#define GL_Black              BLACK
#define GL_Grey               GREY
#define GL_Blue               BLUE
#define GL_Blue2              BLUE
#define GL_Red                RED
#define GL_Magenta            MAGENTA
#define GL_Green              GREEN
#define GL_Cyan               CYAN
#define GL_Yellow             YELLOW
#define GL_Transparent				(0xFFFFFFFF-1)

#define GL_Horizontal         0x00
#define GL_Vertical           0x01

#define FirstPixel            0x01
#define MiddlePixel           0x02
#define LastPixel             0x04
#define SinglePixel           0x08

#define CursorColor           GL_Black
//#define GL_TextColor          TextColor
//#define GL_BackColor          BackColor
/**
  * @}
  */

/** @defgroup LcdHal_Exported_Variables
  * @{
  */
//extern __IO uint8_t           GL_Font;
//extern __IO uint8_t           GL_FontWidth;
//extern __IO uint8_t           GL_FontHeight;
//extern __IO uint16_t 					LCD_Height;
//extern __IO uint16_t 					LCD_Width;
/**
  * @}
  */

/** @defgroup LcdHal_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup LcdHal_Exported_Functions
  * @{
  */
#define COLOR_RGB565(R,G,B) ((color_t)((((R) / 8) << 11) + (((G) / 4) << 5) + ((B) / 8)))
#define COLOR_BGR565(R,G,B) ((color_t)((((B) / 8) << 11) + (((G) / 4) << 5) + ((R) / 8)))

color_t GL_GetTextColor(void);
void GL_SetTextColor(color_t TextColor);
void GL_SetBackColor(color_t BackColor);
void GL_ClearWindow(color_t Color);
void GL_ClearScreen(color_t Color);



void GL_LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii, GL_bool Trasparent_Flag);
void GL_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width, uint32_t Dir);
void GL_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void GL_LCD_DrawRect(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void GL_LCD_DrawCircle(uint8_t Xpos, uint16_t Ypos, uint16_t Radius);
//void GL_DrawBMP(uint8_t* ptrBitmap);
void GL_BackLightSwitch(uint8_t u8_State);
void GL_LCD_WindowModeDisable(void);
void LCD_PutPixel(uint16_t Xpos, uint16_t Ypos, uint16_t Color, uint8_t PixelSpec);
uint16_t LCD_GetPixel(uint16_t Xpos, uint16_t Ypos);
void GL_LCD_DrawCharTransparent(uint16_t Xpos, uint16_t Ypos, const GUI_CHARINFO *pChar, uint16_t YSize);
void GL_LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const GUI_CHARINFO *pChar, uint16_t YSize);


void LCD_Change_Direction(LCD_Direction_TypeDef Direction);
void LCD_WriteChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c);
void LCD_PrintChar(uint16_t Line, uint16_t Column, uint8_t Ascii);
void LCD_DrawColorBMP(uint8_t* ptrBitmap, uint16_t Xpos_Init, uint16_t Ypos_Init, uint16_t Height, uint16_t Width);
void GL_LCD_Init(uint32_t Wigth, uint32_t Height);
#ifdef __cplusplus
}
#endif

#endif /*__LCD_HAL_H */

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
