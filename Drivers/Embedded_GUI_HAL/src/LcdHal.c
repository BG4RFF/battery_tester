/**
  ******************************************************************************
  * @file    LcdHal.c
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

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "LcdHal.h"
//#include "font.h"

//#include "FreeRTOS.h"
//#include "task.h"
#include "lcd_driver/ili9320.h"

/** @addtogroup Embedded_GUI_Library
  * @{
  */

/** @defgroup LcdHal
  * @brief LcdHal main functions
  * @{
  */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variables to handle the right font */

uint16_t LCD_Height = 0;
uint16_t LCD_Width  = 0;
static LCD_Direction_TypeDef LCD_Direction = _0_degree;
static color_t          GL_TextColor;
static color_t          GL_BackColor;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Sets the Text color.
  * @param  GL_TextColor: Text color global variable used by GL_DrawChar
  *         and GL_DrawPicture functions.
  * @retval None
  */
void GL_SetTextColor(color_t GL_NewTextColor)
{
  GL_TextColor = GL_NewTextColor;
}

/**
  * @brief  Gets the Text color.
  * @param  None
  * @retval Specifies the Text color code
  */
color_t GL_GetTextColor()
{
	return GL_TextColor;
}

/**
  * @brief  Sets the Background color.
  * @param  Color: specifies the Background color code RGB(5-6-5).
  * @param  GL_BackColor: Background color global variable used by
  *         GL_DrawChar and GL_DrawPicture functions.
  * @retval None
  */
void GL_SetBackColor(uint32_t GL_NewBackColor)
{
  GL_BackColor = GL_NewBackColor;
}

/**
  * @brief  Clears the whole LCD.
  * @param  Color: specifies the Background color code RGB(5-6-5) for the Display.
  * @retval None
  */
void GL_ClearWindow(color_t Color)
{
	//lcddrv_WriteDataBlock((uint16_t*)&Color, _ClearLen, 0);
}

/**
  * @brief  Clears the whole LCD.
  * @param  Color: specifies the Background color code RGB(5-6-5) for the Display.
  * @retval None
  */
void GL_ClearScreen(color_t Color)
{
	GL_SetDisplayWindow(0, 0, LCD_Width, LCD_Height, 0);

	//lcddrv_WriteDataBlock((uint16_t*)&Color, _ClearLen, 0);
}

/**
  * @brief  Draws a character on LCD without background.
  * @param  Xpos: the Line where to display the character shape.
  *         This parameter can be one of the following values:
  * @param  Ypos: start column address.
  * @param  c: pointer to the character data.
  * @retval None
  */
void GL_LCD_DrawCharTransparent(uint16_t Xpos, uint16_t Ypos, const GUI_CHARINFO *pChar, uint16_t YSize) /* 16bit char ????? */
{
  uint32_t line_index = 0, 	pixel_index = 0, cur_line_index = 0;
  uint16_t Xaddress = 0;
  uint16_t Yaddress = 0;
  //uint16_t tmp_color = 0;

  Xaddress = Xpos;
  Yaddress = Ypos;

	if (pChar->BytesPerLine == 1)
	{
		for (line_index = 3; line_index < YSize; line_index++)
		{
			for (pixel_index = 0; pixel_index <pChar->XSize ; pixel_index++)
			{
				if (pChar->pData[line_index] & (0x80 >> pixel_index))
				{
					LCD_PutPixel(Xaddress, Yaddress, GL_TextColor, FirstPixel);
				}

                Xaddress++;

			}

			Xaddress = Xpos;
			Yaddress++;
		}
	}
	else if (pChar->BytesPerLine == 2)
	{
		for (line_index = 6; line_index < YSize*2; line_index+=2)
		{
			for (pixel_index = 0; pixel_index < pChar->XSize ; pixel_index++)
			{
				if (pChar->pData[(pixel_index>=8)?(line_index+1):(line_index)] & ( (0x80>>(pixel_index %8) )))
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_TextColor, FirstPixel);
				}
				else
				{
					Xaddress++;
				}
			}
			Xaddress = Xpos;
			Yaddress++;
		}
	}
	else if (pChar->BytesPerLine == 3)
	{
		for (line_index = 9; line_index < YSize*3; line_index+=3)
		{
			for (pixel_index = 0; pixel_index < pChar->XSize ; pixel_index++)
			{
				if(pixel_index<8)
				{
					cur_line_index = line_index;
				}
				else if(pixel_index<16)
				{
					cur_line_index = line_index + 1;
				}
				else
				{
					cur_line_index = line_index + 2;
				}

				if (pChar->pData[cur_line_index] & ( (0x80>>(pixel_index %8) )))
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_TextColor, FirstPixel);
				}
				else
				{
					Xaddress++;
				}
			}
			Xaddress = Xpos;
			Yaddress++;
		}
	}
	else if (pChar->BytesPerLine == 6)
	{
		for (line_index = 3*6; line_index < YSize*6; line_index+=6)
		{
			for (pixel_index = 0; pixel_index < pChar->XSize ; pixel_index++)
			{
				if(pixel_index<8)
				{
					cur_line_index = line_index;
				}
				else if(pixel_index<8*2)
				{
					cur_line_index = line_index + 1;
				}
				else  if(pixel_index<8*3)
				{
					cur_line_index = line_index + 2;
				}
				else if(pixel_index<8*4)
				{
					cur_line_index = line_index + 3;
				}
				else if(pixel_index<8*5)
				{
					cur_line_index = line_index + 4;
				}
				else if(pixel_index<8*6)
				{
					cur_line_index = line_index + 5;
				}

				if (pChar->pData[cur_line_index] & ( (0x80>>(pixel_index %8) )))
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_TextColor, FirstPixel);
				}
				else
				{
					Xaddress++;
				}
			}
			Xaddress = Xpos;
			Yaddress++;
		}
	}
}



/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: the Line where to display the character shape.
  *         This parameter can be one of the following values:
  *     @arg  - Linex: where x can be 0..9
  * @param  Ypos: start column address.
  * @param  c: pointer to the character data.
  * @retval None
  */
void GL_LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const GUI_CHARINFO *pChar, uint16_t YSize) /* 16bit char ????? */
{
  uint32_t line_index = 0, 	pixel_index = 0, cur_line_index = 0;
  uint16_t Xaddress = 0;
  uint16_t Yaddress = 0;
  //uint16_t tmp_color = 0;

  Xaddress = Xpos;
  Yaddress = Ypos;

	if (pChar->BytesPerLine == 1)
	{
		for (line_index = 3; line_index < YSize; line_index++)
		{
			for (pixel_index =0 ; pixel_index < pChar->XSize ; pixel_index++)
			{
				if (pChar->pData[line_index] & (0x80 >> pixel_index))
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_TextColor, FirstPixel);
				}
				else
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_BackColor, FirstPixel);
				}
			}

			Xaddress = Xpos;
			Yaddress++;
		}
	}
	else if (pChar->BytesPerLine == 2)
	{
		for (line_index = 6; line_index < (YSize * 2)/*GL_FontHeight */; line_index+=2)
		{
			for (pixel_index = 0; pixel_index < pChar->XSize ; pixel_index++)
			{
				if (pChar->pData[(pixel_index>=8)?(line_index+1):(line_index)] & ( (0x80>>(pixel_index %8) )))
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_TextColor, FirstPixel);
				}
				else
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_BackColor, FirstPixel);
				}
			}
			Xaddress = Xpos;
			Yaddress++;
		}
	}
	else if (pChar->BytesPerLine == 3)
	{
		for (line_index = 9; line_index < (YSize * 3)/*GL_FontHeight */; line_index+=3)
		{
			for (pixel_index = 0; pixel_index < pChar->XSize ; pixel_index++)
			{
				if(pixel_index<8)
				{
					cur_line_index = line_index;
				}
				else if(pixel_index<16)
				{
					cur_line_index = line_index + 1;
				}
				else
				{
					cur_line_index = line_index + 2;
				}

				if (pChar->pData[cur_line_index] & ( (0x80>>(pixel_index %8) )))
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_TextColor, FirstPixel);
				}
				else
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_BackColor, FirstPixel);
				}
			}
			Xaddress = Xpos;
			Yaddress++;
		}
	}
	else if (pChar->BytesPerLine == 6)
	{
		for (line_index = 6*3; line_index < (YSize * 6)/*GL_FontHeight */; line_index+=6)
		{
			for (pixel_index = 0; pixel_index < pChar->XSize ; pixel_index++)
			{
				if(pixel_index<8)
				{
					cur_line_index = line_index;
				}
				else if(pixel_index<8*2)
				{
					cur_line_index = line_index + 1;
				}
				else  if(pixel_index<8*3)
				{
					cur_line_index = line_index + 2;
				}
				else if(pixel_index<8*4)
				{
					cur_line_index = line_index + 3;
				}
				else if(pixel_index<8*5)
				{
					cur_line_index = line_index + 4;
				}
				else if(pixel_index<8*6)
				{
					cur_line_index = line_index + 5;
				}

				if (pChar->pData[cur_line_index] & ( (0x80>>(pixel_index %8) )))
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_TextColor, FirstPixel);
				}
				else
				{
					LCD_PutPixel(Xaddress++, Yaddress, GL_BackColor, FirstPixel);
				}
			}
			Xaddress = Xpos;
			Yaddress++;
		}
	}
}

/**
  * @brief  Sets a display window
  * @param  Xpos: specifies the X bottom left position.
  * @param  Ypos: specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width: display window width.
  * @retval None
  */
void GL_SetDisplayWindow(uint16_t Xmin, uint16_t Ymin, uint16_t Xmax, uint16_t Ymax, uint32_t Dir)
{
//    uint32_t h, w, x, y;
//
//    x = Xmin;
//    y = Ymin;
//    h = Ymax - Ymin;
//    w = Xmax - Xmin;
//	_ClearLen = (w*h);


	//lcddrv_SetDisplayWindow(x, y, h, w, Dir);
}

/**
  * @brief  Displays a line.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: line length.
  * @param  Direction: line direction.
  *         This parameter can be one of the following values:
  *     @arg  Vertical
  *     @arg  Horizontal
  * @retval None
  */
void GL_DrawLine(uint16_t Xmin, uint16_t Ymin, uint16_t Length, uint8_t Direction)
{
  //lcddrv_DrawLine(Xmin, Ymin, Length, Direction, GL_TextColor);
    if(Direction == GL_Vertical) {
        for(uint32_t i = 0; i<Length; i++) {
            ili9320_SetPoint(Xmin + i, Ymin, GL_TextColor);
        }
    } else {

        for(uint32_t i = 0; i<Length; i++) {
            ili9320_SetPoint(Xmin, Ymin + i, GL_TextColor);
        }
    }
}

/**
  * @brief  Displays a rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: display rectangle height.
  * @param  Width: display rectangle width.
  * @retval None
  */
void GL_LCD_DrawRect(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  GL_DrawLine(Xpos, Ypos, Width, GL_Horizontal);
  GL_DrawLine((Xpos + Height), Ypos, Width, GL_Horizontal);

  GL_DrawLine(Xpos, Ypos, Height, GL_Vertical);
  GL_DrawLine(Xpos, (Ypos + Width + 1), Height, GL_Vertical);
}

/**
  * @brief  Switches the backlight either ON or OFF carefully
  * @param  state. This parameter can be one of the following values:
  *     @arg   GL_ON
  *     @arg   GL_OFF
  * @retval None
  */
void GL_BackLightSwitch(uint8_t u8_State)
{
    static uint8_t curr_state = 0;
    int i;

    if(u8_State < curr_state)
    {
			/* Turning OFF the LCD Backlight */
			for(i=curr_state; i>u8_State; i--)
			{
				//vTaskDelay(5);
				//LCD_BackLight(i);
			}
    }
    else
    {
			/* Turning ON the LCD Backlight */
			for(i=curr_state; i<u8_State; i+=2)
			{
				//vTaskDelay(5);
				//LCD_BackLight(i);
			}
    }
    curr_state = u8_State;
}

/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval None
  */
void GL_LCD_Init(uint32_t Wigth, uint32_t Height)
{
    LCD_Height = Height;
    LCD_Width  = Wigth;
    LCD_Direction = _0_degree;
    //_ClearLen = (uint32_t)(LCD_Height*LCD_Width);
}

/**
  * @brief  Disables LCD Window mode.
  * @param  None
  * @retval None
  */
void GL_LCD_WindowModeDisable(void)
{
  //GL_SetDisplayWindow(239, 0x13F, 240, 320);
  //LCD_WriteReg(0x0011,0x6058);
}

/**
  * @brief  Draw one pixel at position given by Xpos, Ypos of color Color.
  *     Three different modes are specified by PixelSpec in order to save time:
  *     1.FirstPixel, MiddlePixel and LasPixel are used for fastened block-writing
  *       to GRAM (for instance: drawing rectangle, horizontal line etc.)
  *     2.SinglePixel is used for drawing stand-alone pixel. (for instance:
  *       drawing circle, character etc.).
  * @param  Xpos: specifies X position
  * @param  Ypos: specifies Y position
  * @param  Color: RGB color of point
  * @param  PixelSpec: specifies Mode of putting pixel.
  *         This parameter can be one of the following values:
  *     @arg  - FirstPixel:  Starting pixel of block-writing sequence.
  *     @arg  - MiddlePixel: Middle-located pixel of block-writing sequence.
  *     @arg  - LasPixel:    Ending pixel of block-writing sequence
  *     @arg  - SinglePixel: Separated pixel.
  * @retval None
  */
void LCD_PutPixel(uint16_t Xpos, uint16_t Ypos, uint16_t Color, uint8_t PixelSpec)
{
	//lcddrv_PutPixel(Xpos, Ypos, Color);
    ili9320_SetPoint(Xpos, Ypos, Color);
}

/**
  * @brief  Get color of pixel located at appropriate position
  * @param  Xpos: specifies X position
  * @param  Ypos: specifies Y position
  * @retval uint16_t - RGB color of required pixel.
  */
uint16_t LCD_GetPixel(uint16_t Xpos, uint16_t Ypos)
{

	return 0;//LCD_GetPoint(Xpos, Ypos);
}




/**
  * @brief  LCD_Change_Direction
  * @param  RCC_APBPeriph: specifies the APB peripheral to gates its clock.
  * @param  Direction: The Drawing Direction
  *         This parameter can be one of the following values:
  *     @arg  _0_degree
  *     @arg  _90_degree
  *     @arg  _180_degree
  *     @arg  _270_degree
  * @retval None
  */
void LCD_Change_Direction(LCD_Direction_TypeDef Direction)
{
  LCD_Direction = Direction;

  if (LCD_Direction == _0_degree)
  {
		//lcddrv_SetDirection(DirX_LeftToRight, DirY_UpToDown);
  }
  else if (LCD_Direction == _90_degree)
  {
		//lcddrv_SetDirection(DirX_RightToLeft,DirY_UpToDown);
  }
  else if (LCD_Direction == _180_degree)
  {
		//lcddrv_SetDirection(DirX_RightToLeft,DirY_DownToUp);
  }
  else if (LCD_Direction == _270_degree)
  {
		//lcddrv_SetDirection(DirX_LeftToRight,DirY_DownToUp);
  }
}

/**
  * @brief  Copy 4 bytes from bitmap array to 32Bit buffer
  * @param  ptrBitmap - Bitmap pointer
  * @param  ptr32BitBuffer - 32Bit buffer to fill
  * @retval None
  */
void BmpBuffer32BitRead(uint32_t* ptr32BitBuffer, uint8_t* ptrBitmap)
{
  *ptr32BitBuffer = 0;
  *ptr32BitBuffer = (*ptrBitmap);
  *ptr32BitBuffer += (*(ptrBitmap + 1)) << 8;
  *ptr32BitBuffer += (*(ptrBitmap + 2)) << 16;
  *ptr32BitBuffer += (*(ptrBitmap + 3)) << 24;
}


/**
  * @brief  LCD_DrawColorBMP
  * @param  *ptrBitmap:   The pointer to the image
  * @param  Xpos_Init: The X axis position
  * @param  Ypos_Init: The Y axis position
  * @param  Height:    The Height of the image
  * @param  Width:     The Width of the image
  * @retval None
  */
void LCD_DrawColorBMP(uint8_t* ptrBitmap, uint16_t Xmin, uint16_t Ymin, uint16_t Xmax, uint16_t Ymax)
{
    uint32_t uDataAddr = 0, uBmpSize = 0;

    BmpBuffer32BitRead(&uBmpSize, ptrBitmap + 2);
    BmpBuffer32BitRead(&uDataAddr, ptrBitmap + 10);

    GL_SetDisplayWindow(Xmin, Ymin, Xmax, Ymax, LCD_Direction);

    //lcddrv_WriteDataBlock((uint16_t*)((uint32_t)ptrBitmap + uDataAddr), (uBmpSize-uDataAddr)/2, 1);
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
