/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
* C-file generated by                                                *
*                                                                    *
*        Font converter for emWin version 5.22                       *
*        Compiled Jul  4 2013, 12:18:44                              *
*        (C) 1998 - 2013 Segger Microcontroller GmbH & Co. KG
*                                                                    *
**********************************************************************
*                                                                    *
* Source file: test_font.c                                           *
* Font:        <unknown>                                             *
* Height:      16                                                    *
*                                                                    *
**********************************************************************
*                                                                    *
* Initial font height:  16                                           *
*                                                                    *
**********************************************************************
*/

#include "font.h"

#ifdef USE_FONT_ARIAL16
#include "fonts/Arial16.c"
#endif
#ifdef USE_FONT_ARIAL17
#include "fonts/Arial17.c"
#endif
#ifdef USE_FONT_ARIAL18
#include "fonts/Arial18.c"
#endif
#ifdef USE_FONT_ARIAL19
#include "fonts/Arial19.c"
#endif
#ifdef USE_FONT_ARIAL22
#include "fonts/Arial22.c"
#endif
#ifdef USE_FONT_ARIAL24
#include "fonts/Arial24.c"
#endif
#ifdef USE_FONT_ARIAL32
#include "fonts/Arial32.c"
#endif


const void *_Fonts[FONT_COUNT] =
{
#ifdef USE_FONT_ARIAL16
	[FONT_ARIAL_16] = (void *)&GUI_FontArial16,
#endif
#ifdef USE_FONT_ARIAL17
	[FONT_ARIAL_17] = (void *)&GUI_FontArial17,
#endif
#ifdef USE_FONT_ARIAL18
	[FONT_ARIAL_18] = (void *)&GUI_FontArial18,
#endif
#ifdef USE_FONT_ARIAL19
	[FONT_ARIAL_19] = (void *)&GUI_FontArial19,
#endif
#ifdef USE_FONT_ARIAL22
	[FONT_ARIAL_22] = (void *)&GUI_FontArial22,
#endif
#ifdef USE_FONT_ARIAL24
	[FONT_ARIAL_24] = (void *)&GUI_FontArial24,
#endif
#ifdef USE_FONT_ARIAL32
	[FONT_ARIAL_32] = (void *)&GUI_FontArial32,
#endif
};

static GUI_FONT *_CurrentFont = (GUI_FONT *)_Fonts;

/**
* @brief
* @param
* @retval None
*/
uint32_t FONT_GetLineLenPix(font_list_t Font, uint8_t *Str)
{
    uint32_t index = 0;
	GUI_FONT_t *pFont = (GUI_FONT_t*)_Fonts[Font];
    uint32_t PixCount = 0;

	/* Send the string character by character on lCD */
	while (*Str != 0)
	{
		/* Decrement the column position by GL_FontWidth */
		PixCount+= pFont->p.pProp->paCharInfo[*Str - pFont->p.pProp->First].XSize;
		/* Point on the next character */
		Str++;
		/* Increment the character counter */
		index++;
	}

	return PixCount;
}


/**
* @brief
* @param
* @retval None
*/
uint32_t FONT_GetMaxHeight(font_list_t Font)
{
	GUI_FONT_t *pFont = (GUI_FONT_t*)_Fonts[Font];

	return pFont->CHeight;
}


/**
* @brief  Sets the Font (Big or Small).
* @param  uFont: specifies the Font (GL_FONT_BIG or GL_FONT_SMALL).
* @retval None
*/
void FONT_SetFont(font_list_t Font)
{
	_CurrentFont = (GUI_FONT*)_Fonts[Font];
}


/**
* @brief  Displays a maximum of 20 char on the LCD.
* @param  Line: the Line where to display the character shape.
* @param  *ptr: pointer to the string to display on LCD.
* @param  Transparent_Flag: if TRUE the character is printed without changing the background
* @retval None
*/
void GL_DisplayAdjStringLine(uint16_t Xmin, uint16_t Ymin, uint8_t *ptr, uint32_t Transparent_Flag)
{
    uint32_t index = 0;
	GUI_FONT_t *pFont = (GUI_FONT_t*)_CurrentFont;
    const uint32_t iMaxChar = 64;


	if (Transparent_Flag)
	{
		/* Send the string character by character on lCD */
		while ((*ptr != 0) & (index < iMaxChar))
		{
			/* Display one character on LCD */
			GL_LCD_DrawCharTransparent(Xmin, Ymin, &pFont->p.pProp->paCharInfo[*ptr - pFont->p.pProp->First], pFont->YSize);
			/* Decrement the column position by GL_FontWidth */
			Xmin += pFont->p.pProp->paCharInfo[*(ptr) - pFont->p.pProp->First].XSize;
			/* Point on the next character */
			ptr++;
			/* Increment the character counter */
			index++;
		}
	}
	else
	{
		/* Send the string character by character on lCD */
		while ((*ptr != 0) & (index < iMaxChar))
		{
			/* Display one character on LCD */
			GL_LCD_DrawChar(Xmin, Ymin,&pFont->p.pProp->paCharInfo[*ptr - pFont->p.pProp->First], pFont->YSize);
			/* Decrement the column position by GL_FontWidth */
			Xmin += pFont->p.pProp->paCharInfo[*(ptr) - pFont->p.pProp->First].XSize;
			/* Point on the next character */
			ptr++;
			/* Increment the character counter */
			index++;
		}
	}
}

