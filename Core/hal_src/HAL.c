
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "hal.h"
#include "../inc/main.h"
#include "../vs_info.h"
#include "../ssd1306_src/ssd1306.h"
#include "../../NetXDuo/App/app_netxduo.h"
////////////////////////////////////////////////////////////////////////////////
void Display(char *_lpszMessage, uint8_t cursorX, uint8_t cursorY )
{
    ssd1306_SetCursor(cursorX, cursorY);
    ssd1306_WriteString(_lpszMessage, Font_7x10, White);
    ssd1306_UpdateScreen();
}
////////////////////////////////////////////////////////////////////////////////
void clearDisplay(void)
{
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}
////////////////////////////////////////////////////////////////////////////////
void InitDisplay()
{
	ssd1306_Init();
}
////////////////////////////////////////////////////////////////////////////////
void Output(char *_lpszBuffer)
{
		clearDisplay();
		Display(_lpszBuffer, 0, 0);      //LCD
}
////////////////////////////////////////////////////////////////////////////////
