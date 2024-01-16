/////////////////////////////////////////////////////////////////////////////////
// HAL: Specific for the Detector Ehternet AKA DE
// Display: ssd1306
#ifndef _HAL_DE_h__
#define _HAL_DE_h__

#include "main.h"
#include "../global.h"
/////////////////////////////////////////////////////////////////////////////////
// Wrapper function
extern void Display(char *, uint8_t cursorX, uint8_t cursorY);
extern void clearDisplay(void);
void Output(char *_lpszBuffer);
/////////////////////////////////////////////////////////////////////////////////
// Hardware specific function
extern SPI_HandleTypeDef 	hspi1;
#endif
