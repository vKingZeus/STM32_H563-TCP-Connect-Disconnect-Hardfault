#include "global.h"
#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// State
int 	g_bStream 		= 0;
int 	g_nHardwareRev 	= -1;
int 	g_bTestLCD		= 0;
int 	g_nBroadcast 	= 7;
int 	g_nADC			= ADC12;
uint8_t g_RXBuffer[12];

char	g_szIPAddressHost[32] = {0};
char 	g_szIPAddressDevice[32] = {0};


