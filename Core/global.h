#ifndef _GLOBAL_h__
#define _GLOBAL_h__

#include "main.h"
#include <stdint.h>

#define     NOADC (2)
#define 	ADC24 (1)
#define 	ADC12 (0)



// State
extern int 					g_bStream;		// Streaming data
extern int 					g_nHardwareRev;  // Hardware revision
extern int 					g_nBroadcast;
extern int 					g_nADC;

extern uint8_t	g_nCount;
extern char    g_ERXBuffer[255];
// For testing
extern int 					g_bTestLCD;
extern char					g_szIPAddressHost[32];
extern char 				g_szIPAddressDevice[32];

extern long GetCount();
extern void Output(char *);

extern void InitVar();

#endif
