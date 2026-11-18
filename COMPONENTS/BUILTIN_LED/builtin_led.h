#ifndef __BUILTIN_LED_H__
#define __BUILTIN_LED_H__

#include "stm32f4xx.h"

#include "COMPONENTS/PEO_PINMAP.H"

void BUILTIN_LED_Init(void);
void BUILTIN_LED_On(void);
void BUILTIN_LED_Off(void);
void BUILTIN_LED_Toggle(void);

#endif

