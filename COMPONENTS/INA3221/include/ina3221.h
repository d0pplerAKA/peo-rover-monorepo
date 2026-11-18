#ifndef __INA3221_H__
#define __INA3221_H__

#include "stm32f4xx.h"

#include "COMPONENTS/PEO_PINMAP.H"

/* 采样电阻（毫欧）—— 模块内置 R100 = 0.1 Ω = 100 mΩ */
#define PEO_PM_INA3221_RSHUNT_MOHM   (100U)

void  INA3221_Init(void);
void  INA3221_PollInit(void);                       /* TIM7 轮询（SENSOR_RATE_INA_HZ） */

#endif
