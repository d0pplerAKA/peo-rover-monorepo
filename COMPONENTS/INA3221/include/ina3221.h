#ifndef __INA3221_H__
#define __INA3221_H__

#include "stm32f4xx.h"

#include "PEO_PINMAP.H"

/* 电机通道索引（0-3 = M1-M4） */
#define INA3221_MOTOR_M1   0
#define INA3221_MOTOR_M2   1
#define INA3221_MOTOR_M3   2
#define INA3221_MOTOR_M4   3

/* 采样电阻（毫欧）—— 模块内置 R100 = 0.1 Ω = 100 mΩ */
#define PEO_PM_INA3221_RSHUNT_MOHM   (100U)

void  INA3221_Init(void);
void  INA3221_PollInit(void);                       /* TIM7 轮询（SENSOR_RATE_INA_HZ） */
float INA3221_GetShuntVoltage_mV(uint8_t motor);   /* 分流电压 mV（有符号） */
float INA3221_GetBusVoltage_V(uint8_t motor);      /* 总线电压 V           */
float INA3221_GetCurrent_mA(uint8_t motor);        /* 电流 mA = shunt/Rshunt */

#endif
