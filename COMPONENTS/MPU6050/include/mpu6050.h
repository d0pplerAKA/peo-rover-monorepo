#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdarg.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "PEO_PINMAP.H"

#include "COMPONENTS/motion_driver_dmp/driver/eMPL/inv_mpu.h"
#include "COMPONENTS/motion_driver_dmp/driver/eMPL/inv_mpu_dmp_motion_driver.h"
#include "COMPONENTS/motion_driver_dmp/mllite/ml_math_func.h"

#include "COMPONENTS/I2C/include/i2c.h"
#include "COMPONENTS/DELAY/include/delay.h"
#include "COMPONENTS/BUILTIN_LED/builtin_led.h"

/* ============================================================================
   MPU6050 DMP 配置
   ----------------------------------------------------------------------------
   MPU6050_SAMPLE_HZ 是【采样率 Hz】，不是 SMPLRT_DIV 分频器。
   eMPL 的 mpu_set_sample_rate(rate) / dmp_set_fifo_rate(rate) 都按 Hz 算，
   内部才换算成分频器（SMPLRT_DIV = 1000/rate - 1）。DMP 输出上限 200 Hz。
   ============================================================================ */
#define MPU6050_SAMPLE_HZ       (100U)          /* 采样率 + DMP 输出率（Hz） */

#define MPU6050_Q30             (1073741824.0f) /* q30 -> float（1 << 30）    */
#define MPU6050_ACCEL_SENS      (16384.0f)      /* ±2g 的 LSB/g              */

#define MPU6050_WHO_AM_I_REG    (0x75)
#define MPU6050_WHO_AM_I_EXP    (0x68)          /* AD0 = GND */

/* ============================================================================
   eMPL 平台函数（在 mpu6050.c 实现）
   ============================================================================ */
int reg_int_cb(struct int_param_s *int_param);
int _MLPrintLog(int priority, const char *tag, const char *fmt, ...);

/* ============================================================================
   MPU6050 DMP 驱动 API
   ============================================================================ */
void    MPU6050_Init(void);                                     /* I2C + DMP 配置（不启动） */
uint8_t MPU6050_DMP_Init(void);                                 /* 0 = 配置成功             */
uint8_t MPU6050_Start(void);                                    /* 0 = 启动成功，开始触发中断 */
uint8_t MPU6050_DataReady(void);                                /* 1 = 有新姿态   */
void    MPU6050_GetQuaternion(float *qw, float *qx, float *qy, float *qz);
void    MPU6050_GetAccel(float *ax, float *ay, float *az);      /* 单位 g        */
uint8_t run_self_test(void);                                   /* 自检 + 偏置校准 */

#endif /* __MPU6050_H__ */
