/**
 ******************************************************************************
 * @file    sensor_hub.h
 * @brief   PEO Rover - SENSOR HUB: 传感器数据汇聚（推送式 + 一致快照）
 *
 * 各传感器在自己的中断里（按各自 sample rate）把最新数据【推送】到 hub，
 * 上层通过 SENSOR_GetHub() 一次性获取带时间戳的一致快照。
 *
 * 推送式（push）数据流：
 *   MPU6050  EXTI3 ISR（100 Hz）     -> SENSOR_PushMpu(...)
 *   VL53L1X  EXTI0/1/2 ISR（~13 Hz） -> SENSOR_PushDist(...)
 *   CONTROL  TIM6 ISR（1 kHz）       -> SENSOR_PushWheel(...)
 *
 * 每个字段组带一个「最后更新时间戳」（TIMER_GetTick() 的 SysTick ms 计数），
 * 快照里能看出每份数据是多久前刷新的。
 *
 * 一致性：SENSOR_GetHub() 关中断拷贝整个结构体，保证字段间一致（同一时刻 T）。
 *
 * 数据访问（唯一入口）：所有传感器数据统一经 SENSOR_GetHub() 读取——读整个
 * SensorData_t 快照后按字段访问（如 s.current_mA[0]、s.dist_front_mm、s.quat_w）。
 * 各传感器组件（MPU6050/VL53/INA3221）不提供单独 getter。
 ******************************************************************************
 */
#ifndef __SENSOR_HUB_H__
#define __SENSOR_HUB_H__

#include "stm32f4xx.h"

/* 测距索引（与 TOF400C_FRONT/LEFT/RIGHT 的 0/1/2 一致） */
#define SENSOR_DIST_FRONT   (0U)
#define SENSOR_DIST_LEFT    (1U)
#define SENSOR_DIST_RIGHT   (2U)

/* 电机数量 */
#define SENSOR_WHEEL_COUNT  (4U)

/* ============================================================================
   传感器采样速率汇总（集中预定义）
   ----------------------------------------------------------------------------
   | 数据源            | 触发方式        | 速率宏 / 档位                | 值      |
   |-------------------|-----------------|------------------------------|---------|
   | 编码器/控制循环   | TIM6 中断        | PEO_CTRL_LOOP_HZ (control.h) | 1000 Hz |
   | MPU6050           | EXTI3 中断       | MPU6050_SAMPLE_HZ (mpu6050.h)| 100 Hz  |
   | VL53L1X 测距      | EXTI0/1/2 中断   | PEO_PM_VL53_PROFILE (tof400c)| 档位    |
   | INA3221 电流电压  | TIM7 轮询        | SENSOR_RATE_INA_HZ (本文件)  | 10 Hz   |
   ============================================================================ */
#define SENSOR_RATE_INA_HZ     (10U)   /* INA3221 电流/电压轮询速率（TIM7） */
#define SENSOR_RATE_REPORT_HZ  (50U)   /* sensorhub 上报周期 */

/* ============================================================================
   模块启用开关（0 = 禁用，1 = 启用）
   ----------------------------------------------------------------------------
   控制 main 里各模块 Init 的编译/调用；boot banner 的 MODULES 节自动同步。
   集中在此（而非 main.c），因为 print_boot_banner() 也需要读这些开关。
   ============================================================================ */
#define PEO_ENABLE_ENCODER    0
#define PEO_ENABLE_MOTOR      0
#define PEO_ENABLE_CONTROL    0
#define PEO_ENABLE_TOF400C    0
#define PEO_ENABLE_INA3221    0
#define PEO_ENABLE_MPU6050    1

/* ============================================================================
   传感器数据包（当前状态快照）
   ============================================================================ */
typedef struct
{
    /* ---- 3 路 VL53L1X 测距 ---- */
    uint16_t dist_front_mm;        /* 前（mm）           */
    uint16_t dist_left_mm;         /* 左（mm）           */
    uint16_t dist_right_mm;        /* 右（mm）           */
    uint8_t  dist_front_status;    /* 前状态（0 = 有效） */
    uint8_t  dist_left_status;     /* 左状态（0 = 有效） */
    uint8_t  dist_right_status;    /* 右状态（0 = 有效） */
    uint32_t dist_timestamp_ms;    /* 最近一次测距更新（SysTick ms） */

    /* ---- MPU6050 ---- */
    float quat_w;                  /* 四元数 (w,x,y,z)   */
    float quat_x;
    float quat_y;
    float quat_z;
    float accel_x;                 /* 三轴加速度（g）    */
    float accel_y;
    float accel_z;
    uint32_t mpu_timestamp_ms;     /* 最近一次 MPU 更新  */

    /* ---- 编码器 / 控制循环 ---- */
    int32_t  speed_d100[4];        /* 四轮速度（0.01 cm/s，正 = 前进） */
    int32_t  odometer[4];          /* 四轮里程计数值                    */
    uint32_t ctrl_timestamp_ms;    /* 最近一次控制更新                  */

    /* ---- INA3221 电流 / 电压（4 路电机） ---- */
    int16_t  current_mA[4];        /* 四路电机电流（mA，有符号）        */
    uint16_t bus_voltage_mV[4];    /* 四路总线电压（mV）                */
    uint32_t ina_timestamp_ms;     /* 最近一次 INA 更新                 */

    /* ---- 整帧总时间戳 + 命令应答（ACK） ---- */
    uint32_t sys_tick_ms;          /* 整帧快照总时间戳（SysTick，快照时更新） */
    uint8_t  last_cmd_id;          /* 最近处理的命令 ID（回显）          */
    uint8_t  last_cmd_status;      /* 执行结果：0=OK，非0=错误码          */
} SensorData_t;

/* ============================================================================
   推送接口（各传感器 ISR 调用，内部自动打时间戳）
   ============================================================================ */

/** @brief MPU6050 推送四元数 + 加速度（EXTI3 中断里调）。 */
void SENSOR_PushMpu(float qw, float qx, float qy, float qz,
                    float ax, float ay, float az);

/** @brief VL53L1X 推送一路测距（which = SENSOR_DIST_FRONT/LEFT/RIGHT）。 */
void SENSOR_PushDist(uint8_t which, uint16_t mm, uint8_t status);

/** @brief CONTROL 推送一个电机的速度 + 里程（TIM6 中断里调）。 */
void SENSOR_PushWheel(uint8_t motor, int32_t speed_d100, int32_t odometer);

/** @brief INA3221 推送四路电机电流 + 电压（TIM7 轮询里调）。 */
void SENSOR_PushIna(int16_t *current_mA, uint16_t *bus_voltage_mV);

/** @brief 设置命令应答（F407 执行命令后调，随 sensorhub 上报回显）。 */
void SENSOR_SetCmdAck(uint8_t cmd_id, uint8_t status);

/* ============================================================================
   获取一致快照
   ============================================================================ */

/**
 * @brief  获取当前所有传感器数据的一致快照（关中断拷贝）。
 * @param  out  输出缓冲区，函数把整个 SensorData_t 拷到这里。
 * @note   关中断保证字段间一致；拷贝很快（微秒级），可安全在主循环调用。
 */
void SENSOR_GetHub(SensorData_t *out);

/* ============================================================================
   启动诊断（boot banner）—— 原在 main.c，迁入此处以保持 main 整洁
   ============================================================================ */

/** @brief 打印全面启动诊断：实际时钟/构建/驱动参数/传感器配置/模块状态。 */
void print_boot_banner(void);

#endif /* __SENSOR_HUB_H__ */
