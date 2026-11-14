/**
 ******************************************************************************
 * @file    sensor_hub.c
 * @brief   PEO Rover - SENSOR HUB 实现：推送写入 + 一致快照
 ******************************************************************************
 */
#include "include/sensor_hub.h"
#include "COMPONENTS/TIMER/include/timer.h"
#include "PEO_PINMAP.H"                              /* boot banner: 驱动/传感器地址宏 */
#include "COMPONENTS/CONTROL/include/control.h"      /* boot banner: PEO_CTRL_KP/KI/LOOP_HZ */
#include "COMPONENTS/TOF400C/include/tof400c.h"      /* boot banner: PEO_PM_VL53_DISTANCE_MODE 等 */
#include <stdio.h>                                   /* boot banner / dbg_f3: printf */

/* 活动数据包：各传感器 ISR 通过 Push 接口写入 */
static volatile SensorData_t g_sensor_data;

/**
 * @brief  MPU6050 推送四元数 + 加速度（EXTI3 中断里调，100 Hz）。
 */
void SENSOR_PushMpu(float qw, float qx, float qy, float qz,
                    float ax, float ay, float az)
{
    g_sensor_data.quat_w = qw;
    g_sensor_data.quat_x = qx;
    g_sensor_data.quat_y = qy;
    g_sensor_data.quat_z = qz;
    g_sensor_data.accel_x = ax;
    g_sensor_data.accel_y = ay;
    g_sensor_data.accel_z = az;
    g_sensor_data.mpu_timestamp_ms = TIMER_GetTick();
}

/**
 * @brief  VL53L1X 推送一路测距（EXTI0/1/2 中断里调，~13 Hz）。
 */
void SENSOR_PushDist(uint8_t which, uint16_t mm, uint8_t status)
{
    switch (which)
    {
        case SENSOR_DIST_FRONT:
            g_sensor_data.dist_front_mm     = mm;
            g_sensor_data.dist_front_status = status;
            break;
        case SENSOR_DIST_LEFT:
            g_sensor_data.dist_left_mm      = mm;
            g_sensor_data.dist_left_status  = status;
            break;
        case SENSOR_DIST_RIGHT:
            g_sensor_data.dist_right_mm     = mm;
            g_sensor_data.dist_right_status = status;
            break;
        default:
            break;
    }
    g_sensor_data.dist_timestamp_ms = TIMER_GetTick();
}

/**
 * @brief  CONTROL 推送一个电机的速度 + 里程（TIM6 中断里调，1 kHz）。
 */
void SENSOR_PushWheel(uint8_t motor, int32_t speed_d100, int32_t odometer)
{
    if (motor < SENSOR_WHEEL_COUNT)
    {
        g_sensor_data.speed_d100[motor] = speed_d100;
        g_sensor_data.odometer[motor]   = odometer;
    }
    g_sensor_data.ctrl_timestamp_ms = TIMER_GetTick();
}

/**
 * @brief  INA3221 推送四路电机电流 + 电压（TIM7 轮询里调）。
 */
void SENSOR_PushIna(int16_t *current_mA, uint16_t *bus_voltage_mV)
{
    uint8_t m;

    for (m = 0; m < SENSOR_WHEEL_COUNT; m++)
    {
        g_sensor_data.current_mA[m]     = current_mA[m];
        g_sensor_data.bus_voltage_mV[m] = bus_voltage_mV[m];
    }
    g_sensor_data.ina_timestamp_ms = TIMER_GetTick();
}

/**
 * @brief  设置命令应答（F407 执行命令后调，随 sensorhub 上报回显）。
 */
void SENSOR_SetCmdAck(uint8_t cmd_id, uint8_t status)
{
    g_sensor_data.last_cmd_id     = cmd_id;
    g_sensor_data.last_cmd_status = status;
}

/**
 * @brief  获取当前所有传感器数据的一致快照（关中断拷贝）。
 */
void SENSOR_GetHub(SensorData_t *out)
{
    uint32_t primask;

    if (out == 0)
        return;

    /* 关中断拷贝，保证字段间一致（同一时刻 T 的快照）。 */
    primask = __get_PRIMASK();
    __disable_irq();
    g_sensor_data.sys_tick_ms = TIMER_GetTick();   /* 快照时刻的总时间戳 */
    *out = g_sensor_data;
    __set_PRIMASK(primask);
}

/* ============================================================================
   启动诊断（boot banner）—— 从 main.c 迁入，保持 main 整洁
   ============================================================================ */

/* MicroLIB printf 不支持 %f，用 x1000 定点打印 float（3 位小数） */
static void dbg_f3(float v)
{
    long iv = (long)(v * 1000.0f);
    char sign = (iv < 0) ? '-' : ' ';
    if (iv < 0) iv = -iv;
    printf("%c%ld.%03ld", sign, iv / 1000, iv % 1000);
}

/* 全面的 boot 诊断信息：时钟/构建/驱动参数/传感器配置/模块状态 */
void print_boot_banner(void)
{
    RCC_ClocksTypeDef clk;
    RCC_GetClocksFreq(&clk);

    printf("\r\n");
    printf("=================================================\r\n");
    printf("  PEO Rover - Boot\r\n");
    printf("=================================================\r\n");
    printf("  Firmware : peo_rover_f407\r\n");
    printf("  Build    : %s %s\r\n", __DATE__, __TIME__);
    printf("  Target   : STM32F407VGT6 @ %lu MHz\r\n",
           (unsigned long)(clk.SYSCLK_Frequency / 1000000UL));
    printf("-------------------------------------------------\r\n");
    printf("  CLOCKS (actual)\r\n");
    printf("    SYSCLK = %lu Hz\r\n", (unsigned long)clk.SYSCLK_Frequency);
    printf("    HCLK   = %lu Hz\r\n", (unsigned long)clk.HCLK_Frequency);
    printf("    PCLK1  = %lu Hz\r\n", (unsigned long)clk.PCLK1_Frequency);
    printf("    PCLK2  = %lu Hz\r\n", (unsigned long)clk.PCLK2_Frequency);
    printf("-------------------------------------------------\r\n");
    printf("  DRIVETRAIN\r\n");
    printf("    Gear       : %u:1\r\n", (unsigned)PEO_PM_GEAR_RATIO);
    printf("    Encoder    : %u counts/output-rev\r\n", (unsigned)PEO_PM_CNT_PER_OUT_REV);
    printf("    Resolution : %u um/count\r\n", (unsigned)PEO_PM_UM_PER_CNT);
    printf("    Speed PI   : Kp=");
    dbg_f3(PEO_CTRL_KP_DEFAULT);
    printf("  Ki=");
    dbg_f3(PEO_CTRL_KI_DEFAULT);
    printf("  @ %u Hz\r\n", (unsigned)PEO_CTRL_LOOP_HZ);
    printf("-------------------------------------------------\r\n");
    printf("  SENSORS (I2C)\r\n");
    printf("    MPU6050 : I2C1 @ 0x%02X  (DMP)\r\n", (unsigned)PEO_PM_MPU6050_ADDR);
    printf("    VL53L1X : I2C3 @ 0x%02X / 0x%02X / 0x%02X\r\n",
           (unsigned)PEO_PM_VL53_FRONT_ADDR,
           (unsigned)PEO_PM_VL53_LEFT_ADDR,
           (unsigned)PEO_PM_VL53_RIGHT_ADDR);
    printf("              DM=%s  TB=%u ms  IMP=%u ms  -> ~%u Hz\r\n",
           (PEO_PM_VL53_DISTANCE_MODE == 1) ? "Short" : "Long",
           (unsigned)PEO_PM_VL53_TIMING_BUDGET_MS,
           (unsigned)PEO_PM_VL53_INTER_MEAS_MS,
           (unsigned)(1000U / PEO_PM_VL53_INTER_MEAS_MS));
    printf("    INA3221 : I2C2 @ 0x%02X + 0x%02X  (2x, 4 motor current)\r\n",
           (unsigned)PEO_PM_INA3221_ADDR_A, (unsigned)PEO_PM_INA3221_ADDR_B);
    printf("-------------------------------------------------\r\n");
    printf("  MODULES\r\n");
    printf("    ENCODER=%s  MOTOR=%s  CONTROL=%s  TOF400C=%s  INA3221=%s\r\n",
           PEO_ENABLE_ENCODER ? "ON " : "OFF",
           PEO_ENABLE_MOTOR   ? "ON " : "OFF",
           PEO_ENABLE_CONTROL ? "ON " : "OFF",
           PEO_ENABLE_TOF400C ? "ON " : "OFF",
           PEO_ENABLE_INA3221 ? "ON " : "OFF");
    printf("    MPU6050=%s   LED=ON\r\n", PEO_ENABLE_MPU6050 ? "ON " : "OFF");
    printf("=================================================\r\n");
    printf("\r\n");
}

