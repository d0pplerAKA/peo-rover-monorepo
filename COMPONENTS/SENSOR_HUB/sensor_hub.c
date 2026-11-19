/**
 * @file  sensor_hub.c
 * @brief Store sensor updates and supply timestamped telemetry snapshots.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#include "include/sensor_hub.h"
#include "COMPONENTS/TIMER/include/timer.h"
#include "COMPONENTS/PEO_PINMAP.H"                              /* Boot banner: driver settings and sensor address macros. */
#include "COMPONENTS/CONTROL/include/control.h"      /* boot banner: PEO_CTRL_KP/KI/LOOP_HZ */
#include "COMPONENTS/GAMEPAD/include/gamepad.h"     /* Include ownership state in SENSOR_GetHub() reports. */
#include "COMPONENTS/TOF400C/include/tof400c.h"      /* Boot banner: range settings such as PEO_PM_VL53_DISTANCE_MODE. */
#include "COMPONENTS/MPU6050/include/mpu6050.h"      /* boot banner: MPU6050_GetSelfTest/GetInitStatus */
#include "COMPONENTS/INA3221/include/ina3221.h"      /* boot banner: INA3221_GetInitStatus */
#include <stdio.h>                                   /* boot banner / dbg_f3: printf */

/* Live packet: sensor interrupt handlers update it through the Push functions. */
static volatile SensorData_t g_sensor_data;  /* Latest sensor readings shared by interrupt writers and snapshot readers. */

/**
 * @brief Store the latest attitude and acceleration as one sensor group.
 *
 * Called by the MPU FIFO callback in EXTI3; adds the current millisecond timestamp.
 * @param qw Quaternion scalar component w, dimensionless.
 * @param qx Quaternion x component, dimensionless.
 * @param qy Quaternion y component, dimensionless.
 * @param qz Quaternion z component, dimensionless.
 * @param ax X-axis acceleration in g (1 g is standard gravity).
 * @param ay Y-axis acceleration in g.
 * @param az Z-axis acceleration in g.
 * @retval None This function does not return a value.
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
 * @brief Store one range sensor result and refresh the distance-group timestamp.
 *
 * Called by EXTI0/1/2 after a successful sensor read. The three distances share a
 * timestamp recording the most recent update to any one of them.
 * @param which SENSOR_DIST_FRONT, SENSOR_DIST_LEFT, or SENSOR_DIST_RIGHT.
 * @param mm Measured distance in millimeters; interpret it together with status.
 * @param status VL53L1X range-status code; zero means a valid measurement.
 * @retval None This function does not return a value.
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
 * @brief Store one measured wheel speed and signed accumulated count.
 *
 * Called for each wheel by the TIM6 control interrupt; updates the control-group
 * timestamp.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @param speed_d100 Measured signed speed in 0.01 cm/s, positive for forward.
 * @param odometer Accumulated signed encoder counts; 1980 counts per wheel revolution.
 * @retval None This function does not return a value.
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
 * @brief Copy all four motor-current and bus-voltage readings into telemetry.
 *
 * Called by TIM7 only after both INA3221 reads succeed; records the update time
 * automatically.
 * @param current_mA Readable array of four signed currents in milliamps, in M1..M4
 *                   order.
 * @param bus_voltage_mV Readable array of four unsigned bus voltages in millivolts, in
 *                       M1..M4 order.
 * @retval None This function does not return a value.
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
 * @brief Record which command was handled and its result for the host.
 *
 * Called by command handling and queue-error paths. A critical section keeps the command
 * ID and result together; only the latest acknowledgement is retained.
 * @param cmd_id Command identifier to echo, from ESP_CMD_*.
 * @param status ESP_CMD_OK or an ESP_CMD_ERR_* result code.
 * @retval None This function does not return a value.
 */
void SENSOR_SetCmdAck(uint8_t cmd_id, uint8_t status)
{
    /* Saved interrupt-mask state; restore it so callers that already masked interrupts stay protected. */
    uint32_t primask = __disable_irq();

    g_sensor_data.last_cmd_id     = cmd_id;
    g_sensor_data.last_cmd_status = status;

    if (!primask)
        __enable_irq();
}

/**
 * @brief Copy the complete current telemetry state into a caller-owned object.
 *
 * Called by ESP_LINK reporting and available to main. Briefly masks interrupts, adds
 * snapshot time/mode/version/ownership, copies the packet, and restores the prior
 * interrupt mask.
 * @param out Writable SensorData_t destination; NULL is accepted and performs no copy.
 * @retval None This function does not return a value.
 */
void SENSOR_GetHub(SensorData_t *out)
{
    uint32_t primask;  /* Saved interrupt-mask state; restore it so callers that already masked interrupts stay protected. */

    if (out == 0)
        return;

    /* Mask interrupts so writers cannot change fields halfway through copying
       the state. Each sensor group retains its own acquisition timestamp. */
    primask = __get_PRIMASK();
    __disable_irq();
    g_sensor_data.sys_tick_ms   = TIMER_GetTick();    /* Timestamp of this snapshot in SysTick milliseconds. */
    g_sensor_data.control_mode  = CONTROL_GetMode();  /* Current drive mode: open-loop PWM or closed-loop speed control. */
    g_sensor_data.fw_version    = PEO_FW_VERSION;     /* F407 firmware version number. */
    g_sensor_data.lock          = GAMEPAD_GetLock();  /* Driving ownership: 0=manual/controller; 1=autonomous/host. */
    *out = g_sensor_data;
    __set_PRIMASK(primask);
}

/* Startup diagnostics (boot banner), moved from main.c to keep main focused on scheduling. */

/* MicroLIB printf does not support %f here; print floats as integers
   scaled by 1000 to show three decimal places. */
/**
 * @brief Print a float with three decimal places without using printf %f.
 *
 * Internal helper used by print_boot_banner for PI gains; multiplies by 1000 and
 * truncates to an integer.
 * @param v Finite value to display, within the range of the scaled long conversion.
 * @retval None This function does not return a value.
 */
static void dbg_f3(float v)
{
    long iv = (long)(v * 1000.0f);  /* Value scaled by 1000 and truncated, allowing three decimal digits without %f. */
    char sign = (iv < 0) ? '-' : ' ';  /* Minus sign for negative values, or a leading space for nonnegative values. */
    if (iv < 0) iv = -iv;
    printf("%c%ld.%03ld", sign, iv / 1000, iv % 1000);
}

/* Initialization labels: 0 -> PASS; 3 (not attempted) -> N/A; otherwise FAIL. */
/**
 * @brief Translate a cached sensor initialization code into a short console label.
 *
 * Internal helper used by print_boot_banner.
 * @param s Cached status: 0=success, 3=not attempted, any other value=failure.
 * @retval value Pointer to static text PASS, N/A, or FAIL; caller must not modify or
 *               free it.
 */
static const char *init_status_str(uint8_t s)
{
    if (s == 0) return "PASS";
    if (s == 3) return "N/A";
    return "FAIL";
}

/* Comprehensive boot diagnostics: clocks, build, drive settings, sensor configuration, and enabled modules. */
/**
 * @brief Print a startup summary so teammates can inspect the rover configuration.
 *
 * Called once by main after component setup. Shows actual clocks, chip/build identity,
 * drivetrain scaling, PI defaults, sensor setup/test results, and enabled modules on
 * USART1.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void print_boot_banner(void)
{
    RCC_ClocksTypeDef clk;  /* Actual system and bus frequencies in Hz, read from clock configuration registers. */
    RCC_GetClocksFreq(&clk);

    printf("\r\n");
    printf("=================================================\r\n");
    printf("  PEO Rover - Boot\r\n");
    printf("=================================================\r\n");
    printf("  Firmware : peo_rover_f407\r\n");
    printf("  Build    : %s %s\r\n", __DATE__, __TIME__);
    printf("  Target   : STM32F407VGT6 @ %lu MHz\r\n",
           (unsigned long)(clk.SYSCLK_Frequency / 1000000UL));
    {
        uint32_t rev_id = DBGMCU_GetREVID();  /* Chip silicon revision identifier for startup diagnostics. */
        uint32_t dev_id = DBGMCU_GetDEVID();  /* Chip device-family identifier for startup diagnostics. */
        uint32_t uid0   = *(volatile uint32_t *)0x1FFF7A10U;  /* UID[31:0]  */
        uint32_t uid1   = *(volatile uint32_t *)0x1FFF7A14U;  /* UID[63:32] */
        uint32_t uid2   = *(volatile uint32_t *)0x1FFF7A18U;  /* UID[95:64] */
        printf("    Revision ID : 0x%04lX\r\n", (unsigned long)rev_id);
        printf("    Device ID   : 0x%03lX\r\n", (unsigned long)dev_id);
        printf("    UID         : %08lX-%08lX-%08lX\r\n",
               (unsigned long)uid2, (unsigned long)uid1, (unsigned long)uid0);
    }
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
    {
        const char *st;  /* Static text describing the cached MPU self-test result. */
        uint8_t     s = MPU6050_GetSelfTest();  /* Cached MPU self-test result; this read does not rerun the test. */
        if (s == 0)      st = "PASS";
        else if (s == 1) st = "FAIL (low confidence)";
        else             st = "N/A";
        printf("    MPU6050 : I2C1 @ 0x%02X  (DMP)  init=%s  self-test=%s\r\n",
               (unsigned)PEO_PM_MPU6050_ADDR,
               init_status_str(MPU6050_GetInitStatus()), st);
    }
    printf("    VL53L1X : I2C3 @ 0x%02X/0x%02X/0x%02X  init=%s/%s/%s\r\n",
           (unsigned)PEO_PM_VL53_FRONT_ADDR,
           (unsigned)PEO_PM_VL53_LEFT_ADDR,
           (unsigned)PEO_PM_VL53_RIGHT_ADDR,
           init_status_str(TOF400C_GetInitStatus(TOF400C_FRONT)),
           init_status_str(TOF400C_GetInitStatus(TOF400C_LEFT)),
           init_status_str(TOF400C_GetInitStatus(TOF400C_RIGHT)));
    printf("              DM=%s  TB=%u ms  IMP=%u ms  -> ~%u Hz\r\n",
           (PEO_PM_VL53_DISTANCE_MODE == 1) ? "Short" : "Long",
           (unsigned)PEO_PM_VL53_TIMING_BUDGET_MS,
           (unsigned)PEO_PM_VL53_INTER_MEAS_MS,
           (unsigned)(1000U / PEO_PM_VL53_INTER_MEAS_MS));
    printf("    INA3221 : I2C2 @ 0x%02X + 0x%02X  init=%s/%s  (2x, 4 motor current)\r\n",
           (unsigned)PEO_PM_INA3221_ADDR_A, (unsigned)PEO_PM_INA3221_ADDR_B,
           init_status_str(INA3221_GetInitStatus(0)),
           init_status_str(INA3221_GetInitStatus(1)));
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

