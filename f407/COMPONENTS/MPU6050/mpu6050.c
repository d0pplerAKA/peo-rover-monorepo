/**
 * @file  mpu6050.c
 * @brief Connect the MPU6050 DMP library to rover interrupts and attitude telemetry.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Layers:
 *   - eMPL platform functions (reg_int_cb, _MLPrintLog): required by the
 *     InvenSense Motion Driver.
 *   - MPU6050_Init / MPU6050_DMP_Init: bring up the MPU and the on-chip DMP.
 *   - Interrupt-driven FIFO read: the DMP asserts INT (PE3/EXTI3) whenever a
 *     quaternion packet is ready; the EXTI3 handler calls the FIFO-read
 *     callback, which converts the latest quaternion and acceleration
 *     and publishes them to SENSOR_HUB. Read them via SENSOR_GetHub().
 *
 * MPU6050 INT = PE3 (EXTI3), active-HIGH (default polarity, push-pull).
 * NVIC enables EXTI3 in NVIC_InitAll() (nvic.c).
 */
#include "include/mpu6050.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"

/* DMP results: updated in the EXTI3 interrupt and then published to sensor_hub. */
static volatile float   g_qw, g_qx, g_qy, g_qz;   /* Orientation quaternion (w,x,y,z), dimensionless. */
static volatile float   g_ax, g_ay, g_az;         /* Acceleration converted from raw samples to g (multiples of gravity). */

/* Cached self-test result: 0=gyro and accelerometer passed; 1=failed
   (measurements may still exist but confidence is reduced); 2=not attempted. */
static volatile uint8_t g_self_test_result = 2;  /* Cached calibration result: 0=passed, 1=failed, 2=not attempted. */
/* Initialization result: 0=success; 1=WHO_AM_I failed; 2=DMP failed; 3=not attempted. */
static volatile uint8_t g_init_status = 3;  /* Cached startup result: 0=ready, 1=identity error, 2=DMP error, 3=not attempted. */

/* Mounting orientation: identity means sensor axes match vehicle axes.
   Change the orientation matrix when the board uses a different mounting direction. */
static const signed char gyro_orientation[9] = { 1, 0, 0,
                                                 0, 1, 0,
                                                 0, 0, 1 };

/* eMPL platform adapter: register the sensor interrupt callback. */
static void (*g_mpu_int_cb)(void) = 0; /* Saved data-ready callback invoked by EXTI3; NULL until registered. */

/**
 * @brief Register the motion-driver callback and connect the MPU data-ready interrupt.
 *
 * Called by the eMPL mpu_init routine. Saves the callback, configures PE3 as an input,
 * and enables rising-edge EXTI3 detection; NVIC setup has already run.
 * @param int_param Callback descriptor; only its cb member is used.
 * @retval 0 Callback descriptor accepted and GPIO/EXTI configured.
 * @retval -1 int_param was NULL.
 */
int reg_int_cb(struct int_param_s *int_param)
{
    GPIO_InitTypeDef gpio;  /* GPIO setup record: selects pin routing, electrical drive, pulls, and switching speed. */
    EXTI_InitTypeDef exti;  /* External-interrupt setup: selected input line and triggering signal edge. */

    if (int_param == 0)
        return -1;
    g_mpu_int_cb = int_param->cb;

    /* PE3 input with pull-down: INT is idle low and rises when data is ready. */
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin   = PEO_PM_MPU_INT_PIN;
    GPIO_Init(PEO_PM_MPU_INT_PORT, &gpio);

    /* Route PE3 to EXTI3 and trigger on rising edges. */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, PEO_PM_MPU_INT_EXTI_SRC);

    exti.EXTI_Line    = PEO_PM_MPU_INT_EXTI_LINE;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    EXTI_ClearITPendingBit(PEO_PM_MPU_INT_EXTI_LINE);

    return 0;
}

/**
 * @brief Acknowledge MPU data-ready and invoke its registered FIFO-reading callback.
 *
 * The processor enters this handler when the MPU INT signal rises on PE3. If no callback
 * is registered, it only clears the pending interrupt.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(PEO_PM_MPU_INT_EXTI_LINE) != RESET)
    {
        EXTI_ClearITPendingBit(PEO_PM_MPU_INT_EXTI_LINE);

        if (g_mpu_int_cb != 0)
            g_mpu_int_cb();
    }
}

/**
 * @brief Send a formatted motion-driver diagnostic message to the USART1 console.
 *
 * Backend called by eMPL logging when logging is enabled. Formats into a bounded
 * 128-byte buffer and optionally prefixes a tag.
 * @param priority Logging priority supplied by eMPL; ignored by this backend.
 * @param tag Optional text label; NULL or empty means no prefix.
 * @param fmt Non-NULL printf-style format string.
 * @param ... Values corresponding to the format placeholders.
 * @retval 0 Logging call completed; this implementation does not report
 *           formatting/transmit errors.
 */
int _MLPrintLog(int priority, const char *tag, const char *fmt, ...)
{
    char buf[128];  /* Bounded text buffer for a formatted log message, including its terminating zero. */
    va_list args;  /* Variable-argument cursor used to format an eMPL log message. */

    (void)priority;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (tag != 0 && tag[0] != '\0')
        printf("[%s] ", tag);
    printf("%s", buf);

    return 0;
}

/* DMP FIFO-ready callback, executed inside the EXTI3 interrupt. */
/**
 * @brief Read one DMP packet and publish orientation and acceleration.
 *
 * Registered during MPU6050_DMP_Init and invoked by EXTI3. Converts Q30 quaternion
 * counts to floats and accelerometer counts to g. A FIFO-read error leaves the previous
 * hub measurement unchanged.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
static void MPU6050_FIFO_Ready_Cb(void)
{
    short gyro[3], accel[3], sensors; /* Raw FIFO gyro/accelerometer vectors and the returned sensor-presence mask. */
    long quat[4];  /* DMP quaternion components w/x/y/z in Q30 fixed point. */
    unsigned long timestamp;  /* Sample timestamp returned by the motion driver, in milliseconds. */
    unsigned char more;  /* Motion-driver indicator/count of FIFO packets remaining; this callback reads one packet. */

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) == 0)
    {
        /* Quaternion order is (w,x,y,z), encoded in Q30 fixed point (divide by 2^30). */
        g_qw = (float)quat[0] / MPU6050_Q30;
        g_qx = (float)quat[1] / MPU6050_Q30;
        g_qy = (float)quat[2] / MPU6050_Q30;
        g_qz = (float)quat[3] / MPU6050_Q30;

        /* Raw acceleration: +/-2 g range, 16384 counts per g. */
        g_ax = (float)accel[0] / MPU6050_ACCEL_SENS;
        g_ay = (float)accel[1] / MPU6050_ACCEL_SENS;
        g_az = (float)accel[2] / MPU6050_ACCEL_SENS;

        /* Publish to the sensor hub at MPU6050_SAMPLE_HZ when sampling is running. */
        SENSOR_PushMpu(g_qw, g_qx, g_qy, g_qz, g_ax, g_ay, g_az);
    }
}

/* Public application API. */

/**
 * @brief Initialize the attitude sensor and cache its startup result.
 *
 * Called by main. Prepares I2C1, verifies WHO_AM_I, and configures DMP; GAMEPAD starts
 * actual DMP output later.
 * @note No arguments are required.
 * @retval 0 Sensor initialization succeeded.
 * @retval 1 WHO_AM_I read or identity check failed.
 * @retval 2 DMP setup failed.
 */
uint8_t MPU6050_Init(void)
{
    uint8_t who = 0;  /* WHO_AM_I register value used to verify the connected sensor identity. */
    uint8_t dmp_status;  /* Numbered DMP initialization result; zero indicates setup success. */
    uint32_t ret;  /* I2C transaction result; zero is NO_ERROR. */

    I2C_MPU_Init();   /* I2C1 @ 400 kHz */
    ret = I2C_MPU_ReadBytes(PEO_PM_MPU6050_ADDR, MPU6050_WHO_AM_I_REG, 1, &who);
    if (ret != NO_ERROR)
    {
        printf("[MPU6050] WHO_AM_I read failed, err=%lu\r\n", (unsigned long)ret);
        g_init_status = 1;
        return 1;
    }
    if (who != MPU6050_WHO_AM_I_EXP)
    {
        printf("[MPU6050] WHO_AM_I mismatch: got=0x%02X, expected=0x%02X\r\n",
               (unsigned)who, (unsigned)MPU6050_WHO_AM_I_EXP);
        g_init_status = 1;
        return 1;
    }

    dmp_status = MPU6050_DMP_Init();
    if (dmp_status != 0)
    {
        printf("[MPU6050] DMP init failed, step=%u\r\n", (unsigned)dmp_status);
        g_init_status = 2;
        return 2;
    }

    g_init_status = 0;   /* Success stays silent; the boot banner reports the result centrally. */
    return 0;
}

/**
 * @brief Load and configure the sensor processing engine without starting its output.
 *
 * Called by MPU6050_Init. Sets sensor/FIFO rates, mounting orientation, and
 * quaternion/acceleration features; self-test outcome is cached separately and does not
 * change this setup return code.
 * @note No arguments are required.
 * @retval 0 Setup succeeded; self-test result must be queried separately.
 * @retval 1 mpu_init failed.
 * @retval 2 Enabling gyro/accelerometer failed.
 * @retval 3 FIFO configuration failed.
 * @retval 4 Setting the sample rate failed.
 * @retval 5 Loading DMP firmware failed.
 * @retval 6 Applying mounting orientation failed.
 * @retval 7 Enabling DMP features failed.
 * @retval 8 Setting DMP FIFO rate failed.
 */
uint8_t MPU6050_DMP_Init(void)
{
    struct int_param_s int_param;

    /* Set the callback before mpu_init(): it calls reg_int_cb() to save it.
       EXTI3 later invokes g_mpu_int_cb to read the FIFO. */
    int_param.cb = MPU6050_FIFO_Ready_Cb;

    if (mpu_init(&int_param))                              return 1;
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))     return 2;
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))  return 3;
    if (mpu_set_sample_rate(MPU6050_SAMPLE_HZ))            return 4;
    if (dmp_load_motion_driver_firmware())                 return 5;
    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
                                                           return 6;
    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL))
                                                           return 7;
    if (dmp_set_fifo_rate(MPU6050_SAMPLE_HZ))              return 8;

    g_self_test_result = run_self_test();   /* Save the self-test result for the boot banner. */

    /* Do not start the DMP engine here. MPU6050_Start() enables data output
       and EXTI3 interrupts later, keeping sensor interrupts out of startup setup. */
    return 0;
}

/**
 * @brief Enable DMP output so attitude samples can arrive by interrupt.
 *
 * Called by the gamepad A toggle after successful setup; data-ready pulses then occur at
 * MPU6050_SAMPLE_HZ.
 * @note No arguments are required.
 * @retval 0 DMP start request succeeded.
 * @retval 1 mpu_set_dmp_state(1) failed.
 * @note Call only after MPU6050_DMP_Init() succeeds. INT then rises at
 *       MPU6050_SAMPLE_HZ, triggering the data-ready interrupt.
 */
uint8_t MPU6050_Start(void)
{
    if (mpu_set_dmp_state(1))
        return 1;
    return 0;
}

/**
 * @brief Disable DMP output when attitude sampling is no longer requested.
 *
 * Called by the gamepad A toggle. Previously published hub data remains available with
 * its old timestamp.
 * @note No arguments are required.
 * @retval 0 DMP stop request succeeded.
 * @retval 1 mpu_set_dmp_state(0) failed.
 */
uint8_t MPU6050_Stop(void)
{
    if (mpu_set_dmp_state(0))
        return 1;
    return 0;
}

/**
 * @brief Check gyro/accelerometer operation and apply measured biases to DMP.
 *
 * Called during DMP initialization. Only the gyro and accelerometer pass bits matter
 * because this board has no compass.
 * @note No arguments are required.
 * @retval 0 Tests passed and both bias writes succeeded.
 * @retval 1 A self-test or bias-write step failed.
 */
uint8_t run_self_test(void)
{
    int res;  /* Self-test pass-bit mask: gyro bit 0 and accelerometer bit 1 are checked. */
    long gyro[3], accel[3]; /* Three-axis self-test bias estimates, then rescaled for DMP calibration. */
    float gyro_sens;  /* Gyroscope raw counts per degree/second, used to scale calibration bias. */
    unsigned short accel_sens;  /* Accelerometer raw counts per g, used to scale calibration bias. */

    res = mpu_run_self_test(gyro, accel);
    /* Self-test bits: bit0=gyro, bit1=accelerometer, bit2=compass. MPU6050
       has no magnetometer (AK89xx_SECONDARY is undefined), so the library sets
       bit2 unconditionally. The complete result is therefore 0x07, not 0x03.
       Mask only bits 0 and 1; ignore the compass bit. */
    if ((res & 0x03) != 0x03)
        return 1;

    /* Convert self-test biases using sensor sensitivity into the format DMP expects, then write them. */
    mpu_get_gyro_sens(&gyro_sens);
    gyro[0] = (long)(gyro[0] * gyro_sens);
    gyro[1] = (long)(gyro[1] * gyro_sens);
    gyro[2] = (long)(gyro[2] * gyro_sens);
    if (dmp_set_gyro_bias(gyro))
        return 1;   /* Failed to write a bias into DMP. */

    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    if (dmp_set_accel_bias(accel))
        return 1;   /* Failed to write a bias into DMP. */

    return 0;
}

/**
 * @brief Read the stored test/calibration result for startup diagnostics.
 *
 * Called by print_boot_banner; no new test is performed.
 * @note No arguments are required.
 * @retval 0 Gyro and accelerometer passed and biases were written.
 * @retval 1 Test or bias write failed.
 * @retval 2 Test has not run.
 */
uint8_t MPU6050_GetSelfTest(void)
{
    return g_self_test_result;
}

/**
 * @brief Read the stored attitude-sensor startup result.
 *
 * Called by print_boot_banner; no I2C transaction is performed.
 * @note No arguments are required.
 * @retval 0 Initialization succeeded.
 * @retval 1 Identity check failed.
 * @retval 2 DMP setup failed.
 * @retval 3 Initialization has not run.
 */
uint8_t MPU6050_GetInitStatus(void)
{
    return g_init_status;
}
