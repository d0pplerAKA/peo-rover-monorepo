/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   PEO Rover - MPU6050 DMP driver (eMPL Motion Driver glue)
 *
 * Layers:
 *   - eMPL platform functions (reg_int_cb, _MLPrintLog): required by the
 *     InvenSense Motion Driver.
 *   - MPU6050_Init / MPU6050_DMP_Init: bring up the MPU and the on-chip DMP.
 *   - Interrupt-driven FIFO read: the DMP asserts INT (PE3/EXTI3) whenever a
 *     quaternion packet is ready; the EXTI3 handler calls the FIFO-read
 *     callback, which stores the latest quaternion + accel and raises
 *     g_data_ready. The app reads them via MPU6050_GetQuaternion/GetAccel.
 *
 * MPU6050 INT = PE3 (EXTI3), active-HIGH (default polarity, push-pull).
 * NVIC enables EXTI3 in NVIC_InitAll() (nvic.c).
 ******************************************************************************
 */
#include "include/mpu6050.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"

/* ---- DMP 输出（EXTI3 中断上下文写入，应用层读取） ---- */
static volatile float   g_qw, g_qx, g_qy, g_qz;   /* 姿态四元数 (w,x,y,z) */
static volatile float   g_ax, g_ay, g_az;         /* 原始加速度（单位 g）*/
static volatile uint8_t g_data_ready = 0;

/* MPU 安装矩阵：单位阵 = 芯片轴与车体轴对齐；若斜装需改成旋转矩阵。 */
static const signed char gyro_orientation[9] = { 1, 0, 0,
                                                 0, 1, 0,
                                                 0, 0, 1 };

/* ---- eMPL 平台函数：INT 回调注册 ---- */
static void (*g_mpu_int_cb)(void) = 0;

/**
 * @brief  eMPL platform function: register the MPU6050 INT callback and
 *         configure PE3 as an EXTI3 input (rising edge, active-high default).
 * @param  int_param  Callback descriptor (only ->cb is used).
 * @return 0 on success, -1 if int_param is NULL.
 */
int reg_int_cb(struct int_param_s *int_param)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;

    if (int_param == 0)
        return -1;
    g_mpu_int_cb = int_param->cb;

    /* PE3 = 输入 + 下拉：INT 空闲为低，数据就绪时拉高。 */
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin   = PEO_PM_MPU_INT_PIN;
    GPIO_Init(PEO_PM_MPU_INT_PORT, &gpio);

    /* EXTI3 -> PE3，上升沿中断。 */
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
 * @brief  EXTI3 handler: MPU6050 data-ready. Clears the pending bit and
 *         invokes the callback registered by reg_int_cb().
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
 * @brief  eMPL Motion Driver logging backend (declared in log.h).
 *         Formats the message and prints it to the debug console (USART1).
 *         Only linked when REMOVE_LOGGING is OFF.
 */
int _MLPrintLog(int priority, const char *tag, const char *fmt, ...)
{
    char buf[128];
    va_list args;

    (void)priority;

    va_start(args, fmt);
    vsprintf(buf, fmt, args);   /* MPL 日志都短（<128 字节） */
    va_end(args);

    if (tag != 0 && tag[0] != '\0')
        printf("[%s] ", tag);
    printf("%s", buf);

    return 0;
}

/* ---- DMP FIFO 就绪回调（在 EXTI3 中断上下文运行） ---- */
static void MPU6050_FIFO_Ready_Cb(void)
{
    short gyro[3], accel[3], sensors;
    long quat[4];
    unsigned long timestamp;
    unsigned char more;

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) == 0)
    {
        /* quat[4] 顺序 = (w, x, y, z)，q30 定点。 */
        g_qw = (float)quat[0] / MPU6050_Q30;
        g_qx = (float)quat[1] / MPU6050_Q30;
        g_qy = (float)quat[2] / MPU6050_Q30;
        g_qz = (float)quat[3] / MPU6050_Q30;

        /* 原始加速度（±2g，16384 LSB/g）。 */
        g_ax = (float)accel[0] / MPU6050_ACCEL_SENS;
        g_ay = (float)accel[1] / MPU6050_ACCEL_SENS;
        g_az = (float)accel[2] / MPU6050_ACCEL_SENS;

        g_data_ready = 1;

        /* 推送到 sensor hub（按 MPU6050_SAMPLE_HZ 频率自动更新） */
        SENSOR_PushMpu(g_qw, g_qx, g_qy, g_qz, g_ax, g_ay, g_az);
    }
}

/* ---- 公开 API ---- */

/**
 * @brief  MPU6050 上电：初始化 I2C1，校验 WHO_AM_I，再初始化 DMP。
 */
void MPU6050_Init(void)
{
    uint8_t who = 0, ret = 0;

    I2C_MPU_Init();   /* I2C1 @ 400 kHz */

    ret = I2C_MPU_ReadBytes(PEO_PM_MPU6050_ADDR, MPU6050_WHO_AM_I_REG, 1, &who);
    if (ret == NO_ERROR)
        printf("[MPU6050] WHO_AM_I = 0x%02X\n", who);
    else
        printf("[MPU6050] WHO_AM_I read failed, err=%d\n", ret);

    if (MPU6050_DMP_Init() == 0)
        printf("[MPU6050] DMP configured (call MPU6050_Start() to begin)\n");
    else
        printf("[MPU6050] DMP init FAILED\n");
}

/**
 * @brief  初始化 MPU + 加载并启用 DMP。
 * @return 0 = 成功；非 0 = 对应步骤失败（返回值即失败步骤序号）。
 */
uint8_t MPU6050_DMP_Init(void)
{
    struct int_param_s int_param;

    /* 必须在 mpu_init() 之前设置：mpu_init() -> reg_int_cb() 会把它存下来，
       EXTI3 中断触发时由 g_mpu_int_cb 调用它读取 FIFO。 */
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

    /* 注意：DMP 引擎不在这里启动——调用 MPU6050_Start() 才会开始产出数据、
       触发 EXTI3 中断，这样上电配置阶段不会被中断打扰。 */
    return 0;
}

/**
 * @brief  启动 DMP 引擎：开始产出数据并触发 EXTI3 中断。
 * @return 0 = 成功；1 = mpu_set_dmp_state(1) 失败。
 * @note   必须在 MPU6050_DMP_Init() 成功后再调用。调用后 INT 脚按
 *         MPU6050_SAMPLE_HZ 频率周期性拉高，EXTI3 中断开始触发。
 */
uint8_t MPU6050_Start(void)
{
    if (mpu_set_dmp_state(1))
        return 1;
    return 0;
}

/**
 * @brief  自上次 MPU6050_GetQuaternion 之后是否有新数据。
 */
uint8_t MPU6050_DataReady(void)
{
    return g_data_ready;
}

/**
 * @brief  读取最新四元数（w, x, y, z），并清掉 data-ready 标志。
 */
void MPU6050_GetQuaternion(float *qw, float *qx, float *qy, float *qz)
{
    *qw = g_qw;
    *qx = g_qx;
    *qy = g_qy;
    *qz = g_qz;
    g_data_ready = 0;
}

/**
 * @brief  读取最新原始加速度（单位 g）。
 */
void MPU6050_GetAccel(float *ax, float *ay, float *az)
{
    *ax = g_ax;
    *ay = g_ay;
    *az = g_az;
}

/**
 * @brief  MPU6050 自检：跑 gyro + accel 自检，通过后把测得的偏置写入 DMP。
 * @return 0 = 通过并写入偏置；1 = 自检未通过。
 */
uint8_t run_self_test(void)
{
    int res;
    long gyro[3], accel[3];
    float gyro_sens;
    unsigned short accel_sens;

    res = mpu_run_self_test(gyro, accel);
    if (res != 0x3)         /* 0x3 = gyro(bit0) + accel(bit1) 都通过 */
        return 1;

    /* 把自检得到的偏置按灵敏度换算成 DMP 需要的格式，再写入 DMP。 */
    mpu_get_gyro_sens(&gyro_sens);
    gyro[0] = (long)(gyro[0] * gyro_sens);
    gyro[1] = (long)(gyro[1] * gyro_sens);
    gyro[2] = (long)(gyro[2] * gyro_sens);
    dmp_set_gyro_bias(gyro);

    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);

    return 0;
}
