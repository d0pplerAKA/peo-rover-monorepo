/**
 * @file    ina3221.c
 * @brief   双 INA3221 驱动：4 路电机电流监测（I2C2 专用总线）。
 *
 *   两片 INA3221 挂在同一条 I2C2（PF1=SCL/PF0=SDA）：
 *     片 A (A0=GND, 0x40): CH1=M1, CH2=M2, CH3=留空
 *     片 B (A0=VS,  0x41): CH1=M3, CH2=M4, CH3=留空
 *   模块内置 R100=0.1Ω 分流电阻；分流电压 LSB=40uV，总线电压 LSB=8mV。
 *   分流电压为 16 位有符号，故电流双向（正=正常耗电，负=再生/反向）。
 */
#include "include/ina3221.h"

#include "COMPONENTS/I2C/include/i2c.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"
#include <stdio.h>

/* INA3221 寄存器地址 */
#define INA3221_REG_CONFIG        0x00
#define INA3221_REG_SHUNT_CH1     0x01
#define INA3221_REG_BUS_CH1       0x02
#define INA3221_REG_SHUNT_CH2     0x03
#define INA3221_REG_BUS_CH2       0x04
#define INA3221_REG_SHUNT_CH3     0x05
#define INA3221_REG_BUS_CH3       0x06
#define INA3221_REG_MANUFACT_ID   0xFE
#define INA3221_REG_DIE_ID        0xFF

/* 芯片身份 */
#define INA3221_MANUFACT_ID       0x5449
#define INA3221_DIE_ID            0x3220

/* 换算系数 */
#define INA3221_SHUNT_LSB_UV      40U     /* 分流电压 LSB = 40 uV  */
#define INA3221_BUS_LSB_MV        8U      /* 总线电压 LSB = 8 mV   */

/* 配置寄存器：MODE[5:3]=111 连续测分流+总线；AVG[14:12]=000 1次平均 */
#define INA3221_CONFIG_CONTINUOUS (0x0038U)


/* 每路电机的 (芯片地址, 分流寄存器, 总线寄存器) 映射 */
typedef struct
{
    uint8_t addr;        /* 7-bit I2C 地址 */
    uint8_t shunt_reg;   /* 分流电压寄存器 */
    uint8_t bus_reg;     /* 总线电压寄存器 */
} INA3221_ChMap_t;

static const INA3221_ChMap_t g_ina_map[4] =
{
    { PEO_PM_INA3221_ADDR_A, INA3221_REG_SHUNT_CH1, INA3221_REG_BUS_CH1 },   /* M1: 片A CH1 */
    { PEO_PM_INA3221_ADDR_A, INA3221_REG_SHUNT_CH2, INA3221_REG_BUS_CH2 },   /* M2: 片A CH2 */
    { PEO_PM_INA3221_ADDR_B, INA3221_REG_SHUNT_CH1, INA3221_REG_BUS_CH1 },   /* M3: 片B CH1 */
    { PEO_PM_INA3221_ADDR_B, INA3221_REG_SHUNT_CH2, INA3221_REG_BUS_CH2 },   /* M4: 片B CH2 */
};


/** 单芯片配置：写连续测量模式（分流 + 总线电压） */
static void INA3221_Config(uint8_t addr)
{
    uint8_t cfg[2] = { (uint8_t)(INA3221_CONFIG_CONTINUOUS >> 8),
                       (uint8_t)(INA3221_CONFIG_CONTINUOUS & 0xFF) };
    I2C_INA_WriteBytes(addr, INA3221_REG_CONFIG, cfg, 2);
}


/** 读制造商 ID，验证芯片在线并打印 */
static void INA3221_CheckId(uint8_t addr)
{
    uint8_t id[2] = {0, 0};
    uint16_t manuf;

    I2C_INA_ReadBytes(addr, INA3221_REG_MANUFACT_ID, id, 2);
    manuf = ((uint16_t)id[0] << 8) | id[1];

    if (manuf == INA3221_MANUFACT_ID)
        printf("[INA3221 @0x%02X] found (ID=0x%04X)\r\n", (unsigned)addr, (unsigned)manuf);
    else
        printf("[INA3221 @0x%02X] NOT found (ID=0x%04X)\r\n", (unsigned)addr, (unsigned)manuf);
}


/**
 * @brief  初始化两片 INA3221：隐式初始化 I2C2，配置连续测量，读两片 ID 验证。
 */
void INA3221_Init(void)
{
    I2C_INA_Init();   /* I2C2 @ 400 kHz, PF1=SCL/PF0=SDA */

    INA3221_Config(PEO_PM_INA3221_ADDR_A);
    INA3221_CheckId(PEO_PM_INA3221_ADDR_A);

    INA3221_Config(PEO_PM_INA3221_ADDR_B);
    INA3221_CheckId(PEO_PM_INA3221_ADDR_B);
}


/**
 * @brief  读某电机通道的分流电压（mV，有符号）。
 */
float INA3221_GetShuntVoltage_mV(uint8_t motor)
{
    const INA3221_ChMap_t *m = &g_ina_map[motor & 0x03];
    uint8_t buf[2] = {0, 0};
    int16_t raw;

    I2C_INA_ReadBytes(m->addr, m->shunt_reg, buf, 2);
    raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

    return (float)raw * INA3221_SHUNT_LSB_UV / 1000.0f;   /* uV -> mV */
}


/**
 * @brief  读某电机通道的总线电压（V）。低 3 位是标志位，需右移 3。
 */
float INA3221_GetBusVoltage_V(uint8_t motor)
{
    const INA3221_ChMap_t *m = &g_ina_map[motor & 0x03];
    uint8_t buf[2] = {0, 0};
    uint16_t raw, bus;

    I2C_INA_ReadBytes(m->addr, m->bus_reg, buf, 2);
    raw = ((uint16_t)buf[0] << 8) | buf[1];
    bus = raw >> 3;                                      /* 去掉低 3 位标志 */

    return (float)bus * INA3221_BUS_LSB_MV / 1000.0f;    /* mV -> V */
}


/**
 * @brief  读某电机通道的电流（mA）。I = Vshunt / Rshunt；mV / mΩ = A。
 */
float INA3221_GetCurrent_mA(uint8_t motor)
{
    float shunt_mV = INA3221_GetShuntVoltage_mV(motor);

    return shunt_mV / (float)PEO_PM_INA3221_RSHUNT_MOHM * 1000.0f;
}

/* ============================================================================
   TIM7 轮询：按 SENSOR_RATE_INA_HZ 频率读 4 路电流/电压，推送 sensorhub
   ----------------------------------------------------------------------------
   TIM7 是 APB1 基本定时器（16-bit）。84 MHz 先预分频到 10 kHz（PSC=8399），
   再分频到 SENSOR_RATE_INA_HZ（ARR = 10000/RATE - 1）。16 位放不下直接
   分频 84MHz->10Hz，所以走两级。改速率只需改 SENSOR_RATE_INA_HZ。
   ============================================================================ */
#define INA3221_TIM      TIM7
#define INA3221_TIM_PSC  (PEO_PM_TIM_APB1_HZ / 10000UL - 1)   /* 84 MHz -> 10 kHz */
#define INA3221_TIM_ARR  (10000UL / SENSOR_RATE_INA_HZ - 1)   /* 10 kHz -> RATE Hz */

/**
 * @brief  配置 TIM7 并按 SENSOR_RATE_INA_HZ 启动轮询。
 * @note   时钟（RCC）和 NVIC 通道由 RCC_Clock_Init()/NVIC_InitAll() 负责，
 *         这里只做 peripheral-side 配置（TIM_TimeBaseInit + TIM_ITConfig），
 *         与 CONTROL 组件用 TIM6 的方式一致。
 */
void INA3221_PollInit(void)
{
    TIM_TimeBaseInitTypeDef tb;

    tb.TIM_Prescaler         = INA3221_TIM_PSC;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = INA3221_TIM_ARR;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(INA3221_TIM, &tb);

    TIM_ClearITPendingBit(INA3221_TIM, TIM_IT_Update);
    TIM_ITConfig(INA3221_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(INA3221_TIM, ENABLE);
}

/**
 * @brief  TIM7 update interrupt = INA3221 轮询（SENSOR_RATE_INA_HZ）。
 * @note   覆盖 startup_stm32f40xx.s 的 [WEAK] TIM7_IRQHandler。
 *         读 4 路电流/电压（8 次 I2C，约 0.8ms @ 400kHz），转定点后
 *         推送给 sensor hub。运行在最低抢占优先级，可被传感器 EXTI 抢占。
 */
void TIM7_IRQHandler(void)
{
    int16_t  cur[4];
    uint16_t vol[4];
    uint8_t  m;

    if (TIM_GetITStatus(INA3221_TIM, TIM_IT_Update) == RESET)
        return;

    TIM_ClearITPendingBit(INA3221_TIM, TIM_IT_Update);

    for (m = 0; m < SENSOR_WHEEL_COUNT; m++)
    {
        cur[m] = (int16_t)INA3221_GetCurrent_mA(m);              /* mA 定点 */
        vol[m] = (uint16_t)(INA3221_GetBusVoltage_V(m) * 1000.0f); /* V -> mV */
    }

    SENSOR_PushIna(cur, vol);
}
