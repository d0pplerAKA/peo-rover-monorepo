/**
 * @file    ina3221.c
 * @brief   双 INA3221 驱动：4 路电机电流监测（I2C2 专用总线）。
 *
 *   两片 INA3221 挂在同一条 I2C2（PB10=SCL/PB11=SDA）：
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

/* 配置寄存器 0x00：RST(bit15)=0；CH1en(bit14)=1、CH2en(bit13)=1、CH3en(bit12)=0；
   AVG[11:9]=000 1次平均；VBUSCT[8:6]=000 140us；VSHCT[5:3]=000 140us；
   MODE[2:0]=111 连续测分流+总线，仅使能 CH1/CH2。 */
#define INA3221_CONFIG_CONTINUOUS (0x6007U)



/** 单芯片配置：写连续测量模式（分流 + 总线电压） */
static void INA3221_Config(uint8_t addr)
{
    uint8_t cfg[2] = { (uint8_t)(INA3221_CONFIG_CONTINUOUS >> 8),
                       (uint8_t)(INA3221_CONFIG_CONTINUOUS & 0xFF) };
    I2C_INA_WriteBytes(addr, INA3221_REG_CONFIG, cfg, 2);
}


/* 初始化状态：ADDR_A/ADDR_B，0=found，1=NOT found，3=未运行。 */
static uint8_t g_init_status[2] = {3, 3};

/** 读制造商 ID 验证芯片在线；成功静默（boot banner 统一显示），失败打印。 */
static void INA3221_CheckId(uint8_t addr, uint8_t idx)
{
    uint8_t id[2] = {0, 0};
    uint16_t manuf;

    I2C_INA_ReadBytes(addr, INA3221_REG_MANUFACT_ID, id, 2);
    manuf = ((uint16_t)id[0] << 8) | id[1];

    if (manuf == INA3221_MANUFACT_ID)
        g_init_status[idx] = 0;   /* found，成功静默 */
    else
    {
        g_init_status[idx] = 1;
        printf("[INA3221 @0x%02X] NOT found (ID=0x%04X)\r\n", (unsigned)addr, (unsigned)manuf);
    }
}


/**
 * @brief  初始化两片 INA3221：隐式初始化 I2C2，配置连续测量，读两片 ID 验证。
 */
void INA3221_Init(void)
{
    I2C_INA_Init();   /* I2C2 @ 400 kHz, PB10=SCL/PB11=SDA */

    INA3221_Config(PEO_PM_INA3221_ADDR_A);
    INA3221_CheckId(PEO_PM_INA3221_ADDR_A, 0);

    INA3221_Config(PEO_PM_INA3221_ADDR_B);
    INA3221_CheckId(PEO_PM_INA3221_ADDR_B, 1);
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
/** 读取某片初始化状态：0=found，1=NOT found，3=未运行。 */
uint8_t INA3221_GetInitStatus(uint8_t idx)
{
    if (idx >= 2)
        return 3;
    return g_init_status[idx];
}

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
 *         2 次突发读（每片 8 字节）读 4 路电流/电压，转定点后推送给
 *         sensor hub。运行在最低抢占优先级，可被传感器 EXTI 抢占。
 */
/* 突发读一片 INA3221 的 0x01 起 8 字节（SHUNT_CH1,BUS_CH1,SHUNT_CH2,BUS_CH2，
   寄存器自动增量），解析出 2 个电机的电流 mA + 电压 mV。
   替代原来「每寄存器单独读」的 8 次事务 -> 2 次（~920µs -> ~500µs @400kHz）。 */
/* 返回 0=读取失败，1=成功；失败时不写输出参数。 */
static uint8_t ina_read_two_motors(uint8_t addr, int16_t *cur0, uint16_t *vol0,
                                int16_t *cur1, uint16_t *vol1)
{
    uint8_t  raw[8];
    int16_t  shunt0, shunt1;
    uint16_t bus0, bus1;

    if (I2C_INA_ReadBytes(addr, INA3221_REG_SHUNT_CH1, raw, 8) != NO_ERROR)
        return 0;

    /* 分流电压寄存器低 3 位为保留（恒 0），右移 3 位得到 bit[15:3] 有符号数据，
       否则电流会放大 8 倍（0x07D0=100mA 会算成 800mA）。 */
    shunt0 = (int16_t)((int16_t)(((uint16_t)raw[0] << 8) | raw[1]) >> 3);  /* CH1 分流电压 */
    bus0   = (uint16_t)(((uint16_t)raw[2] << 8) | raw[3]);                 /* CH1 总线电压 */
    shunt1 = (int16_t)((int16_t)(((uint16_t)raw[4] << 8) | raw[5]) >> 3);  /* CH2 分流电压 */
    bus1   = (uint16_t)(((uint16_t)raw[6] << 8) | raw[7]);                 /* CH2 总线电压 */

    /* 电流 mA = shunt_raw * 40uV / 100mOhm = shunt_raw * 0.4 */
    *cur0 = (int16_t)((float)shunt0 * INA3221_SHUNT_LSB_UV / PEO_PM_INA3221_RSHUNT_MOHM);
    *vol0 = (uint16_t)((bus0 >> 3) * INA3221_BUS_LSB_MV);   /* (raw>>3)*8 mV */
    *cur1 = (int16_t)((float)shunt1 * INA3221_SHUNT_LSB_UV / PEO_PM_INA3221_RSHUNT_MOHM);
    *vol1 = (uint16_t)((bus1 >> 3) * INA3221_BUS_LSB_MV);
    return 1;
}

void TIM7_IRQHandler(void)
{
    int16_t  cur[4];
    uint16_t vol[4];
    uint8_t  ok_a, ok_b;

    if (TIM_GetITStatus(INA3221_TIM, TIM_IT_Update) == RESET)
        return;

    TIM_ClearITPendingBit(INA3221_TIM, TIM_IT_Update);

    /* 2 次突发读（每片 8 字节）替代 8 次单寄存器读 */
    ok_a = ina_read_two_motors(PEO_PM_INA3221_ADDR_A, &cur[0], &vol[0], &cur[1], &vol[1]);  /* M1/M2 */
    ok_b = ina_read_two_motors(PEO_PM_INA3221_ADDR_B, &cur[2], &vol[2], &cur[3], &vol[3]);  /* M3/M4 */

    /* 两片均成功才发布四路数据；任一失败则保留上一轮数据和时间戳。 */
    if (ok_a == 1 && ok_b == 1)
        SENSOR_PushIna(cur, vol);
}
