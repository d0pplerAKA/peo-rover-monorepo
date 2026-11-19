/**
 * @file  ina3221.c
 * @brief Collect four motor currents and voltages from two INA3221 monitors.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Both devices share I2C2 (PB10=SCL, PB11=SDA).
 * Device A: A0=GND, address 0x40; CH1=M1, CH2=M2, CH3 unused.
 * Device B: A0=VS, address 0x41; CH1=M3, CH2=M4, CH3 unused.
 * Each module uses an R100 shunt resistor (0.1 ohm). Shunt-voltage counts
 * represent 40 microvolts; bus-voltage counts represent 8 millivolts.
 * The 16-bit shunt register carries signed data, so current can be positive
 * (normal consumption) or negative (regeneration/reverse flow).
 */
#include "include/ina3221.h"

#include "COMPONENTS/I2C/include/i2c.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"
#include <stdio.h>

/* INA3221 register addresses. */
#define INA3221_REG_CONFIG        0x00  /* Register selecting enabled channels, averaging, timing, and conversion mode. */
#define INA3221_REG_SHUNT_CH1     0x01  /* Channel 1 shunt-voltage register; signed data is stored in bits 15:3. */
#define INA3221_REG_BUS_CH1       0x02  /* Channel 1 bus-voltage register; decoded counts represent 8 mV each. */
#define INA3221_REG_SHUNT_CH2     0x03  /* Channel 2 shunt-voltage register; signed data is stored in bits 15:3. */
#define INA3221_REG_BUS_CH2       0x04  /* Channel 2 bus-voltage register; decoded counts represent 8 mV each. */
#define INA3221_REG_SHUNT_CH3     0x05  /* Channel 3 shunt-voltage register; signed data is stored in bits 15:3. */
#define INA3221_REG_BUS_CH3       0x06  /* Channel 3 bus-voltage register; decoded counts represent 8 mV each. */
#define INA3221_REG_MANUFACT_ID   0xFE  /* Manufacturer-identification register used to check chip presence. */
#define INA3221_REG_DIE_ID        0xFF  /* Chip-identification register address; reserved for diagnostics. */

/* Expected chip-identification values. */
#define INA3221_MANUFACT_ID       0x5449  /* Expected manufacturer ID returned by the current monitor. */
#define INA3221_DIE_ID            0x3220  /* Expected device ID for optional identity diagnostics. */

/* Conversion factors from register counts to physical units. */
#define INA3221_SHUNT_LSB_UV      40U     /* Shunt-voltage resolution: 40 microvolts per decoded count. */
#define INA3221_BUS_LSB_MV        8U      /* Bus-voltage resolution: 8 millivolts per decoded count. */

/* Configuration register 0x00: RST bit 15=0; enable CH1 (bit 14) and
   CH2 (bit 13), disable unused CH3 (bit 12). AVG[11:9]=000: one sample.
   VBUSCT[8:6]=000 and VSHCT[5:3]=000: 140 microseconds per conversion.
   MODE[2:0]=111: continuously measure shunt and bus voltages. */
#define INA3221_CONFIG_CONTINUOUS (0x6007U)  /* Register bit pattern enabling continuous shunt and bus conversion on CH1/CH2. */



/**
 * @brief Request continuous shunt and bus-voltage conversions on one chip.
 *
 * Internal setup helper called by INA3221_Init. Sends the two-byte configuration
 * register in high-byte-first order; this helper does not report the write status.
 * @param addr Unshifted 7-bit INA3221 address, 0x40 or 0x41.
 * @retval None This function does not return a value.
 */
static void INA3221_Config(uint8_t addr)
{
    uint8_t cfg[2] = { (uint8_t)(INA3221_CONFIG_CONTINUOUS >> 8),
                       (uint8_t)(INA3221_CONFIG_CONTINUOUS & 0xFF) };
    I2C_INA_WriteBytes(addr, INA3221_REG_CONFIG, cfg, 2);
}


/* Initialization status for devices A/B: 0=found, 1=not found, 3=not attempted. */
static uint8_t g_init_status[2] = {3, 3};  /* Saved identity-check results for devices A/B; initial 3 means not attempted. */

/**
 * @brief Check one current-monitor identity and cache its presence result.
 *
 * Called by INA3221_Init; a failed identity check is printed immediately, while success
 * is later shown in the boot banner.
 * @param addr Unshifted 7-bit device address.
 * @param idx Status-array index: 0=device A (M1/M2), 1=device B (M3/M4).
 * @retval None This function does not return a value.
 */
static void INA3221_CheckId(uint8_t addr, uint8_t idx)
{
    uint8_t id[2] = {0, 0};  /* Two manufacturer-ID bytes, high byte first. */
    uint16_t manuf;  /* Two received bytes combined into the manufacturer identity value. */

    I2C_INA_ReadBytes(addr, INA3221_REG_MANUFACT_ID, id, 2);
    manuf = ((uint16_t)id[0] << 8) | id[1];

    if (manuf == INA3221_MANUFACT_ID)
        g_init_status[idx] = 0;   /* Device found; successful detection stays silent until the boot banner. */
    else
    {
        g_init_status[idx] = 1;
        printf("[INA3221 @0x%02X] NOT found (ID=0x%04X)\r\n", (unsigned)addr, (unsigned)manuf);
    }
}


/**
 * @brief Configure both current-monitor chips without starting periodic host reads.
 *
 * Called by main at startup. Initializes I2C2 and continuous conversions; the gamepad X
 * toggle later starts TIM7 polling through INA3221_PollInit.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void INA3221_Init(void)
{
    I2C_INA_Init();   /* I2C2 @ 400 kHz, PB10=SCL/PB11=SDA */

    INA3221_Config(PEO_PM_INA3221_ADDR_A);
    INA3221_CheckId(PEO_PM_INA3221_ADDR_A, 0);

    INA3221_Config(PEO_PM_INA3221_ADDR_B);
    INA3221_CheckId(PEO_PM_INA3221_ADDR_B, 1);
}


/* TIM7 polling: read four currents/voltages at SENSOR_RATE_INA_HZ and
   publish them to the sensor hub. TIM7 is a 16-bit APB1 basic timer.
   Prescale 84 MHz to 10 kHz (PSC=8399), then to the requested rate
   (ARR=10000/RATE-1). A direct 84 MHz-to-10 Hz divider would not fit
   in 16 bits. Change SENSOR_RATE_INA_HZ to change the polling frequency. */
#define INA3221_TIM      TIM7  /* Basic timer used to schedule periodic current/voltage reads. */
#define INA3221_TIM_PSC  (PEO_PM_TIM_APB1_HZ / 10000UL - 1)   /* 84 MHz -> 10 kHz */
#define INA3221_TIM_ARR  (10000UL / SENSOR_RATE_INA_HZ - 1)   /* 10 kHz -> RATE Hz */


/**
 * @brief Read the saved startup presence check for one current-monitor chip.
 *
 * Used by print_boot_banner; this does not perform a new I2C transaction.
 * @param idx Device index 0=A or 1=B.
 * @retval 0 Expected manufacturer ID was found.
 * @retval 1 Identity check failed.
 * @retval 3 Initialization not attempted, or index is invalid.
 */
uint8_t INA3221_GetInitStatus(uint8_t idx)
{
    if (idx >= 2)
        return 3;
    return g_init_status[idx];
}

/**
 * @brief Start TIM7 so motor-current measurements are refreshed periodically.
 *
 * Called by the gamepad X toggle after INA3221_Init. Only timer-side configuration is
 * done here; RCC/NVIC are owned centrally.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note RCC_Clock_Init()/NVIC_InitAll() own clocks and the NVIC channel. This function
 *       only configures the timer and its interrupt source, following the same
 *       separation as CONTROL with TIM6.
 */
void INA3221_PollInit(void)
{
    TIM_TimeBaseInitTypeDef tb;  /* Timer setup record containing prescaler and period counts for the requested rate. */

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

/* Stop TIM7 polling when the user toggles current monitoring off. */
/**
 * @brief Stop periodic INA3221 reads by disabling TIM7.
 *
 * Called when the gamepad X toggle turns monitoring off. The chips remain configured and
 * the sensor hub retains its previous readings.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void INA3221_Stop(void)
{
    TIM_Cmd(INA3221_TIM, DISABLE);
}


/* Read eight consecutive bytes starting at register 0x01 on one INA3221:
   SHUNT_CH1, BUS_CH1, SHUNT_CH2, BUS_CH2 (register auto-increment).
   Decode two motor currents in mA and voltages in mV. Across both devices,
   this replaces eight separate register transactions with two bursts
   (historical timing estimate: about 920 us -> 500 us at 400 kHz). */
/* Return 0 on read failure, leaving outputs unchanged; return 1 on success. */
/**
 * @brief Read one chip and convert its two active channels to current and voltage.
 *
 * Internal helper called twice per TIM7 tick. Uses one eight-byte burst and only assigns
 * output values after a successful transfer.
 * @param addr Unshifted 7-bit device address.
 * @param cur0 Non-NULL output for channel 1 signed current in mA.
 * @param vol0 Non-NULL output for channel 1 bus voltage in mV.
 * @param cur1 Non-NULL output for channel 2 signed current in mA.
 * @param vol1 Non-NULL output for channel 2 bus voltage in mV.
 * @retval 1 Both channel measurements were decoded and stored.
 * @retval 0 I2C read failed; output values were not changed.
 */
static uint8_t ina_read_two_motors(uint8_t addr, int16_t *cur0, uint16_t *vol0,
                                int16_t *cur1, uint16_t *vol1)
{
    uint8_t  raw[8];  /* Eight register bytes in chip order: shunt1, bus1, shunt2, bus2. */
    int16_t  shunt0, shunt1; /* Signed channel 1/2 shunt counts after removing three reserved bits; 40 uV per count. */
    uint16_t bus0, bus1; /* Raw channel 1/2 bus-voltage words; shift right three, then multiply by 8 mV. */

    if (I2C_INA_ReadBytes(addr, INA3221_REG_SHUNT_CH1, raw, 8) != NO_ERROR)
        return 0;

    /* The bottom three shunt-register bits are reserved zeros. Arithmetic
       right shift by three extracts signed bits [15:3]; omitting this would
       multiply current by eight (0x07D0 means 100 mA, not 800 mA). */
    shunt0 = (int16_t)((int16_t)(((uint16_t)raw[0] << 8) | raw[1]) >> 3);  /* Channel 1 shunt-voltage register. */
    bus0   = (uint16_t)(((uint16_t)raw[2] << 8) | raw[3]);                 /* Channel 1 bus-voltage register. */
    shunt1 = (int16_t)((int16_t)(((uint16_t)raw[4] << 8) | raw[5]) >> 3);  /* Channel 2 shunt-voltage register. */
    bus1   = (uint16_t)(((uint16_t)raw[6] << 8) | raw[7]);                 /* Channel 2 bus-voltage register. */

    /* Current in mA = shunt_raw * 40 microvolts / 100 milliohms = shunt_raw * 0.4. */
    *cur0 = (int16_t)((float)shunt0 * INA3221_SHUNT_LSB_UV / PEO_PM_INA3221_RSHUNT_MOHM);
    *vol0 = (uint16_t)((bus0 >> 3) * INA3221_BUS_LSB_MV);   /* (raw>>3)*8 mV */
    *cur1 = (int16_t)((float)shunt1 * INA3221_SHUNT_LSB_UV / PEO_PM_INA3221_RSHUNT_MOHM);
    *vol1 = (uint16_t)((bus1 >> 3) * INA3221_BUS_LSB_MV);
    return 1;
}

/**
 * @brief Refresh all four motor-current and voltage readings on a polling tick.
 *
 * Called by the processor on TIM7 updates. Publishes a new set only if both chips were
 * read successfully; otherwise previous readings keep their old timestamp.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Replaces the weak TIM7_IRQHandler in startup_stm32f40xx.s. Two eight-byte burst
 *       reads collect four currents and voltages, convert them to integer units, and
 *       publish them to the hub. This runs at the lowest preemption priority so sensor
 *       EXTI handlers may interrupt it.
 */
void TIM7_IRQHandler(void)
{
    int16_t  cur[4];  /* Four signed currents in mA, published only after both chip reads succeed. */
    uint16_t vol[4];  /* Four bus voltages in mV, in M1..M4 order. */
    uint8_t  ok_a, ok_b; /* Success flags for the two chip reads; both must be one before publishing. */

    if (TIM_GetITStatus(INA3221_TIM, TIM_IT_Update) == RESET)
        return;

    TIM_ClearITPendingBit(INA3221_TIM, TIM_IT_Update);

    /* Use two eight-byte burst reads instead of eight separate register reads. */
    ok_a = ina_read_two_motors(PEO_PM_INA3221_ADDR_A, &cur[0], &vol[0], &cur[1], &vol[1]);  /* M1/M2 */
    ok_b = ina_read_two_motors(PEO_PM_INA3221_ADDR_B, &cur[2], &vol[2], &cur[3], &vol[3]);  /* M3/M4 */

    /* Publish all four channels only if both reads succeed. Otherwise retain
       the previous measurements and their timestamp so their age remains visible. */
    if (ok_a == 1 && ok_b == 1)
        SENSOR_PushIna(cur, vol);
}
