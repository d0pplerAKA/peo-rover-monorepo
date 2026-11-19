/**
 * @file  i2c_mpu.c
 * @brief Adapt the MPU6050 motion driver to register transfers over I2C1.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#include "include/i2c.h"

/* Register-transfer sequence for readers new to I2C:
 * START claims the bus; the device address selects a sensor; register bytes
 * choose the setting or measurement inside it. A write then sends data. A read
 * uses a repeated START to change direction without releasing the bus. ACK asks
 * for another byte; disabling ACK marks the final receive byte; STOP releases
 * the bus. Each polling loop has a fresh iteration limit so a missing sensor
 * can return an error instead of waiting forever at that stage.
 * GPIO open-drain outputs pull low and release high; pull-ups provide the high
 * level. These routines poll hardware flags and do not use I2C interrupts.
 */


/**
 * @brief Prepare I2C1 on PB6/PB7 for sensor register transfers.
 *
 * Called during MPU6050 motion driver initialization. Configures open-drain
 * alternate-function pins and the bus speed; RCC must already have enabled the clocks.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void I2C_MPU_Init(void)    // MPU6050
{
    GPIO_InitTypeDef GPIO_S;  /* GPIO setup record reused to configure this component hardware pins. */
    I2C_InitTypeDef I2C_InitStruct;  /* I2C setup record holding bus frequency, addressing mode, and acknowledgement settings. */
    I2C_StructInit(&I2C_InitStruct);   /* Fill defaults so fields such as OwnAddress1 never contain uninitialized data. */

    GPIO_S.GPIO_Mode = GPIO_Mode_AF;
    GPIO_S.GPIO_OType = GPIO_OType_OD;
    GPIO_S.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_S.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_S.GPIO_Pin = PEO_PM_MPU6050_SCL_PIN;
    GPIO_Init(PEO_PM_MPU6050_SCL_PORT, &GPIO_S);

    GPIO_S.GPIO_Pin = PEO_PM_MPU6050_SDA_PIN;
    GPIO_Init(PEO_PM_MPU6050_SDA_PORT, &GPIO_S);

    GPIO_PinAFConfig(PEO_PM_MPU6050_SCL_PORT, PEO_PM_MPU6050_SCL_SRC, PEO_PM_MPU6050_I2C_AF);            // SCL GPIO_Pin_6      GPIOB
    GPIO_PinAFConfig(PEO_PM_MPU6050_SDA_PORT, PEO_PM_MPU6050_SDA_SRC, PEO_PM_MPU6050_I2C_AF);            // SDA GPIO_Pin_7      GPIOB

    I2C_DeInit(PEO_PM_MPU6050_I2C);
    delay_ms(5);

    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_ClockSpeed = PEO_PM_MPU6050_I2C_CLKSPEED;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(PEO_PM_MPU6050_I2C, &I2C_InitStruct);

    I2C_AcknowledgeConfig(PEO_PM_MPU6050_I2C, ENABLE);
    I2C_Cmd(PEO_PM_MPU6050_I2C, ENABLE);
}


/**
 * @brief Write a block of sensor register bytes using I2C1.
 *
 * Used by the MPU6050 motion driver. This is a blocking, polled transfer with a bounded
 * loop count at each bus event. It writes the register address followed by the data,
 * then requests STOP.
 * @param slave_addr Unshifted 7-bit device address; this function shifts it left once
 *                   for the ST library.
 * @param addr 8-bit register address at which the transfer begins.
 * @param byteNum Number of data bytes; pass a positive count and a buffer of at least
 *                this size.
 * @param data Readable source bytes, retained unchanged.
 * @retval NO_ERROR The transfer completed (zero).
 * @retval nonzero An ERROR_BUSY or ERROR_EV* code identifies the bus stage that timed
 *                 out.
 */
uint32_t I2C_MPU_WriteBytes(unsigned char slave_addr, unsigned char addr, unsigned char byteNum, unsigned char const *data)
{
    uint16_t counter;  /* Remaining polling attempts for the current I2C event, not a millisecond timer. */
	
    counter = I2C_Timeout;
    while(I2C_GetFlagStatus(PEO_PM_MPU6050_I2C, I2C_FLAG_BUSY))
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_BUSY;
        }

        counter--;
    }
    I2C_GenerateSTART(PEO_PM_MPU6050_I2C, ENABLE);
	
    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    /* Convert the 7-bit address to the shifted bus form; the ST library does not shift it. */
    I2C_Send7bitAddress(PEO_PM_MPU6050_I2C, (uint8_t)(slave_addr << 1), I2C_Direction_Transmitter);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_EV6;
        }
        
        counter--;
    }
    I2C_SendData(PEO_PM_MPU6050_I2C, addr);

    for(uint32_t i = 0; i < byteNum; i++)
    {
        counter = I2C_Timeout;
        while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
        {
            if(counter == 0)
            {
                I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
                return ERROR_EV8;
            }
            
            counter--;
        }
        I2C_SendData(PEO_PM_MPU6050_I2C, *data);

        data++;
    }

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_EV8_2;
        }
        
        counter--;
    }
    I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);

    return NO_ERROR;
}

/**
 * @brief Read a block of sensor register bytes using I2C1.
 *
 * Used by the MPU6050 motion driver. This is a blocking, polled transfer with a bounded
 * loop count at each bus event. It first writes the register address, then issues a
 * repeated START to receive data.
 * @param slave_addr Unshifted 7-bit device address; this function shifts it left once
 *                   for the ST library.
 * @param addr 8-bit register address at which the transfer begins.
 * @param byteNum Number of data bytes; pass a positive count and a buffer of at least
 *                this size.
 * @param data Writable destination; initially cleared, but may contain partial data if
 *             an error occurs.
 * @retval NO_ERROR The transfer completed (zero).
 * @retval nonzero An ERROR_BUSY or ERROR_EV* code identifies the bus stage that timed
 *                 out.
 */
uint32_t I2C_MPU_ReadBytes(unsigned char slave_addr, unsigned char addr, unsigned char byteNum, unsigned char *data)
{
    uint16_t counter;  /* Remaining polling attempts for the current I2C event, not a millisecond timer. */

    for(uint32_t i = 0; i < byteNum; i++)
        data[i] = 0x00;

    counter = I2C_Timeout;
    while(I2C_GetFlagStatus(PEO_PM_MPU6050_I2C, I2C_FLAG_BUSY))
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_BUSY;
        }

        counter--;
    }
    I2C_GenerateSTART(PEO_PM_MPU6050_I2C, ENABLE);


    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    /* Convert the 7-bit address to the shifted bus form; the ST library does not shift it. */
    I2C_Send7bitAddress(PEO_PM_MPU6050_I2C, (uint8_t)(slave_addr << 1), I2C_Direction_Transmitter);


    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_EV6;
        }

        counter--;
    }
    I2C_SendData(PEO_PM_MPU6050_I2C, addr);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_EV8;
        }

        counter--;
    }
    I2C_GenerateSTART(PEO_PM_MPU6050_I2C, ENABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_MPU6050_I2C, (uint8_t)(slave_addr << 1), I2C_Direction_Receiver);  /* 7-bit -> 8-bit */
    if (byteNum == 1)
        /* Single-byte read: disable ACK before clearing ADDR so this is the final byte. */
        I2C_AcknowledgeConfig(PEO_PM_MPU6050_I2C, DISABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
            return ERROR_EV6;
        }

        counter--;
    }

    while(byteNum)
    {
        if(byteNum == 1)
            I2C_AcknowledgeConfig(PEO_PM_MPU6050_I2C, DISABLE);
        
        counter = I2C_Timeout;
        while(I2C_CheckEvent(PEO_PM_MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
        {
            if(counter == 0)
            {
                for(uint32_t i = 0; i < byteNum; i++)
                    data[i] = 0x00;

                I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);
                return ERROR_EV7;
            }

            counter--;
        }
        *data = I2C_ReceiveData(PEO_PM_MPU6050_I2C);

        data++;

        byteNum--;
    }

    I2C_GenerateSTOP(PEO_PM_MPU6050_I2C, ENABLE);

    I2C_AcknowledgeConfig(PEO_PM_MPU6050_I2C, ENABLE);

    return NO_ERROR;
}
