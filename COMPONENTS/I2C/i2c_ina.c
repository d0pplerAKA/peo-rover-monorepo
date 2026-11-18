/**
 * @file    i2c_ina.c
 * @brief   I2C2 driver for the INA3221 current monitor (PF1=SCL/PF0=SDA).
 *
 * 8-bit register address, 7-bit slave address left-shifted internally
 * (same convention as i2c_mpu.c). INA3221 sits alone on I2C2 so the
 * VL53L1X bus (I2C3) and MPU6050 bus (I2C1) stay clean.
 */
#include "include/i2c.h"


void I2C_INA_Init(void)    // INA3221
{
    GPIO_InitTypeDef GPIO_S;
    I2C_InitTypeDef I2C_InitStruct;
    I2C_StructInit(&I2C_InitStruct);   /* 填默认值，避免 OwnAddress1 用未初始化值 */

    GPIO_S.GPIO_Mode = GPIO_Mode_AF;
    GPIO_S.GPIO_OType = GPIO_OType_OD;
    GPIO_S.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_S.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_S.GPIO_Pin = PEO_PM_INA_SCL_PIN;
    GPIO_Init(PEO_PM_INA_SCL_PORT, &GPIO_S);

    GPIO_S.GPIO_Pin = PEO_PM_INA_SDA_PIN;
    GPIO_Init(PEO_PM_INA_SDA_PORT, &GPIO_S);

    GPIO_PinAFConfig(PEO_PM_INA_SCL_PORT, PEO_PM_INA_SCL_SRC, PEO_PM_INA_I2C_AF);   // SCL 引脚：GPIOB GPIO_Pin_10
    GPIO_PinAFConfig(PEO_PM_INA_SDA_PORT, PEO_PM_INA_SDA_SRC, PEO_PM_INA_I2C_AF);   // SDA 引脚：GPIOB GPIO_Pin_11

    I2C_DeInit(PEO_PM_INA_I2C);
    delay_ms(5);

    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_ClockSpeed = PEO_PM_INA_I2C_CLKSPEED;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(PEO_PM_INA_I2C, &I2C_InitStruct);

    I2C_AcknowledgeConfig(PEO_PM_INA_I2C, ENABLE);
    I2C_Cmd(PEO_PM_INA_I2C, ENABLE);
}


uint8_t I2C_INA_WriteBytes(uint8_t sl_addr, uint8_t reg_addr, uint8_t *byte_buffer_write, uint32_t byte_num)
{
    uint16_t counter;

    counter = I2C_Timeout;
    while(I2C_GetFlagStatus(PEO_PM_INA_I2C, I2C_FLAG_BUSY))
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_BUSY;
        }
        counter--;
    }
    I2C_GenerateSTART(PEO_PM_INA_I2C, ENABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_EV5;
        }
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_INA_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Transmitter);  /* 7-bit -> 8-bit */

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_EV6;
        }
        counter--;
    }
    I2C_SendData(PEO_PM_INA_I2C, reg_addr);

    for(uint32_t i = 0; i < byte_num; i++)
    {
        counter = I2C_Timeout;
        while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
        {
            if(counter == 0)
            {
                I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
                return ERROR_EV8;
            }
            counter--;
        }
        I2C_SendData(PEO_PM_INA_I2C, *byte_buffer_write);
        byte_buffer_write++;
    }

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_EV8_2;
        }
        counter--;
    }
    I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);

    return NO_ERROR;
}


uint8_t I2C_INA_ReadBytes(uint8_t sl_addr, uint8_t reg_addr, uint8_t *byte_buffer_receive, uint32_t byte_num)
{
    uint16_t counter;

    for(uint32_t i = 0; i < byte_num; i++)
        byte_buffer_receive[i] = 0x00;

    counter = I2C_Timeout;
    while(I2C_GetFlagStatus(PEO_PM_INA_I2C, I2C_FLAG_BUSY))
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_BUSY;
        }
        counter--;
    }
    I2C_GenerateSTART(PEO_PM_INA_I2C, ENABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_EV5;
        }
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_INA_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Transmitter);  /* 7-bit -> 8-bit */

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_EV6;
        }
        counter--;
    }
    I2C_SendData(PEO_PM_INA_I2C, reg_addr);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_EV8;
        }
        counter--;
    }
    I2C_GenerateSTART(PEO_PM_INA_I2C, ENABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_EV5;
        }
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_INA_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Receiver);  /* 7-bit -> 8-bit */
    if (byte_num == 1)
        I2C_AcknowledgeConfig(PEO_PM_INA_I2C, DISABLE);  /* 单字节：清 ADDR 前禁 ACK */

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
            return ERROR_EV6;
        }
        counter--;
    }

    while(byte_num)
    {
        if(byte_num == 1)
            I2C_AcknowledgeConfig(PEO_PM_INA_I2C, DISABLE);

        counter = I2C_Timeout;
        while(I2C_CheckEvent(PEO_PM_INA_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
        {
            if(counter == 0)
            {
                for(uint32_t i = 0; i < byte_num; i++)
                    byte_buffer_receive[i] = 0x00;

                I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);
                return ERROR_EV7;
            }
            counter--;
        }
        *byte_buffer_receive = I2C_ReceiveData(PEO_PM_INA_I2C);

        byte_buffer_receive++;
        byte_num--;
    }

    I2C_GenerateSTOP(PEO_PM_INA_I2C, ENABLE);

    I2C_AcknowledgeConfig(PEO_PM_INA_I2C, ENABLE);

    return NO_ERROR;
}
