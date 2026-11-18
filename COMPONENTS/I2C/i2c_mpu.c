#include "include/i2c.h"


void I2C_MPU_Init(void)    // MPU6050
{
    GPIO_InitTypeDef GPIO_S;
    I2C_InitTypeDef I2C_InitStruct;
    I2C_StructInit(&I2C_InitStruct);   /* 填默认值，避免 OwnAddress1 用未初始化值 */

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


uint32_t I2C_MPU_WriteBytes(unsigned char slave_addr, unsigned char addr, unsigned char byteNum, unsigned char const *data)
{
    uint16_t counter;
	
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
    I2C_Send7bitAddress(PEO_PM_MPU6050_I2C, (uint8_t)(slave_addr << 1), I2C_Direction_Transmitter);  /* 7-bit -> 8-bit：标准库不左移 */

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

uint32_t I2C_MPU_ReadBytes(unsigned char slave_addr, unsigned char addr, unsigned char byteNum, unsigned char *data)
{
    uint16_t counter;

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
    I2C_Send7bitAddress(PEO_PM_MPU6050_I2C, (uint8_t)(slave_addr << 1), I2C_Direction_Transmitter);  /* 7-bit -> 8-bit：标准库不左移 */


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
        I2C_AcknowledgeConfig(PEO_PM_MPU6050_I2C, DISABLE);  /* 单字节：清 ADDR 前禁 ACK */

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
