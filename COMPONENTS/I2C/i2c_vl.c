#include "include/i2c.h"


void I2C_TOF_Init(void)    // TOF400C
{
    GPIO_InitTypeDef GPIO_S;
    I2C_InitTypeDef I2C_InitStruct;

    GPIO_S.GPIO_Mode = GPIO_Mode_AF;
    GPIO_S.GPIO_OType = GPIO_OType_OD;
    GPIO_S.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_S.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_S.GPIO_Pin = PEO_PM_VL53_SCL_PIN;
    GPIO_Init(PEO_PM_VL53_SCL_PORT, &GPIO_S);
    GPIO_SetBits(PEO_PM_VL53_SCL_PORT, PEO_PM_VL53_SCL_PIN);

    GPIO_S.GPIO_Pin = PEO_PM_VL53_SDA_PIN;
    GPIO_Init(PEO_PM_VL53_SDA_PORT, &GPIO_S);
    GPIO_SetBits(PEO_PM_VL53_SDA_PORT, PEO_PM_VL53_SDA_PIN);

    GPIO_PinAFConfig(PEO_PM_VL53_SCL_PORT, PEO_PM_VL53_SCL_SRC, PEO_PM_VL53_I2C_AF);            // SCL GPIO_Pin_8      GPIOA
    GPIO_PinAFConfig(PEO_PM_VL53_SDA_PORT, PEO_PM_VL53_SDA_SRC, PEO_PM_VL53_I2C_AF);            // SDA GPIO_Pin_9      GPIOC

    I2C_DeInit(PEO_PM_VL53_I2C);
    delay_ms(5);

    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_ClockSpeed = PEO_PM_VL53_I2C_CLKSPEED;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(PEO_PM_VL53_I2C, &I2C_InitStruct);

    I2C_AcknowledgeConfig(PEO_PM_VL53_I2C, ENABLE);
    I2C_Cmd(PEO_PM_VL53_I2C, ENABLE);
}


uint8_t I2C_TOF_WriteByte(uint8_t sl_addr, uint16_t reg_addr, uint8_t byte_write)
{
    uint8_t index_H = (uint8_t) (reg_addr >> 8);
    uint8_t index_L = (uint8_t) reg_addr;
    

    uint16_t counter;
	
    counter = I2C_Timeout;
    while(I2C_GetFlagStatus(PEO_PM_VL53_I2C, I2C_FLAG_BUSY))
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_BUSY;
        }

        counter--;
    }
    I2C_GenerateSTART(PEO_PM_VL53_I2C, ENABLE);
	
    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Transmitter);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV6;
        }
        
        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, index_H);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV8;
        }
        
        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, index_L);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV8;
        }
        
        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, byte_write);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV8_2;
        }
        
        counter--;
    }
    I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);

    return NO_ERROR;
}

uint8_t I2C_TOF_ReadByte(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_receive)
{
    uint8_t index_H = (uint8_t) (reg_addr >> 8);
    uint8_t index_L = (uint8_t) reg_addr;

    *byte_receive = 0x00;

    uint16_t counter;

    counter = I2C_Timeout;
    while(I2C_GetFlagStatus(PEO_PM_VL53_I2C, I2C_FLAG_BUSY))
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_BUSY;
        }
        
        counter--;
    }
    I2C_GenerateSTART(PEO_PM_VL53_I2C, ENABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Transmitter);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV6;
        }
        
        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, index_H);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV6;
        }
        
        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, index_L);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV8;
        }
        
        counter--;
    }
    I2C_GenerateSTART(PEO_PM_VL53_I2C, ENABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Receiver);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV6;
        }
        
        counter--;
    }
    I2C_AcknowledgeConfig(PEO_PM_VL53_I2C, DISABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV7;
        }
        
        counter--;
    }

    *byte_receive = I2C_ReceiveData(PEO_PM_VL53_I2C);

    I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);

    I2C_AcknowledgeConfig(PEO_PM_VL53_I2C, ENABLE);

    return NO_ERROR;
}


uint8_t I2C_TOF_WriteBytes(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_buffer_write, uint32_t byte_num)
{
    uint8_t index_H = (uint8_t) (reg_addr >> 8);
    uint8_t index_L = (uint8_t) reg_addr;

    uint16_t counter;
	
    counter = I2C_Timeout;
    while(I2C_GetFlagStatus(PEO_PM_VL53_I2C, I2C_FLAG_BUSY))
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_BUSY;
        }

        counter--;
    }
    I2C_GenerateSTART(PEO_PM_VL53_I2C, ENABLE);
	
    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Transmitter);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV6;
        }
        
        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, index_H);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV8;
        }
        
        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, index_L);

    for(uint32_t i = 0; i < byte_num; i++)
    {
        counter = I2C_Timeout;
        while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
        {
            if(counter == 0)
            {
                I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
                return ERROR_EV8;
            }
            
            counter--;
        }
        I2C_SendData(PEO_PM_VL53_I2C, *byte_buffer_write);

        byte_buffer_write++;
    }

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV8_2;
        }
        
        counter--;
    }
    I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);

    return NO_ERROR;
}

uint8_t I2C_TOF_ReadBytes(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_buffer_receive, uint32_t byte_num)
{
    uint8_t index_H = (uint8_t) (reg_addr >> 8);
    uint8_t index_L = (uint8_t) reg_addr;

    for(uint32_t i = 0; i < byte_num; i++)
        byte_buffer_receive[i] = 0x00;
    
    uint16_t counter;

    counter = I2C_Timeout;
    while(I2C_GetFlagStatus(PEO_PM_VL53_I2C, I2C_FLAG_BUSY))
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_BUSY;
        }

        counter--;
    }
    I2C_GenerateSTART(PEO_PM_VL53_I2C, ENABLE);


    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Transmitter);


    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV6;
        }

        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, index_H);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV6;
        }

        counter--;
    }
    I2C_SendData(PEO_PM_VL53_I2C, index_L);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV8;
        }

        counter--;
    }
    I2C_GenerateSTART(PEO_PM_VL53_I2C, ENABLE);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV5;
        }
        
        counter--;
    }
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)(sl_addr << 1), I2C_Direction_Receiver);

    counter = I2C_Timeout;
    while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
    {
        if(counter == 0)
        {
            I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
            return ERROR_EV6;
        }

        counter--;
    }

    while(byte_num)
    {
        if(byte_num == 1)
            I2C_AcknowledgeConfig(PEO_PM_VL53_I2C, DISABLE);
        
        counter = I2C_Timeout;
        while(I2C_CheckEvent(PEO_PM_VL53_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
        {
            if(counter == 0)
            {
                for(uint32_t i = 0; i < byte_num; i++)
                    byte_buffer_receive[i] = 0x00;

                I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);
                return ERROR_EV7;
            }

            counter--;
        }
        *byte_buffer_receive = I2C_ReceiveData(PEO_PM_VL53_I2C);

        byte_buffer_receive++;

        byte_num--;
    }

    I2C_GenerateSTOP(PEO_PM_VL53_I2C, ENABLE);

    I2C_AcknowledgeConfig(PEO_PM_VL53_I2C, ENABLE);

    return NO_ERROR;
}

