/**
 * @file  i2c_vl.c
 * @brief Transfer VL53L1X ranging registers over the shared I2C3 sensor bus.
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
 * @brief Prepare I2C3 on PA8/PC9 for sensor register transfers.
 *
 * Called during VL53L1X ranging platform adapter initialization. Configures open-drain
 * alternate-function pins and the bus speed; RCC must already have enabled the clocks.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void I2C_TOF_Init(void)    // TOF400C
{
    GPIO_InitTypeDef GPIO_S;  /* GPIO setup record reused to configure this component hardware pins. */
    I2C_InitTypeDef I2C_InitStruct;  /* I2C setup record holding bus frequency, addressing mode, and acknowledgement settings. */
    I2C_StructInit(&I2C_InitStruct);   /* Fill defaults so fields such as OwnAddress1 never contain uninitialized data. */

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


/* sl_addr is already in 8-bit bus form (7-bit address << 1); do not shift again. */
/**
 * @brief Write one sensor register byte using I2C3.
 *
 * Used by the VL53L1X ranging platform adapter. This is a blocking, polled transfer with
 * a bounded loop count at each bus event. It writes the register address followed by the
 * data, then requests STOP.
 * @param sl_addr Already-shifted 8-bit bus address, such as 0x52; do not shift it again.
 * @param reg_addr 16-bit register address; the high byte is sent before the low byte.
 * @param byte_write One 8-bit value to store in the selected register.
 * @retval NO_ERROR The transfer completed (zero).
 * @retval nonzero An ERROR_BUSY or ERROR_EV* code identifies the bus stage that timed
 *                 out.
 */
uint8_t I2C_TOF_WriteByte(uint8_t sl_addr, uint16_t reg_addr, uint8_t byte_write)
{
    uint8_t index_H = (uint8_t) (reg_addr >> 8);  /* High byte of the 16-bit range-sensor register address, sent first. */
    uint8_t index_L = (uint8_t) reg_addr;  /* Low byte of the 16-bit range-sensor register address, sent second. */
    

    uint16_t counter;  /* Remaining polling attempts for the current I2C event, not a millisecond timer. */
	
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
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)sl_addr, I2C_Direction_Transmitter);

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

/* sl_addr is already in 8-bit bus form (7-bit address << 1); do not shift again. */
/**
 * @brief Read one sensor register byte using I2C3.
 *
 * Used by the VL53L1X ranging platform adapter. This is a blocking, polled transfer with
 * a bounded loop count at each bus event. It first writes the register address, then
 * issues a repeated START to receive data.
 * @param sl_addr Already-shifted 8-bit bus address, such as 0x52; do not shift it again.
 * @param reg_addr 16-bit register address; the high byte is sent before the low byte.
 * @param byte_receive Non-NULL output pointer for the received byte; cleared before
 *                     starting.
 * @retval NO_ERROR The transfer completed (zero).
 * @retval nonzero An ERROR_BUSY or ERROR_EV* code identifies the bus stage that timed
 *                 out.
 */
uint8_t I2C_TOF_ReadByte(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_receive)
{
    uint8_t index_H = (uint8_t) (reg_addr >> 8);  /* High byte of the 16-bit range-sensor register address, sent first. */
    uint8_t index_L = (uint8_t) reg_addr;  /* Low byte of the 16-bit range-sensor register address, sent second. */

    *byte_receive = 0x00;

    uint16_t counter;  /* Remaining polling attempts for the current I2C event, not a millisecond timer. */

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
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)sl_addr, I2C_Direction_Transmitter);

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
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)sl_addr, I2C_Direction_Receiver);
    I2C_AcknowledgeConfig(PEO_PM_VL53_I2C, DISABLE);  /* Single-byte read: disable ACK before clearing ADDR so this is the final byte. */

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


/* sl_addr is already in 8-bit bus form (7-bit address << 1); do not shift again. */
/**
 * @brief Write a block of sensor register bytes using I2C3.
 *
 * Used by the VL53L1X ranging platform adapter. This is a blocking, polled transfer with
 * a bounded loop count at each bus event. It writes the register address followed by the
 * data, then requests STOP.
 * @param sl_addr Already-shifted 8-bit bus address, such as 0x52; do not shift it again.
 * @param reg_addr 16-bit register address; the high byte is sent before the low byte.
 * @param byte_buffer_write Readable source with at least byte_num bytes.
 * @param byte_num Number of data bytes to transfer; callers provide a positive count.
 * @retval NO_ERROR The transfer completed (zero).
 * @retval nonzero An ERROR_BUSY or ERROR_EV* code identifies the bus stage that timed
 *                 out.
 */
uint8_t I2C_TOF_WriteBytes(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_buffer_write, uint32_t byte_num)
{
    uint8_t index_H = (uint8_t) (reg_addr >> 8);  /* High byte of the 16-bit range-sensor register address, sent first. */
    uint8_t index_L = (uint8_t) reg_addr;  /* Low byte of the 16-bit range-sensor register address, sent second. */

    uint16_t counter;  /* Remaining polling attempts for the current I2C event, not a millisecond timer. */
	
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
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)sl_addr, I2C_Direction_Transmitter);

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

/* sl_addr is already in 8-bit bus form (7-bit address << 1); do not shift again. */
/**
 * @brief Read a block of sensor register bytes using I2C3.
 *
 * Used by the VL53L1X ranging platform adapter. This is a blocking, polled transfer with
 * a bounded loop count at each bus event. It first writes the register address, then
 * issues a repeated START to receive data.
 * @param sl_addr Already-shifted 8-bit bus address, such as 0x52; do not shift it again.
 * @param reg_addr 16-bit register address; the high byte is sent before the low byte.
 * @param byte_buffer_receive Writable destination of at least byte_num bytes; failure
 *                            may leave partial data.
 * @param byte_num Number of data bytes to transfer; callers provide a positive count.
 * @retval NO_ERROR The transfer completed (zero).
 * @retval nonzero An ERROR_BUSY or ERROR_EV* code identifies the bus stage that timed
 *                 out.
 */
uint8_t I2C_TOF_ReadBytes(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_buffer_receive, uint32_t byte_num)
{
    uint8_t index_H = (uint8_t) (reg_addr >> 8);  /* High byte of the 16-bit range-sensor register address, sent first. */
    uint8_t index_L = (uint8_t) reg_addr;  /* Low byte of the 16-bit range-sensor register address, sent second. */

    for(uint32_t i = 0; i < byte_num; i++)
        byte_buffer_receive[i] = 0x00;
    
    uint16_t counter;  /* Remaining polling attempts for the current I2C event, not a millisecond timer. */

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
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)sl_addr, I2C_Direction_Transmitter);


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
    I2C_Send7bitAddress(PEO_PM_VL53_I2C, (uint8_t)sl_addr, I2C_Direction_Receiver);

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

