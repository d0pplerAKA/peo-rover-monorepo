#ifndef __I2C_H__
#define __I2C_H__

#include <stdio.h>
#include "stm32f4xx.h"

#include "COMPONENTS/PEO_PINMAP.H"

#include "COMPONENTS/DELAY/include/delay.h"



// Mounting below APB1 Peripheral bus with a clock of (100MHz / 2) -> 50MHz
#define I2C_Timeout                         (uint16_t ) 500

#define NO_ERROR                            0
#define ERROR_EV5                           5
#define ERROR_EV6                           6
#define ERROR_EV7                           7
#define ERROR_EV8                           8
#define ERROR_EV8_2                         82
#define ERROR_BUSY                          9

#define I2C_Fail                            0xCE
#define I2C_Success                         0x00


// I2C1 - MPU6050
void I2C_MPU_Init(void);
uint32_t I2C_MPU_WriteBytes(unsigned char slave_addr, unsigned char addr, unsigned char byteNum, unsigned char const *data);
uint32_t I2C_MPU_ReadBytes(unsigned char slave_addr, unsigned char addr, unsigned char byteNum, unsigned char *data);


// I2C3 - TOF400C (VL53L1X only)
void I2C_TOF_Init(void);
uint8_t I2C_TOF_WriteByte(uint8_t sl_addr, uint16_t reg_addr, uint8_t byte_write);
uint8_t I2C_TOF_ReadByte(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_receive);
uint8_t I2C_TOF_WriteBytes(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_buffer_write, uint32_t byte_num);
uint8_t I2C_TOF_ReadBytes(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_buffer_receive, uint32_t byte_num);


// I2C2 - INA3221 (8-bit register address, 7-bit slave addr left-shifted inside)
void I2C_INA_Init(void);
uint8_t I2C_INA_WriteBytes(uint8_t sl_addr, uint8_t reg_addr, uint8_t *byte_buffer_write, uint32_t byte_num);
uint8_t I2C_INA_ReadBytes(uint8_t sl_addr, uint8_t reg_addr, uint8_t *byte_buffer_receive, uint32_t byte_num);



#endif
