/**
 * @file  i2c.h
 * @brief Declare the three sensor-bus interfaces and their transfer result codes.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#ifndef __I2C_H__
#define __I2C_H__

#include <stdio.h>
#include "stm32f4xx.h"

#include "COMPONENTS/PEO_PINMAP.H"

#include "COMPONENTS/DELAY/include/delay.h"



/* I2C1/2/3 are on APB1, whose bus clock is 42 MHz in this 168 MHz design.
 * I2C_Timeout limits polling iterations per event, not milliseconds. */
/* Maximum polling iterations for each I2C event; not a duration in milliseconds. */
#define I2C_Timeout                         (uint16_t ) 500

#define NO_ERROR                            0  /* Successful register transfer; all I2C adapters return zero on success. */
#define ERROR_EV5                           5  /* Timed out waiting for the controller to enter master mode after START. */
#define ERROR_EV6                           6  /* Timed out during address selection (also used by the VL register-index stage). */
#define ERROR_EV7                           7  /* Timed out waiting for a received data byte. */
#define ERROR_EV8                           8  /* Timed out waiting for a transmitted byte/register-index stage. */
#define ERROR_EV8_2                         82  /* Timed out waiting for the final transmitted byte to complete. */
#define ERROR_BUSY                          9  /* Bus stayed busy beyond the polling limit. */

#define I2C_Fail                            0xCE  /* Legacy generic I2C failure marker; event-specific functions return ERROR_* instead. */
#define I2C_Success                         0x00  /* Legacy generic success marker; equivalent to NO_ERROR. */


// I2C1 - MPU6050
/**
 * @brief Prepare I2C1 on PB6/PB7 for sensor register transfers.
 *
 * Called during MPU6050 motion driver initialization. Configures open-drain
 * alternate-function pins and the bus speed; RCC must already have enabled the clocks.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void I2C_MPU_Init(void);
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
uint32_t I2C_MPU_WriteBytes(unsigned char slave_addr, unsigned char addr, unsigned char byteNum, unsigned char const *data);
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
uint32_t I2C_MPU_ReadBytes(unsigned char slave_addr, unsigned char addr, unsigned char byteNum, unsigned char *data);


// I2C3 - TOF400C (VL53L1X only)
/**
 * @brief Prepare I2C3 on PA8/PC9 for sensor register transfers.
 *
 * Called during VL53L1X ranging platform adapter initialization. Configures open-drain
 * alternate-function pins and the bus speed; RCC must already have enabled the clocks.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void I2C_TOF_Init(void);
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
uint8_t I2C_TOF_WriteByte(uint8_t sl_addr, uint16_t reg_addr, uint8_t byte_write);
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
uint8_t I2C_TOF_ReadByte(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_receive);
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
uint8_t I2C_TOF_WriteBytes(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_buffer_write, uint32_t byte_num);
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
uint8_t I2C_TOF_ReadBytes(uint8_t sl_addr, uint16_t reg_addr, uint8_t *byte_buffer_receive, uint32_t byte_num);


// I2C2 - INA3221 (8-bit register address, 7-bit slave addr left-shifted inside)
/**
 * @brief Prepare I2C2 on PB10/PB11 for sensor register transfers.
 *
 * Called during INA3221 current-monitor driver initialization. Configures open-drain
 * alternate-function pins and the bus speed; RCC must already have enabled the clocks.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void I2C_INA_Init(void);
/**
 * @brief Write a block of sensor register bytes using I2C2.
 *
 * Used by the INA3221 current-monitor driver. This is a blocking, polled transfer with a
 * bounded loop count at each bus event. It writes the register address followed by the
 * data, then requests STOP.
 * @param sl_addr Unshifted 7-bit device address; this function shifts it left once for
 *                the ST library.
 * @param reg_addr 8-bit register address at which the transfer begins.
 * @param byte_buffer_write Readable source with at least byte_num bytes.
 * @param byte_num Number of data bytes to transfer; callers provide a positive count.
 * @retval NO_ERROR The transfer completed (zero).
 * @retval nonzero An ERROR_BUSY or ERROR_EV* code identifies the bus stage that timed
 *                 out.
 */
uint8_t I2C_INA_WriteBytes(uint8_t sl_addr, uint8_t reg_addr, uint8_t *byte_buffer_write, uint32_t byte_num);
/**
 * @brief Read a block of sensor register bytes using I2C2.
 *
 * Used by the INA3221 current-monitor driver. This is a blocking, polled transfer with a
 * bounded loop count at each bus event. It first writes the register address, then
 * issues a repeated START to receive data.
 * @param sl_addr Unshifted 7-bit device address; this function shifts it left once for
 *                the ST library.
 * @param reg_addr 8-bit register address at which the transfer begins.
 * @param byte_buffer_receive Writable destination of at least byte_num bytes; failure
 *                            may leave partial data.
 * @param byte_num Number of data bytes to transfer; callers provide a positive count.
 * @retval NO_ERROR The transfer completed (zero).
 * @retval nonzero An ERROR_BUSY or ERROR_EV* code identifies the bus stage that timed
 *                 out.
 */
uint8_t I2C_INA_ReadBytes(uint8_t sl_addr, uint8_t reg_addr, uint8_t *byte_buffer_receive, uint32_t byte_num);



#endif
