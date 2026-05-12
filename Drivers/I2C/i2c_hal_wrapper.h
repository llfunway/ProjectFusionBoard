/**
  ******************************************************************************
  * @file    i2c_hal_wrapper.h
  * @author  Embedded System Team
  * @brief   I2C HAL Wrapper Header - Unified I2C Interface
  *          Provides simplified I2C read/write operations for STM32 HAL
  ******************************************************************************
  */

#ifndef __I2C_HAL_WRAPPER_H
#define __I2C_HAL_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported Macros -----------------------------------------------------------*/
#define I2C_OK                      ((int8_t)0)    /*!< Operation successful */
#define I2C_ERROR_TIMEOUT           ((int8_t)-1)   /*!< Timeout error */
#define I2C_ERROR_NOACK             ((int8_t)-2)   /*!< No ACK received */
#define I2C_ERROR_BUS               ((int8_t)-3)   /*!< Bus error */
#define I2C_ERROR_PARAM             ((int8_t)-4)   /*!< Parameter error */

#define I2C_DEV_ADDR_MASK           ((uint8_t)0xFE) /*!< Device address mask */

/* Exported Types ------------------------------------------------------------*/
/**
  * @brief  I2C Device Handle Structure
  */
typedef struct
{
  I2C_HandleTypeDef *hi2c;      /*!< Pointer to I2C handle */
  uint8_t DevAddress;            /*!< Device 7-bit address */
  uint32_t Timeout;              /*!< Communication timeout in ms */
} I2C_Device_TypeDef;

/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Initialize I2C device handle
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  hi2c: Pointer to I2C_HandleTypeDef
  * @param  devAddr: Device 7-bit address
  * @param  timeout: Communication timeout in ms
  * @retval I2C status code
  */
int8_t I2C_Device_Init(I2C_Device_TypeDef *hi2c_dev, 
                       I2C_HandleTypeDef *hi2c, 
                       uint8_t devAddr, 
                       uint32_t timeout);

/**
  * @brief  Write a single byte to a register
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  regAddr: Register address to write to
  * @param  data: Byte to write
  * @retval I2C status code
  */
int8_t I2C_WriteByte(I2C_Device_TypeDef *hi2c_dev, 
                     uint8_t regAddr, 
                     uint8_t data);

/**
  * @brief  Read a single byte from a register
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  regAddr: Register address to read from
  * @param  pData: Pointer to store read data
  * @retval I2C status code
  */
int8_t I2C_ReadByte(I2C_Device_TypeDef *hi2c_dev, 
                    uint8_t regAddr, 
                    uint8_t *pData);

/**
  * @brief  Write multiple bytes to consecutive registers
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  regAddr: Starting register address
  * @param  pData: Pointer to data buffer
  * @param  size: Number of bytes to write
  * @retval I2C status code
  */
int8_t I2C_WriteBytes(I2C_Device_TypeDef *hi2c_dev, 
                      uint8_t regAddr, 
                      uint8_t *pData, 
                      uint16_t size);

/**
  * @brief  Read multiple bytes from consecutive registers
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  regAddr: Starting register address
  * @param  pData: Pointer to data buffer
  * @param  size: Number of bytes to read
  * @retval I2C status code
  */
int8_t I2C_ReadBytes(I2C_Device_TypeDef *hi2c_dev, 
                     uint8_t regAddr, 
                     uint8_t *pData, 
                     uint16_t size);

/**
  * @brief  Read a single bit from a register
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  regAddr: Register address
  * @param  bitNum: Bit position (0-7)
  * @param  pData: Pointer to store bit value (0 or 1)
  * @retval I2C status code
  */
int8_t I2C_ReadBit(I2C_Device_TypeDef *hi2c_dev, 
                   uint8_t regAddr, 
                   uint8_t bitNum, 
                   uint8_t *pData);

/**
  * @brief  Read multiple bits from a register
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  regAddr: Register address
  * @param  bitStart: Starting bit position (0-7)
  * @param  length: Number of bits to read (1-8)
  * @param  pData: Pointer to store bits value
  * @retval I2C status code
  */
int8_t I2C_ReadBits(I2C_Device_TypeDef *hi2c_dev, 
                    uint8_t regAddr, 
                    uint8_t bitStart, 
                    uint8_t length, 
                    uint8_t *pData);

/**
  * @brief  Write a single bit to a register
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  regAddr: Register address
  * @param  bitNum: Bit position (0-7)
  * @param  data: Bit value (0 or 1)
  * @retval I2C status code
  */
int8_t I2C_WriteBit(I2C_Device_TypeDef *hi2c_dev, 
                    uint8_t regAddr, 
                    uint8_t bitNum, 
                    uint8_t data);

/**
  * @brief  Check if device is connected and responding
  * @param  hi2c_dev: Pointer to I2C device structure
  * @retval I2C_OK if device responds, error code otherwise
  */
int8_t I2C_CheckDevice(I2C_Device_TypeDef *hi2c_dev);

int8_t I2C_BusRecovery(I2C_Device_TypeDef *hi2c_dev, GPIO_TypeDef *sclPort, uint16_t sclPin);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_HAL_WRAPPER_H */
