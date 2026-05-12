/**
  ******************************************************************************
  * @file    i2c_hal_wrapper.c
  * @author  Embedded System Team
  * @brief   I2C HAL Wrapper Implementation
  *          Simplified I2C operations for STM32 HAL library
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c_hal_wrapper.h"
#include <stdlib.h>
#include <string.h>

/* Private Macros ------------------------------------------------------------*/

/* Private Functions Prototypes ----------------------------------------------*/
static uint8_t I2C_WaitForFlag(I2C_HandleTypeDef *hi2c, uint32_t Flag, 
                                FlagStatus Status, uint32_t Timeout);

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initialize I2C device handle
  */
int8_t I2C_Device_Init(I2C_Device_TypeDef *hi2c_dev, 
                       I2C_HandleTypeDef *hi2c, 
                       uint8_t devAddr, 
                       uint32_t timeout)
{
  if (hi2c_dev == NULL || hi2c == NULL)
  {
    return I2C_ERROR_PARAM;
  }
  
  hi2c_dev->hi2c = hi2c;
  hi2c_dev->DevAddress = (devAddr & I2C_DEV_ADDR_MASK);
  hi2c_dev->Timeout = timeout;
  
  return I2C_CheckDevice(hi2c_dev);
}

/**
  * @brief  Write a single byte to a register
  */
int8_t I2C_WriteByte(I2C_Device_TypeDef *hi2c_dev, 
                     uint8_t regAddr, 
                     uint8_t data)
{
  if (hi2c_dev == NULL || hi2c_dev->hi2c == NULL)
  {
    return I2C_ERROR_PARAM;
  }

  if (HAL_I2C_Mem_Write(hi2c_dev->hi2c,
                        hi2c_dev->DevAddress,
                        regAddr,
                        I2C_MEMADD_SIZE_8BIT,
                        &data,
                        1,
                        hi2c_dev->Timeout) != HAL_OK)
  {
    return I2C_ERROR_TIMEOUT;
  }
  
  return I2C_OK;
}

/**
  * @brief  Read a single byte from a register
  */
int8_t I2C_ReadByte(I2C_Device_TypeDef *hi2c_dev, 
                    uint8_t regAddr, 
                    uint8_t *pData)
{
  if (hi2c_dev == NULL || hi2c_dev->hi2c == NULL || pData == NULL)
  {
    return I2C_ERROR_PARAM;
  }
  
  if (HAL_I2C_Mem_Read(hi2c_dev->hi2c,
                       hi2c_dev->DevAddress,
                       regAddr,
                       I2C_MEMADD_SIZE_8BIT,
                       pData,
                       1,
                       hi2c_dev->Timeout) != HAL_OK)
  {
    return I2C_ERROR_TIMEOUT;
  }
  
  return I2C_OK;
}

/**
  * @brief  Write multiple bytes to consecutive registers
  */
int8_t I2C_WriteBytes(I2C_Device_TypeDef *hi2c_dev, 
                      uint8_t regAddr, 
                      uint8_t *pData, 
                      uint16_t size)
{
  if (hi2c_dev == NULL || hi2c_dev->hi2c == NULL || pData == NULL || size == 0)
  {
    return I2C_ERROR_PARAM;
  }

  if (HAL_I2C_Mem_Write(hi2c_dev->hi2c,
                        hi2c_dev->DevAddress,
                        regAddr,
                        I2C_MEMADD_SIZE_8BIT,
                        pData,
                        size,
                        hi2c_dev->Timeout) != HAL_OK)
  {
    return I2C_ERROR_TIMEOUT;
  }

  return I2C_OK;
}

/**
  * @brief  Read multiple bytes from consecutive registers
  */
int8_t I2C_ReadBytes(I2C_Device_TypeDef *hi2c_dev, 
                     uint8_t regAddr, 
                     uint8_t *pData, 
                     uint16_t size)
{
  if (hi2c_dev == NULL || hi2c_dev->hi2c == NULL || pData == NULL || size == 0)
  {
    return I2C_ERROR_PARAM;
  }
  
  if (HAL_I2C_Mem_Read(hi2c_dev->hi2c,
                       hi2c_dev->DevAddress,
                       regAddr,
                       I2C_MEMADD_SIZE_8BIT,
                       pData,
                       size,
                       hi2c_dev->Timeout) != HAL_OK)
  {
    return I2C_ERROR_TIMEOUT;
  }
  
  return I2C_OK;
}

/**
  * @brief  Read a single bit from a register
  */
int8_t I2C_ReadBit(I2C_Device_TypeDef *hi2c_dev, 
                   uint8_t regAddr, 
                   uint8_t bitNum, 
                   uint8_t *pData)
{
  uint8_t byte;
  int8_t ret;
  
  ret = I2C_ReadByte(hi2c_dev, regAddr, &byte);
  if (ret != I2C_OK)
  {
    return ret;
  }
  
  *pData = (byte >> bitNum) & 0x01;
  return I2C_OK;
}

/**
  * @brief  Read multiple bits from a register
  */
int8_t I2C_ReadBits(I2C_Device_TypeDef *hi2c_dev, 
                    uint8_t regAddr, 
                    uint8_t bitStart, 
                    uint8_t length, 
                    uint8_t *pData)
{
  uint8_t byte;
  int8_t ret;
  
  if (length > 8 || bitStart >= 8)
  {
    return I2C_ERROR_PARAM;
  }
  
  ret = I2C_ReadByte(hi2c_dev, regAddr, &byte);
  if (ret != I2C_OK)
  {
    return ret;
  }
  
  /* Mask and shift */
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  *pData = (byte & mask) >> (bitStart - length + 1);
  
  return I2C_OK;
}

/**
  * @brief  Write a single bit to a register
  */
int8_t I2C_WriteBit(I2C_Device_TypeDef *hi2c_dev, 
                    uint8_t regAddr, 
                    uint8_t bitNum, 
                    uint8_t data)
{
  uint8_t byte;
  int8_t ret;
  
  ret = I2C_ReadByte(hi2c_dev, regAddr, &byte);
  if (ret != I2C_OK)
  {
    return ret;
  }
  
  /* Modify specific bit */
  if (data)
  {
    byte |= (1 << bitNum);
  }
  else
  {
    byte &= ~(1 << bitNum);
  }
  
  return I2C_WriteByte(hi2c_dev, regAddr, byte);
}

/**
  * @brief  Check if device is connected and responding
  */
int8_t I2C_CheckDevice(I2C_Device_TypeDef *hi2c_dev)
{
  if (hi2c_dev == NULL || hi2c_dev->hi2c == NULL)
  {
    return I2C_ERROR_PARAM;
  }
  
  if (HAL_I2C_IsDeviceReady(hi2c_dev->hi2c, 
                             hi2c_dev->DevAddress, 
                             2, 
                             100) != HAL_OK)
  {
    return I2C_ERROR_NOACK;
  }
  
  return I2C_OK;
}

/* Public Functions — Bus Recovery -------------------------------------------*/

/**
  * @brief  Recover I2C bus from stuck-BUSY condition (STM32F1 errata).
  *         Toggles SCL to flush any held slave state, then resets the peripheral.
  * @param  hi2c_dev: Pointer to I2C device structure
  * @param  sclPort: GPIO port for SCL pin
  * @param  sclPin:  GPIO pin for SCL
  * @retval I2C_OK on success, error code otherwise
  */
int8_t I2C_BusRecovery(I2C_Device_TypeDef *hi2c_dev, GPIO_TypeDef *sclPort, uint16_t sclPin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hi2c_dev == NULL || hi2c_dev->hi2c == NULL || sclPort == NULL)
  {
    return I2C_ERROR_PARAM;
  }

  /* Disable the I2C peripheral */
  hi2c_dev->hi2c->Instance->CR1 &= ~I2C_CR1_PE;
  HAL_Delay(2);

  /* Configure SCL as open-drain output and toggle it to flush stuck slaves */
  GPIO_InitStruct.Pin = sclPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(sclPort, &GPIO_InitStruct);

  for (uint8_t i = 0; i < 16; i++)
  {
    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  /* Re-enable the I2C peripheral — the application must re-init via HAL afterwards */
  hi2c_dev->hi2c->Instance->CR1 |= I2C_CR1_PE;
  HAL_Delay(2);

  /* Restore SCL to its alternate-function role */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  HAL_GPIO_Init(sclPort, &GPIO_InitStruct);

  return I2C_OK;
}

/* Private Functions ---------------------------------------------------------*/

/**
  * @brief  Wait for I2C flag with timeout
  */
static uint8_t I2C_WaitForFlag(I2C_HandleTypeDef *hi2c, uint32_t Flag,
                                FlagStatus Status, uint32_t Timeout)
{
  uint32_t tickstart = HAL_GetTick();

  while ((__HAL_I2C_GET_FLAG(hi2c, Flag) ? SET : RESET) != Status)
  {
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((HAL_GetTick() - tickstart) > Timeout)
      {
        return 0; /* Timeout */
      }
    }
  }

  return 1; /* Success */
}
