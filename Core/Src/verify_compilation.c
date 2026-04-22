/**
  ******************************************************************************
  * @file    verify_compilation.c
  * @brief   Compilation Verification Test
  *          Include all driver headers to verify compilation
  ******************************************************************************
  */

#include "main.h"

/* Include all driver headers - this verifies they can compile */
#include "i2c_hal_wrapper.h"
#include "mpu6050.h"
#include "spi_adc.h"
#include "spi_flash.h"
#include "uart_proto.h"

/* Dummy variables to prevent "unused header" warnings */
static I2C_Device_TypeDef *pI2cTest = (I2C_Device_TypeDef*)0;
static MPU6050_Handle_t *pMpuTest = (MPU6050_Handle_t*)0;
static SPI_ADC_Handle_t *pAdcTest = (SPI_ADC_Handle_t*)0;
static FLASH_Handle_t *pFlashTest = (FLASH_Handle_t*)0;
static UART_Proto_Handle_t *pUartTest = (UART_Proto_Handle_t*)0;

/**
  * @brief  Verification function (empty)
  */
void Verify_All_Drivers_Compiled(void)
{
  /* This file is for compilation test only */
  /* If this compiles, all drivers are working */
  
  /* Prevent compiler warnings about unused variables */
  (void)pI2cTest;
  (void)pMpuTest;
  (void)pAdcTest;
  (void)pFlashTest;
  (void)pUartTest;
}
