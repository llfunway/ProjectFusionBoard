/**
  ******************************************************************************
  * @file    test_drivers.c
  * @author  Embedded System Team
  * @brief   Driver Compilation Test File
  *          Verify all drivers can be compiled successfully
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private Variables ---------------------------------------------------------*/
static uint8_t test_counter = 0;

/* Test Variables for Each Driver --------------------------------------------*/

/* I2C Wrapper Test */
#ifdef TEST_I2C_WRAPPER
I2C_Device_TypeDef i2c_test_device;
#endif

/* MPU6050 Test */
#ifdef TEST_MPU6050
MPU6050_Handle_t mpu_test_handle;
MPU6050_Config_t mpu_test_config;
#endif

/* SPI ADC Test */
#ifdef TEST_SPI_ADC
SPI_ADC_Handle_t adc_test_handle;
SPI_ADC_Config_t adc_test_config;
#endif

/* SPI Flash Test */
#ifdef TEST_SPI_FLASH
FLASH_Handle_t flash_test_handle;
FLASH_Config_t flash_test_config;
#endif

/* UART Protocol Test */
#ifdef TEST_UART_PROTO
UART_Proto_Handle_t uart_test_handle;
UART_Proto_Config_t uart_test_config;
#endif

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Test entry point (not called by default)
  */
void Test_All_Drivers(void)
{
  test_counter++;
  
  /* Add your test code here */
  /* This function is for compilation test only */
}

/**
  * @brief  Get test counter
  */
uint8_t Get_Test_Counter(void)
{
  return test_counter;
}
