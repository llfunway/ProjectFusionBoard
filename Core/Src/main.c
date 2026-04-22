/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - 3x AD7177 test
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI    ((float)3.14159265358979323846f)
#endif

#include "i2c_hal_wrapper.h"
#include "mpu6050.h"
#include "ad7177.h"
#include "uart_proto.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSTEM_FREQUENCY_HZ     ((uint32_t)50)   /* Keep <= configured AD7177 ODR (currently 59 SPS). */
#define SENSOR_READ_PERIOD_MS   ((uint32_t)(1000U / SYSTEM_FREQUENCY_HZ))
#define TEST_REPORT_PERIOD_MS   SENSOR_READ_PERIOD_MS
#define AD7177_DEVICE_COUNT     ((uint8_t)3)

#define AD7177_CS1_PORT         GPIOC
#define AD7177_CS1_PIN          GPIO_PIN_12
#define AD7177_CS2_PORT         GPIOB
#define AD7177_CS2_PIN          GPIO_PIN_3
#define AD7177_CS3_PORT         GPIOB
#define AD7177_CS3_PIN          GPIO_PIN_5

#define ERR_SPI_ADC_INIT        ((uint8_t)2)
#define ERR_UART_INIT           ((uint8_t)3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t g_SystemTick = 0;
static volatile uint8_t g_ErrorCode = 0;
volatile uint8_t g_IsInitialized = 0;

static AD7177_Handle_t hAD7177[AD7177_DEVICE_COUNT];
UART_Proto_Handle_t hUART_Proto;

static uint32_t lastSensorRead = 0;
static uint32_t lastTestReport = 0;

static float adc_voltage[AD7177_DEVICE_COUNT] = {0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static void Init_System_Tick(void);
static uint8_t Init_All_Drivers(void);
static void Process_Sensor_Data(void);
static void Process_Test_Report(void);
static void Handle_Init_Error(uint8_t error_code);
static void System_Status_Update(void);
static void UART_SendString(const char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_SendString(const char *str)
{
  if (str != NULL)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), 100);
  }
}

static void UART_SendFrequencyInfo(void)
{
  char txBuf[48];
  int len = snprintf(txBuf,
                     sizeof(txBuf),
                     "frequency=%luHz\r\n",
                     (unsigned long)SYSTEM_FREQUENCY_HZ);

  if (len > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, (uint16_t)len, 100);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  Init_System_Tick();

  if (Init_All_Drivers() != 0)
  {
    while (1)
    {
      Handle_Init_Error(g_ErrorCode);
    }
  }

  g_IsInitialized = 1;
  UART_SendFrequencyInfo();
  UART_SendString("3xAD7177 test start\r\n");

  while (1)
  {
    if ((g_SystemTick - lastSensorRead) >= SENSOR_READ_PERIOD_MS)
    {
      lastSensorRead = g_SystemTick;

      /* MPU6050 test path is temporarily disabled during 3-channel AD7177 bring-up. */

      for (uint8_t i = 0; i < AD7177_DEVICE_COUNT; i++)
      {
        if (AD7177_ReadData(&hAD7177[i], 0, &hAD7177[i].RawData[0]) == AD7177_OK)
        {
          AD7177_GetVoltage(&hAD7177[i], 0, &adc_voltage[i]);
        }
      }
    }

    Process_Sensor_Data();

    if ((g_SystemTick - lastTestReport) >= TEST_REPORT_PERIOD_MS)
    {
      lastTestReport = g_SystemTick;
      Process_Test_Report();
    }

    System_Status_Update();
    HAL_Delay(1);
  }
}

/* USER CODE BEGIN 4 */
static void Init_System_Tick(void)
{
  g_SystemTick = 0;
}

static uint8_t Init_All_Drivers(void)
{
  AD7177_Config_t adc_config = {
    .hspi = &hspi1,
    .CsPort = AD7177_CS1_PORT,
    .CsPin = AD7177_CS1_PIN,
    .CrcMode = AD7177_CRC_DISABLE,
    .Timeout = 100,
    .ReferenceVoltage_mV = 5000.0f,
    .UseExternalRef = 1
  };

  if (huart1.Instance != USART1)
  {
    g_ErrorCode = ERR_UART_INIT;
    return g_ErrorCode;
  }

  /* MPU6050 initialization is temporarily disabled during 3-channel AD7177 bring-up. */

  adc_config.Channels[0].Channel = 0;
  adc_config.Channels[0].AinPos = 3;
  adc_config.Channels[0].AinNeg = 4;
  adc_config.Channels[0].SetupSel = 0;
  adc_config.Channels[0].Enabled = 1;

  adc_config.Setups[0].Bipolar = 1;
  adc_config.Setups[0].RefSel = 0;
  /* Keep ADC ODR slightly above the system loop rate so each cycle can fetch fresh data. */
  adc_config.Setups[0].Odr = AD7177_ODR_59SPS;
  adc_config.Setups[0].FilterOrder = 0;

  if (AD7177_Init(&hAD7177[0], &adc_config) != AD7177_OK)
  {
    g_ErrorCode = ERR_SPI_ADC_INIT;
    return g_ErrorCode;
  }

  adc_config.CsPort = AD7177_CS2_PORT;
  adc_config.CsPin = AD7177_CS2_PIN;
  if (AD7177_Init(&hAD7177[1], &adc_config) != AD7177_OK)
  {
    g_ErrorCode = ERR_SPI_ADC_INIT;
    return g_ErrorCode;
  }

  adc_config.CsPort = AD7177_CS3_PORT;
  adc_config.CsPin = AD7177_CS3_PIN;
  if (AD7177_Init(&hAD7177[2], &adc_config) != AD7177_OK)
  {
    g_ErrorCode = ERR_SPI_ADC_INIT;
    return g_ErrorCode;
  }

  return 0;
}

static void Process_Sensor_Data(void)
{
  /* Reserved for future threshold/event logic during bring-up. */
}

static void Process_Test_Report(void)
{
  char txBuf[128];
  int len = snprintf(
      txBuf,
      sizeof(txBuf),
      "{plotter}%.6f,%.6f,%.6f\r\n",
      adc_voltage[0],
      adc_voltage[1],
      adc_voltage[2]);

  if (len > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, (uint16_t)len, 100);
  }
}

static void Handle_Init_Error(uint8_t error_code)
{
  char txBuf[48];
  int len = snprintf(txBuf, sizeof(txBuf), "INIT_ERROR:%u\r\n", error_code);

  if (len > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, (uint16_t)len, 100);
  }

  HAL_Delay(500);
}

static void System_Status_Update(void)
{
  /* Keep hook for future board-level status management. */
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin = AD7177_CS1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AD7177_CS1_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(AD7177_CS1_PORT, AD7177_CS1_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = AD7177_CS2_PIN;
  HAL_GPIO_Init(AD7177_CS2_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(AD7177_CS2_PORT, AD7177_CS2_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = AD7177_CS3_PIN;
  HAL_GPIO_Init(AD7177_CS3_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(AD7177_CS3_PORT, AD7177_CS3_PIN, GPIO_PIN_SET);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function (for AD7177 ADC)
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;

  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    UART_SendString("FATAL_ERROR\r\n");
    HAL_Delay(200);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
