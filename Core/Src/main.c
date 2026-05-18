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
#include "mag_residual_filter.h"
#include "uart_proto.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_FREQUENCY_HZ        ((uint32_t)50)   /* Keep <= configured AD7177 ODR (currently 59 SPS). */
#define IMU_FREQUENCY_HZ        ((uint32_t)200)  /* Faster attitude update than ADC loop. */
#define ADC_READ_PERIOD_MS      ((uint32_t)(1000U / ADC_FREQUENCY_HZ))
#define IMU_READ_PERIOD_MS      ((uint32_t)(1000U / IMU_FREQUENCY_HZ))
#define TEST_REPORT_PERIOD_MS   ADC_READ_PERIOD_MS
#define AD7177_DEVICE_COUNT     ((uint8_t)3)
#define MAG_FRONTEND_SCALE      ((float)4.0f)
#define MAG_NT_PER_OE           ((float)100000.0f)

#define AD7177_CS1_PORT         GPIOC
#define AD7177_CS1_PIN          GPIO_PIN_12
#define AD7177_CS2_PORT         GPIOB
#define AD7177_CS2_PIN          GPIO_PIN_3
#define AD7177_CS3_PORT         GPIOB
#define AD7177_CS3_PIN          GPIO_PIN_5

#define STATUS_LED_RUN_PORT     GPIOB
#define STATUS_LED_RUN_PIN      GPIO_PIN_12
#define STATUS_LED_ADC_PORT     GPIOB
#define STATUS_LED_ADC_PIN      GPIO_PIN_13
#define STATUS_LED_IMU_PORT     GPIOB
#define STATUS_LED_IMU_PIN      GPIO_PIN_14
#define STATUS_LED_ERR_PORT     GPIOB
#define STATUS_LED_ERR_PIN      GPIO_PIN_15
#define STATUS_LED_ACTIVE       GPIO_PIN_SET
#define STATUS_LED_INACTIVE     GPIO_PIN_RESET
#define STATUS_LED_RUN_PERIOD_MS       ((uint32_t)500)
#define STATUS_LED_OK_HOLD_MS          ((uint32_t)250)
#define STATUS_LED_ERR_BLINK_MS        ((uint32_t)200)
#define STATUS_LED_ADC_TIMEOUT_LIMIT   ((uint32_t)3)
#define STATUS_LED_IMU_ERROR_LIMIT     ((uint32_t)10)

#define ERR_MPU6050_INIT        ((uint8_t)1)
#define ERR_SPI_ADC_INIT        ((uint8_t)2)
#define ERR_UART_INIT           ((uint8_t)3)
#define MPU6050_STARTUP_DELAY_MS ((uint32_t)100)
#define MPU6050_INIT_RETRY_COUNT ((uint8_t)5)
#define MPU6050_INIT_RETRY_DELAY_MS ((uint32_t)20)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t g_SystemTick = 0;
static volatile uint8_t g_ErrorCode = 0;
volatile uint8_t g_IsInitialized = 0;
volatile uint8_t g_UartReady = 0;
static volatile uint8_t g_SensorDataFresh = 0;
static volatile uint8_t g_ImuDataFresh = 0;
static volatile uint32_t g_ImuConsecutiveErrors = 0;
static volatile uint32_t g_AdcConsecutiveTimeouts = 0;

static MPU6050_Handle_t hMPU6050;
static AD7177_Handle_t hAD7177[AD7177_DEVICE_COUNT];
UART_Proto_Handle_t hUART_Proto;

static uint32_t lastSensorRead = 0;
static uint32_t lastTestReport = 0;
static uint32_t lastImuRead = 0;

static float adc_voltage[AD7177_DEVICE_COUNT] = {0.0f};
static float mag_nt[AD7177_DEVICE_COUNT] = {0.0f};
static float imu_roll = 0.0f;
static float imu_pitch = 0.0f;
static float imu_temperature = 0.0f;
static float vertical_proj_raw = 0.0f;
static float vertical_proj = 0.0f;
static MagResidualFilter_t mag_residual_filter;
static const float mag_sensitivity_v_per_oe[AD7177_DEVICE_COUNT] = {
  10.3904f,
  10.5235f,
  10.9590f
};
static const float mag_zero_offset_v[AD7177_DEVICE_COUNT] = {
  -0.1350f,
  -0.1949f,
  -0.1795f
};
static int8_t mpu_init_status = MPU6050_OK;
static uint8_t mpu_init_address = 0;
static uint8_t mpu_init_device_id = 0;
static int8_t mpu_probe_status[2] = {MPU6050_ERROR, MPU6050_ERROR};
static uint8_t mpu_probe_device_id[2] = {0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static void Init_System_Tick(void);
static uint8_t Init_All_Drivers(void);
static void Process_Sensor_Data(void);
static void Process_Test_Report(void);
static void Handle_Init_Error(uint8_t error_code);
static void UART_SendString(const char *str);
static void UART_SendFormattedBuffer(const char *buf, size_t buf_size, int len, uint32_t timeout);
static void UART_SendMpuInitDebug(void);
static void UART_SendI2CBusScan(void);
static void UART_SendDataHeader(void);
static void Update_Status_LEDs(uint8_t init_error_code);
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

static void UART_SendFormattedBuffer(const char *buf, size_t buf_size, int len, uint32_t timeout)
{
  if ((buf != NULL) && (len > 0) && ((size_t)len < buf_size))
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, timeout);
  }
}

static void UART_SendFrequencyInfo(void)
{
  char txBuf[48];
  int len = snprintf(txBuf,
                     sizeof(txBuf),
                     "frequency=%luHz\r\n",
                     (unsigned long)ADC_FREQUENCY_HZ);

  UART_SendFormattedBuffer(txBuf, sizeof(txBuf), len, 100);
}

static void UART_SendDataHeader(void)
{
  UART_SendString("{text}b1_nT,b2_nT,b3_nT,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,stm32_roll_deg,stm32_pitch_deg,vertical_raw_nT,vertical_corr_nT\r\n");
}

static void UART_SendMpuInitDebug(void)
{
  char txBuf[128];
  int len = snprintf(txBuf,
                     sizeof(txBuf),
                     "MPU_INIT status=%d addr=0x%02X whoami=0x%02X probe[D0]=%d/0x%02X probe[D2]=%d/0x%02X\r\n",
                     (int)mpu_init_status,
                     mpu_init_address,
                     mpu_init_device_id,
                     (int)mpu_probe_status[0],
                     mpu_probe_device_id[0],
                     (int)mpu_probe_status[1],
                     mpu_probe_device_id[1]);

  UART_SendFormattedBuffer(txBuf, sizeof(txBuf), len, 100);
}

static void UART_SendI2CBusScan(void)
{
  char txBuf[160];
  int len = snprintf(txBuf, sizeof(txBuf), "I2C_SCAN:");
  uint8_t found = 0;

  for (uint8_t address = 0x08; address < 0x78; address++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 2, 20) == HAL_OK)
    {
      if (len > 0 && len < (int)(sizeof(txBuf) - 8))
      {
        len += snprintf(&txBuf[len], sizeof(txBuf) - (size_t)len, " 0x%02X", address);
        found = 1;
      }
    }
  }

  if (!found && len > 0 && len < (int)(sizeof(txBuf) - 5))
  {
    len += snprintf(&txBuf[len], sizeof(txBuf) - (size_t)len, " none");
  }

  if (len > 0 && len < (int)(sizeof(txBuf) - 3))
  {
    len += snprintf(&txBuf[len], sizeof(txBuf) - (size_t)len, "\r\n");
    UART_SendFormattedBuffer(txBuf, sizeof(txBuf), len, 200);
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
  g_UartReady = 1;

  Init_System_Tick();
  UART_SendString("BOOT\r\n");

  if (Init_All_Drivers() != 0)
  {
    while (1)
    {
      Handle_Init_Error(g_ErrorCode);
    }
  }

  imu_roll = hMPU6050.Attitude.Roll;
  imu_pitch = hMPU6050.Attitude.Pitch;
  imu_temperature = hMPU6050.Data.Temp;
  MagResidualFilter_Init(&mag_residual_filter);

  g_IsInitialized = 1;
  (void)imu_temperature;
  UART_SendFrequencyInfo();
  UART_SendString("3xAD7177 test start\r\n");
  UART_SendDataHeader();

  while (1)
  {
    if ((g_SystemTick - lastImuRead) >= IMU_READ_PERIOD_MS)
    {
      lastImuRead = g_SystemTick;

      if ((MPU6050_ReadRawData(&hMPU6050) == MPU6050_OK) &&
          (MPU6050_UpdateAttitude(&hMPU6050) == MPU6050_OK))
      {
        imu_roll = hMPU6050.Attitude.Roll;
        imu_pitch = hMPU6050.Attitude.Pitch;
        imu_temperature = hMPU6050.Data.Temp;
        g_ImuDataFresh = 1;
        g_ImuConsecutiveErrors = 0;
      }
      else
      {
        g_ImuConsecutiveErrors++;
        g_ImuDataFresh = 0;
      }
    }

    if ((g_SystemTick - lastSensorRead) >= ADC_READ_PERIOD_MS)
    {
      float adc_voltage_next[AD7177_DEVICE_COUNT];
      uint8_t all_adc_fresh = 1;

      lastSensorRead = g_SystemTick;
      g_SensorDataFresh = 0;

      for (uint8_t i = 0; i < AD7177_DEVICE_COUNT; i++)
      {
        if ((AD7177_ReadData(&hAD7177[i], 0, &hAD7177[i].RawData[0]) != AD7177_OK) ||
            (AD7177_GetVoltage(&hAD7177[i], 0, &adc_voltage_next[i]) != AD7177_OK))
        {
          all_adc_fresh = 0;
        }
      }

      if (all_adc_fresh)
      {
        for (uint8_t i = 0; i < AD7177_DEVICE_COUNT; i++)
        {
          adc_voltage[i] = adc_voltage_next[i];
        }
        g_SensorDataFresh = 1;
        g_AdcConsecutiveTimeouts = 0;
      }
      else
      {
        g_AdcConsecutiveTimeouts++;
      }
    }

    if (g_SensorDataFresh)
    {
      Process_Sensor_Data();
      g_SensorDataFresh = 0;
      g_ImuDataFresh = 0;
    }
    else if (g_ImuDataFresh)
    {
      g_ImuDataFresh = 0;
    }

    if ((g_SystemTick - lastTestReport) >= TEST_REPORT_PERIOD_MS)
    {
      lastTestReport = g_SystemTick;
      Process_Test_Report();
    }

    Update_Status_LEDs(0);
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
  MPU6050_Config_t mpu_config = {
    .hi2c = &hi2c1,
    .I2cAddress = 0,
    .AccelRange = MPU6050_ACCEL_RANGE_2G,
    .GyroRange = MPU6050_GYRO_RANGE_500DPS,
    .DlpfBandwidth = MPU6050_DLPF_BW_42HZ,
    .SampleRateDiv = 4,
    .Timeout = 100
  };
  const uint8_t mpu_probe_addresses[] = {
    (uint8_t)(MPU6050_ADDR_AD0_LOW << 1),
    (uint8_t)(MPU6050_ADDR_AD0_HIGH << 1)
  };
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

  mpu_init_status = MPU6050_ERROR;
  mpu_init_address = 0;
  mpu_init_device_id = 0;
  mpu_probe_status[0] = MPU6050_ERROR;
  mpu_probe_status[1] = MPU6050_ERROR;
  mpu_probe_device_id[0] = 0;
  mpu_probe_device_id[1] = 0;

  HAL_Delay(MPU6050_STARTUP_DELAY_MS);

  for (uint8_t i = 0; i < (sizeof(mpu_probe_addresses) / sizeof(mpu_probe_addresses[0])); i++)
  {
    for (uint8_t retry = 0; retry < MPU6050_INIT_RETRY_COUNT; retry++)
    {
      mpu_config.I2cAddress = mpu_probe_addresses[i];
      mpu_init_address = mpu_probe_addresses[i];
      mpu_init_status = MPU6050_Init(&hMPU6050, &mpu_config);
      mpu_init_device_id = hMPU6050.DeviceID;
      mpu_probe_status[i] = mpu_init_status;
      mpu_probe_device_id[i] = hMPU6050.DeviceID;

      if (mpu_init_status == MPU6050_OK)
      {
        break;
      }

      HAL_Delay(MPU6050_INIT_RETRY_DELAY_MS);
    }

    if (mpu_init_status == MPU6050_OK)
    {
      break;
    }
  }

  if (mpu_init_status != MPU6050_OK)
  {
    g_ErrorCode = ERR_MPU6050_INIT;
    return g_ErrorCode;
  }

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
  float roll_rad;
  float pitch_rad;

  if (isnan(imu_roll) || isinf(imu_roll) ||
      isnan(imu_pitch) || isinf(imu_pitch))
  {
    roll_rad = 0.0f;
    pitch_rad = 0.0f;
  }
  else
  {
    roll_rad = imu_roll * ((float)M_PI / 180.0f);
    pitch_rad = imu_pitch * ((float)M_PI / 180.0f);
  }

  const float vertical_unit[3] = {
    -sinf(pitch_rad),
    sinf(roll_rad) * cosf(pitch_rad),
    cosf(roll_rad) * cosf(pitch_rad)
  };

  for (uint8_t i = 0; i < AD7177_DEVICE_COUNT; i++)
  {
    float sensor_voltage;

    if (isnan(adc_voltage[i]) || isinf(adc_voltage[i]))
    {
      adc_voltage[i] = 0.0f;
    }

    sensor_voltage = adc_voltage[i] * MAG_FRONTEND_SCALE;
    mag_nt[i] = ((sensor_voltage - mag_zero_offset_v[i]) /
                 mag_sensitivity_v_per_oe[i]) * MAG_NT_PER_OE;
  }

  vertical_proj_raw = (mag_nt[0] * vertical_unit[0]) +
                      (mag_nt[1] * vertical_unit[1]) +
                      (mag_nt[2] * vertical_unit[2]);

  if (isnan(vertical_proj_raw) || isinf(vertical_proj_raw))
  {
    vertical_proj_raw = 0.0f;
  }

  vertical_proj = MagResidualFilter_Update(&mag_residual_filter,
                                           mag_nt,
                                           vertical_unit,
                                           vertical_proj_raw);
}

static void Process_Test_Report(void)
{
  char txBuf[280];
  int len = snprintf(
      txBuf,
      sizeof(txBuf),
      "{plotter}%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
      mag_nt[0],
      mag_nt[1],
      mag_nt[2],
      hMPU6050.Data.AccelX,
      hMPU6050.Data.AccelY,
      hMPU6050.Data.AccelZ,
      hMPU6050.Data.GyroX,
      hMPU6050.Data.GyroY,
      hMPU6050.Data.GyroZ,
      hMPU6050.Attitude.Roll,
      hMPU6050.Attitude.Pitch,
      vertical_proj_raw,
      vertical_proj);

  UART_SendFormattedBuffer(txBuf, sizeof(txBuf), len, 100);
}

static void Handle_Init_Error(uint8_t error_code)
{
  char txBuf[48];
  int len = snprintf(txBuf, sizeof(txBuf), "INIT_ERROR:%u\r\n", error_code);

  UART_SendFormattedBuffer(txBuf, sizeof(txBuf), len, 100);

  if (error_code == ERR_MPU6050_INIT)
  {
    UART_SendMpuInitDebug();
    UART_SendI2CBusScan();
  }

  for (uint8_t i = 0; i < 10; i++)
  {
    Update_Status_LEDs(error_code);
    HAL_Delay(50);
  }
}

static void Update_Status_LEDs(uint8_t init_error_code)
{
  (void)init_error_code;
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
  GPIO_InitTypeDefdrift removal GPIO_InitStruct = {0};

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
  if (g_UartReady)
  {
    UART_SendString("FATAL_ERROR\r\n");
  }
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
