/**
  ******************************************************************************
  * @file    spi_adc.c
  * @author  Embedded System Team
  * @brief   SPI ADC Driver Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi_adc.h"

/* Private Macros ------------------------------------------------------------*/
#define ADC_CS_LOW()    HAL_GPIO_WritePin(hadc->Config.CsPort, hadc->Config.CsPin, GPIO_PIN_RESET)
#define ADC_CS_HIGH()   HAL_GPIO_WritePin(hadc->Config.CsPort, hadc->Config.CsPin, GPIO_PIN_SET)

#define ADS1256_CMD_RDATA       ((uint8_t)0x01)
#define ADS1256_CMD_RDATAC      ((uint8_t)0x03)
#define ADS1256_CMD_SDATAC      ((uint8_t)0x0F)

#define MCP3208_CMD_SINGLE      ((uint8_t)0x08)
#define MCP3208_CMD_DIFF        ((uint8_t)0x00)

/* Private Function Prototypes -----------------------------------------------*/
static void SPI_ADC_DelayUs(uint32_t delay);
static SPI_ADC_Status_t SPI_ADC_SelectChannel(SPI_ADC_Handle_t *hadc, uint8_t channel);

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initialize SPI ADC driver
  */
SPI_ADC_Status_t SPI_ADC_Init(SPI_ADC_Handle_t *hadc, SPI_ADC_Config_t *pConfig)
{
  if (hadc == NULL || pConfig == NULL)
  {
    return SPI_ADC_ERROR_PARAM;
  }
  
  if (pConfig->hspi == NULL)
  {
    return SPI_ADC_ERROR_PARAM;
  }
  
  hadc->Config = *pConfig;
  hadc->ReferenceVoltage_mV = 3300;
  hadc->IsConverting = 0;
  
  for (uint8_t i = 0; i < ADC_MAX_CHANNELS; i++)
  {
    hadc->RawData[i] = 0;
    hadc->VoltageData[i] = 0.0f;
    hadc->Config.Channels[i].Channel = i;
    hadc->Config.Channels[i].IsEnabled = 0;
  }
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = hadc->Config.CsPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(hadc->Config.CsPort, &GPIO_InitStruct);
  
  ADC_CS_HIGH();
  
  SPI_ADC_DelayUs(10);
  
  hadc->IsInitialized = 1;
  
  return SPI_ADC_OK;
}

/**
  * @brief  Read single channel ADC value
  */
SPI_ADC_Status_t SPI_ADC_ReadChannel(SPI_ADC_Handle_t *hadc, uint8_t channel, uint32_t *pRawValue)
{
  if (hadc == NULL || !hadc->IsInitialized)
  {
    return SPI_ADC_ERROR_NOT_INIT;
  }
  
  if (channel >= ADC_MAX_CHANNELS || pRawValue == NULL)
  {
    return SPI_ADC_ERROR_PARAM;
  }
  
  uint8_t txBuffer[3] = {0};
  uint8_t rxBuffer[3] = {0};
  
  if (hadc->Config.Resolution == ADC_RESOLUTION_24BIT)
  {
    txBuffer[0] = ADS1256_CMD_RDATA;
    
    ADC_CS_LOW();
    SPI_ADC_DelayUs(1);
    
    if (HAL_SPI_TransmitReceive(hadc->Config.hspi, txBuffer, rxBuffer, 1, hadc->Config.Timeout) != HAL_OK)
    {
      ADC_CS_HIGH();
      return SPI_ADC_ERROR_TIMEOUT;
    }
    
    if (HAL_SPI_Receive(hadc->Config.hspi, rxBuffer, 3, hadc->Config.Timeout) != HAL_OK)
    {
      ADC_CS_HIGH();
      return SPI_ADC_ERROR_TIMEOUT;
    }
    
    ADC_CS_HIGH();
    SPI_ADC_DelayUs(1);
    
    *pRawValue = ((uint32_t)rxBuffer[0] << 16) | ((uint32_t)rxBuffer[1] << 8) | rxBuffer[2];
    
    if (*pRawValue & 0x800000)
    {
      *pRawValue |= 0xFF000000;
    }
  }
  else
  {
    txBuffer[0] = MCP3208_CMD_SINGLE | ((channel & 0x07) << 3);
    
    ADC_CS_LOW();
    SPI_ADC_DelayUs(1);
    
    uint8_t temp[2] = {0};
    if (HAL_SPI_TransmitReceive(hadc->Config.hspi, txBuffer, temp, 1, hadc->Config.Timeout) != HAL_OK)
    {
      ADC_CS_HIGH();
      return SPI_ADC_ERROR_TIMEOUT;
    }
    
    if (HAL_SPI_Receive(hadc->Config.hspi, temp, 2, hadc->Config.Timeout) != HAL_OK)
    {
      ADC_CS_HIGH();
      return SPI_ADC_ERROR_TIMEOUT;
    }
    
    ADC_CS_HIGH();
    SPI_ADC_DelayUs(1);
    
    *pRawValue = ((uint32_t)(temp[0] & 0x0F) << 8) | temp[1];
  }
  
  hadc->RawData[channel] = *pRawValue;
  
  return SPI_ADC_OK;
}

/**
  * @brief  Start continuous conversion mode
  */
SPI_ADC_Status_t SPI_ADC_StartContinuous(SPI_ADC_Handle_t *hadc)
{
  if (hadc == NULL || !hadc->IsInitialized)
  {
    return SPI_ADC_ERROR_NOT_INIT;
  }
  
  if (hadc->Config.Resolution != ADC_RESOLUTION_24BIT)
  {
    return SPI_ADC_ERROR;
  }
  
  uint8_t cmd = ADS1256_CMD_RDATAC;
  
  ADC_CS_LOW();
  SPI_ADC_DelayUs(1);
  
  if (HAL_SPI_Transmit(hadc->Config.hspi, &cmd, 1, hadc->Config.Timeout) != HAL_OK)
  {
    ADC_CS_HIGH();
    return SPI_ADC_ERROR_TIMEOUT;
  }
  
  ADC_CS_HIGH();
  SPI_ADC_DelayUs(1);
  
  hadc->IsConverting = 1;
  
  return SPI_ADC_OK;
}

/**
  * @brief  Stop continuous conversion mode
  */
SPI_ADC_Status_t SPI_ADC_StopContinuous(SPI_ADC_Handle_t *hadc)
{
  if (hadc == NULL || !hadc->IsInitialized)
  {
    return SPI_ADC_ERROR_NOT_INIT;
  }
  
  uint8_t cmd = ADS1256_CMD_SDATAC;
  
  ADC_CS_LOW();
  SPI_ADC_DelayUs(1);
  
  if (HAL_SPI_Transmit(hadc->Config.hspi, &cmd, 1, hadc->Config.Timeout) != HAL_OK)
  {
    ADC_CS_HIGH();
    return SPI_ADC_ERROR_TIMEOUT;
  }
  
  ADC_CS_HIGH();
  SPI_ADC_DelayUs(1);
  
  hadc->IsConverting = 0;
  
  return SPI_ADC_OK;
}

/**
  * @brief  Scan all enabled channels
  */
SPI_ADC_Status_t SPI_ADC_ScanChannels(SPI_ADC_Handle_t *hadc)
{
  if (hadc == NULL || !hadc->IsInitialized)
  {
    return SPI_ADC_ERROR_NOT_INIT;
  }
  
  for (uint8_t ch = 0; ch < ADC_MAX_CHANNELS; ch++)
  {
    if (hadc->Config.Channels[ch].IsEnabled)
    {
      uint32_t rawValue;
      if (SPI_ADC_ReadChannel(hadc, ch, &rawValue) == SPI_ADC_OK)
      {
        float voltage = (float)rawValue * hadc->ReferenceVoltage_mV / ((1U << hadc->Config.Resolution) - 1);
        hadc->VoltageData[ch] = voltage / 1000.0f;
      }
    }
  }
  
  return SPI_ADC_OK;
}

/**
  * @brief  Get voltage data for specified channel
  */
SPI_ADC_Status_t SPI_ADC_GetVoltage(SPI_ADC_Handle_t *hadc, uint8_t channel, float *pVoltage)
{
  if (hadc == NULL || !hadc->IsInitialized || pVoltage == NULL)
  {
    return SPI_ADC_ERROR_PARAM;
  }
  
  if (channel >= ADC_MAX_CHANNELS)
  {
    return SPI_ADC_ERROR_CHANNEL;
  }
  
  *pVoltage = hadc->VoltageData[channel];
  
  return SPI_ADC_OK;
}

/**
  * @brief  Set reference voltage
  */
void SPI_ADC_SetReferenceVoltage(SPI_ADC_Handle_t *hadc, uint32_t RefVoltage_mV)
{
  if (hadc != NULL && hadc->IsInitialized)
  {
    hadc->ReferenceVoltage_mV = RefVoltage_mV;
  }
}

/**
  * @brief  Enable ADC channel
  */
SPI_ADC_Status_t SPI_ADC_EnableChannel(SPI_ADC_Handle_t *hadc, uint8_t channel)
{
  if (hadc == NULL || !hadc->IsInitialized)
  {
    return SPI_ADC_ERROR_NOT_INIT;
  }
  
  if (channel >= ADC_MAX_CHANNELS)
  {
    return SPI_ADC_ERROR_CHANNEL;
  }
  
  hadc->Config.Channels[channel].IsEnabled = 1;
  hadc->Config.NumChannels++;
  
  return SPI_ADC_OK;
}

/**
  * @brief  Disable ADC channel
  */
SPI_ADC_Status_t SPI_ADC_DisableChannel(SPI_ADC_Handle_t *hadc, uint8_t channel)
{
  if (hadc == NULL || !hadc->IsInitialized)
  {
    return SPI_ADC_ERROR_NOT_INIT;
  }
  
  if (channel >= ADC_MAX_CHANNELS)
  {
    return SPI_ADC_ERROR_CHANNEL;
  }
  
  if (hadc->Config.Channels[channel].IsEnabled)
  {
    hadc->Config.Channels[channel].IsEnabled = 0;
    if (hadc->Config.NumChannels > 0)
    {
      hadc->Config.NumChannels--;
    }
  }
  
  return SPI_ADC_OK;
}

/* Private Functions ---------------------------------------------------------*/

/**
  * @brief  Microsecond delay (using simple loop)
  */
static void SPI_ADC_DelayUs(uint32_t delay)
{
  volatile uint32_t count;
  /* At 72MHz, approximately 18 cycles per us */
  uint32_t cycles = delay * 18;
  
  for (count = 0; count < cycles; count++)
  {
    __NOP();
  }
}

/**
  * @brief  Select ADC channel (internal)
  */
static SPI_ADC_Status_t SPI_ADC_SelectChannel(SPI_ADC_Handle_t *hadc, uint8_t channel)
{
  (void)hadc;
  (void)channel;
  
  return SPI_ADC_OK;
}
