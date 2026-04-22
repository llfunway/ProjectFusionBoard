/**
  ******************************************************************************
  * @file    spi_adc.h
  * @author  Embedded System Team
  * @brief   SPI ADC Driver for External ADC Chips (ADS1256/MCP3208/AD7177)
  ******************************************************************************
  */

#ifndef __SPI_ADC_H
#define __SPI_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported Macros -----------------------------------------------------------*/
#define ADC_SPI_INSTANCE                SPI1
#define ADC_CS_GPIO_Port                GPIOA
#define ADC_CS_Pin                      GPIO_PIN_4

#define ADC_MAX_CHANNELS                ((uint8_t)8)
#define ADC_MIN_CHANNELS                ((uint8_t)1)

#define ADC_RESOLUTION_16BIT            ((uint8_t)16)
#define ADC_RESOLUTION_24BIT            ((uint8_t)24)
#define ADC_RESOLUTION_24BIT_AD7177     ((uint8_t)25)  // AD7177 special identifier

#define ADC_MODE_SINGLE                 ((uint8_t)0x00)
#define ADC_MODE_CONTINUOUS             ((uint8_t)0x01)

/* Exported Types ------------------------------------------------------------*/
typedef enum
{
  SPI_ADC_OK = 0,
  SPI_ADC_ERROR = -1,
  SPI_ADC_ERROR_NOT_INIT = -2,
  SPI_ADC_ERROR_TIMEOUT = -3,
  SPI_ADC_ERROR_PARAM = -4,
  SPI_ADC_ERROR_CHANNEL = -5
} SPI_ADC_Status_t;

typedef struct
{
  uint8_t Channel;
  uint8_t IsEnabled;
  float SampleRate;
} ADC_ChannelConfig_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *CsPort;
  uint16_t CsPin;
  uint8_t Resolution;
  uint8_t NumChannels;
  uint8_t Mode;
  ADC_ChannelConfig_t Channels[ADC_MAX_CHANNELS];
  uint32_t Timeout;
} SPI_ADC_Config_t;

typedef struct
{
  SPI_ADC_Config_t Config;
  uint32_t RawData[ADC_MAX_CHANNELS];
  float VoltageData[ADC_MAX_CHANNELS];
  uint8_t IsInitialized;
  uint32_t ReferenceVoltage_mV;
  uint8_t IsConverting;
} SPI_ADC_Handle_t;

/* Exported Functions --------------------------------------------------------*/

SPI_ADC_Status_t SPI_ADC_Init(SPI_ADC_Handle_t *hadc, SPI_ADC_Config_t *pConfig);

SPI_ADC_Status_t SPI_ADC_ReadChannel(SPI_ADC_Handle_t *hadc, uint8_t channel, uint32_t *pRawValue);

SPI_ADC_Status_t SPI_ADC_StartContinuous(SPI_ADC_Handle_t *hadc);

SPI_ADC_Status_t SPI_ADC_StopContinuous(SPI_ADC_Handle_t *hadc);

SPI_ADC_Status_t SPI_ADC_ScanChannels(SPI_ADC_Handle_t *hadc);

SPI_ADC_Status_t SPI_ADC_GetVoltage(SPI_ADC_Handle_t *hadc, uint8_t channel, float *pVoltage);

void SPI_ADC_SetReferenceVoltage(SPI_ADC_Handle_t *hadc, uint32_t RefVoltage_mV);

SPI_ADC_Status_t SPI_ADC_EnableChannel(SPI_ADC_Handle_t *hadc, uint8_t channel);

SPI_ADC_Status_t SPI_ADC_DisableChannel(SPI_ADC_Handle_t *hadc, uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_ADC_H */
