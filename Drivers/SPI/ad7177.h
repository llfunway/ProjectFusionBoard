/**
  ******************************************************************************
  * @file    ad7177.h
  * @author  Embedded System Team
  * @brief   AD7177-2 24-bit Sigma-Delta ADC Driver Header
  *          Based on Analog Devices standard library, ported to HAL
  ******************************************************************************
  */

#ifndef __AD7177_H
#define __AD7177_H

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

/* AD7177 Device Constants */
#define AD7177_MAX_CHANNELS     ((uint8_t)4)   /* Maximum 4 channels (CHMAP0-3) */
#define AD7177_MAX_SETUPS       ((uint8_t)4)   /* Maximum 4 setups (SETUPCON0-3) */
#define AD7177_DEVICE_ID        ((uint16_t)0x0FD0)  /* Expected device ID for AD7177-2 */

/* AD7177 Register Addresses */
#define AD7177_REG_COMM       ((uint8_t)0x00)
#define AD7177_REG_STATUS     ((uint8_t)0x00)
#define AD7177_REG_ADCMODE    ((uint8_t)0x01)
#define AD7177_REG_IFMODE     ((uint8_t)0x02)
#define AD7177_REG_REGCHECK   ((uint8_t)0x03)
#define AD7177_REG_DATA       ((uint8_t)0x04)
#define AD7177_REG_GPIOCON    ((uint8_t)0x06)
#define AD7177_REG_ID         ((uint8_t)0x07)
#define AD7177_REG_CHMAP0     ((uint8_t)0x10)
#define AD7177_REG_CHMAP1     ((uint8_t)0x11)
#define AD7177_REG_CHMAP2     ((uint8_t)0x12)
#define AD7177_REG_CHMAP3     ((uint8_t)0x13)
#define AD7177_REG_SETUPCON0  ((uint8_t)0x20)
#define AD7177_REG_SETUPCON1  ((uint8_t)0x21)
#define AD7177_REG_SETUPCON2  ((uint8_t)0x22)
#define AD7177_REG_SETUPCON3  ((uint8_t)0x23)
#define AD7177_REG_FILTCON0   ((uint8_t)0x28)
#define AD7177_REG_FILTCON1   ((uint8_t)0x29)
#define AD7177_REG_FILTCON2   ((uint8_t)0x2A)
#define AD7177_REG_FILTCON3   ((uint8_t)0x2B)
#define AD7177_REG_OFFSET0    ((uint8_t)0x30)
#define AD7177_REG_OFFSET1    ((uint8_t)0x31)
#define AD7177_REG_OFFSET2    ((uint8_t)0x32)
#define AD7177_REG_OFFSET3    ((uint8_t)0x33)
#define AD7177_REG_GAIN0      ((uint8_t)0x38)
#define AD7177_REG_GAIN1      ((uint8_t)0x39)
#define AD7177_REG_GAIN2      ((uint8_t)0x3A)
#define AD7177_REG_GAIN3      ((uint8_t)0x3B)

/* Communication Register bits */
#define AD7177_COMM_WEN       ((uint8_t)(0 << 7))
#define AD7177_COMM_WR        ((uint8_t)(0 << 6))
#define AD7177_COMM_RD        ((uint8_t)(1 << 6))
#define AD7177_COMM_RA(x)     ((uint8_t)((x) & 0x3F))

/* ADC Mode Register bits */
#define AD7177_ADCMODE_REF_EN            ((uint16_t)(1 << 15))
#define AD7177_ADCMODE_MODE_CONTINUOUS   ((uint16_t)(0 << 4))
#define AD7177_ADCMODE_CLOCK_INT         ((uint16_t)(0 << 2))
#define AD7177_ADCMODE_CLOCK_INT_OUT     ((uint16_t)(1 << 2))
#define AD7177_ADCMODE_CLOCK_EXT         ((uint16_t)(2 << 2))
#define AD7177_ADCMODE_CLOCK_CRYSTAL     ((uint16_t)(3 << 2))

/* Status Register bits */
#define AD7177_STATUS_RDY     ((uint8_t)(1 << 7))
#define AD7177_STATUS_ADC_ERR ((uint8_t)(1 << 6))
#define AD7177_STATUS_CRC_ERR ((uint8_t)(1 << 5))
#define AD7177_STATUS_REG_ERR ((uint8_t)(1 << 4))
#define AD7177_STATUS_CH(x)   ((uint8_t)((x) & 0x03))

/* Interface Mode Register bits */
#define AD7177_IFMODE_DOUT_RESET  ((uint16_t)(1 << 8))
#define AD7177_IFMODE_CRC_EN      ((uint16_t)(1 << 3))
#define AD7177_IFMODE_XOR_EN      ((uint16_t)(1 << 2))
#define AD7177_IFMODE_DATA_WL32   ((uint16_t)(1 << 1))

/* Channel Map Register bits */
#define AD7177_CHMAP_CH_EN        ((uint16_t)(1 << 15))
#define AD7177_CHMAP_SETUP_SEL(x) ((uint16_t)(((x) & 0x3) << 12))
#define AD7177_CHMAP_AINPOS(x)    ((uint16_t)(((x) & 0x1F) << 5))
#define AD7177_CHMAP_AINNEG(x)    ((uint16_t)((x) & 0x1F))

/* Setup Configuration Register bits */
#define AD7177_SETUP_BI_UNIPOLAR  ((uint16_t)(1 << 12))
#define AD7177_SETUP_REF_SEL(x)   ((uint16_t)(((x) & 0x3) << 4))

/* Filter Configuration Register bits */
#define AD7177_FILT_SINC3_MAP     ((uint16_t)(1 << 15))
#define AD7177_FILT_ENHFILTEN     ((uint16_t)(1 << 11))
#define AD7177_FILT_ENHFILT(x)    ((uint16_t)(((x) & 0x7) << 8))
#define AD7177_FILT_ORDER(x)      ((uint16_t)(((x) & 0x3) << 5))
#define AD7177_FILT_ODR(x)        ((uint16_t)((x) & 0x1F))

/* Data rates (ODR values for SINC3 filter) */
#define AD7177_ODR_5SPS       ((uint8_t)0x04)   // 5 SPS
#define AD7177_ODR_10SPS      ((uint8_t)0x03)   // 10 SPS
#define AD7177_ODR_20SPS      ((uint8_t)0x01)   // 20 SPS (Recommended)
#define AD7177_ODR_50SPS      ((uint8_t)0x0E)   // 50 SPS
#define AD7177_ODR_59SPS      ((uint8_t)0x0F)   // 59.52 SPS (Standard lib: 0x056f)
#define AD7177_ODR_100SPS     ((uint8_t)0x0E)   // 100.2 SPS
#define AD7177_ODR_200SPS     ((uint8_t)0x0D)   // 200 SPS
#define AD7177_ODR_500SPS     ((uint8_t)0x0B)   // 500 SPS
#define AD7177_ODR_1000SPS    ((uint8_t)0x0A)   // 1007 SPS
#define AD7177_ODR_2500SPS    ((uint8_t)0x09)   // 2500 SPS

/* CRC Polynomial */
#define AD7177_CRC8_POLY      ((uint8_t)0x07)

/* Default Configuration Values (20 SPS, Differential AIN3+/AIN4-) */
#define AD7177_DEFAULT_CHMAP0     ((uint16_t)0x8064)  // CH0: AIN3+ / AIN4-, enabled
#define AD7177_DEFAULT_SETUPCON0  ((uint16_t)0x1300)  // Bipolar, Internal reference
#define AD7177_DEFAULT_FILTCON0   ((uint16_t)0x0571)  // 20 SPS, SINC3 filter

/* Exported Types ------------------------------------------------------------*/

typedef enum
{
  AD7177_OK = 0,
  AD7177_ERROR = -1,
  AD7177_ERROR_NOT_INIT = -2,
  AD7177_ERROR_TIMEOUT = -3,
  AD7177_ERROR_PARAM = -4,
  AD7177_ERROR_COMM = -5,
  AD7177_ERROR_CRC = -6
} AD7177_Status_t;

typedef enum
{
  AD7177_CRC_DISABLE = 0,
  AD7177_CRC_ENABLE = 1
} AD7177_CrcMode_t;

typedef struct
{
  uint8_t Channel;
  uint8_t AinPos;         // Positive input (0-15)
  uint8_t AinNeg;         // Negative input (0-15)
  uint8_t SetupSel;       // Setup configuration (0-3)
  uint8_t Enabled;
} AD7177_ChannelConfig_t;

typedef struct
{
  uint8_t Bipolar;        // 0: Unipolar, 1: Bipolar
  uint8_t RefSel;         // Reference select (0: Internal, 1: External)
  uint8_t Odr;            // Output data rate
  uint8_t FilterOrder;    // Filter order (0: SINC3, 1: Enhanced)
} AD7177_SetupConfig_t;

/**
  * @brief  AD7177 Configuration Structure
  */
typedef struct
{
  SPI_HandleTypeDef *hspi;        /**< SPI handle */
  
  /* Chip Select Configuration */
  GPIO_TypeDef *CsPort;           /**< CS GPIO port */
  uint16_t CsPin;                 /**< CS GPIO pin */
  
  /* CRC Configuration */
  AD7177_CrcMode_t CrcMode;       /**< CRC mode */
  
  /* Reference Voltage Configuration */
  float ReferenceVoltage_mV;      /**< Reference voltage in mV (e.g., 5000.0 for 5V) */
  uint8_t UseExternalRef;         /**< 1 = External reference, 0 = Internal reference */
  
  /* Timing */
  uint32_t Timeout;               /**< Timeout in ms */
  
  /* Channel Configuration (up to 4 channels) */
  AD7177_ChannelConfig_t Channels[AD7177_MAX_CHANNELS];
  
  /* Setup Configuration (up to 4 setups) */
  AD7177_SetupConfig_t Setups[AD7177_MAX_SETUPS];
  
} AD7177_Config_t;

typedef struct
{
  AD7177_Config_t Config;
  int32_t RawData[4];           // Raw ADC data for 4 channels
  float VoltageData[4];         // Converted voltage data
  uint8_t IsInitialized;
  uint8_t DeviceID;
  uint16_t StatusReg;
} AD7177_Handle_t;

/* Exported Functions --------------------------------------------------------*/

AD7177_Status_t AD7177_Init(AD7177_Handle_t *had7177, AD7177_Config_t *pConfig);

AD7177_Status_t AD7177_ReadRegister(AD7177_Handle_t *had7177, uint8_t regAddr, uint32_t *pData);

AD7177_Status_t AD7177_WriteRegister(AD7177_Handle_t *had7177, uint8_t regAddr, uint32_t data);

AD7177_Status_t AD7177_Reset(AD7177_Handle_t *had7177);

AD7177_Status_t AD7177_WaitForReady(AD7177_Handle_t *had7177, uint32_t timeout);

AD7177_Status_t AD7177_ReadData(AD7177_Handle_t *had7177, uint8_t channel, int32_t *pData);

AD7177_Status_t AD7177_GetVoltage(AD7177_Handle_t *had7177, uint8_t channel, float *pVoltage);

void AD7177_SetReferenceVoltage(AD7177_Handle_t *had7177, float refVoltage_mV);

uint8_t AD7177_ComputeCRC8(uint8_t *pBuf, uint8_t bufSize);

#ifdef __cplusplus
}
#endif

#endif /* __AD7177_H */
