/**
  ******************************************************************************
  * @file    ad7177.c
  * @author  Embedded System Team
  * @brief   AD7177-2 24-bit Sigma-Delta ADC Driver Implementation
  *          Ported from Analog Devices standard library to HAL
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ad7177.h"
#include <string.h>

/* Private Macros ------------------------------------------------------------*/
#define AD7177_CS_LOW()   HAL_GPIO_WritePin(had7177->Config.CsPort, had7177->Config.CsPin, GPIO_PIN_RESET)
#define AD7177_CS_HIGH()  HAL_GPIO_WritePin(had7177->Config.CsPort, had7177->Config.CsPin, GPIO_PIN_SET)

/* Private Function Prototypes -----------------------------------------------*/
static AD7177_Status_t AD7177_SpiTransfer(AD7177_Handle_t *had7177, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
static void AD7177_DelayUs(uint32_t delay);

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initialize AD7177 driver
  * @param  had7177: Pointer to AD7177 handle
  * @param  pConfig: Pointer to configuration structure
  * @retval Status
  */
AD7177_Status_t AD7177_Init(AD7177_Handle_t *had7177, AD7177_Config_t *pConfig)
{
  if (had7177 == NULL || pConfig == NULL)
  {
    return AD7177_ERROR_PARAM;
  }
  
  if (pConfig->hspi == NULL)
  {
    return AD7177_ERROR_PARAM;
  }
  
  /* Copy configuration */
  had7177->Config = *pConfig;
  had7177->IsInitialized = 0;
  had7177->DeviceID = 0;
  
  /* Initialize CS pin */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (had7177->Config.CsPort == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if (had7177->Config.CsPort == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if (had7177->Config.CsPort == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if (had7177->Config.CsPort == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  
  GPIO_InitStruct.Pin = had7177->Config.CsPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(had7177->Config.CsPort, &GPIO_InitStruct);
  
  AD7177_CS_HIGH();
  AD7177_DelayUs(10);
  
  /* Reset device */
  if (AD7177_Reset(had7177) != AD7177_OK)
  {
    return AD7177_ERROR_COMM;
  }

  HAL_Delay(10);

  /* Allow register access during init. Cleared on failure below. */
  had7177->IsInitialized = 1;
  
  /* Read Device ID */
  uint32_t idReg = 0;
  if (AD7177_ReadRegister(had7177, AD7177_REG_ID, &idReg) != AD7177_OK)
  {
    had7177->IsInitialized = 0;
    return AD7177_ERROR_COMM;
  }
  had7177->DeviceID = (uint8_t)(idReg & 0xFF);
  
  /* Configure Interface Mode Register */
  uint32_t ifMode = AD7177_IFMODE_DOUT_RESET;
  if (had7177->Config.CrcMode == AD7177_CRC_ENABLE)
  {
    ifMode |= AD7177_IFMODE_CRC_EN;
  }
  ifMode |= AD7177_IFMODE_DATA_WL32;  // 32-bit data width
  
  if (AD7177_WriteRegister(had7177, AD7177_REG_IFMODE, ifMode) != AD7177_OK)
  {
    had7177->IsInitialized = 0;
    return AD7177_ERROR_COMM;
  }

  /* Configure ADC mode: continuous conversion, selected reference source. */
  uint32_t adcMode = AD7177_ADCMODE_MODE_CONTINUOUS | AD7177_ADCMODE_CLOCK_INT;
  if (!had7177->Config.UseExternalRef)
  {
    adcMode |= AD7177_ADCMODE_REF_EN;
  }

  if (AD7177_WriteRegister(had7177, AD7177_REG_ADCMODE, adcMode) != AD7177_OK)
  {
    had7177->IsInitialized = 0;
    return AD7177_ERROR_COMM;
  }

  HAL_Delay(10);
  
  /* Write configuration registers */
  /* Channel Map 0 */
  if (had7177->Config.Channels[0].Enabled)
  {
    uint32_t chMap = AD7177_CHMAP_CH_EN |
                     AD7177_CHMAP_SETUP_SEL(had7177->Config.Channels[0].SetupSel) |
                     AD7177_CHMAP_AINPOS(had7177->Config.Channels[0].AinPos) |
                     AD7177_CHMAP_AINNEG(had7177->Config.Channels[0].AinNeg);
    
    if (AD7177_WriteRegister(had7177, AD7177_REG_CHMAP0, chMap) != AD7177_OK)
    {
      had7177->IsInitialized = 0;
      return AD7177_ERROR_COMM;
    }
  }
  
  /* Setup Configuration 0 */
  uint32_t setupCon = 0;
  
  /* Bipolar/Unipolar mode */
  if (had7177->Config.Setups[0].Bipolar)
  {
    setupCon |= AD7177_SETUP_BI_UNIPOLAR;
  }
  
  /* Reference voltage selection */
  /* RefSel values:
   * 0b00 = REFIN+ / REFIN- (External reference)
   * 0b01 = Internal 2.5V reference
   * 0b10 = AVDD1 / AVSS
   * 0b11 = Reserved
   */
  if (had7177->Config.UseExternalRef)
  {
    setupCon |= AD7177_SETUP_REF_SEL(0);  // External reference (REFIN+/REFIN-)
  }
  else
  {
    setupCon |= AD7177_SETUP_REF_SEL(1);  // Internal 2.5V reference
  }
  
  if (AD7177_WriteRegister(had7177, AD7177_REG_SETUPCON0, setupCon) != AD7177_OK)
  {
    had7177->IsInitialized = 0;
    return AD7177_ERROR_COMM;
  }
  
  /* Filter Configuration 0 */
  uint32_t filtCon = AD7177_FILT_ORDER(had7177->Config.Setups[0].FilterOrder) |
                     AD7177_FILT_ODR(had7177->Config.Setups[0].Odr);
  
  if (AD7177_WriteRegister(had7177, AD7177_REG_FILTCON0, filtCon) != AD7177_OK)
  {
    had7177->IsInitialized = 0;
    return AD7177_ERROR_COMM;
  }
  
  HAL_Delay(10);
  
  /* Set default reference voltage */
  if (had7177->Config.ReferenceVoltage_mV == 0)
  {
    had7177->Config.ReferenceVoltage_mV = 2500.0f;  // Internal 2.5V reference
  }
  
  had7177->IsInitialized = 1;
  
  return AD7177_OK;
}

/**
  * @brief  Read register from AD7177
  * @param  had7177: Pointer to AD7177 handle
  * @param  regAddr: Register address
  * @param  pData: Pointer to store data
  * @retval Status
  */
AD7177_Status_t AD7177_ReadRegister(AD7177_Handle_t *had7177, uint8_t regAddr, uint32_t *pData)
{
  if (had7177 == NULL || !had7177->IsInitialized || pData == NULL)
  {
    return AD7177_ERROR_PARAM;
  }
  
  uint8_t txBuffer[5] = {0};
  uint8_t rxBuffer[5] = {0};
  uint16_t transferSize = 0;
  
  /* Determine register size */
  uint8_t regSize = 0;
  switch (regAddr)
  {
    case AD7177_REG_STATUS:
      regSize = 1;
      break;
    case AD7177_REG_ADCMODE:
    case AD7177_REG_IFMODE:
    case AD7177_REG_GPIOCON:
    case AD7177_REG_ID:
    case AD7177_REG_CHMAP0:
    case AD7177_REG_CHMAP1:
    case AD7177_REG_CHMAP2:
    case AD7177_REG_CHMAP3:
    case AD7177_REG_SETUPCON0:
    case AD7177_REG_SETUPCON1:
    case AD7177_REG_SETUPCON2:
    case AD7177_REG_SETUPCON3:
    case AD7177_REG_FILTCON0:
    case AD7177_REG_FILTCON1:
    case AD7177_REG_FILTCON2:
    case AD7177_REG_FILTCON3:
      regSize = 2;
      break;
    case AD7177_REG_DATA:
      regSize = 4;
      break;
    case AD7177_REG_OFFSET0:
    case AD7177_REG_OFFSET1:
    case AD7177_REG_OFFSET2:
    case AD7177_REG_OFFSET3:
    case AD7177_REG_GAIN0:
    case AD7177_REG_GAIN1:
    case AD7177_REG_GAIN2:
    case AD7177_REG_GAIN3:
      regSize = 3;
      break;
    default:
      return AD7177_ERROR_PARAM;
  }
  
  /* Build command byte */
  txBuffer[0] = AD7177_COMM_WEN | AD7177_COMM_RD | AD7177_COMM_RA(regAddr);
  
  /* Calculate transfer size (with optional CRC) */
  transferSize = (had7177->Config.CrcMode == AD7177_CRC_ENABLE) ? (regSize + 2) : (regSize + 1);
  
  /* SPI transfer */
  AD7177_CS_LOW();
  AD7177_DelayUs(1);
  
  AD7177_Status_t status = AD7177_SpiTransfer(had7177, txBuffer, rxBuffer, transferSize);
  
  AD7177_CS_HIGH();
  AD7177_DelayUs(1);
  
  if (status != AD7177_OK)
  {
    return status;
  }
  
  /* Verify CRC if enabled */
  if (had7177->Config.CrcMode == AD7177_CRC_ENABLE)
  {
    uint8_t crcCalc = AD7177_ComputeCRC8(rxBuffer, regSize + 1);
    if (crcCalc != rxBuffer[regSize + 1])
    {
      return AD7177_ERROR_CRC;
    }
  }
  
  /* Extract data */
  *pData = 0;
  for (uint8_t i = 1; i <= regSize; i++)
  {
    *pData <<= 8;
    *pData |= rxBuffer[i];
  }
  
  return AD7177_OK;
}

/**
  * @brief  Write register to AD7177
  * @param  had7177: Pointer to AD7177 handle
  * @param  regAddr: Register address
  * @param  data: Data to write
  * @retval Status
  */
AD7177_Status_t AD7177_WriteRegister(AD7177_Handle_t *had7177, uint8_t regAddr, uint32_t data)
{
  if (had7177 == NULL || !had7177->IsInitialized)
  {
    return AD7177_ERROR_PARAM;
  }
  
  uint8_t txBuffer[6] = {0};
  uint16_t transferSize = 0;
  
  /* Determine register size */
  uint8_t regSize = 0;
  switch (regAddr)
  {
    case AD7177_REG_STATUS:
      regSize = 1;
      break;
    case AD7177_REG_ADCMODE:
    case AD7177_REG_IFMODE:
    case AD7177_REG_GPIOCON:
    case AD7177_REG_ID:
    case AD7177_REG_CHMAP0:
    case AD7177_REG_CHMAP1:
    case AD7177_REG_CHMAP2:
    case AD7177_REG_CHMAP3:
    case AD7177_REG_SETUPCON0:
    case AD7177_REG_SETUPCON1:
    case AD7177_REG_SETUPCON2:
    case AD7177_REG_SETUPCON3:
    case AD7177_REG_FILTCON0:
    case AD7177_REG_FILTCON1:
    case AD7177_REG_FILTCON2:
    case AD7177_REG_FILTCON3:
      regSize = 2;
      break;
    case AD7177_REG_DATA:
      regSize = 4;
      break;
    case AD7177_REG_OFFSET0:
    case AD7177_REG_OFFSET1:
    case AD7177_REG_OFFSET2:
    case AD7177_REG_OFFSET3:
    case AD7177_REG_GAIN0:
    case AD7177_REG_GAIN1:
    case AD7177_REG_GAIN2:
    case AD7177_REG_GAIN3:
      regSize = 3;
      break;
    default:
      return AD7177_ERROR_PARAM;
  }
  
  /* Build command byte */
  txBuffer[0] = AD7177_COMM_WEN | AD7177_COMM_WR | AD7177_COMM_RA(regAddr);
  
  /* Fill data bytes (MSB first) */
  for (uint8_t i = 0; i < regSize; i++)
  {
    txBuffer[regSize - i] = (uint8_t)(data & 0xFF);
    data >>= 8;
  }
  
  /* Calculate CRC if enabled */
  if (had7177->Config.CrcMode == AD7177_CRC_ENABLE)
  {
    txBuffer[regSize + 1] = AD7177_ComputeCRC8(txBuffer, regSize + 1);
    transferSize = regSize + 2;
  }
  else
  {
    transferSize = regSize + 1;
  }
  
  /* SPI transfer */
  AD7177_CS_LOW();
  AD7177_DelayUs(1);
  
  AD7177_Status_t status = AD7177_SpiTransfer(had7177, txBuffer, NULL, transferSize);
  
  AD7177_CS_HIGH();
  AD7177_DelayUs(1);
  
  return status;
}

/**
  * @brief  Reset AD7177 device
  * @param  had7177: Pointer to AD7177 handle
  * @retval Status
  */
AD7177_Status_t AD7177_Reset(AD7177_Handle_t *had7177)
{
  if (had7177 == NULL)
  {
    return AD7177_ERROR_PARAM;
  }
  
  /* Send 64 consecutive 1's to reset SPI interface */
  uint8_t resetBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  AD7177_CS_LOW();
  AD7177_DelayUs(1);
  
  AD7177_Status_t status = AD7177_SpiTransfer(had7177, resetBuf, NULL, 8);
  
  AD7177_CS_HIGH();
  AD7177_DelayUs(1);
  
  return status;
}

/**
  * @brief  Wait for ADC conversion to complete
  * @param  had7177: Pointer to AD7177 handle
  * @param  timeout: Timeout in milliseconds
  * @retval Status
  */
AD7177_Status_t AD7177_WaitForReady(AD7177_Handle_t *had7177, uint32_t timeout)
{
  if (had7177 == NULL || !had7177->IsInitialized)
  {
    return AD7177_ERROR_NOT_INIT;
  }
  
  uint32_t startTime = HAL_GetTick();
  uint32_t statusReg = 0;
  
  while ((HAL_GetTick() - startTime) < timeout)
  {
    if (AD7177_ReadRegister(had7177, AD7177_REG_STATUS, &statusReg) != AD7177_OK)
    {
      return AD7177_ERROR_COMM;
    }
    
    /* Check RDY bit (bit 7) */
    if ((statusReg & AD7177_STATUS_RDY) == 0)
    {
      return AD7177_OK;
    }
    
    HAL_Delay(1);
  }
  
  return AD7177_ERROR_TIMEOUT;
}

/**
  * @brief  Read ADC conversion data
  * @param  had7177: Pointer to AD7177 handle
  * @param  channel: Channel number (0-3)
  * @param  pData: Pointer to store raw data
  * @retval Status
  */
AD7177_Status_t AD7177_ReadData(AD7177_Handle_t *had7177, uint8_t channel, int32_t *pData)
{
  if (had7177 == NULL || !had7177->IsInitialized || pData == NULL)
  {
    return AD7177_ERROR_PARAM;
  }
  
  if (channel > 3)
  {
    return AD7177_ERROR_PARAM;
  }
  
  /* Wait for data ready */
  if (AD7177_WaitForReady(had7177, 100) != AD7177_OK)
  {
    return AD7177_ERROR_TIMEOUT;
  }
  
  /* Read data register */
  uint32_t rawData = 0;
  if (AD7177_ReadRegister(had7177, AD7177_REG_DATA, &rawData) != AD7177_OK)
  {
    return AD7177_ERROR_COMM;
  }
  
  /* In bipolar mode, AD7177 output coding is offset binary: midscale is 0 V. */
  int64_t bipolarCode = (int64_t)rawData - 2147483648LL;
  int32_t adcData = (int32_t)bipolarCode;
  
  *pData = adcData;
  had7177->RawData[channel] = adcData;
  
  /* Convert to voltage */
  float voltage = ((float)bipolarCode / 2147483648.0f) * had7177->Config.ReferenceVoltage_mV;
  had7177->VoltageData[channel] = voltage / 1000.0f;  // Convert mV to V
  
  return AD7177_OK;
}

/**
  * @brief  Get voltage data for specified channel
  * @param  had7177: Pointer to AD7177 handle
  * @param  channel: Channel number (0-3)
  * @param  pVoltage: Pointer to store voltage
  * @retval Status
  */
AD7177_Status_t AD7177_GetVoltage(AD7177_Handle_t *had7177, uint8_t channel, float *pVoltage)
{
  if (had7177 == NULL || !had7177->IsInitialized || pVoltage == NULL)
  {
    return AD7177_ERROR_PARAM;
  }
  
  if (channel > 3)
  {
    return AD7177_ERROR_PARAM;
  }
  
  *pVoltage = had7177->VoltageData[channel];
  
  return AD7177_OK;
}

/**
  * @brief  Set reference voltage
  * @param  had7177: Pointer to AD7177 handle
  * @param  refVoltage_mV: Reference voltage in millivolts
  */
void AD7177_SetReferenceVoltage(AD7177_Handle_t *had7177, float refVoltage_mV)
{
  if (had7177 != NULL && had7177->IsInitialized)
  {
    had7177->Config.ReferenceVoltage_mV = refVoltage_mV;
  }
}

/**
  * @brief  Compute CRC8 checksum
  * @param  pBuf: Data buffer
  * @param  bufSize: Buffer size
  * @retval CRC value
  */
uint8_t AD7177_ComputeCRC8(uint8_t *pBuf, uint8_t bufSize)
{
  uint8_t crc = 0;
  
  while (bufSize--)
  {
    for (uint8_t i = 0x80; i != 0; i >>= 1)
    {
      if (((crc & 0x80) != 0) != ((*pBuf & i) != 0))
      {
        crc <<= 1;
        crc ^= AD7177_CRC8_POLY;
      }
      else
      {
        crc <<= 1;
      }
    }
    pBuf++;
  }
  
  return crc;
}

/* Private Functions ---------------------------------------------------------*/

/**
  * @brief  SPI transfer helper function
  * @param  had7177: Pointer to AD7177 handle
  * @param  pTxData: Transmit buffer
  * @param  pRxData: Receive buffer
  * @param  Size: Number of bytes
  * @retval Status
  */
static AD7177_Status_t AD7177_SpiTransfer(AD7177_Handle_t *had7177, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
  HAL_StatusTypeDef halStatus;
  
  if (pRxData != NULL)
  {
    /* Full-duplex transfer (read operation) */
    halStatus = HAL_SPI_TransmitReceive(had7177->Config.hspi, pTxData, pRxData, Size, had7177->Config.Timeout);
  }
  else
  {
    /* Transmit only (write operation) */
    halStatus = HAL_SPI_Transmit(had7177->Config.hspi, pTxData, Size, had7177->Config.Timeout);
  }
  
  if (halStatus != HAL_OK)
  {
    return AD7177_ERROR_COMM;
  }
  
  return AD7177_OK;
}

/**
  * @brief  Microsecond delay
  * @param  delay: Delay in microseconds
  */
static void AD7177_DelayUs(uint32_t delay)
{
  volatile uint32_t count;
  /* At 72MHz, approximately 18 cycles per us */
  uint32_t cycles = delay * 18;
  
  for (count = 0; count < cycles; count++)
  {
    __NOP();
  }
}
