/**
  ******************************************************************************
  * @file    spi_flash.c
  * @author  Embedded System Team
  * @brief   SPI Flash Driver Implementation for W25Qxx
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi_flash.h"

/* Private Macros ------------------------------------------------------------*/
#define FLASH_CS_LOW()    HAL_GPIO_WritePin(hflash->Config.CsPort, hflash->Config.CsPin, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()   HAL_GPIO_WritePin(hflash->Config.CsPort, hflash->Config.CsPin, GPIO_PIN_SET)

/* Private Function Prototypes -----------------------------------------------*/
static void FLASH_DelayMs(uint32_t delay);

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initialize SPI Flash driver
  */
FLASH_Status_t FLASH_Init(FLASH_Handle_t *hflash, FLASH_Config_t *pConfig)
{
  if (hflash == NULL || pConfig == NULL)
  {
    return FLASH_ERROR_PARAM;
  }
  
  if (pConfig->hspi == NULL)
  {
    return FLASH_ERROR_PARAM;
  }
  
  hflash->Config = *pConfig;
  hflash->IsInitialized = 0;
  hflash->IsWriteEnabled = 0;
  hflash->ManufacturerID = 0;
  hflash->DeviceID = 0;
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = hflash->Config.CsPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(hflash->Config.CsPort, &GPIO_InitStruct);
  
  FLASH_CS_HIGH();
  
  FLASH_DelayMs(10);

  /* Allow ID probe through the shared public read helper. */
  hflash->IsInitialized = 1;
  
  if (FLASH_ReadID(hflash) != FLASH_OK)
  {
    hflash->IsInitialized = 0;
    return FLASH_ERROR;
  }
  
  return FLASH_OK;
}

/**
  * @brief  Read Flash manufacturer and device ID
  */
FLASH_Status_t FLASH_ReadID(FLASH_Handle_t *hflash)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  uint8_t cmd = FLASH_CMD_JEDEC_ID;
  uint8_t rxBuffer[3] = {0};
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, &cmd, 1, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  if (HAL_SPI_Receive(hflash->Config.hspi, rxBuffer, 3, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  hflash->ManufacturerID = rxBuffer[0];
  hflash->DeviceID = ((uint16_t)rxBuffer[1] << 8) | rxBuffer[2];
  
  return FLASH_OK;
}

/**
  * @brief  Read data from Flash
  */
FLASH_Status_t FLASH_ReadData(FLASH_Handle_t *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size)
{
  if (hflash == NULL || !hflash->IsInitialized || pBuffer == NULL || Size == 0)
  {
    return FLASH_ERROR_PARAM;
  }
  
  if (Address >= hflash->Config.TotalSize)
  {
    return FLASH_ERROR_PARAM;
  }
  
  uint8_t cmd[4];
  cmd[0] = FLASH_CMD_SEQUENTIAL_READ;
  cmd[1] = (Address >> 16) & 0xFF;
  cmd[2] = (Address >> 8) & 0xFF;
  cmd[3] = Address & 0xFF;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, cmd, 4, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  if (HAL_SPI_Receive(hflash->Config.hspi, pBuffer, Size, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  return FLASH_OK;
}

/**
  * @brief  Write one page to Flash
  */
FLASH_Status_t FLASH_WritePage(FLASH_Handle_t *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size)
{
  if (hflash == NULL || !hflash->IsInitialized || pBuffer == NULL || Size == 0)
  {
    return FLASH_ERROR_PARAM;
  }
  
  if (Size > FLASH_W25Q_PAGE_SIZE)
  {
    Size = FLASH_W25Q_PAGE_SIZE;
  }
  
  if (Address >= hflash->Config.TotalSize)
  {
    return FLASH_ERROR_PARAM;
  }
  
  if (FLASH_WaitBusy(hflash, FLASH_TIMEOUT_WRITE) != FLASH_OK)
  {
    return FLASH_ERROR_TIMEOUT;
  }
  
  if (FLASH_WriteEnable(hflash) != FLASH_OK)
  {
    return FLASH_ERROR;
  }
  
  uint8_t cmd[4];
  cmd[0] = FLASH_CMD_PAGE_PROGRAM;
  cmd[1] = (Address >> 16) & 0xFF;
  cmd[2] = (Address >> 8) & 0xFF;
  cmd[3] = Address & 0xFF;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, cmd, 4, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, pBuffer, Size, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  FLASH_WriteDisable(hflash);
  
  if (FLASH_WaitBusy(hflash, FLASH_TIMEOUT_WRITE) != FLASH_OK)
  {
    return FLASH_ERROR_TIMEOUT;
  }
  
  return FLASH_OK;
}

/**
  * @brief  Erase a sector (4KB)
  */
FLASH_Status_t FLASH_EraseSector(FLASH_Handle_t *hflash, uint32_t SectorAddress)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  if (SectorAddress >= hflash->Config.TotalSize)
  {
    return FLASH_ERROR_PARAM;
  }
  
  if (FLASH_WaitBusy(hflash, FLASH_TIMEOUT_ERASE) != FLASH_OK)
  {
    return FLASH_ERROR_TIMEOUT;
  }
  
  if (FLASH_WriteEnable(hflash) != FLASH_OK)
  {
    return FLASH_ERROR;
  }
  
  uint8_t cmd[4];
  cmd[0] = FLASH_CMD_SECTOR_ERASE;
  cmd[1] = (SectorAddress >> 16) & 0xFF;
  cmd[2] = (SectorAddress >> 8) & 0xFF;
  cmd[3] = SectorAddress & 0xFF;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, cmd, 4, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  FLASH_WriteDisable(hflash);
  
  if (FLASH_WaitBusy(hflash, FLASH_TIMEOUT_ERASE) != FLASH_OK)
  {
    return FLASH_ERROR_TIMEOUT;
  }
  
  return FLASH_OK;
}

/**
  * @brief  Erase a block (64KB)
  */
FLASH_Status_t FLASH_EraseBlock(FLASH_Handle_t *hflash, uint32_t BlockAddress)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  if (BlockAddress >= hflash->Config.TotalSize)
  {
    return FLASH_ERROR_PARAM;
  }
  
  if (FLASH_WaitBusy(hflash, FLASH_TIMEOUT_ERASE) != FLASH_OK)
  {
    return FLASH_ERROR_TIMEOUT;
  }
  
  if (FLASH_WriteEnable(hflash) != FLASH_OK)
  {
    return FLASH_ERROR;
  }
  
  uint8_t cmd[4];
  cmd[0] = FLASH_CMD_BLOCK_ERASE;
  cmd[1] = (BlockAddress >> 16) & 0xFF;
  cmd[2] = (BlockAddress >> 8) & 0xFF;
  cmd[3] = BlockAddress & 0xFF;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, cmd, 4, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  FLASH_WriteDisable(hflash);
  
  if (FLASH_WaitBusy(hflash, FLASH_TIMEOUT_ERASE) != FLASH_OK)
  {
    return FLASH_ERROR_TIMEOUT;
  }
  
  return FLASH_OK;
}

/**
  * @brief  Erase entire chip
  */
FLASH_Status_t FLASH_EraseChip(FLASH_Handle_t *hflash)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  if (FLASH_WaitBusy(hflash, FLASH_TIMEOUT_ERASE * 10) != FLASH_OK)
  {
    return FLASH_ERROR_TIMEOUT;
  }
  
  if (FLASH_WriteEnable(hflash) != FLASH_OK)
  {
    return FLASH_ERROR;
  }
  
  uint8_t cmd = FLASH_CMD_CHIP_ERASE;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, &cmd, 1, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  FLASH_WriteDisable(hflash);
  
  if (FLASH_WaitBusy(hflash, FLASH_TIMEOUT_ERASE * 10) != FLASH_OK)
  {
    return FLASH_ERROR_TIMEOUT;
  }
  
  return FLASH_OK;
}

/**
  * @brief  Wait until Flash is not busy
  */
FLASH_Status_t FLASH_WaitBusy(FLASH_Handle_t *hflash, uint32_t Timeout)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  uint8_t status;
  uint32_t tickstart = HAL_GetTick();
  
  do
  {
    if (FLASH_ReadStatus(hflash, &status) != FLASH_OK)
    {
      return FLASH_ERROR_TIMEOUT;
    }
    
    if ((status & FLASH_STATUS_BUSY) == 0)
    {
      return FLASH_OK;
    }
    
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((HAL_GetTick() - tickstart) > Timeout)
      {
        return FLASH_ERROR_TIMEOUT;
      }
    }
    
  } while (1);
}

/**
  * @brief  Enable write operations
  */
FLASH_Status_t FLASH_WriteEnable(FLASH_Handle_t *hflash)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, &cmd, 1, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  hflash->IsWriteEnabled = 1;
  
  return FLASH_OK;
}

/**
  * @brief  Disable write operations
  */
FLASH_Status_t FLASH_WriteDisable(FLASH_Handle_t *hflash)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  uint8_t cmd = FLASH_CMD_WRITE_DISABLE;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, &cmd, 1, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  hflash->IsWriteEnabled = 0;
  
  return FLASH_OK;
}

/**
  * @brief  Read status register
  */
FLASH_Status_t FLASH_ReadStatus(FLASH_Handle_t *hflash, uint8_t *pStatus)
{
  if (hflash == NULL || !hflash->IsInitialized || pStatus == NULL)
  {
    return FLASH_ERROR_PARAM;
  }
  
  uint8_t cmd = FLASH_CMD_READ_STATUS;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, &cmd, 1, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  if (HAL_SPI_Receive(hflash->Config.hspi, pStatus, 1, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  hflash->StatusRegister = *pStatus;
  
  return FLASH_OK;
}

/**
  * @brief  Enter power-down mode
  */
FLASH_Status_t FLASH_PowerDown(FLASH_Handle_t *hflash)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  uint8_t cmd = FLASH_CMD_POWER_DOWN;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, &cmd, 1, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  return FLASH_OK;
}

/**
  * @brief  Release from power-down mode
  */
FLASH_Status_t FLASH_ReleasePowerDown(FLASH_Handle_t *hflash)
{
  if (hflash == NULL || !hflash->IsInitialized)
  {
    return FLASH_ERROR_NOT_INIT;
  }
  
  uint8_t cmd = FLASH_CMD_RELEASE_POWER_DOWN;
  
  FLASH_CS_LOW();
  
  if (HAL_SPI_Transmit(hflash->Config.hspi, &cmd, 1, hflash->Config.Timeout) != HAL_OK)
  {
    FLASH_CS_HIGH();
    return FLASH_ERROR_TIMEOUT;
  }
  
  FLASH_CS_HIGH();
  
  return FLASH_OK;
}

/* Private Functions ---------------------------------------------------------*/

/**
  * @brief  Millisecond delay
  */
static void FLASH_DelayMs(uint32_t delay)
{
  HAL_Delay(delay);
}
