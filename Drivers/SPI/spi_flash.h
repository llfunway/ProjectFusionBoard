/**
  ******************************************************************************
  * @file    spi_flash.h
  * @author  Embedded System Team
  * @brief   SPI Flash Driver for W25Qxx Series
  ******************************************************************************
  */

#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported Macros -----------------------------------------------------------*/
#define FLASH_SPI_INSTANCE              SPI2
#define FLASH_CS_GPIO_Port              GPIOB
#define FLASH_CS_Pin                    GPIO_PIN_12

#define FLASH_SECTOR_SIZE               ((uint32_t)4096)
#define FLASH_W25Q_PAGE_SIZE            ((uint32_t)256)
#define FLASH_BLOCK_SIZE                ((uint32_t)65536)

#define FLASH_CMD_WRITE_ENABLE          ((uint8_t)0x06)
#define FLASH_CMD_WRITE_DISABLE         ((uint8_t)0x04)
#define FLASH_CMD_READ_STATUS           ((uint8_t)0x05)
#define FLASH_CMD_PAGE_PROGRAM          ((uint8_t)0x02)
#define FLASH_CMD_SEQUENTIAL_READ       ((uint8_t)0x03)
#define FLASH_CMD_SECTOR_ERASE          ((uint8_t)0x20)
#define FLASH_CMD_BLOCK_ERASE           ((uint8_t)0xD8)
#define FLASH_CMD_CHIP_ERASE            ((uint8_t)0xC7)
#define FLASH_CMD_POWER_DOWN            ((uint8_t)0xB9)
#define FLASH_CMD_RELEASE_POWER_DOWN    ((uint8_t)0xAB)
#define FLASH_CMD_JEDEC_ID              ((uint8_t)0x9F)

#define FLASH_STATUS_BUSY               ((uint8_t)0x01)
#define FLASH_STATUS_WEL                ((uint8_t)0x02)

#define FLASH_TIMEOUT_ERASE             ((uint32_t)10000)
#define FLASH_TIMEOUT_WRITE             ((uint32_t)1000)
#define FLASH_TIMEOUT_DEFAULT           ((uint32_t)100)

/* Exported Types ------------------------------------------------------------*/
typedef enum
{
  FLASH_OK = 0,
  FLASH_ERROR = -1,
  FLASH_ERROR_NOT_INIT = -2,
  FLASH_ERROR_TIMEOUT = -3,
  FLASH_ERROR_PARAM = -4,
  FLASH_ERROR_BUSY = -5,
  FLASH_ERROR_WRITE_PROTECT = -6
} FLASH_Status_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *CsPort;
  uint16_t CsPin;
  uint32_t TotalSize;
  uint32_t SectorSize;
  uint32_t PageSize;
  uint32_t Timeout;
} FLASH_Config_t;

typedef struct
{
  FLASH_Config_t Config;
  uint8_t ManufacturerID;
  uint16_t DeviceID;
  uint8_t IsInitialized;
  uint8_t IsWriteEnabled;
  uint8_t StatusRegister;
} FLASH_Handle_t;

/* Exported Functions --------------------------------------------------------*/

FLASH_Status_t FLASH_Init(FLASH_Handle_t *hflash, FLASH_Config_t *pConfig);

FLASH_Status_t FLASH_ReadID(FLASH_Handle_t *hflash);

FLASH_Status_t FLASH_ReadData(FLASH_Handle_t *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size);

FLASH_Status_t FLASH_WritePage(FLASH_Handle_t *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size);

FLASH_Status_t FLASH_EraseSector(FLASH_Handle_t *hflash, uint32_t SectorAddress);

FLASH_Status_t FLASH_EraseBlock(FLASH_Handle_t *hflash, uint32_t BlockAddress);

FLASH_Status_t FLASH_EraseChip(FLASH_Handle_t *hflash);

FLASH_Status_t FLASH_WaitBusy(FLASH_Handle_t *hflash, uint32_t Timeout);

FLASH_Status_t FLASH_WriteEnable(FLASH_Handle_t *hflash);

FLASH_Status_t FLASH_WriteDisable(FLASH_Handle_t *hflash);

FLASH_Status_t FLASH_ReadStatus(FLASH_Handle_t *hflash, uint8_t *pStatus);

FLASH_Status_t FLASH_PowerDown(FLASH_Handle_t *hflash);

FLASH_Status_t FLASH_ReleasePowerDown(FLASH_Handle_t *hflash);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_FLASH_H */
