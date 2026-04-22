/**
  ******************************************************************************
  * @file    uart_proto.h
  * @author  Embedded System Team
  * @brief   UART Communication Protocol Header
  *          Custom frame format for PC communication
  ******************************************************************************
  */

#ifndef __UART_PROTO_H
#define __UART_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported Macros -----------------------------------------------------------*/
#define UART_PROTO_INSTANCE             USART1
#define UART_PROTO_BAUDRATE             ((uint32_t)115200)

#define UART_PROTO_RX_BUFFER_SIZE       ((uint16_t)256)
#define UART_PROTO_TX_BUFFER_SIZE       ((uint16_t)256)
#define UART_PROTO_MAX_PAYLOAD_SIZE     ((uint16_t)128)

#define UART_PROTO_FRAME_HEADER         ((uint8_t)0xAA)
#define UART_PROTO_FRAME_FOOTER         ((uint8_t)0x55)
#define UART_PROTO_FRAME_MAX_SIZE       ((uint16_t)(UART_PROTO_MAX_PAYLOAD_SIZE + 8))

#define UART_PROTO_CMD_READ_SENSORS     ((uint8_t)0x01)
#define UART_PROTO_CMD_START_LOGGING    ((uint8_t)0x02)
#define UART_PROTO_CMD_STOP_LOGGING     ((uint8_t)0x03)
#define UART_PROTO_CMD_GET_STATUS       ((uint8_t)0x04)
#define UART_PROTO_CMD_SET_CONFIG       ((uint8_t)0x05)
#define UART_PROTO_CMD_ACK              ((uint8_t)0x80)
#define UART_PROTO_CMD_NACK             ((uint8_t)0x81)

/* Exported Types ------------------------------------------------------------*/
typedef enum
{
  UART_PROTO_OK = 0,
  UART_PROTO_ERROR = -1,
  UART_PROTO_ERROR_NOT_INIT = -2,
  UART_PROTO_ERROR_TIMEOUT = -3,
  UART_PROTO_ERROR_PARAM = -4,
  UART_PROTO_ERROR_CHECKSUM = -5,
  UART_PROTO_ERROR_FRAME = -6
} UART_Proto_Status_t;

typedef struct
{
  uint8_t Header;
  uint8_t Command;
  uint16_t Length;
  uint8_t Payload[UART_PROTO_MAX_PAYLOAD_SIZE];
  uint8_t Checksum;
  uint8_t Footer;
} UART_Proto_Frame_t;

typedef void (*UART_Proto_CmdCallback_t)(uint8_t cmd, uint8_t* data, uint16_t len);

typedef enum
{
  PARSER_STATE_IDLE = 0,
  PARSER_STATE_WAIT_HEADER,
  PARSER_STATE_WAIT_COMMAND,
  PARSER_STATE_WAIT_LENGTH_L,
  PARSER_STATE_WAIT_LENGTH_H,
  PARSER_STATE_WAIT_PAYLOAD,
  PARSER_STATE_WAIT_CHECKSUM,
  PARSER_STATE_WAIT_FOOTER,
  PARSER_STATE_COMPLETE,
  PARSER_STATE_ERROR
} Parser_State_t;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint32_t BaudRate;
  uint8_t UseInterrupt;
  uint8_t UseDMA;
} UART_Proto_Config_t;

typedef struct
{
  UART_Proto_Config_t Config;
  UART_Proto_Frame_t RxFrame;
  UART_Proto_Frame_t TxFrame;
  Parser_State_t ParserState;
  uint16_t RxIndex;
  uint8_t IsFrameReady;
  uint8_t ChecksumError;
  
  uint8_t RxBuf[UART_PROTO_RX_BUFFER_SIZE];
  uint8_t TxBuf[UART_PROTO_TX_BUFFER_SIZE];
  volatile uint16_t RxCount;
  volatile uint8_t IsReceiving;
  
  UART_Proto_CmdCallback_t CmdCallback;
  uint8_t IsInitialized;
} UART_Proto_Handle_t;

/* Exported Functions --------------------------------------------------------*/

UART_Proto_Status_t UART_Proto_Init(UART_Proto_Handle_t *hproto, UART_Proto_Config_t *pConfig);

UART_Proto_Status_t UART_Proto_SendPacket(UART_Proto_Handle_t *hproto, uint8_t cmd, uint8_t *pData, uint16_t len);

UART_Proto_Status_t UART_Proto_Parse(UART_Proto_Handle_t *hproto, uint8_t data);

void UART_Proto_RegisterCallback(UART_Proto_Handle_t *hproto, UART_Proto_CmdCallback_t callback);

UART_Proto_Status_t UART_Proto_SendAck(UART_Proto_Handle_t *hproto, uint8_t cmd, uint8_t status);

UART_Proto_Status_t UART_Proto_Poll(UART_Proto_Handle_t *hproto);

void UART_Proto_IRQHandler(UART_Proto_Handle_t *hproto);

uint8_t UART_Proto_CalculateChecksum(UART_Proto_Frame_t *pFrame);

#ifdef __cplusplus
}
#endif

#endif /* __UART_PROTO_H */
