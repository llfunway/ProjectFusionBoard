/**
  ******************************************************************************
  * @file    uart_proto.c
  * @author  Embedded System Team
  * @brief   UART Communication Protocol Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart_proto.h"
#include "main.h"
#include <string.h>

/* Private Function Prototypes -----------------------------------------------*/
static void UART_Proto_ProcessFrame(UART_Proto_Handle_t *hproto);

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initialize UART protocol handler
  */
UART_Proto_Status_t UART_Proto_Init(UART_Proto_Handle_t *hproto, UART_Proto_Config_t *pConfig)
{
  if (hproto == NULL || pConfig == NULL)
  {
    return UART_PROTO_ERROR_PARAM;
  }
  
  if (pConfig->huart == NULL)
  {
    return UART_PROTO_ERROR_PARAM;
  }
  
  hproto->Config = *pConfig;
  hproto->ParserState = PARSER_STATE_IDLE;
  hproto->RxIndex = 0;
  hproto->IsFrameReady = 0;
  hproto->ChecksumError = 0;
  hproto->CmdCallback = NULL;
  hproto->RxCount = 0;
  hproto->IsReceiving = 0;
  
  for (uint16_t i = 0; i < UART_PROTO_RX_BUFFER_SIZE; i++)
  {
    hproto->RxBuf[i] = 0;
  }
  
  for (uint16_t i = 0; i < UART_PROTO_TX_BUFFER_SIZE; i++)
  {
    hproto->TxBuf[i] = 0;
  }
  
  hproto->IsInitialized = 1;
  
  if (hproto->Config.UseInterrupt)
  {
    __HAL_UART_ENABLE_IT(hproto->Config.huart, UART_IT_RXNE);
  }
  
  return UART_PROTO_OK;
}

/**
  * @brief  Send data packet with custom frame format
  */
UART_Proto_Status_t UART_Proto_SendPacket(UART_Proto_Handle_t *hproto, uint8_t cmd, uint8_t *pData, uint16_t len)
{
  if (hproto == NULL || !hproto->IsInitialized)
  {
    return UART_PROTO_ERROR_NOT_INIT;
  }
  
  if (len > UART_PROTO_MAX_PAYLOAD_SIZE)
  {
    return UART_PROTO_ERROR_PARAM;
  }
  
  hproto->TxFrame.Header = UART_PROTO_FRAME_HEADER;
  hproto->TxFrame.Command = cmd;
  hproto->TxFrame.Length = len;
  
  for (uint16_t i = 0; i < len; i++)
  {
    hproto->TxFrame.Payload[i] = pData[i];
  }
  
  hproto->TxFrame.Checksum = UART_Proto_CalculateChecksum(&hproto->TxFrame);
  hproto->TxFrame.Footer = UART_PROTO_FRAME_FOOTER;
  
  uint8_t txBuffer[UART_PROTO_FRAME_MAX_SIZE];
  uint16_t txSize = 0;
  
  txBuffer[txSize++] = hproto->TxFrame.Header;
  txBuffer[txSize++] = hproto->TxFrame.Command;
  txBuffer[txSize++] = (hproto->TxFrame.Length >> 8) & 0xFF;
  txBuffer[txSize++] = hproto->TxFrame.Length & 0xFF;
  
  for (uint16_t i = 0; i < len; i++)
  {
    txBuffer[txSize++] = hproto->TxFrame.Payload[i];
  }
  
  txBuffer[txSize++] = hproto->TxFrame.Checksum;
  txBuffer[txSize++] = hproto->TxFrame.Footer;
  
  if (hproto->Config.UseDMA)
  {
    if (HAL_UART_Transmit_DMA(hproto->Config.huart, txBuffer, txSize) != HAL_OK)
    {
      return UART_PROTO_ERROR_TIMEOUT;
    }
  }
  else
  {
    if (HAL_UART_Transmit(hproto->Config.huart, txBuffer, txSize, 100) != HAL_OK)
    {
      return UART_PROTO_ERROR_TIMEOUT;
    }
  }
  
  return UART_PROTO_OK;
}

/**
  * @brief  Parse received byte (state machine)
  */
UART_Proto_Status_t UART_Proto_Parse(UART_Proto_Handle_t *hproto, uint8_t data)
{
  if (hproto == NULL || !hproto->IsInitialized)
  {
    return UART_PROTO_ERROR_NOT_INIT;
  }
  
  hproto->IsFrameReady = 0;
  
  switch (hproto->ParserState)
  {
    case PARSER_STATE_IDLE:
    case PARSER_STATE_WAIT_HEADER:
      if (data == UART_PROTO_FRAME_HEADER)
      {
        hproto->RxFrame.Header = data;
        hproto->ParserState = PARSER_STATE_WAIT_COMMAND;
        hproto->RxIndex = 0;
      }
      break;
    
    case PARSER_STATE_WAIT_COMMAND:
      hproto->RxFrame.Command = data;
      hproto->ParserState = PARSER_STATE_WAIT_LENGTH_L;
      break;
    
    case PARSER_STATE_WAIT_LENGTH_L:
      hproto->RxFrame.Length = (uint16_t)data << 8;
      hproto->ParserState = PARSER_STATE_WAIT_LENGTH_H;
      break;
    
    case PARSER_STATE_WAIT_LENGTH_H:
      hproto->RxFrame.Length |= data;
      
      if (hproto->RxFrame.Length > UART_PROTO_MAX_PAYLOAD_SIZE)
      {
        hproto->ParserState = PARSER_STATE_ERROR;
      }
      else if (hproto->RxFrame.Length == 0)
      {
        hproto->ParserState = PARSER_STATE_WAIT_CHECKSUM;
      }
      else
      {
        hproto->ParserState = PARSER_STATE_WAIT_PAYLOAD;
        hproto->RxIndex = 0;
      }
      break;
    
    case PARSER_STATE_WAIT_PAYLOAD:
      if (hproto->RxIndex < hproto->RxFrame.Length)
      {
        hproto->RxFrame.Payload[hproto->RxIndex++] = data;
        
        if (hproto->RxIndex >= hproto->RxFrame.Length)
        {
          hproto->ParserState = PARSER_STATE_WAIT_CHECKSUM;
        }
      }
      break;
    
    case PARSER_STATE_WAIT_CHECKSUM:
      hproto->RxFrame.Checksum = data;
      hproto->ParserState = PARSER_STATE_WAIT_FOOTER;
      break;
    
    case PARSER_STATE_WAIT_FOOTER:
      if (data == UART_PROTO_FRAME_FOOTER)
      {
        hproto->RxFrame.Footer = data;
        
        if (UART_Proto_CalculateChecksum(&hproto->RxFrame) == hproto->RxFrame.Checksum)
        {
          hproto->IsFrameReady = 1;
          UART_Proto_ProcessFrame(hproto);
        }
        else
        {
          hproto->ChecksumError = 1;
        }
      }
      else
      {
        hproto->ParserState = PARSER_STATE_ERROR;
      }
      
      hproto->ParserState = PARSER_STATE_IDLE;
      break;
    
    case PARSER_STATE_ERROR:
    default:
      if (data == UART_PROTO_FRAME_HEADER)
      {
        hproto->RxFrame.Header = data;
        hproto->ParserState = PARSER_STATE_WAIT_COMMAND;
        hproto->RxIndex = 0;
      }
      else
      {
        hproto->ParserState = PARSER_STATE_IDLE;
      }
      break;
  }
  
  return UART_PROTO_OK;
}

/**
  * @brief  Register command callback function
  */
void UART_Proto_RegisterCallback(UART_Proto_Handle_t *hproto, UART_Proto_CmdCallback_t callback)
{
  if (hproto != NULL && hproto->IsInitialized)
  {
    hproto->CmdCallback = callback;
  }
}

/**
  * @brief  Send ACK/NACK response
  */
UART_Proto_Status_t UART_Proto_SendAck(UART_Proto_Handle_t *hproto, uint8_t cmd, uint8_t status)
{
  uint8_t ackData[2];
  ackData[0] = cmd;
  ackData[1] = status;
  
  if (status == 0)
  {
    return UART_Proto_SendPacket(hproto, UART_PROTO_CMD_ACK, ackData, 2);
  }
  else
  {
    return UART_Proto_SendPacket(hproto, UART_PROTO_CMD_NACK, ackData, 2);
  }
}

/**
  * @brief  Poll UART protocol handler (call in main loop)
  */
UART_Proto_Status_t UART_Proto_Poll(UART_Proto_Handle_t *hproto)
{
  if (hproto == NULL || !hproto->IsInitialized)
  {
    return UART_PROTO_ERROR_NOT_INIT;
  }
  
  while (hproto->IsReceiving && hproto->RxCount > 0)
  {
    uint8_t data = 0;

    __disable_irq();
    if (hproto->RxCount > 0)
    {
      data = hproto->RxBuf[0];
      if (hproto->RxCount > 1)
      {
        memmove(&hproto->RxBuf[0], &hproto->RxBuf[1], hproto->RxCount - 1);
      }
      hproto->RxCount--;
      if (hproto->RxCount == 0)
      {
        hproto->IsReceiving = 0;
      }
    }
    __enable_irq();

    UART_Proto_Parse(hproto, data);
  }
  
  return UART_PROTO_OK;
}

/**
  * @brief  UART interrupt handler (call from stm32f1xx_it.c)
  */
void UART_Proto_IRQHandler(UART_Proto_Handle_t *hproto)
{
  if (hproto == NULL || !hproto->IsInitialized)
  {
    return;
  }
  
  UART_HandleTypeDef *huart = hproto->Config.huart;
  
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
  {
    if (hproto->RxCount < UART_PROTO_RX_BUFFER_SIZE)
    {
      hproto->RxBuf[hproto->RxCount] = (uint8_t)(huart->Instance->DR & 0xFF);
      hproto->RxCount++;
      hproto->IsReceiving = 1;
    }
    
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
  }
  
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
    (void)huart->Instance->DR; /* Clear ORE by reading DR */
  }
}

/**
  * @brief  Calculate checksum for frame
  */
uint8_t UART_Proto_CalculateChecksum(UART_Proto_Frame_t *pFrame)
{
  uint8_t checksum = 0;
  
  checksum ^= pFrame->Header;
  checksum ^= pFrame->Command;
  checksum ^= (pFrame->Length >> 8) & 0xFF;
  checksum ^= pFrame->Length & 0xFF;
  
  for (uint16_t i = 0; i < pFrame->Length; i++)
  {
    checksum ^= pFrame->Payload[i];
  }
  
  return checksum;
}

/* Private Functions ---------------------------------------------------------*/

/**
  * @brief  Process valid received frame
  */
static void UART_Proto_ProcessFrame(UART_Proto_Handle_t *hproto)
{
  if (hproto->CmdCallback != NULL)
  {
    hproto->CmdCallback(hproto->RxFrame.Command, 
                        hproto->RxFrame.Payload, 
                        hproto->RxFrame.Length);
  }
}
