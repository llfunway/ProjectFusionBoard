/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* External Global Variables */
extern volatile uint32_t g_SystemTick;
extern volatile uint8_t g_IsInitialized;
extern volatile uint8_t g_UartReady;

/* Include UART protocol header for type definition */
#include "uart_proto.h"

/* External Function Prototypes */
extern void UART_Proto_IRQHandler(UART_Proto_Handle_t *hproto);
extern UART_Proto_Handle_t hUART_Proto;

#include <stdio.h>

/*
 * Fault capture structure placed in a retained section.
 * On Cortex-M3 the stacked registers are at the SP value when the exception
 * was taken.  The captured values help post-mortem diagnosis.
 */
typedef struct {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t psr;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t mmfar;
  uint32_t bfar;
  uint8_t  fault_type; /* 1=HardFault, 2=MemManage, 3=BusFault, 4=UsageFault */
} FaultCapture_t;

static FaultCapture_t g_FaultCapture;

static void FaultCaptureAndReport(uint8_t fault_type)
{
  uint32_t *stack_frame;
  uint32_t stacked_pc, stacked_lr;

  /* Determine which stack was in use when the exception occurred.
   * Bit 2 of the exception LR (EXC_RETURN) indicates the stack: 0=MSP, 1=PSP. */
  register uint32_t exc_lr __ASM("lr");

  if (exc_lr & 0x04)
  {
    stack_frame = (uint32_t *)__get_PSP();
  }
  else
  {
    stack_frame = (uint32_t *)__get_MSP();
  }

  stacked_pc = stack_frame[6];
  stacked_lr = stack_frame[5];

  g_FaultCapture.r0  = stack_frame[0];
  g_FaultCapture.r1  = stack_frame[1];
  g_FaultCapture.r2  = stack_frame[2];
  g_FaultCapture.r3  = stack_frame[3];
  g_FaultCapture.r12 = stack_frame[4];
  g_FaultCapture.lr  = stacked_lr;
  g_FaultCapture.pc  = stacked_pc;
  g_FaultCapture.psr = stack_frame[7];
  g_FaultCapture.cfsr = (*((volatile uint32_t *)0xE000ED28));
  g_FaultCapture.hfsr = (*((volatile uint32_t *)0xE000ED2C));
  g_FaultCapture.mmfar = (*((volatile uint32_t *)0xE000ED34));
  g_FaultCapture.bfar = (*((volatile uint32_t *)0xE000ED38));
  g_FaultCapture.fault_type = fault_type;

  if (g_UartReady)
  {
    char buf[80];
    int n;
    n = snprintf(buf, sizeof(buf),
                 "FLT%u:PC=%08lX,LR=%08lX,CFSR=%08lX,HFSR=%08lX\r\n",
                 (unsigned long)fault_type,
                 (unsigned long)stacked_pc,
                 (unsigned long)stacked_lr,
                 (unsigned long)g_FaultCapture.cfsr,
                 (unsigned long)g_FaultCapture.hfsr);
    if (n > 0)
    {
      for (int i = 0; i < n; i++)
      {
        while (!(USART1->SR & USART_SR_TXE)) {}
        USART1->DR = (uint8_t)buf[i];
      }
    }
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  FaultCaptureAndReport(1);
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  FaultCaptureAndReport(2);
  while (1)
  {
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  FaultCaptureAndReport(3);
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  FaultCaptureAndReport(4);
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  
  /* Increment system tick counter */
  g_SystemTick++;
  
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/*                 STM32F1xx Peripheral Interrupt Handlers                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
#ifdef USART1_IRQn
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  
  /* Call UART protocol handler if system is initialized */
  if (g_IsInitialized)
  {
    UART_Proto_IRQHandler(&hUART_Proto);
  }
  
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
#endif

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
