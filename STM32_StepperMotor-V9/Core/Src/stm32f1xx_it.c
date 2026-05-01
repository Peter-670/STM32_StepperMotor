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
#define UART_BUF_SIZE 64  // must match main.c
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
extern uint8_t  uart_buf[UART_BUF_SIZE];
extern uint16_t uart_buf_head;
extern uint16_t uart_buf_tail;
extern UART_HandleTypeDef huart1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)
{
  while (1) {}
}

void HardFault_Handler(void)
{
  while (1) {}
}

void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief USART1 ISR: ring-buffer RX with overflow protection.
  *        Direct DR read avoids HAL callback dual-processing.
  *        RXNE IT re-enabled after HAL_UART_IRQHandler in case HAL clears it.
  */
void USART1_IRQHandler(void)
{
  if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
  {
    uint8_t rx_data = (uint8_t)(huart1.Instance->DR);
    uint16_t next_head = (uart_buf_head + 1) % UART_BUF_SIZE;
    if(next_head != uart_buf_tail)
    {
      uart_buf[uart_buf_head] = rx_data;
      uart_buf_head = next_head;
    }
  }
  HAL_UART_IRQHandler(&huart1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}
