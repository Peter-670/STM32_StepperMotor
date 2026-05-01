/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stepper.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define DEFAULT_HALF_T_LOW 500
#define DEFAULT_HALF_T_MID 250
#define DEFAULT_HALF_T_HIGH 180

#define STEPS_PER_CIRCLE 6400
#define STEPS_PER_HALF_CIRCLE 3200
#define UART_BUF_SIZE 64

#define LIMIT_FILTER_COUNT 3

#define LIMIT_CW_PIN GPIO_PIN_3
#define LIMIT_CCW_PIN GPIO_PIN_4
#define LIMIT_ORG_PIN GPIO_PIN_5
#define LIMIT_PORT GPIOB

#define DISP_HALF_CIRCLE 0.5f
#define DISP_FULL_CIRCLE 1.0f
/* USER CODE END PD */

uint8_t uart_buf[UART_BUF_SIZE];
uint16_t uart_buf_head = 0;
uint16_t uart_buf_tail = 0;

/* USER CODE BEGIN PV */
uint8_t motor_speed = 1;
uint8_t motor_dir = 0;
uint16_t motor_steps = 0;
uint8_t cmd_ready = 0;

float motor_displacement = 0.0f;
uint8_t motor_stop_flag = 0;

uint8_t alarm_lock = 0;
uint8_t cw_limit_locked = 0;
uint8_t ccw_limit_locked = 0;

uint8_t last_cw_limit = 0;
uint8_t last_ccw_limit = 0;
uint8_t last_org_limit = 0;
uint8_t cw_filter_cnt = 0;
uint8_t ccw_filter_cnt = 0;
uint8_t org_filter_cnt = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_Parse_Cmd(void);
void UART_PutChar(uint8_t ch);
void UART_PutString(char *str);
void HAL_Delay_us(uint32_t us);
void Motor_Emergency_Stop(void);
void Limit_Switch_Check(void);
void Alarm_Info(char *str);
void Displacement_Update(uint8_t dir, uint32_t steps);
uint8_t Read_Limit_Stable(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void System_Reset(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    uint16_t next_head = (uart_buf_head + 1) % UART_BUF_SIZE;
    if(next_head != uart_buf_tail)
    {
      uart_buf[uart_buf_head] = huart->Instance->DR;
      uart_buf_head = next_head;
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  Step_GPIO_Config();
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  HAL_Delay(100);
  last_cw_limit = 0;
  last_ccw_limit = 0;
  last_org_limit = 0;

  UART_PutString("\r\n=============================\r\n");
  UART_PutString("电机控制系统初始化完成\r\n");
  UART_PutString("指令规则:\r\n");
  UART_PutString("1. 3位数字指令: 速度方向圈数\r\n");
  UART_PutString("   速度: 1=低速  2=中速  3=高速\r\n");
  UART_PutString("   方向: 0=正转  1=反转\r\n");
  UART_PutString("   圈数: 1=整圈  2=半圈\r\n");
  UART_PutString("2. 复位指令: 发送R解除报警\r\n");
  UART_PutString("=============================\r\n");
  UART_PutString("> ");

  while (1)
  {
    UART_Parse_Cmd();
    Limit_Switch_Check();
    HAL_Delay(10);

    if(alarm_lock == 1)
      continue;

    if(cmd_ready)
    {
      // 核心逻辑修正：
      // 正转(0)被CW限位锁住 -> 禁止
      // 反转(1)被CCW限位锁住 -> 禁止
      if((motor_dir == 0 && cw_limit_locked) || (motor_dir == 1 && ccw_limit_locked))
      {
        UART_PutString("禁止: 该方向已限位,仅允许反向转动\r\n> ");
        cmd_ready = 0;
        continue;
      }

      motor_stop_flag = 0;
      cmd_ready = 0;
      Set_Direction(motor_dir);

      uint32_t delay = DEFAULT_HALF_T_LOW;
      if(motor_speed ==2) delay = DEFAULT_HALF_T_MID;
      else if(motor_speed ==3) delay = DEFAULT_HALF_T_HIGH;

      Step(motor_steps, delay);

      if(!motor_stop_flag)
      {
        Displacement_Update(motor_dir, motor_steps);
      }

      UART_PutString("执行完成: ");
      UART_PutChar(motor_speed + '0');
      UART_PutChar(motor_dir + '0');
      UART_PutChar((motor_steps == STEPS_PER_CIRCLE)?'1':'2');
      UART_PutString("\r\n当前位移: ");
      char disp_buf[32];
      sprintf(disp_buf, "%.1f mm\r\n", motor_displacement);
      UART_PutString(disp_buf);
      UART_PutString("> ");
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 复位逻辑：只清除报警，不清除限位锁定，防止复位后立即再次报警
void System_Reset(void)
{
  alarm_lock = 0;
  motor_stop_flag = 0;
  last_cw_limit = 0;
  last_ccw_limit = 0;
  cmd_ready = 0;
  UART_PutString("系统: 报警已复位,仅允许反向转动\r\n> ");
}

uint8_t Read_Limit_Stable(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  uint8_t level1 = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  HAL_Delay(2);
  uint8_t level2 = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  return (level1 == level2) ? level1 : 0;
}

void HAL_Delay_us(uint32_t us)
{
  uint32_t cnt = us * (SystemCoreClock / 1000000) / 4;
  while(cnt--);
}

void UART_PutChar(uint8_t ch)
{
  HAL_UART_Transmit(&huart1, &ch, 1, 10);
}

void UART_PutString(char *str)
{
  if(str == NULL) return;
  uint16_t len = strlen(str);
  HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 100);
}

void UART_Parse_Cmd(void)
{
  uint16_t data_len = (uart_buf_head - uart_buf_tail + UART_BUF_SIZE) % UART_BUF_SIZE;
  if(data_len == 0) return;

  uint8_t cmd = uart_buf[uart_buf_tail];
  if(cmd == '\r' || cmd == '\n')
  {
    uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
    return;
  }

  if(cmd == 'R' || cmd == 'r')
  {
    System_Reset();
    uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
    return;
  }

  if(data_len >=3)
  {
    uint8_t s = uart_buf[uart_buf_tail];
    uint8_t d = uart_buf[(uart_buf_tail+1)%UART_BUF_SIZE];
    uint8_t c = uart_buf[(uart_buf_tail+2)%UART_BUF_SIZE];

    if((s>='1'&&s<='3')&&(d>='0'&&d<='1')&&(c>='1'&&c<='2'))
    {
      motor_speed = s-'0';
      motor_dir = d-'0';
      motor_steps = (c=='1')? STEPS_PER_CIRCLE:STEPS_PER_HALF_CIRCLE;
      cmd_ready =1;
      uart_buf_tail = (uart_buf_tail + 3) % UART_BUF_SIZE;
    }else{
      uart_buf_head = uart_buf_tail = 0;
      UART_PutString("错误:无效指令\r\n> ");
    }
  }
}

void Alarm_Info(char *str)
{
  UART_PutString("报警: ");
  UART_PutString(str);
  UART_PutString("\r\n> ");
}

void Motor_Emergency_Stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  motor_stop_flag = 1;
  alarm_lock = 1;
}

// 核心逻辑修正：
// 1. CW限位（正转限位）：触发后锁定CW，只允许反转（离开限位）。
// 2. CCW限位（反转限位）：触发后锁定CCW，只允许正转（离开限位）。
void Limit_Switch_Check(void)
{
  uint8_t cw_now = Read_Limit_Stable(LIMIT_PORT, LIMIT_CW_PIN);
  uint8_t ccw_now = Read_Limit_Stable(LIMIT_PORT, LIMIT_CCW_PIN);
  uint8_t org_now = Read_Limit_Stable(LIMIT_PORT, LIMIT_ORG_PIN);

  // --- 正转限位逻辑 (CW) ---
  // 如果CW限位被触发，且尚未锁定
  if(cw_now == 1 && !cw_limit_locked)
  {
    cw_filter_cnt++;
    if(cw_filter_cnt >= LIMIT_FILTER_COUNT && last_cw_limit == 0)
    {
      last_cw_limit = 1;
      cw_limit_locked = 1; // 锁定正转方向
      Motor_Emergency_Stop();
      Alarm_Info("正转限位触发 发送R复位");
    }
  }
  // 只有在没有报警锁定的情况下（即复位后），且限位开关松开，才解除锁定
  else if(cw_now == 0 && alarm_lock == 0)
  {
    cw_filter_cnt = 0;
    cw_limit_locked = 0;
  }

  // --- 反转限位逻辑 (CCW) ---
  // 如果CCW限位被触发，且尚未锁定
  if(ccw_now == 1 && !ccw_limit_locked)
  {
    ccw_filter_cnt++;
    if(ccw_filter_cnt >= LIMIT_FILTER_COUNT && last_ccw_limit == 0)
    {
      last_ccw_limit = 1;
      ccw_limit_locked = 1; // 锁定反转方向
      Motor_Emergency_Stop();
      Alarm_Info("反转限位触发 发送R复位");
    }
  }
  // 只有在没有报警锁定的情况下（即复位后），且限位开关松开，才解除锁定
  else if(ccw_now == 0 && alarm_lock == 0)
  {
    ccw_filter_cnt = 0;
    ccw_limit_locked = 0;
  }

  // --- 原点逻辑 ---
  if(org_now == 1)
  {
    org_filter_cnt++;
    if(org_filter_cnt >= LIMIT_FILTER_COUNT && last_org_limit == 0)
    {
      last_org_limit = 1;
      motor_displacement = 0.0f;
      UART_PutString("提示: 原点触发 位移已清零\r\n> ");
    }
  }
  else
  {
    org_filter_cnt = 0;
    last_org_limit = 0;
  }
}

void Displacement_Update(uint8_t dir, uint32_t steps)
{
  float step2mm = (float)steps / STEPS_PER_CIRCLE * DISP_FULL_CIRCLE;
  if(dir == 0) // 正转
  {
    motor_displacement += step2mm;
  }
  else // 反转
  {
    motor_displacement -= step2mm;
  }
  char buf[64];
  sprintf(buf, "当前位置: %.1f mm\r\n", motor_displacement);
  UART_PutString(buf);
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while(1){}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif