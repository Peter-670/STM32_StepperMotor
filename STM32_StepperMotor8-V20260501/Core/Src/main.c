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
#define LIMIT_RELEASE_FILTER_COUNT 20

#define LIMIT_CW_PIN GPIO_PIN_3
#define LIMIT_CCW_PIN GPIO_PIN_4
#define LIMIT_ORG_PIN GPIO_PIN_5
#define LIMIT_PORT GPIOB

#define DISP_HALF_CIRCLE 0.5f
#define DISP_FULL_CIRCLE 1.0f

#define DIR_TO_CW 0
#define DIR_TO_CCW 1

#define TEST_DEFAULT_GROUPS 20
#define TEST_REF_STEPS 40000UL
#define TEST_SEARCH_MAX_STEPS 120000UL
#define TEST_HALF_T DEFAULT_HALF_T_MID
#define TEST_BACKOFF_STEPS 800UL
/* USER CODE END PD */

uint8_t uart_buf[UART_BUF_SIZE];
uint16_t uart_buf_head = 0;
uint16_t uart_buf_tail = 0;

/* USER CODE BEGIN PV */
uint8_t motor_speed = 1;
uint8_t motor_dir = 0;
uint16_t motor_steps = 0;
uint8_t cmd_ready = 0;
uint8_t test_cmd_ready = 0;
uint8_t test_group_count = 0;

float motor_displacement = 0.0f;
uint8_t motor_stop_flag = 0;

uint8_t alarm_lock = 0;
uint8_t cw_limit_locked = 0;
uint8_t ccw_limit_locked = 0;
uint8_t cw_limit_armed = 1;
uint8_t ccw_limit_armed = 1;

uint8_t last_cw_limit = 0;
uint8_t last_ccw_limit = 0;
uint8_t last_org_limit = 0;
uint8_t cw_filter_cnt = 0;
uint8_t ccw_filter_cnt = 0;
uint8_t org_filter_cnt = 0;
uint8_t cw_release_cnt = 0;
uint8_t ccw_release_cnt = 0;
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
uint32_t Motor_Move_With_Monitor(uint8_t dir, uint32_t steps, uint32_t half_t, uint8_t stop_on_alarm);
void Run_Repeatability_Test(uint8_t groups);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next_head = (uart_buf_head + 1) % UART_BUF_SIZE;
    if (next_head != uart_buf_tail)
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
  UART_PutString("3. Test command: T or T20 (repeatability)\r\n");
  UART_PutString("> ");

  while (1)
  {
    UART_Parse_Cmd();
    Limit_Switch_Check();
    HAL_Delay(10);

    if (test_cmd_ready)
    {
      Run_Repeatability_Test(test_group_count);
      test_cmd_ready = 0;
      UART_PutString("> ");
      continue;
    }

    if (alarm_lock == 1)
      continue;

    if (cmd_ready)
    {
      // 鏍稿績閫昏緫淇锛?
      // 姝ｈ浆(0)琚獵W闄愪綅閿佷綇 -> 绂佹
      // 鍙嶈浆(1)琚獵CW闄愪綅閿佷綇 -> 绂佹
      if ((motor_dir == 0 && cw_limit_locked) || (motor_dir == 1 && ccw_limit_locked))
      {
        UART_PutString("禁止: 该方向已限位,仅允许反向转动\r\n>");
        cmd_ready = 0;
        continue;
      }

      motor_stop_flag = 0;
      cmd_ready = 0;
      Set_Direction(motor_dir);

      uint32_t delay = DEFAULT_HALF_T_LOW;
      if (motor_speed == 2)
        delay = DEFAULT_HALF_T_MID;
      else if (motor_speed == 3)
        delay = DEFAULT_HALF_T_HIGH;

      uint32_t moved_steps = Motor_Move_With_Monitor(motor_dir, motor_steps, delay, 1);

      if (!motor_stop_flag && moved_steps > 0)
      {
        Displacement_Update(motor_dir, moved_steps);
      }

      UART_PutString("鎵ц瀹屾垚: ");
      UART_PutChar(motor_speed + '0');
      UART_PutChar(motor_dir + '0');
      UART_PutChar((motor_steps == STEPS_PER_CIRCLE) ? '1' : '2');
      UART_PutString("\r\n褰撳墠浣嶇Щ: ");
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
// 澶嶄綅閫昏緫锛氬彧娓呴櫎鎶ヨ锛屼笉娓呴櫎闄愪綅閿佸畾锛岄槻姝㈠浣嶅悗绔嬪嵆鍐嶆鎶ヨ
void System_Reset(void)
{
  uint8_t cw_now = Read_Limit_Stable(LIMIT_PORT, LIMIT_CW_PIN);
  uint8_t ccw_now = Read_Limit_Stable(LIMIT_PORT, LIMIT_CCW_PIN);

  alarm_lock = 0;
  motor_stop_flag = 0;
  cmd_ready = 0;
  cw_filter_cnt = 0;
  ccw_filter_cnt = 0;

  if (cw_now == 0)
  {
    cw_limit_locked = 0;
    cw_limit_armed = 1;
    cw_release_cnt = 0;
    last_cw_limit = 0;
  }

  if (ccw_now == 0)
  {
    ccw_limit_locked = 0;
    ccw_limit_armed = 1;
    ccw_release_cnt = 0;
    last_ccw_limit = 0;
  }
  UART_PutString("系统: 报警已复位,仅允许反向转动\r\n> ");
}

uint8_t Read_Limit_Stable(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  return (uint8_t)HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

void HAL_Delay_us(uint32_t us)
{
  uint32_t cnt = us * (SystemCoreClock / 1000000) / 4;
  while (cnt--)
    ;
}

void UART_PutChar(uint8_t ch)
{
  HAL_UART_Transmit(&huart1, &ch, 1, 10);
}

void UART_PutString(char *str)
{
  if (str == NULL)
    return;
  uint16_t len = strlen(str);
  HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 100);
}

void UART_Parse_Cmd(void)
{
  uint16_t data_len = (uart_buf_head - uart_buf_tail + UART_BUF_SIZE) % UART_BUF_SIZE;
  if (data_len == 0)
    return;

  uint8_t cmd = uart_buf[uart_buf_tail];
  if (cmd == '\r' || cmd == '\n')
  {
    uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
    return;
  }

  if (cmd == 'R' || cmd == 'r')
  {
    System_Reset();
    uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
    return;
  }

  if (cmd == 'T' || cmd == 't')
  {
    uint8_t groups = TEST_DEFAULT_GROUPS;
    uint8_t consume = 1;

    if (data_len >= 3)
    {
      uint8_t d1 = uart_buf[(uart_buf_tail + 1) % UART_BUF_SIZE];
      uint8_t d2 = uart_buf[(uart_buf_tail + 2) % UART_BUF_SIZE];
      if ((d1 >= '0' && d1 <= '9') && (d2 >= '0' && d2 <= '9'))
      {
        groups = (uint8_t)((d1 - '0') * 10 + (d2 - '0'));
        consume = 3;
      }
    }
    else if (data_len >= 2)
    {
      uint8_t d1 = uart_buf[(uart_buf_tail + 1) % UART_BUF_SIZE];
      if (d1 >= '1' && d1 <= '9')
      {
        groups = (uint8_t)(d1 - '0');
        consume = 2;
      }
    }

    if (groups == 0)
    {
      UART_PutString("Error: invalid test group count\r\n> ");
      uart_buf_tail = (uart_buf_tail + consume) % UART_BUF_SIZE;
      return;
    }

    test_group_count = groups;
    test_cmd_ready = 1;
    cmd_ready = 0;
    uart_buf_tail = (uart_buf_tail + consume) % UART_BUF_SIZE;
    return;
  }

  if (data_len >= 3)
  {
    uint8_t s = uart_buf[uart_buf_tail];
    uint8_t d = uart_buf[(uart_buf_tail + 1) % UART_BUF_SIZE];
    uint8_t c = uart_buf[(uart_buf_tail + 2) % UART_BUF_SIZE];

    if ((s >= '1' && s <= '3') && (d >= '0' && d <= '1') && (c >= '1' && c <= '2'))
    {
      motor_speed = s - '0';
      motor_dir = d - '0';
      motor_steps = (c == '1') ? STEPS_PER_CIRCLE : STEPS_PER_HALF_CIRCLE;
      cmd_ready = 1;
      uart_buf_tail = (uart_buf_tail + 3) % UART_BUF_SIZE;
    }
    else
    {
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

// 鏍稿績閫昏緫淇锛?
// 1. CW闄愪綅锛堟杞檺浣嶏級锛氳Е鍙戝悗閿佸畾CW锛屽彧鍏佽鍙嶈浆锛堢寮€闄愪綅锛夈€?
// 2. CCW闄愪綅锛堝弽杞檺浣嶏級锛氳Е鍙戝悗閿佸畾CCW锛屽彧鍏佽姝ｈ浆锛堢寮€闄愪綅锛夈€?
void Limit_Switch_Check(void)
{
  uint8_t cw_now = Read_Limit_Stable(LIMIT_PORT, LIMIT_CW_PIN);
  uint8_t ccw_now = Read_Limit_Stable(LIMIT_PORT, LIMIT_CCW_PIN);
  uint8_t org_now = Read_Limit_Stable(LIMIT_PORT, LIMIT_ORG_PIN);

  // --- CW limit ---
  if (cw_now == 1)
  {
    cw_release_cnt = 0;
    if (cw_limit_armed && !cw_limit_locked)
    {
      cw_filter_cnt++;
      if (cw_filter_cnt >= LIMIT_FILTER_COUNT)
      {
        last_cw_limit = 1;
        cw_limit_locked = 1;
        cw_limit_armed = 0;
        Motor_Emergency_Stop();
        Alarm_Info("CW limit triggered, send R to reset");
      }
    }
  }
  else
  {
    cw_filter_cnt = 0;
    last_cw_limit = 0;
    if (!cw_limit_armed)
    {
      if (cw_release_cnt < 0xFF)
        cw_release_cnt++;
      if (cw_release_cnt >= LIMIT_RELEASE_FILTER_COUNT)
      {
        cw_limit_armed = 1;
        if (alarm_lock == 0)
          cw_limit_locked = 0;
      }
    }
    else if (alarm_lock == 0)
    {
      cw_limit_locked = 0;
    }
  }

  // --- CCW limit ---
  if (ccw_now == 1)
  {
    ccw_release_cnt = 0;
    if (ccw_limit_armed && !ccw_limit_locked)
    {
      ccw_filter_cnt++;
      if (ccw_filter_cnt >= LIMIT_FILTER_COUNT)
      {
        last_ccw_limit = 1;
        ccw_limit_locked = 1;
        ccw_limit_armed = 0;
        Motor_Emergency_Stop();
        Alarm_Info("CCW limit triggered, send R to reset");
      }
    }
  }
  else
  {
    ccw_filter_cnt = 0;
    last_ccw_limit = 0;
    if (!ccw_limit_armed)
    {
      if (ccw_release_cnt < 0xFF)
        ccw_release_cnt++;
      if (ccw_release_cnt >= LIMIT_RELEASE_FILTER_COUNT)
      {
        ccw_limit_armed = 1;
        if (alarm_lock == 0)
          ccw_limit_locked = 0;
      }
    }
    else if (alarm_lock == 0)
    {
      ccw_limit_locked = 0;
    }
  }

  // --- ORG ---
  if (org_now == 1)
  {
    org_filter_cnt++;
    if (org_filter_cnt >= LIMIT_FILTER_COUNT && last_org_limit == 0)
    {
      last_org_limit = 1;
      motor_displacement = 0.0f;
      UART_PutString("Tip: ORG triggered, displacement reset\r\n> ");
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
  if (dir == 0) // 姝ｈ浆
  {
    motor_displacement += step2mm;
  }
  else // 鍙嶈浆
  {
    motor_displacement -= step2mm;
  }
  char buf[64];
  sprintf(buf, "褰撳墠浣嶇疆: %.1f mm\r\n", motor_displacement);
  UART_PutString(buf);
}
uint32_t Motor_Move_With_Monitor(uint8_t dir, uint32_t steps, uint32_t half_t, uint8_t stop_on_alarm)
{
  uint32_t moved = 0;
  Set_Direction(dir);

  while (moved < steps)
  {
    if (stop_on_alarm && alarm_lock)
      break;

    Step(1, half_t);
    moved++;

    Limit_Switch_Check();

    if (stop_on_alarm && alarm_lock)
      break;
  }

  return moved;
}

void Run_Repeatability_Test(uint8_t groups)
{
  uint8_t i = 0;
  int32_t total_delta = 0;
  uint32_t total_abs_delta = 0;
  uint8_t ok_count = 0;

  UART_PutString("\r\n=== Repeatability Test Start ===\r\n");
  UART_PutString("Sequence: CCW limit -> +40000 -> CCW limit\r\n");

  for (i = 0; i < groups; i++)
  {
    char msg[128];
    sprintf(msg, "\r\n[Group %u/%u]\r\n", (unsigned int)(i + 1), (unsigned int)groups);
    UART_PutString(msg);

    System_Reset();
    UART_PutString("Phase0: backoff from CCW...\r\n");
    uint32_t backoff_steps = Motor_Move_With_Monitor(DIR_TO_CW, TEST_BACKOFF_STEPS, TEST_HALF_T, 1);
    if (alarm_lock)
    {
      sprintf(msg, "FAIL: alarm in phase0, pulses=%lu\r\n", (unsigned long)backoff_steps);
      UART_PutString(msg);
      continue;
    }

    System_Reset();

    UART_PutString("Phase1: move to CCW limit...\r\n");
    (void)Motor_Move_With_Monitor(DIR_TO_CCW, TEST_SEARCH_MAX_STEPS, TEST_HALF_T, 1);
    if (!alarm_lock || !ccw_limit_locked)
    {
      UART_PutString("FAIL: CCW limit not reached in phase1\r\n");
      continue;
    }

    System_Reset();

    UART_PutString("Phase2: move toward ORG for 40000 pulses...\r\n");
    uint32_t out_steps = Motor_Move_With_Monitor(DIR_TO_CW, TEST_REF_STEPS, TEST_HALF_T, 1);
    if (alarm_lock)
    {
      sprintf(msg, "FAIL: unexpected alarm in phase2, pulses=%lu\r\n", (unsigned long)out_steps);
      UART_PutString(msg);
      continue;
    }

    UART_PutString("Phase3: move toward CCW until limit alarm...\r\n");
    uint32_t back_steps = Motor_Move_With_Monitor(DIR_TO_CCW, TEST_SEARCH_MAX_STEPS, TEST_HALF_T, 1);
    if (!alarm_lock || !ccw_limit_locked)
    {
      sprintf(msg, "FAIL: CCW limit not reached in phase3, pulses=%lu\r\n", (unsigned long)back_steps);
      UART_PutString(msg);
      continue;
    }

    int32_t delta = (int32_t)back_steps - (int32_t)TEST_REF_STEPS;
    uint32_t abs_delta = (uint32_t)((delta >= 0) ? delta : -delta);
    total_delta += delta;
    total_abs_delta += abs_delta;
    ok_count++;

    sprintf(msg,
            "Result: back=%lu, ref=%lu, delta=%ld, abs=%lu\r\n",
            (unsigned long)back_steps,
            (unsigned long)TEST_REF_STEPS,
            (long)delta,
            (unsigned long)abs_delta);
    UART_PutString(msg);
  }

  UART_PutString("\r\n=== Repeatability Test Summary ===\r\n");
  if (ok_count == 0)
  {
    UART_PutString("No valid group completed.\r\n");
  }
  else
  {
    char summary[128];
    int32_t avg_delta = total_delta / ok_count;
    uint32_t avg_abs_delta = total_abs_delta / ok_count;
    sprintf(summary,
            "Valid=%u/%u, avg_delta=%ld, avg_abs_delta=%lu\r\n",
            (unsigned int)ok_count,
            (unsigned int)groups,
            (long)avg_delta,
            (unsigned long)avg_abs_delta);
    UART_PutString(summary);
  }
  UART_PutString("=== Repeatability Test End ===\r\n");
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
