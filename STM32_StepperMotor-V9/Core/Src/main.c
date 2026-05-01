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
// Speed presets: half-period delay (smaller = faster)
#define DEFAULT_HALF_T_LOW   500
#define DEFAULT_HALF_T_MID   250
#define DEFAULT_HALF_T_HIGH  180

#define UART_BUF_SIZE 64

// Limit debounce filter count
#define LIMIT_FILTER_COUNT  3

// Limit pins (use CubeMX-generated names internally, A/B aliases for clarity)
#define LIMIT_A_PIN   CW_Pin
#define LIMIT_A_PORT  CW_GPIO_Port
#define LIMIT_B_PIN   CCW_Pin
#define LIMIT_B_PORT  CCW_GPIO_Port
#define LIMIT_ORG_PIN  ORG_Pin
#define LIMIT_ORG_PORT ORG_GPIO_Port
/* USER CODE END PD */

uint8_t  uart_buf[UART_BUF_SIZE];
uint16_t uart_buf_head = 0;
uint16_t uart_buf_tail = 0;

/* USER CODE BEGIN PV */
uint8_t  motor_speed = 1;          // 1=slow 2=mid 3=fast
uint8_t  motor_dir   = 0;          // 0=A dir, 1=B dir
uint32_t motor_steps = 0;          // Target step count
uint8_t  cmd_ready   = 0;          // Command ready flag
uint8_t  motor_busy  = 0;          // Motor is currently stepping

int32_t motor_position_steps = 0;  // Current position in steps (A+, B-)
uint8_t motor_stop_flag = 0;       // Emergency stop flag

// Direction locks: each direction locked independently
uint8_t limit_a_locked = 0;        // A-direction locked (A limit triggered)
uint8_t limit_b_locked = 0;        // B-direction locked (B limit triggered)

// Limit debounce state
uint8_t last_limit_a  = 0;
uint8_t last_limit_b  = 0;
uint8_t last_limit_org = 0;
uint8_t filter_a_cnt  = 0;
uint8_t filter_b_cnt  = 0;
uint8_t filter_org_cnt = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_Parse_Cmd(void);
void UART_PutChar(uint8_t ch);
void UART_PutString(char *str);
void Motor_Emergency_Stop(void);
void Limit_Switch_Check(void);
uint8_t Read_Limit_Stable(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void System_Reset(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
  last_limit_a   = 0;
  last_limit_b   = 0;
  last_limit_org = 0;
  motor_position_steps = 0;

  UART_PutString("\r\n========================================\r\n");
  UART_PutString(" Stepper Motor Control System V9\r\n");
  UART_PutString(" 6400 pulse/rev, 1mm lead\r\n");
  UART_PutString("========================================\r\n");
  UART_PutString(" Commands:\r\n");
  UART_PutString("  1) 3-digit: <Speed1-3><Dir0-1><Turn1-2>\r\n");
  UART_PutString("     0=A-dir  1=B-dir\r\n");
  UART_PutString("     ex: 201=Mid_A_Full  321=Fast_B_Half\r\n");
  UART_PutString("  2) Micron: A<dist> or B<dist>\r\n");
  UART_PutString("     ex: A100=A+100um  B1000=B+1000um\r\n");
  UART_PutString("  3) Reset: send R to clear locks\r\n");
  UART_PutString("  4) Query: send ? to show position\r\n");
  UART_PutString("========================================\r\n");
  UART_PutString("> ");

  while (1)
  {
    UART_Parse_Cmd();
    Limit_Switch_Check();
    HAL_Delay(10);

    if (cmd_ready)
    {
      // Direction lock: A-dir(0) blocked by A-limit, B-dir(1) blocked by B-limit
      if ((motor_dir == 0 && limit_a_locked) ||
          (motor_dir == 1 && limit_b_locked))
      {
        UART_PutString("Blocked: limit active, reverse dir only\r\n> ");
        cmd_ready = 0;
        continue;
      }

      motor_stop_flag = 0;
      motor_busy = 1;
      cmd_ready = 0;
      Set_Direction(motor_dir);

      uint32_t delay = DEFAULT_HALF_T_LOW;
      if (motor_speed == 2)      delay = DEFAULT_HALF_T_MID;
      else if (motor_speed == 3) delay = DEFAULT_HALF_T_HIGH;

      Step(motor_steps, delay);
      motor_busy = 0;

      if (!motor_stop_flag)
      {
        if (motor_dir == 0)
          motor_position_steps += (int32_t)motor_steps;
        else
          motor_position_steps -= (int32_t)motor_steps;
      }

      // Display current position (0.1um precision)
      int32_t pos_0p1um = STEPS_TO_0P1UM(motor_position_steps);
      char sign = '+';
      int32_t abs_val = pos_0p1um;
      if (abs_val < 0) { sign = '-'; abs_val = -abs_val; }

      char disp_buf[48];
      sprintf(disp_buf, "Done  Pos: %c%ld.%ld um\r\n> ",
              sign, (long)(abs_val / 10), (long)(abs_val % 10));
      UART_PutString(disp_buf);
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

// Software reset: clear alarm state without clearing direction locks.
// Direction locks auto-clear when sensors release in Limit_Switch_Check().
void System_Reset(void)
{
  motor_stop_flag  = 0;
  // Keep limit_a_locked / limit_b_locked — they auto-clear on sensor release
  last_limit_a     = 0;
  last_limit_b     = 0;
  filter_a_cnt     = 0;
  filter_b_cnt     = 0;
  cmd_ready        = 0;
  UART_PutString("System: alarm reset, reverse dir available\r\n> ");
}

// Double-read for stable level (2ms interval debounce)
uint8_t Read_Limit_Stable(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  uint8_t level1 = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  HAL_Delay(2);
  uint8_t level2 = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  return (level1 == level2) ? level1 : 0;
}

void UART_PutChar(uint8_t ch)
{
  HAL_UART_Transmit(&huart1, &ch, 1, 10);
}

void UART_PutString(char *str)
{
  if (str == NULL) return;
  uint16_t len = strlen(str);
  HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 100);
}

// Parse micron digits after A/B prefix has been consumed.
// On entry, uart_buf_tail points to first digit.
static void parse_micron_value(uint8_t dir)
{
  uint16_t data_len = (uart_buf_head - uart_buf_tail + UART_BUF_SIZE) % UART_BUF_SIZE;
  if (data_len == 0) return;

  uint32_t value = 0;
  uint8_t  digit_count = 0;
  uint16_t idx = uart_buf_tail;

  while (idx != uart_buf_head && digit_count < 10)
  {
    uint8_t ch = uart_buf[idx];
    if (ch >= '0' && ch <= '9')
    {
      value = value * 10 + (ch - '0');
      digit_count++;
      idx = (idx + 1) % UART_BUF_SIZE;
    }
    else if (ch == '\r' || ch == '\n')
    {
      idx = (idx + 1) % UART_BUF_SIZE;
      break;
    }
    else
    {
      uart_buf_head = uart_buf_tail = 0;
      UART_PutString("Error: invalid command\r\n> ");
      return;
    }
  }

  if (digit_count > 0 && value > 0 && value <= 15000)
  {
    motor_speed = 2;  // micron commands default: mid speed
    motor_dir   = dir;
    motor_steps = UM_TO_STEPS(value);
    if (motor_steps == 0) motor_steps = 1;
    cmd_ready = 1;
    uart_buf_tail = idx;

    char info[64];
    sprintf(info, "%s move %lu um (%lu steps)\r\n",
            (dir == 0) ? "A" : "B",
            (unsigned long)value, (unsigned long)motor_steps);
    UART_PutString(info);
  }
  else
  {
    uart_buf_head = uart_buf_tail = 0;
    UART_PutString("Error: invalid distance (1~15000um)\r\n> ");
  }
}

// UART command parser: 3-digit / A/B micron / R reset / ? query
void UART_Parse_Cmd(void)
{
  uint16_t data_len = (uart_buf_head - uart_buf_tail + UART_BUF_SIZE) % UART_BUF_SIZE;
  if (data_len == 0) return;

  uint8_t cmd = uart_buf[uart_buf_tail];

  // Ignore CR/LF
  if (cmd == '\r' || cmd == '\n')
  {
    uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
    return;
  }

  // ?: query position
  if (cmd == '?')
  {
    int32_t pos_0p1um = STEPS_TO_0P1UM(motor_position_steps);
    char sign = '+';
    int32_t abs_val = pos_0p1um;
    if (abs_val < 0) { sign = '-'; abs_val = -abs_val; }
    char buf[48];
    sprintf(buf, "Pos: %c%ld.%ld um\r\n> ",
            sign, (long)(abs_val / 10), (long)(abs_val % 10));
    UART_PutString(buf);
    uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
    return;
  }

  // R/r standalone = reset
  if (cmd == 'R' || cmd == 'r')
  {
    uint16_t next_idx = (uart_buf_tail + 1) % UART_BUF_SIZE;

    // Only 'R' in buffer, wait for follow-up char
    if (next_idx == uart_buf_head)
      return;

    uint8_t next_ch = uart_buf[next_idx];

    // Standalone R with CR/LF or end-of-buffer -> reset
    if (next_ch == '\r' || next_ch == '\n')
    {
      System_Reset();
      uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
      return;
    }

    // R followed by anything else -> error (B-micron uses 'B' prefix now)
    uart_buf_head = uart_buf_tail = 0;
    UART_PutString("Error: use 'B<dist>' for B-micron, 'R' alone for reset\r\n> ");
    return;
  }

  // A/a: A-direction micron command
  if (cmd == 'A' || cmd == 'a')
  {
    uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE; // consume 'A'

    data_len = (uart_buf_head - uart_buf_tail + UART_BUF_SIZE) % UART_BUF_SIZE;
    if (data_len == 0) return;

    uint8_t first_digit = uart_buf[uart_buf_tail];
    if (!(first_digit >= '0' && first_digit <= '9'))
    {
      uart_buf_head = uart_buf_tail = 0;
      UART_PutString("Error: A requires a number, ex: A100\r\n> ");
      return;
    }
    parse_micron_value(0);  // dir=0 (A)
    return;
  }

  // B/b: B-direction micron command
  if (cmd == 'B' || cmd == 'b')
  {
    uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE; // consume 'B'

    data_len = (uart_buf_head - uart_buf_tail + UART_BUF_SIZE) % UART_BUF_SIZE;
    if (data_len == 0) return;

    uint8_t first_digit = uart_buf[uart_buf_tail];
    if (!(first_digit >= '0' && first_digit <= '9'))
    {
      uart_buf_head = uart_buf_tail = 0;
      UART_PutString("Error: B requires a number, ex: B1000\r\n> ");
      return;
    }
    parse_micron_value(1);  // dir=1 (B)
    return;
  }

  // 3-digit command: <Speed1-3><Dir0-1><Turn1-2>
  if (data_len >= 3)
  {
    uint8_t s = uart_buf[uart_buf_tail];
    uint8_t d = uart_buf[(uart_buf_tail + 1) % UART_BUF_SIZE];
    uint8_t c = uart_buf[(uart_buf_tail + 2) % UART_BUF_SIZE];

    if ((s >= '1' && s <= '3') &&
        (d >= '0' && d <= '1') &&
        (c >= '1' && c <= '2'))
    {
      motor_speed = s - '0';
      motor_dir   = d - '0';
      motor_steps = (c == '1') ? STEPS_PER_CIRCLE : STEPS_PER_HALF_CIRCLE;
      cmd_ready = 1;
      uart_buf_tail = (uart_buf_tail + 3) % UART_BUF_SIZE;

      char info[64];
      sprintf(info, "%s %s %s turn (%lu steps)\r\n",
              (motor_speed == 1) ? "Slow" : (motor_speed == 2) ? "Mid" : "Fast",
              (motor_dir == 0) ? "A" : "B",
              (motor_steps == STEPS_PER_CIRCLE) ? "Full" : "Half",
              (unsigned long)motor_steps);
      UART_PutString(info);
    }
    else
    {
      uart_buf_head = uart_buf_tail = 0;
      UART_PutString("Error: invalid command\r\n> ");
    }
  }
}

void Alarm_Info(char *str)
{
  UART_PutString("ALARM: ");
  UART_PutString(str);
  UART_PutString("\r\n> ");
}

// Emergency stop: pull STEP low, set flag. Direction lock set by caller.
void Motor_Emergency_Stop(void)
{
  HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_GPIO_PIN, GPIO_PIN_RESET);
  motor_stop_flag = 1;
}

/*
 * Limit switch state machine (debounced, called every 10ms).
 *
 * A-limit (was CW): triggers -> lock A-dir -> stop motor -> alarm
 * B-limit (was CCW): triggers -> lock B-dir -> stop motor -> alarm
 * Origin: info only, no stop, no lock.
 *
 * Direction lock auto-clears when sensor releases (after debounce).
 * Opposite direction always available without needing R.
 * R is only needed if both limits are stuck or for software recovery.
 */
void Limit_Switch_Check(void)
{
  uint8_t a_now   = Read_Limit_Stable(LIMIT_A_PORT, LIMIT_A_PIN);
  uint8_t b_now   = Read_Limit_Stable(LIMIT_B_PORT, LIMIT_B_PIN);
  uint8_t org_now = Read_Limit_Stable(LIMIT_ORG_PORT, LIMIT_ORG_PIN);

  // === A-limit (was CW) ===
  if (a_now == 1 && !limit_a_locked)
  {
    filter_a_cnt++;
    if (filter_a_cnt >= LIMIT_FILTER_COUNT && last_limit_a == 0)
    {
      last_limit_a = 1;
      limit_a_locked = 1;
      Motor_Emergency_Stop();
      Alarm_Info("A-limit triggered, reverse dir (B) available");
    }
  }
  else if (a_now == 0)
  {
    filter_a_cnt = 0;
    last_limit_a = 0;
    limit_a_locked = 0;  // auto-clear when sensor releases
  }

  // === B-limit (was CCW) ===
  if (b_now == 1 && !limit_b_locked)
  {
    filter_b_cnt++;
    if (filter_b_cnt >= LIMIT_FILTER_COUNT && last_limit_b == 0)
    {
      last_limit_b = 1;
      limit_b_locked = 1;
      Motor_Emergency_Stop();
      Alarm_Info("B-limit triggered, reverse dir (A) available");
    }
  }
  else if (b_now == 0)
  {
    filter_b_cnt = 0;
    last_limit_b = 0;
    limit_b_locked = 0;  // auto-clear when sensor releases
  }

  // === Origin: info only, no stop, no lock ===
  if (org_now == 1)
  {
    filter_org_cnt++;
    if (filter_org_cnt >= LIMIT_FILTER_COUNT && last_limit_org == 0)
    {
      last_limit_org = 1;
      if (!motor_busy)
      {
        motor_position_steps = 0;
      }
      UART_PutString("Info: origin reached");
      if (!motor_busy)
      {
        UART_PutString(", position reset to 0");
      }
      else
      {
        UART_PutString(" (motor busy, position unchanged)");
      }
      UART_PutString("\r\n> ");
    }
  }
  else
  {
    filter_org_cnt = 0;
    last_limit_org = 0;
  }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
