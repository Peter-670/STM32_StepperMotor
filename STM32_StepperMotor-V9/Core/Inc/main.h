/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR_PIN_Pin GPIO_PIN_10
#define DIR_PIN_GPIO_Port GPIOB
#define STEP_PIN_Pin GPIO_PIN_11
#define STEP_PIN_GPIO_Port GPIOB
#define CW_Pin GPIO_PIN_3
#define CW_GPIO_Port GPIOB
#define CCW_Pin GPIO_PIN_4
#define CCW_GPIO_Port GPIOB
#define ORG_Pin GPIO_PIN_5
#define ORG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// Step motor pin aliases
#define DIR_GPIO_PORT       GPIOB
#define DIR_GPIO_PIN        GPIO_PIN_10
#define STEP_GPIO_PORT      GPIOB
#define STEP_GPIO_PIN       GPIO_PIN_11

// Direction A/B aliases (A = CW pin, B = CCW pin)
#define LIMIT_A_PIN   CW_Pin
#define LIMIT_A_PORT  CW_GPIO_Port
#define LIMIT_B_PIN   CCW_Pin
#define LIMIT_B_PORT  CCW_GPIO_Port

// 6400 pulse/rev, 1mm lead
#define STEPS_PER_CIRCLE      6400
#define STEPS_PER_HALF_CIRCLE 3200

// Step <-> micron conversion (1 step = 1000/6400 = 0.15625um)
// steps = microns * 6400 / 1000 = microns * 32 / 5
#define UM_TO_STEPS(um)       ((uint32_t)(um) * 32 / 5)
// 0.1um = steps * 10000 / 6400 = steps * 25 / 16
#define STEPS_TO_0P1UM(steps) ((int32_t)(steps) * 25 / 16)

// Limit check interval during stepping (fast GPIO read, no delay)
#define STEP_CHECK_INTERVAL   100

// Default half-period
#define DEFAULT_HALF_T 5000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
