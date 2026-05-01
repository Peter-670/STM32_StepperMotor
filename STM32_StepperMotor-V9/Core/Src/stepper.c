/* stepper.c */
#include "stepper.h"

void Step_GPIO_Config(void)
{
    // GPIO already configured in MX_GPIO_Init
}

void Delay(uint32_t nCount)
{
    volatile uint32_t index;
    for(index = nCount; index != 0; index--);
}

/**
  * @brief  Send step pulses with fast limit detection.
  *         Fast GPIO read every STEP_CHECK_INTERVAL steps — no debounce delay,
  *         so pulse timing stays smooth. Debounced detection runs in main loop.
  */
void Step(uint32_t num, uint32_t half_T)
{
    for(uint32_t i = 0; i < num; i++)
    {
        if((i % STEP_CHECK_INTERVAL) == 0)
        {
            // Fast limit check with 2-sample debounce:
            // back-to-back GPIO reads filter single-sample noise spikes
            // while responding quickly to real (sustained) limit triggers.
            // Only check limits that are NOT already locked.
            if(!limit_a_locked)
            {
                if(HAL_GPIO_ReadPin(LIMIT_A_PORT, LIMIT_A_PIN) &&
                   HAL_GPIO_ReadPin(LIMIT_A_PORT, LIMIT_A_PIN))
                {
                    limit_a_locked = 1;
                    Motor_Emergency_Stop();
                    return;
                }
            }
            if(!limit_b_locked)
            {
                if(HAL_GPIO_ReadPin(LIMIT_B_PORT, LIMIT_B_PIN) &&
                   HAL_GPIO_ReadPin(LIMIT_B_PORT, LIMIT_B_PIN))
                {
                    limit_b_locked = 1;
                    Motor_Emergency_Stop();
                    return;
                }
            }
        }

        HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_GPIO_PIN, GPIO_PIN_SET);
        Delay(half_T);
        HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_GPIO_PIN, GPIO_PIN_RESET);
        Delay(half_T);
    }
}

/**
  * @brief  Set motor direction.
  * @param  dir: 0 = A direction, 1 = B direction
  */
void Set_Direction(uint8_t dir)
{
    if(dir)
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_SET);
}
