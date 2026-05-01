/* stepper.h */
#ifndef __STEPPER_H
#define __STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// External variables/functions from main.c used by Step()
extern uint8_t motor_stop_flag;
extern uint8_t limit_a_locked;
extern uint8_t limit_b_locked;
extern void Motor_Emergency_Stop(void);

// Public API
void Step_GPIO_Config(void);
void Delay(uint32_t nCount);
void Step(uint32_t num, uint32_t half_T);
void Set_Direction(uint8_t dir);  // 0=A, 1=B

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_H */
