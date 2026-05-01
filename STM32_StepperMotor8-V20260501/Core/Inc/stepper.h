/* stepper.h */
#ifndef __STEPPER_H
#define __STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 函数声明
void Step_GPIO_Config(void);
void Delay(uint32_t nCount);
void Step(uint32_t num, uint32_t half_T);
void Set_Direction(uint8_t dir);  // 0: 反转，1: 正转

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_H */

