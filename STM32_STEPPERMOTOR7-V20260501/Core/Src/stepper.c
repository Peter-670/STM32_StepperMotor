/* stepper.c */
#include "stepper.h"

/**
  * @brief  配置步进电机 GPIO 引脚
  *         CubeMX 已初始化，此函数可省略或用于额外配置
  */
void Step_GPIO_Config(void)
{
    // GPIO 已在 CubeMX 中配置，无需额外初始化
    // 如需动态配置，可在此添加代码
}

/**
  * @brief  延时函数
  * @param  nCount: 延时计数值
  */
void Delay(uint32_t nCount)
{
    volatile uint32_t index;
    for(index = nCount; index != 0; index--);
}

/**
  * @brief  发送步进脉冲
  * @param  num: 脉冲数量
  * @param  half_T: 半周期时间（控制速度）
  */
void Step(uint32_t num, uint32_t half_T)
{
    for(; num > 0; num--)
    {
        HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_GPIO_PIN, GPIO_PIN_SET);
        Delay(half_T);
        HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_GPIO_PIN, GPIO_PIN_RESET);
        Delay(half_T);
    }
}

/**
  * @brief  设置电机方向
  * @param  dir: 方向 (0: 反转，1: 正转)
  */
void Set_Direction(uint8_t dir)
{
    if(dir)
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_SET);
}

