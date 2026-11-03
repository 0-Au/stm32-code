/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   TIM2 编码器接口实现
  *
  * CubeMX 里 TIM2 = Encoder Mode, TI1 and TI2, PSC=0, ARR=65535
  * PA0 = TIM2_CH1 = A 相,  PA1 = TIM2_CH2 = B 相
  ******************************************************************************
  */
#include "encoder.h"
#include "tim.h"

static int16_t s_last  = 0;   /* 上次读到的 CNT（按 16 位解释） */
static int32_t s_total = 0;   /* 累计值，标定 CPR 用 */

void Encoder_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    s_last  = 0;
    s_total = 0;
}

int16_t Encoder_ReadDelta(void)
{
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t d   = (int16_t)(now - s_last);   /* 16 位回绕自动正确 */

    s_last   = now;
    s_total += d;

    return d;
}

int32_t Encoder_GetTotal(void)
{
    return s_total;
}

void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    s_last  = 0;
    s_total = 0;
}
