/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   TIM2 正交编码器接口（A/B 四倍频）
  *
  * 为什么不自己写 EXTI 中断数脉冲：
  *   - 四倍频 + 去抖靠硬件，M 法测速在 20kHz 级别的边沿下 STM32 中断会丢脉冲；
  *   - 硬件编码器模式天然给出方向，正转 CNT++、反转 CNT--。
  *   这是后面所有闭环的"眼睛"，不要省。
  ******************************************************************************
  */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

/* 启动 TIM2 编码器模式，计数器清零 */
void Encoder_Init(void);

/**
  * @brief  读取自上次调用以来的增量（带方向符号）
  * @retval 有符号计数差
  * @note   内部用 static 记录上次的值，所以【必须固定周期调用】。
  *         本工程在 main 循环里每 20ms 调一次，这个 20ms 就是测速窗口。
  *         16 位计数器差用 (int16_t) 强转，天然处理溢出和反向。
  */
int16_t Encoder_ReadDelta(void);

/* 累计总计数（从 Encoder_Init 或 Encoder_Reset 起算），用于手动标定 CPR */
int32_t Encoder_GetTotal(void);

/* 清零计数器与累计值 */
void Encoder_Reset(void);

#endif /* __ENCODER_H__ */
