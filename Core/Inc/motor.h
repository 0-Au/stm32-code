/**
  ******************************************************************************
  * @file    motor.h
  * @brief   单路直流电机驱动（TB6612FNG + TIM1_CH1 PWM）—— 开环，不含 PID
  *
  * 设计要点：
  *   1. 上层只用 ±MOTOR_PWM_MAX 这个"逻辑量程"，不关心 ARR/PSC 具体是多少。
  *      以后改 PWM 频率（比如 20kHz -> 16kHz）只需要重新 CubeMX 生成，上层代码不动。
  *   2. 方向由 AIN1/AIN2 决定，速度由 CCR 决定，两者在 Motor_SetPWM() 内一起处理，
  *      避免出现"先改方向再改占空比"这种中间态。
  ******************************************************************************
  */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/* 逻辑满量程：Motor_SetPWM(1000) 约等于 100% 占空比，-1000 为反向满速 */
#define MOTOR_PWM_MAX   1000

/* 上电初始化：拉高 STBY、把 PWM 拉 0、启动 TIM1 CH1 */
void Motor_Init(void);

/**
  * @brief  设置电机输出
  * @param  pwm  -1000 ~ +1000，正数为正转，负数为反转，0 为滑行停止(coast)
  * @note   带饱和保护，超出量程按满量程处理
  */
void Motor_SetPWM(int16_t pwm);

/* 滑行停止：IN1=IN2=0，电机自由转动（电流最小，推荐默认用这个） */
void Motor_Coast(void);

/* 刹车：IN1=IN2=1，电机绕组短接，制动但会有电流（大惯性负载才需要） */
void Motor_Brake(void);

#endif /* __MOTOR_H__ */
