/**
  ******************************************************************************
  * @file    motor.c
  * @brief   TB6612FNG 单路电机驱动
  *
  * 接线（默认）：
  *   PA8  -> PWMA    (TIM1_CH1)
  *   PB12 -> AIN1
  *   PB13 -> AIN2
  *   PB14 -> STBY
  *
  * TB6612 真值表（PWM 接高电平时的表现）：
  *   AIN1  AIN2  STBY | 输出
  *    1     0     1   | 正转
  *    0     1     1   | 反转
  *    0     0     1   | 滑行停止 (coast)
  *    1     1     1   | 刹车 (brake)
  *    x     x     0   | 待机，输出高阻
  *
  * 注意 TB6612 的 PWM 引脚是"高电平有效"：PWMA=1 时输出，PWMA=0 时短路制动。
  * 所以我们用 PWM Mode 1 + 高极性，CCR 越大转速越高，语义一致。
  ******************************************************************************
  */
#include "motor.h"
#include "tim.h"

/* ---------- 引脚定义：改板子只改这里 ---------- */
#define AIN1_PORT   GPIOB
#define AIN1_PIN    GPIO_PIN_12
#define AIN2_PORT   GPIOB
#define AIN2_PIN    GPIO_PIN_13
#define STBY_PORT   GPIOB
#define STBY_PIN    GPIO_PIN_14

static uint32_t s_arr = 0;   /* TIM1 的 ARR，Init 时缓存下来 */

static inline void AIN1(uint8_t lv)
{
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, lv ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void AIN2(uint8_t lv)
{
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, lv ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Init(void)
{
    s_arr = __HAL_TIM_GET_AUTORELOAD(&htim1);   /* 3599 @ 20kHz/72MHz */

    /* 先给安全态，再使能驱动，最后才开 PWM，避免上电瞬间乱转 */
    AIN1(0);
    AIN2(0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);  /* 退出待机 */

    /* TIM1 是高级定时器，HAL 里会自动打开 MOE，不需要手动写 BDTR */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Motor_SetPWM(int16_t pwm)
{
    uint32_t mag;
    uint32_t pulse;

    /* 1. 饱和 */
    if (pwm >  MOTOR_PWM_MAX) pwm =  MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;

    mag = (pwm >= 0) ? (uint32_t)pwm : (uint32_t)(-pwm);

    /* 2. 逻辑量程 -> 计数值。ARR=3599 时 1000 档分辨率约 0.28% */
    pulse = (mag * (s_arr + 1u)) / MOTOR_PWM_MAX;
    if (pulse > s_arr) pulse = s_arr;   /* 钳到 99.97%，避免 CCR>ARR 的边界行为 */

    /* 3. 方向 + 占空比一起落地 */
    if (pwm > 0) {
        AIN1(1); AIN2(0);
    } else if (pwm < 0) {
        AIN1(0); AIN2(1);
    } else {
        AIN1(0); AIN2(0);   /* 0 档走 coast，不要用 brake，否则停机时电流大 */
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

void Motor_Coast(void)
{
    AIN1(0); AIN2(0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

void Motor_Brake(void)
{
    AIN1(1); AIN2(1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}
