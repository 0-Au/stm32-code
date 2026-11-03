/**
  ******************************************************************************
  * bsp_motor.c - 电机 PWM + 编码器测速（BSP 层）
  *
  * 依赖 CubeMX 生成的：
  *   htim1 : TIM1 CH1 PWM 输出（PA8），ARR = 7199，PSC = 0 -> 10kHz
  *   htim2 : TIM2 编码器模式（PA0/PA1），ARR = 65535
  *   GPIO  : PB12(AIN1) / PB13(AIN2) 推挽输出
  *
  * 引脚改了只改本文件，上层 app_tasks.c 不用动。
  ******************************************************************************
  */
#include "bsp_motor.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"

/* ---------- 引脚宏定义：换板子只改这里 ---------- */
#define AIN1_PIN        GPIO_PIN_12
#define AIN1_PORT       GPIOB
#define AIN2_PIN        GPIO_PIN_13
#define AIN2_PORT       GPIOB

#define PWM_TIM         (&htim1)
#define PWM_CHANNEL     TIM_CHANNEL_1
#define PWM_MAX         7200        /* = ARR + 1，与 app_tasks.c 保持一致 */

#define ENC_TIM         (&htim2)

/* ---------- 内部状态 ---------- */
static int   s_last_pwm     = 0;
static int16_t s_last_count = 0;    /* 上次采样的计数值（16 位） */
static uint32_t s_last_tick = 0;    /* 上次采样时刻（ms） */
static long  s_total_count  = 0;    /* 累计脉冲 */

/**
  * @brief 初始化：启动 PWM 和编码器，方向脚置低
  */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(PWM_TIM, PWM_CHANNEL);
    HAL_TIM_Encoder_Start(ENC_TIM, TIM_CHANNEL_ALL);

    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CHANNEL, 0);
    __HAL_TIM_SET_COUNTER(ENC_TIM, 0);

    s_last_count = 0;
    s_last_tick  = HAL_GetTick();
    s_total_count = 0;
}

/**
  * @brief 设置 PWM 与方向
  * @param pwm 正数正转，负数反转，绝对值不超过 PWM_MAX
  */
void Motor_SetPWM(int pwm)
{
    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm > 0) {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    } else if (pwm < 0) {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
        pwm = -pwm;
    } else {
        /* 0 输出：两脚都低 = 滑行停止（如需刹车可都置高） */
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CHANNEL, (uint32_t)pwm);
    s_last_pwm = pwm;
}

int Motor_GetPWM(void)
{
    return s_last_pwm;
}

/**
  * @brief 读取两次调用之间的平均转速（rpm）
  * @note  由 motor_task 每 10ms 调用一次，内部自动处理 16 位计数器溢出
  */
float Encoder_GetSpeed(void)
{
    uint32_t now   = HAL_GetTick();
    uint32_t dt_ms = now - s_last_tick;

    int16_t now_count = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM);
    int16_t delta     = (int16_t)(now_count - s_last_count);

    /* 每转输出轴脉冲数 = 线数 × 倍频 × 减速比 */
    const float counts_per_rev = (float)ENCODER_PPR * ENCODER_MULTIPLY * GEAR_RATIO;
    float rev_per_s;

    s_last_count = now_count;
    s_last_tick  = now;
    s_total_count += delta;

    if (dt_ms == 0) return 0.0f;   /* 防止除零 */

    /* rpm = (脉冲数 / 每转脉冲数) / 秒数 × 60 */
    rev_per_s = (float)delta / counts_per_rev / ((float)dt_ms / 1000.0f);
    return rev_per_s * 60.0f;
}

long Encoder_GetTotalCount(void)
{
    return s_total_count;
}

void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(ENC_TIM, 0);
    s_last_count  = 0;
    s_total_count = 0;
    s_last_tick   = HAL_GetTick();
}
