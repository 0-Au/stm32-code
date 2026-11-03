/**
  ******************************************************************************
  * pid.c - 位置式 PID 控制器实现
  *
  * 要点（验收时会问）：
  *   Kp：影响响应速度，太大会振荡
  *   Ki：消除静差，太大 + 目标突变 -> 积分饱和（所以要积分限幅）
  *   Kd：抑制超调和振荡趋势，对噪声敏感
  *   控制周期改变 -> 积分/微分项都变 -> 参数必须重调
  ******************************************************************************
  */
#include "pid.h"

static float clamp(float x, float max)
{
    if (x >  max) return  max;
    if (x < -max) return -max;
    return x;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float integral_max, float output_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral     = 0.0f;
    pid->last_err     = 0.0f;
    pid->integral_max = integral_max;
    pid->output_max   = output_max;
}

float PID_Calc(PID_t *pid, float target, float actual)
{
    float err = target - actual;

    /* 积分 + 积分限幅：防止目标快速变化时积分饱和 */
    pid->integral += err;
    pid->integral  = clamp(pid->integral, pid->integral_max);

    /* 位置式 PID */
    float out = pid->Kp * err
              + pid->Ki * pid->integral
              + pid->Kd * (err - pid->last_err);

    pid->last_err = err;

    /* 输出限幅：不能超过 PWM 能力范围 */
    return clamp(out, pid->output_max);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_err = 0.0f;
}
