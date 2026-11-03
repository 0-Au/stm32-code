/**
  ******************************************************************************
  * pid.h - 位置式 PID 控制器（带积分限幅 + 输出限幅）
  * 配套培训第 2 点：FreeRTOS + PID 任务训练
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;        /* 积分累加项 */
    float last_err;        /* 上一次误差，用于微分 */

    float integral_max;    /* 积分限幅（绝对值），防止积分饱和 */
    float output_max;      /* 输出限幅（绝对值），对应 PWM 上限 */
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd,
               float integral_max, float output_max);
float PID_Calc(PID_t *pid, float target, float actual);
void  PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
