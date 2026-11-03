/**
  ******************************************************************************
  * bsp_motor.h - 电机 + 编码器底层驱动（BSP 层）
  * 对应 app_tasks.c 里的两个 extern 接口
  ******************************************************************************
  */
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* 编码器参数：按你电机铭牌改 */
#define ENCODER_PPR        13      /* 编码器线数（每转脉冲数，单相） */
#define ENCODER_MULTIPLY   4       /* 四倍频（TIM 编码器模式默认 4 倍频） */
#define GEAR_RATIO         30.0f   /* 减速比，直驱电机填 1.0f */

void  Motor_Init(void);                  /* CubeMX 初始化之后调用一次 */
void  Motor_SetPWM(int pwm);             /* 输入 -PWM_MAX ~ +PWM_MAX，正负代表方向 */
int   Motor_GetPWM(void);                /* 返回当前 PWM 值，调试用 */
float Encoder_GetSpeed(void);            /* 返回两次调用之间的平均转速，单位 rpm */
long  Encoder_GetTotalCount(void);       /* 累计脉冲数，可用于位置环 */
void  Encoder_Reset(void);               /* 清零计数与速度计算基准 */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MOTOR_H */
