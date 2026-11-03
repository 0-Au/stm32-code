/**
  ******************************************************************************
  * @file    hwt101.h
  * @brief   JY60 六轴 IMU 驱动接口（WitMotion 标准协议，与 HWT101 通用）
  * @note    硬件：维特智能 JY60，TTL 串口，出厂 9600-8-N-1，回传速率 20Hz，
  *          输出加速度(0x51) / 角速度(0x52) / 角度(0x53)，3.3~5V 供电。
  *          接口命名沿用培训文档要求（hwt101.c / hwt101.h），可无缝换成 HWT101。
  ******************************************************************************
  */
#ifndef __HWT101_H__
#define __HWT101_H__

#include "main.h"          /* 提供 UART_HandleTypeDef、HAL_GetTick 等 */
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== 可调参数 ============================== */
#define HWT101_FRAME_LEN        11u       /* 固定帧长：0x55 + type + 6 数据 + 1 类型 + 校验 */
#define HWT101_DMA_BUF_SIZE     64u       /* DMA 循环缓冲，需 >= 2 帧(22B) */
#define HWT101_RING_BUF_SIZE    256u      /* 软件环形缓冲，必须是 2 的幂 */

#define HWT101_OFFLINE_MS       300u      /* 超过该时间无有效帧 -> 判定掉线 */
#define HWT101_RESTART_MS       500u      /* 掉线后重启接收的最小间隔 */
#define HWT101_GYRO_CALI_MS     500u      /* 静止零偏校准采样时长 */

/* 物理量换算（JY60 出厂量程：±16g / ±2000dps / ±180°） */
#define HWT101_ACC_SCALE        (16.0f   / 32768.0f)   /* g   per LSB */
#define HWT101_GYRO_SCALE       (2000.0f / 32768.0f)   /* dps per LSB */
#define HWT101_ANGLE_SCALE      (180.0f  / 32768.0f)   /* deg per LSB */

/* ============================== 数据结构 ============================== */
/**
 * @brief 对外提供的 IMU 数据（业务层只读，不接触任何缓冲区）
 */
typedef struct
{
    float    acc_g[3];        /* 加速度 x,y,z，单位 g            */
    float    gyro_dps[3];     /* 原始角速度 x,y,z，单位 °/s      */
    float    roll_deg;        /* 横滚角，单位 °                  */
    float    pitch_deg;       /* 俯仰角，单位 °                  */
    float    yaw_raw_deg;     /* 传感器原始 yaw，±180            */
    float    yaw_deg;         /* 相对零点 yaw，±180              */
    float    yaw_rate_dps;    /* 去零偏后的 Z 轴角速度，°/s      */

    uint32_t frame_cnt;       /* 有效帧计数（判断是否在出数）    */
    uint32_t err_cnt;         /* 校验错/帧头错/缓冲溢出计数      */
    uint32_t last_frame_ms;   /* 最近一帧有效数据的时间戳        */
    bool     online;          /* 在线标志                        */
} HWT101_Data_t;

/* ============================== 对外接口 ============================== */
void  HWT101_Init(UART_HandleTypeDef *huart); /* 初始化并使能 DMA+IDLE 接收 */
void  HWT101_RestartRx(void);                 /* 仅重启接收（保留零点/数据，用于 ORE 恢复） */
void  HWT101_RxEventCallback(void);           /* 串口中断里调用，处理 IDLE  */
void  HWT101_Parse(void);                     /* 主循环/任务中周期调用，解析字节流 */
void  HWT101_CalibrateGyro(void);              /* 上电静止校准 Z 轴角速度零偏 */

void  HWT101_ResetYawZero(void);               /* 把当前 yaw 记为 0°（上电归零） */
void  HWT101_SetYawZero(float zero_deg);       /* 把当前 yaw 记为 zero_deg      */
float HWT101_GetYaw(void);                     /* 相对零点 yaw，±180           */
float HWT101_GetYawRate(void);                 /* 去零偏角速度，°/s            */
const HWT101_Data_t *HWT101_GetData(void);     /* 完整数据快照                 */

bool     HWT101_IsOnline(void);
uint32_t HWT101_GetFrameCount(void);
uint32_t HWT101_GetErrCount(void);

/* ---------- 诊断接口（排查"收不到数据"时用） ---------- */
/* HWT101_GetInitStatus() 返回下列位组合 */
#define HWT101_ST_UART_NULL    0x01u   /* Init 传入的串口句柄为空          */
#define HWT101_ST_DMA_NULL     0x02u   /* huart->hdmarx 为空：CubeMX 没配 DMA */
#define HWT101_ST_DMA_FAIL     0x04u   /* HAL_UART_Receive_DMA 返回非 HAL_OK */
#define HWT101_ST_RX_STARTED   0x08u   /* 接收已成功启动                    */

uint32_t HWT101_GetRxBytes(void);      /* 累计进入环形缓冲的字节数：0 = 链路完全没通 */
uint8_t  HWT101_GetInitStatus(void);   /* 初始化状态位，见上面的宏 */
uint32_t HWT101_GetIrqCount(void);     /* 串口中断进过几次（0 = 中断没挂上） */
uint32_t HWT101_GetIdleCount(void);    /* 其中确认是 IDLE 的次数（0 = 没收到完整帧） */

#ifdef __cplusplus
}
#endif

#endif /* __HWT101_H__ */
