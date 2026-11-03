/**
  ******************************************************************************
  * @file    hwt101.c
  * @brief   JY60 六轴 IMU 驱动（UART + DMA 循环接收 + IDLE 空闲中断 + 协议解析）
  *
  * 数据通路（三层解耦）：
  *   硬件 --DMA--> s_dma_buf(原始接收区) --IDLE中断--> s_ring(软件环形缓冲)
  *        --HWT101_Parse--> 协议状态机 --> s_data(对外数据)
  *
  * 业务层调用 HWT101_GetYaw()/HWT101_GetYawRate() 即可，永远不碰缓冲区。
  ******************************************************************************
  */

#include "hwt101.h"
#include <string.h>

/* ============================== 私有变量 ============================== */
static UART_HandleTypeDef *s_huart = NULL;      /* 绑定的串口句柄 */

static uint8_t  s_dma_buf[HWT101_DMA_BUF_SIZE]; /* DMA 原始接收缓冲（循环模式） */
static uint16_t s_dma_last_pos = 0;             /* 上一次已搬运到的写指针位置 */

static uint8_t  s_ring[HWT101_RING_BUF_SIZE];   /* 软件环形缓冲（中断写、主循环读） */
static volatile uint16_t s_ring_head = 0;
static volatile uint16_t s_ring_tail = 0;

static uint8_t  s_frame[HWT101_FRAME_LEN];      /* 组帧暂存 */
static uint8_t  s_frame_idx = 0;                /* 组帧进度 */

static HWT101_Data_t s_data;                    /* 对外数据 */
static float    s_yaw_zero_deg   = 0.0f;        /* 角度零点（上电朝向） */
static float    s_gyro_bias_z    = 0.0f;        /* Z 轴角速度零偏 */
static uint32_t s_last_restart_ms = 0;          /* 上次重启接收的时间 */

static volatile uint32_t s_rx_bytes = 0;        /* 累计进环形缓冲的字节数（诊断） */
static volatile uint32_t s_irq_cnt  = 0;        /* HWT101_RxEventCallback 被调用次数 */
static volatile uint32_t s_idle_cnt = 0;        /* 其中确认是 IDLE 的次数 */
static uint8_t  s_init_status = 0;              /* 初始化状态位（诊断） */

/* ============================== 内部函数 ============================== */
static __inline uint16_t ring_next(uint16_t idx)
{
    return (uint16_t)((idx + 1u) & (HWT101_RING_BUF_SIZE - 1u));
}

static __inline void ring_push(uint8_t byte)
{
    uint16_t next = ring_next(s_ring_head);
    if (next == s_ring_tail)          /* 满：丢弃新字节，保证旧帧完整性 */
    {
        s_data.err_cnt++;
        return;
    }
    s_ring[s_ring_head] = byte;
    s_ring_head = next;
    s_rx_bytes++;
}

static __inline bool ring_pop(uint8_t *byte)
{
    if (s_ring_head == s_ring_tail)
    {
        return false;
    }
    *byte = s_ring[s_ring_tail];
    s_ring_tail = ring_next(s_ring_tail);
    return true;
}

static __inline void ring_reset(void)
{
    s_ring_head = 0;
    s_ring_tail = 0;
}

static __inline void uart_clear_idle_flag(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(s_huart);
}

/**
 * @brief 启动一次 DMA 循环接收，并打开 IDLE 中断
 */
static void uart_start_rx(void)
{
    s_init_status = 0;

    if (s_huart == NULL)
    {
        s_init_status |= HWT101_ST_UART_NULL;
        return;
    }
    if (s_huart->hdmarx == NULL)
    {
        s_init_status |= HWT101_ST_DMA_NULL;   /* CubeMX 里没给 USART1 加 DMA 请求 */
        return;
    }

    (void)HAL_UART_DMAStop(s_huart);       /* 先停，避免重复启动返回 BUSY */

    /* 强制 DMA 为循环模式：缓冲区写满一圈自动回头，不会停 */
    s_huart->hdmarx->Init.Mode = DMA_CIRCULAR;
    (void)HAL_DMA_Init(s_huart->hdmarx);

    s_dma_last_pos = 0;
    uart_clear_idle_flag();
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);

    if (HAL_UART_Receive_DMA(s_huart, s_dma_buf, HWT101_DMA_BUF_SIZE) == HAL_OK)
    {
        s_init_status |= HWT101_ST_RX_STARTED;
    }
    else
    {
        s_init_status |= HWT101_ST_DMA_FAIL;
    }
}

/**
 * @brief 把 DMA 缓冲区增量搬进软件环形缓冲
 */
static void dma_to_ring(void)
{
    uint16_t pos = (uint16_t)(HWT101_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(s_huart->hdmarx));

    if (pos > HWT101_DMA_BUF_SIZE)
    {
        pos = 0;                            /* 异常保护 */
    }

    while (s_dma_last_pos != pos)
    {
        ring_push(s_dma_buf[s_dma_last_pos]);
        s_dma_last_pos = (uint16_t)((s_dma_last_pos + 1u) % HWT101_DMA_BUF_SIZE);
    }
}

static __inline int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static __inline float wrap_pm180(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static bool frame_checksum_ok(const uint8_t *f)
{
    uint8_t sum = 0;
    uint8_t i;

    for (i = 0; i < (HWT101_FRAME_LEN - 1u); i++)
    {
        sum = (uint8_t)(sum + f[i]);
    }
    return (sum == f[HWT101_FRAME_LEN - 1u]);
}

/**
 * @brief 处理一个校验通过的数据帧
 *        帧结构：0x55 | type | data[8] | sum
 *        type = 0x51 加速度, 0x52 角速度, 0x53 角度
 */
static void handle_frame(const uint8_t *f)
{
    switch (f[1])
    {
    case 0x51:  /* 加速度 ±16g */
        s_data.acc_g[0] = (float)le16(&f[2]) * HWT101_ACC_SCALE;
        s_data.acc_g[1] = (float)le16(&f[4]) * HWT101_ACC_SCALE;
        s_data.acc_g[2] = (float)le16(&f[6]) * HWT101_ACC_SCALE;
        break;

    case 0x52:  /* 角速度 ±2000dps */
        s_data.gyro_dps[0] = (float)le16(&f[2]) * HWT101_GYRO_SCALE;
        s_data.gyro_dps[1] = (float)le16(&f[4]) * HWT101_GYRO_SCALE;
        s_data.gyro_dps[2] = (float)le16(&f[6]) * HWT101_GYRO_SCALE;
        s_data.yaw_rate_dps = s_data.gyro_dps[2] - s_gyro_bias_z;
        break;

    case 0x53:  /* 角度 ±180° */
        s_data.roll_deg    = (float)le16(&f[2]) * HWT101_ANGLE_SCALE;
        s_data.pitch_deg   = (float)le16(&f[4]) * HWT101_ANGLE_SCALE;
        s_data.yaw_raw_deg = (float)le16(&f[6]) * HWT101_ANGLE_SCALE;
        s_data.yaw_deg     = wrap_pm180(s_data.yaw_raw_deg - s_yaw_zero_deg);
        break;

    default:
        return;                             /* 未使用的帧类型（0x54 磁场等） */
    }

    s_data.last_frame_ms = HAL_GetTick();
    s_data.online        = true;
    s_data.frame_cnt++;
}

/* ============================== 对外接口 ============================== */

/**
  * @brief  初始化 IMU 驱动：清状态 + 启动 DMA 循环接收 + 打开 IDLE 中断
  * @param  huart 已由 CubeMX 初始化好的串口句柄（如 &huart1）
  */
void HWT101_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    memset(&s_data, 0, sizeof(s_data));
    memset(s_frame, 0, sizeof(s_frame));
    s_frame_idx       = 0;
    s_dma_last_pos    = 0;
    s_yaw_zero_deg    = 0.0f;
    s_gyro_bias_z     = 0.0f;
    s_last_restart_ms = HAL_GetTick();
    s_data.last_frame_ms = HAL_GetTick();
    ring_reset();

    uart_start_rx();
}

/**
  * @brief  只重启接收链路，不清除零点与已解析数据
  * @note   串口 ORE/噪声错误后 HAL 会关掉 DMA 接收，在 HAL_UART_ErrorCallback 里调本函数
  */
void HWT101_RestartRx(void)
{
    s_frame_idx       = 0;
    s_dma_last_pos    = 0;
    s_last_restart_ms = HAL_GetTick();
    ring_reset();
    uart_start_rx();
}

/**
 * @brief 检查 IDLE 标志并搬运（核心逻辑，中断和轮询共用）
 */
static void idle_drain(void)
{
    if (s_huart == NULL)
    {
        return;
    }
    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) == RESET)
    {
        return;                             /* 没有 IDLE，直接返回，开销极小 */
    }

    s_idle_cnt++;
    uart_clear_idle_flag();                 /* 必须先清标志，避免反复进中断 */
    dma_to_ring();                          /* 增量搬运，绝不整块 memcpy */
}

/**
  * @brief  串口 IDLE 处理，必须在 USARTx_IRQHandler 中调用
  * @note   一次 IDLE 表示"数据流出现了一个空闲间隙"，即一帧（或一批帧）收完
  */
void HWT101_RxEventCallback(void)
{
    s_irq_cnt++;
    idle_drain();
}

/**
  * @brief  协议解析：必须在主循环（或 FreeRTOS 任务）里持续调用
  * @note   本函数不阻塞、不做长延时，单次耗时与已缓存字节数成正比
  */
void HWT101_Parse(void)
{
    uint8_t  byte;
    uint8_t  i;
    uint32_t now;

    if (s_huart == NULL)
    {
        return;
    }

    /* ---------- 0. 兜底：即使 IDLE 中断没挂上，主循环也主动搬 ---------- */
    idle_drain();                   /* 中断正常时这里是空操作（标志已清），开销极小 */

    /* ---------- 1. 逐字节组帧，靠 0x55 帧头同步 ---------- */
    while (ring_pop(&byte))
    {
        if ((s_frame_idx == 0u) && (byte != 0x55u))
        {
            continue;                       /* 等待帧头，丢弃杂散字节 */
        }

        s_frame[s_frame_idx++] = byte;

        if (s_frame_idx >= HWT101_FRAME_LEN)
        {
            s_frame_idx = 0;

            if (frame_checksum_ok(s_frame))
            {
                handle_frame(s_frame);
            }
            else
            {
                s_data.err_cnt++;           /* 校验失败：整帧丢弃，重新同步 */

                /* 若帧内又出现 0x55，可能是上一帧错位，做一次重同步 */
                for (i = 1u; i < HWT101_FRAME_LEN; i++)
                {
                    if (s_frame[i] == 0x55u)
                    {
                        uint8_t rest = (uint8_t)(HWT101_FRAME_LEN - i);
                        memmove(s_frame, &s_frame[i], rest);
                        s_frame_idx = rest;
                        break;
                    }
                }
            }
        }
    }

    /* ---------- 2. 掉线检测 + 自动重连 ---------- */
    now = HAL_GetTick();

    if (s_data.online && ((now - s_data.last_frame_ms) > HWT101_OFFLINE_MS))
    {
        s_data.online = false;              /* 长时间无数据：判掉线 */
    }

    if (((now - s_data.last_frame_ms) > HWT101_OFFLINE_MS) &&
        ((now - s_last_restart_ms) > HWT101_RESTART_MS))
    {
        HWT101_RestartRx();                 /* 重新挂 DMA + IDLE，可从断线恢复 */
    }
}

/**
  * @brief  静止状态下校准 Z 轴角速度零偏
  * @note   阻塞约 HWT101_GYRO_CALI_MS 毫秒，期间必须保持传感器静止
  */
void HWT101_CalibrateGyro(void)
{
    uint32_t start = HAL_GetTick();
    float    sum   = 0.0f;
    uint32_t count = 0;

    while ((HAL_GetTick() - start) < HWT101_GYRO_CALI_MS)
    {
        HWT101_Parse();
        if (s_data.online)
        {
            sum += s_data.gyro_dps[2];
            count++;
        }
        HAL_Delay(5);
    }

    if (count > 0u)
    {
        s_gyro_bias_z = sum / (float)count;
    }
}

/**
  * @brief 把当前朝向记为 0°（上电归零）
  */
void HWT101_ResetYawZero(void)
{
    s_yaw_zero_deg = s_data.yaw_raw_deg;
    s_data.yaw_deg = 0.0f;
}

/**
  * @brief 把当前朝向记为指定角度（用于对齐机械零位，如车头朝前 = 0°）
  */
void HWT101_SetYawZero(float zero_deg)
{
    s_yaw_zero_deg = wrap_pm180(s_data.yaw_raw_deg - zero_deg);
    s_data.yaw_deg = wrap_pm180(s_data.yaw_raw_deg - s_yaw_zero_deg);
}

float HWT101_GetYaw(void)
{
    return s_data.yaw_deg;
}

float HWT101_GetYawRate(void)
{
    return s_data.yaw_rate_dps;
}

const HWT101_Data_t *HWT101_GetData(void)
{
    return &s_data;
}

bool HWT101_IsOnline(void)
{
    return s_data.online;
}

uint32_t HWT101_GetFrameCount(void)
{
    return s_data.frame_cnt;
}

uint32_t HWT101_GetErrCount(void)
{
    return s_data.err_cnt;
}

uint32_t HWT101_GetRxBytes(void)
{
    return s_rx_bytes;
}

uint8_t HWT101_GetInitStatus(void)
{
    return s_init_status;
}

uint32_t HWT101_GetIrqCount(void)
{
    return s_irq_cnt;
}

uint32_t HWT101_GetIdleCount(void)
{
    return s_idle_cnt;
}

