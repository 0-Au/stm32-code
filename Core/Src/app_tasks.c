/**
  ******************************************************************************
  * app_tasks.c - 培训第 2 点：sin_generate_task + motor_task（FreeRTOS + PID）
  *
  * 用法：
  *   1. CubeMX: FREERTOS 选 CMSIS_V1, SYS Timebase Source 改成 TIM3
  *   2. 本文件加入 Keil 工程，main.c 中调度器启动前调用 Motor_Init() + App_TasksInit()
  *   3. Encoder_GetSpeed() / Motor_SetPWM() 由 bsp_motor.c 实现
  *
  * 数据接口提供两种（二选一即可，默认方法一）：
  *   方法一：volatile 全局变量（float 读写原子，一写一读安全）
  *   方法二：长度 1 队列 + xQueueOverwrite（只保留最新值）
  ******************************************************************************
  */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pid.h"
#include "bsp_motor.h"
#include <math.h>
#include <stdio.h>

/* ---------- 参数配置 ---------- */
#define SIN_AMPLITUDE    50.0f   /* 目标速度幅值 ±50 rpm */
#define SIN_PERIOD_S     4.0f    /* 正弦周期 4 秒 */
#define SIN_TASK_DT_MS   20      /* sin 任务周期 20ms */
#define MOTOR_TASK_DT_MS 10      /* 控制任务周期 10ms */
#define PWM_MAX          7200    /* PWM 上限，按你的定时器 ARR+1 改 */

#define ENABLE_VOFA      1       /* 1=向 VOFA+ 输出波形数据（两路数值） */
#define ENABLE_MONITOR   1       /* 1=每 2 秒打印任务栈余量 */

/* ---------- 方法一：受控全局变量 ---------- */
volatile float g_target_speed = 0;

/* ---------- 方法二：队列（备用，默认未启用） ---------- */
#define USE_QUEUE 0
#if USE_QUEUE
static QueueHandle_t speed_queue;
#endif

static PID_t motor_pid;

static TaskHandle_t s_sin_task   = NULL;
static TaskHandle_t s_motor_task = NULL;
#if ENABLE_MONITOR
static TaskHandle_t s_mon_task   = NULL;
#endif

static volatile uint32_t s_motor_overrun = 0;   /* 控制周期超时次数，由监控任务打印 */

/**
  * @brief 周期生成正弦目标速度
  *        优先级低于 motor_task
  */
void sin_generate_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    float t = 0.0f;
    float target;
    const float dt = SIN_TASK_DT_MS / 1000.0f;

    for (;;) {
        /* 绝对延时：保证严格 20ms 一拍，不随执行时间漂移 */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SIN_TASK_DT_MS));
        t += dt;

        /* sinf 需要 C99；若 Keil 报未定义，勾选 Options→C/C++→C99 Mode，或改用 sin() */
        target = SIN_AMPLITUDE * sinf(2.0f * 3.1415926f * t / SIN_PERIOD_S);

#if USE_QUEUE
        xQueueOverwrite(speed_queue, &target);   /* 队列满也覆盖，只留最新值 */
#else
        g_target_speed = target;
#endif
    }
}

/**
  * @brief 固定周期执行 PID 速度闭环
  *        优先级高于 sin_generate_task 和显示/日志类任务
  */
void motor_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    TickType_t now;
    float target = 0.0f;
    float actual;
    float out;
#if USE_QUEUE
    float latest = 0.0f;
#endif

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(MOTOR_TASK_DT_MS));

        /* 检查是否错过唤醒时刻（说明本周期的活干超时了） */
        now = xTaskGetTickCount();
        if ((now - wake) > 1) {
            s_motor_overrun++;      /* 记一次"错过唤醒时刻" */
        }

#if USE_QUEUE
        if (xQueuePeek(speed_queue, &latest, 0) == pdTRUE) {
            target = latest;   /* 队列空则沿用上次的值 */
        }
#else
        target = g_target_speed;
#endif

        actual = Encoder_GetSpeed();
        out    = PID_Calc(&motor_pid, target, actual);
        Motor_SetPWM((int)out);

#if ENABLE_VOFA
        /* VOFA+ 数据引擎选 FireWater：两路数值，按出现顺序对应 ch0 / ch1 */
        printf("%.1f,%.1f\n", target, actual);
#endif
    }
}

#if ENABLE_MONITOR
/**
  * @brief 每 2 秒打印各任务栈余量与超时次数（验收要求"观察任务栈占用"）
  *        栈余量单位 word(4 字节)，低于 50 说明该加栈了
  */
void monitor_task(void *argument)
{
    (void)argument;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        printf("[MON] sin stack=%u, motor stack=%u, overrun=%lu\r\n",
               (unsigned)uxTaskGetStackHighWaterMark(s_sin_task),
               (unsigned)uxTaskGetStackHighWaterMark(s_motor_task),
               (unsigned long)s_motor_overrun);

        /* 余量告警：小于 50 word 就该调大 xTaskCreate 的栈参数 */
        if (uxTaskGetStackHighWaterMark(s_motor_task) < 50) {
            printf("[MON] WARNING: motor task stack too low!\r\n");
        }
    }
}
#endif

/**
  * @brief 创建两个任务，main.c 中 vTaskStartScheduler()/osKernelStart() 前调用
  */
void App_TasksInit(void)
{
    /* PID 参数需实机整定，这里只是能跑的起始值 */
    PID_Init(&motor_pid,
             70.0f,   /* Kp */
             10.0f,     /* Ki */
             0.0f,     /* Kd：速度环通常先不用微分 */
             500.0f,   /* 积分限幅 */
             PWM_MAX); /* 输出限幅 */

#if USE_QUEUE
    speed_queue = xQueueCreate(1, sizeof(float));   /* 长度1：只保留最新值 */
#endif

    /* 栈单位是 word（4 字节）；控制任务优先级更高 */
    xTaskCreate(sin_generate_task, "sin",   256, NULL, 2, &s_sin_task);
    xTaskCreate(motor_task,        "motor", 256, NULL, 3, &s_motor_task);
#if ENABLE_MONITOR
    xTaskCreate(monitor_task,      "mon",   256, NULL, 1, &s_mon_task);
#endif
}
