# Motorsinpid 直流电机正弦跟踪（STM32F103 + FreeRTOS + PID）

基于 STM32F103C8T6，在 FreeRTOS 上实现直流有刷电机的**速度闭环**：一个任务周期生成
正弦波目标转速，另一个任务用增量式编码器测速 + 位置式 PID 做速度控制，通过 TIM PWM
驱动电机，并把目标/实际转速输出到 VOFA+ 实时看波形。

工程是「FreeRTOS + PID」培训任务，`app_tasks.c` 里预留了两种任务间通信方式
（volatile 全局变量 / 长度 1 队列），默认用全局变量。

## 硬件

| 项目 | 参数 |
|---|---|
| MCU | STM32F103C8T6，HSE 8MHz × 9 = 72MHz |
| 电机 | 直流有刷减速电机（带增量式编码器） |
| 编码器 | 13 线，TIM2 四倍频，减速比 30:1（按电机铭牌修改） |
| 驱动 | TB6612 类 H 桥，AIN1/AIN2 方向 + PWM 调速 |
| 调试输出 | USART1 115200-8-N-1，接 VOFA+ |

### 接线

| STM32 引脚 | 方向 | 外设 | 说明 |
|---|---|---|---|
| PA8  (TIM1_CH1) | → | 驱动板 PWM | 10kHz PWM，ARR=7199 |
| PB12 (AIN1) | → | 驱动板 AIN1 | 方向脚 1 |
| PB13 (AIN2) | → | 驱动板 AIN2 | 方向脚 2 |
| PA0  (TIM2_CH1) | ← | 编码器 A 相 | 编码器模式输入 |
| PA1  (TIM2_CH2) | ← | 编码器 B 相 | 编码器模式输入 |
| PA9  (USART1_TX) | → | USB-TTL 的 RX | VOFA+ 波形，115200 |
| PA10 (USART1_RX) | ← | USB-TTL 的 TX | 备用 |
| PA13 / PA14 | — | ST-Link | SWD 烧录调试 |

## 系统架构

```
[sin_generate_task]                 [motor_task]                  [monitor_task]
优先级 2，20ms 一拍               优先级 3，10ms 一拍             优先级 1，2s 一拍
生成正弦目标转速 ──(全局变量)──► 读目标 → 编码器测速           打印栈余量与超时
                                  → PID 计算                     (验收: 观察栈占用)
                                  → Motor_SetPWM 驱动
                                  → printf 输出到 VOFA+
```

三个任务的关系：

- `sin_generate_task` 用 `sinf()` 生成 ±50 rpm、周期 4s 的正弦目标速度
- `motor_task` 每 10ms 采样一次编码器转速，跑 PID 得到 PWM 输出，驱动电机
- `monitor_task` 每 2s 打印各任务栈余量，`< 50` word 时告警

## 快速开始

1. Keil 打开 `MDK-ARM/Motorsinpid.uvprojx`，编译烧录。
2. **Options for Target → Target → 勾选 `Use MicroLIB`**（printf 重定向必需）。
3. **Options for Target → C/C++ → 勾选 `C99 Mode`**（`sinf()` 依赖 C99）。
4. 串口助手或 VOFA+ 打开 USART1，115200-8-N-1。
5. 上电后电机开始跟踪正弦目标转速，串口每秒输出约 100 行：

```
-12.3,-10.8
-35.1,-33.9
-50.0,-49.2
...
```

每行两个数：`目标转速, 实际转速`（单位 rpm）。

### 用 VOFA+ 看波形

- VOFA+ 数据引擎选 **FireWater**
- 串口配置 115200，打开后**右键波形区 → 选择通道**，两个数值按出现顺序对应 **ch0 / ch1**
- ch0 是目标转速，ch1 是实际转速，两者越贴合说明 PID 跟踪越好

## 关键参数与调参

### PID 参数（`app_tasks.c: App_TasksInit`）

| 参数 | 初值 | 说明 |
|---|---|---|
| Kp | 70.0 | 响应速度，太大会振荡 |
| Ki | 10.0 | 消除静差，太大 + 目标突变会积分饱和 |
| Kd | 0.0 | 速度环通常先不用微分 |
| integral_max | 500.0 | 积分限幅，防饱和 |
| output_max | 7200 | 输出限幅，对应 PWM 上限 |

> 这是能跑起来的起始值，**必须实机整定**。经验顺序：先只调 Kp 到不振荡、稍有静差，
> 再逐步加 Ki 消除静差，最后按需加 Kd 抑制超调。

### 结构参数（`bsp_motor.h`）

```c
#define ENCODER_PPR        13      /* 编码器线数，按铭牌改 */
#define ENCODER_MULTIPLY   4       /* 四倍频 */
#define GEAR_RATIO         30.0f   /* 减速比，直驱填 1.0f */
```

编码器线数和减速比**必须按实际电机铭牌改**，否则转速读数是错的，PID 无从调起。

### 控制周期

| 宏 | 值 | 说明 |
|---|---|---|
| `SIN_TASK_DT_MS` | 20 | 正弦任务周期 |
| `MOTOR_TASK_DT_MS` | 10 | PID 控制周期 |
| `PWM_MAX` | 7200 | = ARR + 1，与定时器一致 |

> **控制周期一旦改变，PID 的 Ki、Kd 都必须重调**（积分、微分项都与周期相关），
> 这是验收常问的点。

## 定时器与外设配置

| 外设 | 配置 | 用途 |
|---|---|---|
| TIM1 CH1 | PWM，PSC=0，ARR=7199 → 10kHz，PA8 | 电机调速 |
| TIM2 | 编码器模式 TI12，ARR=65535，PA0/PA1 | 编码器测速 |
| TIM3 | 时基中断（`HAL_IncTick`） | FreeRTOS 时基，取代 SysTick |
| USART1 | 异步 115200-8-N-1 | printf → VOFA+ |
| GPIO | PB12/PB13 推挽输出 | H 桥方向 |

## 代码结构

| 文件 | 职责 |
|---|---|
| `app_tasks.c` | FreeRTOS 任务：正弦生成、PID 控制、监控 |
| `bsp_motor.c/.h` | 电机 PWM + 编码器测速（BSP 层） |
| `pid.c/.h` | 位置式 PID（积分限幅 + 输出限幅） |
| `retarget.c` | printf 重定向到 USART1 |
| `freertos.c` | CubeMX 生成的 FreeRTOS 对象 |
| `main.c` | 初始化后调 `Motor_Init()` + `App_TasksInit()` |

分层原则：`app_tasks.c` 只调 `bsp_motor.h` 和 `pid.h` 的接口，不碰寄存器；
引脚改了只改 `bsp_motor.c` 顶部的宏，上层不用动。

## API 速查

### 电机与编码器（bsp_motor.h）

| 函数 | 说明 |
|---|---|
| `Motor_Init()` | 启动 PWM 和编码器，方向脚清零 |
| `Motor_SetPWM(int)` | 输入 ±7200，正负代表方向 |
| `Motor_GetPWM()` | 返回当前 PWM，调试用 |
| `Encoder_GetSpeed()` | 两次调用间平均转速，rpm |
| `Encoder_GetTotalCount()` | 累计脉冲，位置环用 |
| `Encoder_Reset()` | 清零计数与测速基准 |

### PID（pid.h）

| 函数 | 说明 |
|---|---|
| `PID_Init(pid, kp, ki, kd, i_max, o_max)` | 初始化参数与限幅 |
| `PID_Calc(pid, target, actual)` | 算一次，返回限幅后的输出 |
| `PID_Reset(pid)` | 清零积分与上次误差 |

## 已知限制与注意

- **PID 是位置式**，积分项持续累加，目标突变或长时间堵转会积分饱和，靠 `integral_max`
  限幅缓解；需要更强抗饱和可改增量式或加积分分离。
- **编码器计数用 16 位**（`int16_t`），两次采样间若转过半圈以上会溢出，低速正弦场景
  足够，高速或大减速比时需注意采样周期不能太长。
- **PWM=0 是滑行停止**（两方向脚都低），不是刹车；需要快速停车可把两脚都置高实现制动。
- **`sinf()` 依赖 C99**，Keil 报未定义就勾 C99 Mode，或把 `sinf` 换成 `sin`。
- 栈余量单位是 **word（4 字节）**&#8203;，`< 50` 表示该加大 `xTaskCreate` 的栈参数了。

## 目录结构

```
Motorsinpid/
├── Core/
│   ├── Inc/         main.h  bsp_motor.h  pid.h  FreeRTOSConfig.h ...
│   └── Src/         main.c  app_tasks.c  bsp_motor.c  pid.c  retarget.c  freertos.c ...
├── Drivers/         STM32F1xx HAL Driver + CMSIS
├── Middlewares/     FreeRTOS
├── MDK-ARM/         Keil 工程（构建输出不入库）
├── Motorsinpid.ioc  CubeMX 工程配置
└── .gitignore       排除构建产物与个人环境文件
```

## 许可

未指定。
