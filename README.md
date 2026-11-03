# JY60 六轴 IMU 姿态采集（STM32F103 + HAL）

基于 STM32F103C8T6，通过 USART1 + DMA 循环接收 + IDLE 中断，读取维特智能 JY60
六轴 IMU 的姿态数据，输出 yaw / roll / pitch、三轴角速度与三轴加速度。

接口命名沿用 HWT101 培训模板（`hwt101.c` / `hwt101.h`），两者协议完全一致，
换成 HWT101 硬件可直接使用。

## 硬件

| 项目 | 参数 |
|---|---|
| MCU | STM32F103C8T6（Blue Pill），HSE 8MHz × 9 = 72MHz |
| 传感器 | 维特智能 JY60，TTL 串口，9600-8-N-1 |
| 出厂配置 | 回传 20Hz，±16g / ±2000dps / ±180°，3.3~5V 供电 |
| 协议 | WitMotion 标准协议，11 字节定长帧 |

### 接线

| STM32 引脚 | 方向 | JY60 引脚 | 说明 |
|---|---|---|---|
| PA9  (USART1_TX) | → | RX | 下发配置指令，**必须接** |
| PA10 (USART1_RX) | ← | TX | 姿态数据 |
| 3V3 或 5V | → | VCC | 模块支持 3.3~5V |
| GND | — | GND | 必须共地 |
| PA2  (USART2_TX) | → | USB-TTL 的 RX | printf 调试输出，115200-8-N-1 |
| PA13 / PA14 | — | ST-Link | SWD 烧录调试 |

> 容易漏的一条：**PA9 必须接到模块的 RX**。只接 RX 也能收到数据，但改不了回传速率，
> 也无法下发解锁/保存指令。

## 工作流程

```
USART1(9600) → DMA1_Channel5 循环缓冲 64B
             → IDLE 中断：增量搬运到软件环形缓冲 256B
             → HWT101_Parse()：逐字节组帧 + 累加和校验
             → HWT101_Data_t（业务层只读快照）
```

为什么用 DMA + IDLE 而不是逐字节中断：9600bps 下每 50ms 才来一帧（11 字节），
CPU 完全没必要为每个字节进一次中断；IDLE 中断的含义是"数据流出现空闲间隙"，
即一帧收完，此时一次性把新到的字节搬走，中断开销降到每帧一次。

解析放在主循环里做（`HWT101_Parse()`），中断里只搬运不解析，避免长中断。

## 快速开始

1. 用 Keil 打开 `MDK-ARM/JY60.uvprojx`，编译烧录。
2. **Options for Target → Target → 勾选 `Use MicroLIB`**。`main.c` 里 `fputc` 把
   printf 重定向到 USART2，缺了这项 printf 会卡死。
3. PA2 接 USB-TTL，串口助手 115200-8-N-1 打开。
4. 上电后应看到：

```
[JY60] IMU start
[JY60] online, frames=20
[JY60] zero done, yaw=0.00
yaw:0.12
yaw:0.15
yaw:0.14
```

`[JY60] online` 表示链路通了，随后主循环以 10Hz 打印相对零点的 yaw。

### 重新生成代码时 CubeMX 的配置要求

- USART1：Asynchronous，9600
- USART1_RX 添加 DMA：`DMA1_Channel5`，Mode = **Circular**，Byte/Byte
- NVIC：勾选 `USART1 global interrupt`（IDLE 中断依赖它）
- Project Manager：勾选 *Generate peripheral initialization as a pair of .c/.h files
  per peripheral*（否则没有 `gpio.c` / `usart.c` / `dma.c`，`main.c` 不能整份替换）

## 使用示例

```c
#include "hwt101.h"

HWT101_Init(&huart1);                      /* 挂 DMA 循环接收 + 开 IDLE 中断 */

uint32_t t0 = HAL_GetTick();
while (!HWT101_IsOnline() && (HAL_GetTick() - t0) < 1000u)
{
    HWT101_Parse();                        /* 等模块出数，最多等 1s */
}

HWT101_ResetYawZero();                     /* 当前位置记为 0° */
HWT101_CalibrateGyro();                    /* 静止 500ms 校准 Z 轴零偏，此时不要动 */

while (1)
{
    HWT101_Parse();                        /* 非阻塞，必须持续调用 */

    float yaw   = HWT101_GetYaw();         /* ±180°，相对零点 */
    float yaw_w = HWT101_GetYawRate();     /* °/s，已扣除零偏 */

    /* PID 航向控制写在这里 */
}
```

## API 速查

| 函数 | 说明 |
|---|---|
| `HWT101_Init(UART_HandleTypeDef*)` | 初始化并使能 DMA + IDLE 接收 |
| `HWT101_Parse(void)` | 解析字节流，主循环周期调用，非阻塞 |
| `HWT101_RxEventCallback(void)` | 在 `USARTx_IRQHandler` 中调用，处理 IDLE |
| `HWT101_GetYaw()` | 相对零点 yaw，±180° |
| `HWT101_GetYawRate()` | 去零偏角速度，°/s |
| `HWT101_GetData()` | 完整数据快照指针（只读） |
| `HWT101_ResetYawZero()` / `SetYawZero(float)` | 把当前朝向设为 0° / 指定角度 |
| `HWT101_CalibrateGyro()` | 静止校准 Z 轴角速度零偏，阻塞 500ms |
| `HWT101_IsOnline()` | 300ms 内收到有效帧则为 true |
| `HWT101_GetFrameCount()` / `GetErrCount()` | 有效帧数 / 校验错帧数 |
| `HWT101_RestartRx()` | 仅重启接收，保留零点（串口 ORE 后恢复用） |

诊断接口：`HWT101_GetInitStatus()`（位标志 0x01 串口空 / 0x02 DMA 空 / 0x04 启动失败 /
0x08 已启动）、`HWT101_GetRxBytes()`（0 表示链路完全没通）、`HWT101_GetIrqCount()`、
`HWT101_GetIdleCount()`。

## 协议说明

数据帧固定 11 字节：

```
0x55 | type | data[8] | sum
       ↑               ↑
   帧类型          前 10 字节累加和
```

| type | 内容 | 量程 | 换算 |
|---|---|---|---|
| 0x51 | 加速度 x,y,z | ±16g | 16.0 / 32768 per LSB |
| 0x52 | 角速度 x,y,z | ±2000dps | 2000.0 / 32768 per LSB |
| 0x53 | 角度 roll,pitch,yaw | ±180° | 180.0 / 32768 per LSB |

配置指令（5 字节，带 `0xFF 0xAA` 头），见 `main.c: JY60_SetRate20Hz()`：

| 指令 | 字节 | 作用 |
|---|---|---|
| 解锁 | `FF AA 69 88 B5` | 进入可配置状态 |
| 速率 | `FF AA 03 07 00` | 回传 20Hz（0x03=1Hz, 0x06=10Hz, 0x07=20Hz, 0x08=50Hz, 0x09=100Hz） |
| 保存 | `FF AA 00 00 00` | 写入模块 Flash，掉电不丢 |

速率设置成功并保存后，可把 `JY60_SetRate20Hz()` 的调用注释掉，缩短上电时间。

## 调试排查

先看串口打印的这行：

```
[JY60] NO DATA -> st=0x08 rx=0
```

| 现象 | 含义 | 处理 |
|---|---|---|
| `rx=0` | 一个字节都没收到 | 检查 PA10 是否接模块 TX、是否共地、模块供电 |
| `rx>0` 但 `frames=0` | 收到字节但组不了帧 | 波特率不匹配（JY60 出厂 9600），或 TX/RX 接反 |
| `st` 含 `0x02` | DMA 没配 | CubeMX 里给 USART1_RX 加 DMA（Circular） |
| `st` 含 `0x04` | DMA 启动失败 | 检查 DMA 通道冲突 |
| `irq` 一直为 0 | 中断没挂上 | NVIC 里勾 USART1 global interrupt |
| 偶尔掉线 | 干扰或线太长 | 缩短接线、确保共地；模块与 MCU 分开供电时务必共地 |

**硬件探针**：把 `main.c` 里的 `#define IMU_PROBE 0` 改成 `1`，编译烧录一次。
探针绕开 DMA 和中断，直接阻塞读 11 字节，用于区分"接线/波特率问题"和"DMA/中断配置问题"：

- `st=0` 且 `buf[0]=0x55` → 链路正常，问题在 DMA 或中断配置
- `st=0` 但 `buf[0]≠0x55` → 波特率不对，收到的是乱码
- `st=3`（timeout）→ PA10 上一个字节都没有，查接线、供电、模块

## 目录结构

```
JY60/
├── Core/
│   ├── Inc/         main.h  hwt101.h  usart.h  dma.h  gpio.h  stm32f1xx_hal_conf.h
│   └── Src/         main.c  hwt101.c  usart.c  dma.c  gpio.c  stm32f1xx_it.c
├── Drivers/         STM32F1xx HAL Driver + CMSIS（Device / Include）
├── MDK-ARM/         Keil 工程 JY60.uvprojx（构建输出不入库）
├── JY60.ioc         CubeMX 工程配置
└── .gitignore       排除 Keil 构建产物与个人环境文件
```

## 已知限制

- **单任务轮询**，无 RTOS。`HWT101_Parse()` 必须在主循环持续调用；在 20Hz 下环形缓冲
  256B 约能撑 1.1s，主循环里若有超过 1s 的阻塞操作会丢帧。
- `HWT101_CalibrateGyro()` 阻塞 500ms，期间虽不解析但 DMA 仍在收，缓冲足够容纳。
- 只处理 `0x51 / 0x52 / 0x53` 三种帧，若模块开启了磁场输出（`0x54`）会被静默丢弃。
- yaw 来自模块内部融合，静止长时间放置仍会缓慢漂移；需要绝对航向请换九轴模块或加磁力计校正。
- 当前 `main.c` 只打印 yaw，roll / pitch / 加速度已在 `HWT101_Data_t` 中，用 `HWT101_GetData()` 取。

## 许可

未指定。