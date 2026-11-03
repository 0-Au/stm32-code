/**
  ******************************************************************************
  * @file    retarget.c
  * @brief   printf 重定向到 USART1 (PA9/PA10, 115200 8N1)
  *
  * 使用方法：
  *   1. 把本文件加入工程（Keil: 右键 Application/User -> Add Existing Files）
  *   2. Keil 勾选 Options for Target -> Target -> Use MicroLIB
  *      （STM32CubeIDE 不用勾，默认就有 _write，但两边不要同时定义）
  *   3. 打印浮点 %f：Keil 的 MicroLIB 默认不支持，需要在
  *      Options -> C/C++ -> Misc Controls 里加 -u_printf_float
  *
  * 注意：HAL_UART_Transmit 是阻塞的。115200 下一行 40 字节约 3.5ms，
  *       所以主循环里 printf 的频率要压到 100ms 以上，别放进控制周期。
  *       后面上 PID 时，串口打印必须改成 DMA 或环形缓冲，否则会啃掉控制时间。
  ******************************************************************************
  */
#include "main.h"
#include "usart.h"
#include <stdio.h>

#ifdef __GNUC__
/* STM32CubeIDE / GCC 走 _write */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}
#else
/* Keil MDK / ARMCC 走 fputc */
int fputc(int ch, FILE *f)
{
    uint8_t c = (uint8_t)ch;
    (void)f;
    HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
    return ch;
}
#endif
