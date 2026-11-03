/**
  ******************************************************************************
  * retarget.c - printf 重定向到 USART1（供 VOFA+ 看曲线）
  *
  * Keil 设置：Options for Target -> Target -> 勾选 Use MicroLIB
  * CubeMX 设置：USART1 波特率 115200，8N1，异步模式
  ******************************************************************************
  */
#include "usart.h"
#include "main.h"
#include <stdio.h>

#ifdef __GNUC__
/* GCC 版（STM32CubeIDE / CLion） */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
#else
/* Keil ARMCC / ARMCLANG 版 */
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif
