/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : JY60 六轴 IMU 采集（USART1 + DMA 循环接收 + IDLE 中断）
  * @note           直接整份替换 CubeMX 生成的 Core/Src/main.c 即可。
  *                 前提：CubeMX 已勾选
  *                 "Generate peripheral initialization as a pair of .c/.h files
  *                  per peripheral"（工程里存在 gpio.c / usart.c / dma.c）。
  *                 若未勾选，请不要整份替换，只把 USER CODE 区内容抄过去。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hwt101.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 上电自动配置 JY60：解锁 → 设回传速率 → 保存
 * @note  必须先把 PA9 (USART1_TX) 接到模块的 RX 引脚，否则指令发不出去
 *        速率码：0x03=1Hz 0x06=10Hz 0x07=20Hz 0x08=50Hz 0x09=100Hz
 *        已保存后掉电不丢，之后可把本函数调用注释掉
 */
static void JY60_SetRate20Hz(void)
{
  uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};  /* 解锁 */
  uint8_t rate[]   = {0xFF, 0xAA, 0x03, 0x07, 0x00};  /* 回传速率 20Hz */
  uint8_t save[]   = {0xFF, 0xAA, 0x00, 0x00, 0x00};  /* 保存到 Flash */

  HAL_UART_Transmit(&huart1, unlock, 5u, 100u);
  HAL_Delay(50);
  HAL_UART_Transmit(&huart1, rate,   5u, 100u);
  HAL_Delay(50);
  HAL_UART_Transmit(&huart1, save,   5u, 100u);
  HAL_Delay(50);
}

/* printf 重定向到 USART2（需在 Options for Target 勾选 Use MicroLIB） */
int fputc(int ch, FILE *f)
{
  (void)f;
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1u, 10u);
  return ch;
}

/* ============================================================
 * 诊断探针：把 IMU_PROBE 改成 1，编译烧录一次即可判断
 *   链路问题（接线/波特率/模块）还是 DMA/中断配置问题。
 *   探针绕过 DMA 和中断，直接阻塞读 11 字节。
 * 判读：
 *   st=0 且 buf[0]=0x55  -> 硬件链路完全正常，问题在 DMA/中断
 *   st=0 但 buf[0]!=0x55 -> 波特率不对，收到的是乱码
 *   st=3 (timeout)       -> PA10 上一个字节都没有：接线/供电/模块
 * ============================================================ */
#define IMU_PROBE   0

#if IMU_PROBE
static void IMU_Probe(void)
{
  uint8_t  buf[11];
  uint8_t  i;
  HAL_StatusTypeDef st;

  for (i = 0; i < 11u; i++) { buf[i] = 0; }

  printf("[probe] waiting 11 bytes on USART1 @9600...\r\n");
  st = HAL_UART_Receive(&huart1, buf, 11u, 2000u);

  printf("[probe] st=%d data:", (int)st);
  for (i = 0; i < 11u; i++) { printf(" %02X", buf[i]); }
  printf("\r\n[probe] st=0 ok / 1 error / 2 busy / 3 timeout\r\n");
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t t0;
  uint32_t last_print;
  const HWT101_Data_t *d;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n[JY60] IMU start\r\n");

#if IMU_PROBE
  IMU_Probe();                 /* 探针要在 HWT101_Init 之前，否则 DMA 会抢走数据 */
#endif

  /* 0. 上电先配置模块回传速率为 20Hz（需要 PA9 已接到模块 RX） */
  JY60_SetRate20Hz();

  /* 1. 挂 DMA 循环接收 + 打开 IDLE 中断 */
  HWT101_Init(&huart1);

  /* 2. 等模块出数（JY60 上电约 100ms 开始回传，20Hz） */
  t0 = HAL_GetTick();
  while (!HWT101_IsOnline() && ((HAL_GetTick() - t0) < 1000u))
  {
    HWT101_Parse();
  }

  if (!HWT101_IsOnline())
  {
    printf("[JY60] NO DATA -> st=0x%02X rx=%lu\r\n",
           HWT101_GetInitStatus(), (unsigned long)HWT101_GetRxBytes());
    printf("       st: 01=uart null 02=dma null 04=dma fail 08=started\r\n");
    printf("       rx=0 -> wiring/baud/idle-callback ; rx>0 -> baud or protocol\r\n");
  }
  else
  {
    printf("[JY60] online, frames=%lu\r\n", (unsigned long)HWT101_GetFrameCount());
  }

  /* 3. 上电归零：当前朝向记为 0° */
  HWT101_ResetYawZero();

  /* 4. 静止 0.5s 校准 Z 轴角速度零偏（此时不要动模块） */
  HWT101_CalibrateGyro();

  printf("[JY60] zero done, yaw=%.2f\r\n", HWT101_GetYaw());

  last_print = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while(1)
	 {
    HWT101_Parse();            /* 非阻塞，必须持续调用 */

    if ((HAL_GetTick() - last_print) >= 100u)   /* 10Hz 打印 */
    {
      d = HWT101_GetData();
      last_print = HAL_GetTick();

      printf("yaw:%.2f\n",
             d->yaw_deg
            );
    }
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 控制逻辑放这里：PID 用 HWT101_GetYaw() / HWT101_GetYawRate() */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
