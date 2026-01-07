/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include <string.h>
#include <stdio.h>
#include "bsp_fdcan.h"
#include <math.h>  

#include "as5047p.h"
#include "odrive_can.h"
#include "joint_mit.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== 1kHz loop -> 10Hz print =====
#define PRINT_DIV               100   // 1kHz/100=10Hz

#define AXIS_STATE_CLOSED_LOOP  8
#define CONTROL_MODE_TORQUE     1
#define INPUT_MODE_PASSTHROUGH  1

#define JOINT_NODE_ID           6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UNUSED(x) ((void)(x))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_tick_1khz = 0;
volatile uint8_t  g_tick_flag = 0;
static char uart_buf[160];

static AS5047P   g_enc1;
static ODriveCan g_od1;
static JointMIT  g_j1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART10_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // 1) Power enable (如果你的板子需要)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(50);

  // 2) CAN init/start
  bsp_can_init();
  HAL_Delay(200);

  // encoder
  g_enc1.hspi = &hspi1;
  g_enc1.cs_port = GPIOE;
  g_enc1.cs_pin  = GPIO_PIN_15;
  as5047p_init(&g_enc1);

  // odrive
  g_od1.hcan = &hfdcan1;
  g_od1.node_id = JOINT_NODE_ID;

  // joint
  joint_mit_init(&g_j1);
  g_j1.kp = 2.0f;
  g_j1.kd = 0.05f;
  g_j1.tau_max = 0.30f;
  g_j1.torque_sign = -1;   // 反方向

  // ODrive state
  odrive_can_clear_errors(&g_od1);
  HAL_Delay(20);

  odrive_can_set_axis_state(&g_od1, AXIS_STATE_CLOSED_LOOP);
  HAL_Delay(100);

  odrive_can_set_controller_mode(&g_od1, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
  HAL_Delay(100);

  // zero
  uint8_t e = 0;
  uint16_t raw0 = as5047p_read_angle_raw14(&g_enc1, &e);
  if (e == 0) {
      joint_mit_zero_here(&g_j1, (int32_t)raw0);
  } else {
      const char *msg = "[WARN] encoder zero failed\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 50);
  }

  // 5) start 1kHz tick
  HAL_TIM_Base_Start_IT(&htim2);

  const char *boot = "\r\n[H725] Joint1 MIT: AS5047P + MiniODrive torque @1kHz\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)boot, (uint16_t)strlen(boot), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_tick = 0;
  uint32_t print_ctr = 0;

  while (1)
  {
      if (!g_tick_flag) continue;
      g_tick_flag = 0;

      if (g_tick_1khz == last_tick) continue;
      last_tick = g_tick_1khz;

      uint8_t e = 0;
      uint16_t raw = as5047p_read_angle_raw14(&g_enc1, &e);

      if (e != 0) {
          odrive_can_set_input_torque(&g_od1, 0.0f);
          continue;
      }

      joint_mit_step_1khz(&g_j1, (int32_t)raw);
      odrive_can_set_input_torque(&g_od1, g_j1.tau);

      if (++print_ctr >= PRINT_DIV) {
          print_ctr = 0;
          int n = snprintf(uart_buf, sizeof(uart_buf),
              "[t=%lu] raw=%u zero=%ld q=%ldmrad qd=%ldmrad/s tau=%ldmNm err=%u\r\n",
              (unsigned long)g_tick_1khz,
              (unsigned)raw,
              (long)g_j1.raw_zero,
              (long)(g_j1.q * 1000.0f),
              (long)(g_j1.qd * 1000.0f),
              (long)(g_j1.tau * 1000.0f),
              (unsigned)e);

          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n, 100);
      }
  }


/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
	g_tick_1khz++;
	g_tick_flag = 1;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
