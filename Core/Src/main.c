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

#define JOINT_NODE_ID_1           2
#define JOINT_NODE_ID_2           6
#define JOINT_NODE_ID_3           1

#define HOME_RAW_ABS_1   8000   // TODO: 改成你的真实值（输出轴绝对raw）
#define HOME_RAW_ABS_2   7500   // TODO: 改成你的真实值（输出轴绝对raw）
#define HOME_RAW_ABS_3   3300   // TODO: 改成你的真实值（输出轴绝对raw）
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

// encoder #1
static AS5047P   g_enc1;
static ODriveCan g_od1;
static JointMIT  g_j1;
// encoder #2
static AS5047P   g_enc2;
static ODriveCan g_od2;
static JointMIT  g_j2;

// encoder #3
static AS5047P   g_enc3;
static ODriveCan g_od3;
static JointMIT  g_j3;
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

  g_enc2.hspi = &hspi1;
  g_enc2.cs_port = GPIOB;
  g_enc2.cs_pin  = GPIO_PIN_10;
  as5047p_init(&g_enc2);

  g_enc3.hspi = &hspi1;
  g_enc3.cs_port = GPIOB;
  g_enc3.cs_pin  = GPIO_PIN_11;
  as5047p_init(&g_enc3);

  // odrive
  g_od1.hcan = &hfdcan1;
  g_od1.node_id = JOINT_NODE_ID_1;
  
  g_od2.hcan = &hfdcan1;
  g_od2.node_id = JOINT_NODE_ID_2;

  g_od3.hcan = &hfdcan1;
  g_od3.node_id = JOINT_NODE_ID_3;
  // joint
  joint_mit_init(&g_j1);
  joint_mit_init(&g_j2);
  joint_mit_init(&g_j3);
  /* 你的实际参数 */
  joint_mit_config_raw(&g_j1, 3000, 10000, 1.0f); // 10:1 gear
  joint_mit_config_raw(&g_j2, 5000, 10000, 1.0f);
  joint_mit_config_raw(&g_j3, 2500, 4000, 1.0f);

  g_j1.kp = 0.45f;        // 齿隙系统：kp 不要大
  g_j1.kd = 0.025f;
  g_j1.tau_max = 0.8f;

  g_j1.q_db        = 0.015f; // 1 deg
  g_j1.q_hold_band = 0.020f;
  g_j1.qd_alpha    = 0.05f;
  g_j1.tau_rate_limit = 10.0f;

  g_j1.torque_sign = -1; // 如果方向反

  g_j2.kp = 0.35f;
  g_j2.kd = 0.025f;
  g_j2.tau_max = 0.8f;

  g_j2.q_db = 0.015f;
  g_j2.q_hold_band = 0.020f;
  g_j2.qd_alpha = 0.05f;
  g_j2.tau_rate_limit = 10.0f;

  g_j2.torque_sign = -1;   // 方向需要单独确认

  g_j3.kp = 0.45f;
  g_j3.kd = 0.025f;
  g_j3.tau_max = 0.8f;

  g_j3.q_db = 0.015f;
  g_j3.q_hold_band = 0.020f;
  g_j3.qd_alpha = 0.05f;
  g_j3.tau_rate_limit = 10.0f;

  g_j3.torque_sign = -1;   // 方向需要单独确认
  // ODrive state
  odrive_can_clear_errors(&g_od1);
  odrive_can_clear_errors(&g_od2);
  odrive_can_clear_errors(&g_od3);
  HAL_Delay(20);

  odrive_can_set_axis_state(&g_od1, AXIS_STATE_CLOSED_LOOP);
  odrive_can_set_axis_state(&g_od2, AXIS_STATE_CLOSED_LOOP);
  odrive_can_set_axis_state(&g_od3, AXIS_STATE_CLOSED_LOOP);
  HAL_Delay(100);

  odrive_can_set_controller_mode(&g_od1, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
  odrive_can_set_controller_mode(&g_od2, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
  odrive_can_set_controller_mode(&g_od3, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
  HAL_Delay(100);

  // zero
  uint8_t e1 = 0;
  uint16_t raw1 = as5047p_read_angle_raw14(&g_enc1, &e1);
  uint8_t e2 = 0;
  uint16_t raw2 = as5047p_read_angle_raw14(&g_enc2, &e2);
  uint8_t e3 = 0;
  uint16_t raw3 = as5047p_read_angle_raw14(&g_enc3, &e3);

  if (e1 == 0) joint_mit_zero_here(&g_j1, HOME_RAW_ABS_1);
  else HAL_UART_Transmit(&huart1, (uint8_t*)"[WARN] enc1 zero failed\r\n", 25, 50);

  if (e2 == 0) joint_mit_zero_here(&g_j2, HOME_RAW_ABS_2);
  else HAL_UART_Transmit(&huart1, (uint8_t*)"[WARN] enc2 zero failed\r\n", 25, 50);

  if (e3 == 0) joint_mit_zero_here(&g_j3, HOME_RAW_ABS_3);
  else HAL_UART_Transmit(&huart1, (uint8_t*)"[WARN] enc2 zero failed\r\n", 25, 50);

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

      uint8_t e1 = 0;
      uint16_t raw1 = as5047p_read_angle_raw14(&g_enc1, &e1);
      uint8_t e2 = 0;
      uint16_t raw2 = as5047p_read_angle_raw14(&g_enc2, &e2);
      uint8_t e3 = 0;
      uint16_t raw3 = as5047p_read_angle_raw14(&g_enc3, &e3);

      if (e1 == 0 && g_j1.zero_valid) {
          joint_mit_step_1khz(&g_j1, (int32_t)raw1);
          odrive_can_set_input_torque(&g_od1, g_j1.tau);
      } else {
          odrive_can_set_input_torque(&g_od1, 0.0f);
      }

      if (e2 == 0 && g_j2.zero_valid) {
          joint_mit_step_1khz(&g_j2, (int32_t)raw2);
          odrive_can_set_input_torque(&g_od2, g_j2.tau);
      } else {
          odrive_can_set_input_torque(&g_od2, 0.0f);
      }

      if (e3 == 0 && g_j3.zero_valid) {
          joint_mit_step_1khz(&g_j3, (int32_t)raw3);
          odrive_can_set_input_torque(&g_od3, g_j3.tau);
      } else {
          odrive_can_set_input_torque(&g_od3, 0.0f);
      }

      if (++print_ctr >= PRINT_DIV) {
          print_ctr = 0;
          int n = snprintf(uart_buf, sizeof(uart_buf),
              "[t=%lu] raw1=%u raw2=%u raw3=%u q=%ldmrad qd=%ldmrad/s tau=%ldmNm err=%u\r\n",
              (unsigned long)g_tick_1khz,
              (unsigned)raw1,
		      (unsigned)raw2,
			  (unsigned)raw3,
              (long)(g_j1.q * 1000.0f),
              (long)(g_j1.qd * 1000.0f),
              (long)(g_j1.tau * 1000.0f),
              (unsigned)e1);

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
