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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    // encoder
    int32_t raw_abs;     // 当前绝对 raw (5500~10500)
    int32_t raw_zero;    // “零点”绝对 raw（上电/按键校准得到）
    int32_t raw_rel;     // raw_abs - raw_zero

    // state
    float q;             // rad
    float q_prev;
    float qd;            // rad/s
    float qd_f;

    // cmd / gains
    float q_des;         // rad
    float qd_des;        // rad/s
    float kp;
    float kd;
    float tau_ff;
    float tau;
    float tau_max;

    uint8_t zero_valid;
} JointState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== 1kHz loop -> 10Hz print =====
#define PRINT_DIV               100   // 1kHz/100=10Hz

// ===== Encoder raw boundary (ABS raw) =====
#define RAW_MIN                 5500
#define RAW_MAX                 10500
#define RAW_MID                 ((RAW_MIN + RAW_MAX)/2)      // 8000
#define RAW_HALF_RANGE          ((RAW_MAX - RAW_MIN)/2)      // 2500

// 你还没给真实关节行程，先定义一个“临时映射”跑通：±1 rad
#define JOINT_Q_MAX_RAD         1.0f

// ===== Soft limit =====
#define MARGIN_RAW              200
#define K_SOFT                  0.002f     // Nm/raw 先保守，确认方向后再加
#define TAU_MAX_DEFAULT         0.30f      // Nm 先保守

// ===== ODrive CAN (Mini ODrive) =====
#define JOINT_NODE_ID           6
#define CMD_SET_AXIS_STATE      0x07
#define CMD_SET_CONTROLLER_MODE 0x0B
#define CMD_SET_INPUT_TORQUE    0x0E
#define CMD_CLEAR_ERRORS        0x18

#define AXIS_STATE_CLOSED_LOOP  8
#define CONTROL_MODE_TORQUE     1
#define INPUT_MODE_PASSTHROUGH  1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UNUSED(x) ((void)(x))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_tick_1khz = 0;
volatile uint8_t  g_tick_flag = 0;

static uint16_t enc_raw[1];
static uint8_t  enc_err[1];

static char uart_buf[160];

// 关节状态（注意：不要叫 j1）
static JointState g_joint1 = {
    .q_des = 0.0f,
    .qd_des = 0.0f,
    .kp = 1.0f,            // POC 先小
    .kd = 0.0f,            // 后面加阻尼
    .tau_ff = 0.0f,
    .tau_max = TAU_MAX_DEFAULT,
    .zero_valid = 0,
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void joint1_zero_here(int32_t raw_now);
static void joint1_control_1khz(int32_t raw_abs);

static void odrive_clear_errors(FDCAN_HandleTypeDef* hcan, uint8_t node_id);
static void odrive_set_axis_state(FDCAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t state);
static void odrive_set_controller_mode(FDCAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t control_mode, uint32_t input_mode);
static void odrive_set_input_torque(FDCAN_HandleTypeDef* hcan, uint8_t node_id, float tau_nm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ===== AS5047P protocol =====
#define AS5047P_CMD_READ       0x4000u
#define AS5047P_REG_ANGLECOM   0x3FFFu
#define AS5047P_DATA_MASK      0x3FFFu
#define AS5047P_ERR_MASK       0x4000u

typedef struct {
  GPIO_TypeDef *cs_port;
  uint16_t      cs_pin;
} EncoderCS;

static EncoderCS enc_cs[1] = {
  {GPIOE, GPIO_PIN_15},
};

static uint16_t g_last_rx2 = 0;

static inline void CS_Low(const EncoderCS *cs)  { HAL_GPIO_WritePin(cs->cs_port, cs->cs_pin, GPIO_PIN_RESET); }
static inline void CS_High(const EncoderCS *cs) { HAL_GPIO_WritePin(cs->cs_port, cs->cs_pin, GPIO_PIN_SET); }

static HAL_StatusTypeDef spi1_txrx_u16(uint16_t tx, uint16_t *rx)
{
  uint8_t txb[2] = {(uint8_t)(tx >> 8), (uint8_t)(tx & 0xFF)};
  uint8_t rxb[2] = {0, 0};
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, txb, rxb, 2, 2);
  if (st == HAL_OK && rx) *rx = ((uint16_t)rxb[0] << 8) | (uint16_t)rxb[1];
  return st;
}

static inline uint16_t as5047p_apply_parity(uint16_t v)
{
  uint16_t x = v;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  uint16_t odd = x & 1u;
  if (odd) v |= 0x8000u; else v &= 0x7FFFu;
  return v;
}


// ===== small helpers =====
static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ===== ODrive CAN ID builder (11-bit) =====
static inline uint16_t odrive_build_id(uint8_t node_id, uint8_t cmd_id) {
    return ((uint16_t)node_id << 5) | (cmd_id & 0x1F);
}

static void odrive_clear_errors(FDCAN_HandleTypeDef *hcan, uint8_t node_id) {
    uint8_t data[8] = {0};
    uint16_t id = odrive_build_id(node_id, CMD_CLEAR_ERRORS);
    fdcanx_send_data(hcan, id, data, 1);
}

static void odrive_set_axis_state(FDCAN_HandleTypeDef *hcan, uint8_t node_id, uint32_t state) {
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(state & 0xFF);
    data[1] = (uint8_t)((state >> 8) & 0xFF);
    data[2] = (uint8_t)((state >> 16) & 0xFF);
    data[3] = (uint8_t)((state >> 24) & 0xFF);

    uint16_t id = odrive_build_id(node_id, CMD_SET_AXIS_STATE);
    fdcanx_send_data(hcan, id, data, 4);
}

static void odrive_set_controller_mode(FDCAN_HandleTypeDef *hcan,
                                       uint8_t node_id,
                                       uint32_t control_mode,
                                       uint32_t input_mode) {
    uint8_t data[8];

    data[0] = (uint8_t)(control_mode & 0xFF);
    data[1] = (uint8_t)((control_mode >> 8) & 0xFF);
    data[2] = (uint8_t)((control_mode >> 16) & 0xFF);
    data[3] = (uint8_t)((control_mode >> 24) & 0xFF);

    data[4] = (uint8_t)(input_mode & 0xFF);
    data[5] = (uint8_t)((input_mode >> 8) & 0xFF);
    data[6] = (uint8_t)((input_mode >> 16) & 0xFF);
    data[7] = (uint8_t)((input_mode >> 24) & 0xFF);

    uint16_t id = odrive_build_id(node_id, CMD_SET_CONTROLLER_MODE);
    fdcanx_send_data(hcan, id, data, 8);
}

static void odrive_set_input_torque(FDCAN_HandleTypeDef *hcan, uint8_t node_id, float torque) {
    uint8_t data[8] = {0};

    union { float f; uint8_t b[4]; } u;
    u.f = torque;

    data[0] = u.b[0];
    data[1] = u.b[1];
    data[2] = u.b[2];
    data[3] = u.b[3];

    uint16_t id = odrive_build_id(node_id, CMD_SET_INPUT_TORQUE);
    fdcanx_send_data(hcan, id, data, 4);
}

// ===== raw -> q(rad) mapping (临时) =====
static inline float raw_rel_to_q_rad(int32_t raw_rel) {
    float norm = (float)raw_rel / (float)RAW_HALF_RANGE; // roughly [-1..1]
    norm = clampf(norm, -1.2f, +1.2f);
    return norm * JOINT_Q_MAX_RAD;
}

// ===== zero: set current position as q=0 =====
static void joint1_zero_here(int32_t raw_now) {
    g_joint1.raw_zero = raw_now;
    g_joint1.zero_valid = 1;
    g_joint1.q = 0.0f;
    g_joint1.q_prev = 0.0f;
    g_joint1.qd = 0.0f;
    g_joint1.qd_f = 0.0f;

    // 默认：目标位置=当前（上电保持）
    g_joint1.q_des = 0.0f;
}

// ===== 1kHz control: read raw_abs externally then call this =====
static void joint1_control_1khz(int32_t raw_abs) {
    g_joint1.raw_abs = raw_abs;

    // 没有zero就不输出扭矩（安全）
    if (!g_joint1.zero_valid) {
        odrive_set_input_torque(&hfdcan1, JOINT_NODE_ID, 0.0f);
        return;
    }

    // 1) raw rel
    g_joint1.raw_rel = g_joint1.raw_abs - g_joint1.raw_zero;

    // 2) q / qd
    g_joint1.q = raw_rel_to_q_rad(g_joint1.raw_rel);

    const float dt = 0.001f;
    float qd = (g_joint1.q - g_joint1.q_prev) / dt;
    g_joint1.q_prev = g_joint1.q;

    const float alpha = 0.1f;
    g_joint1.qd_f = g_joint1.qd_f + alpha * (qd - g_joint1.qd_f);
    g_joint1.qd = g_joint1.qd_f;

    // 3) torque: P/PD + ff
    float tau = g_joint1.kp * (g_joint1.q_des - g_joint1.q)
              + g_joint1.kd * (g_joint1.qd_des - g_joint1.qd)
              + g_joint1.tau_ff;

    // 4) soft limits (基于绝对 raw 更直观)
    //    如果方向反了：把 + / - 交换即可
    if (g_joint1.raw_abs < (RAW_MIN + MARGIN_RAW)) {
        float x = (float)((RAW_MIN + MARGIN_RAW) - g_joint1.raw_abs);
        tau += K_SOFT * x;
    } else if (g_joint1.raw_abs > (RAW_MAX - MARGIN_RAW)) {
        float x = (float)(g_joint1.raw_abs - (RAW_MAX - MARGIN_RAW));
        tau -= K_SOFT * x;
    }

    // 5) clamp
    tau = clampf(tau, -g_joint1.tau_max, +g_joint1.tau_max);
    g_joint1.tau = tau;

    // 6) send
    odrive_set_input_torque(&hfdcan1, JOINT_NODE_ID, -tau);
}
// Pipelined read: send READ cmd (ignore first response), then send NOP to receive the requested register
static uint16_t as5047p_read_reg14(uint8_t enc_idx, uint16_t addr, uint8_t *err)
{
  if (err) *err = 0;

  uint16_t cmd = AS5047P_CMD_READ | (addr & 0x3FFFu);
  cmd = as5047p_apply_parity(cmd);

  uint16_t rx1 = 0, rx2 = 0;
  const EncoderCS *cs = &enc_cs[enc_idx];

  CS_Low(cs);
  (void)spi1_txrx_u16(cmd, &rx1);
  CS_High(cs);

  __NOP(); __NOP(); __NOP();

  CS_Low(cs);
  (void)spi1_txrx_u16(0x0000u, &rx2);
  CS_High(cs);

  if ((rx2 & AS5047P_ERR_MASK) != 0u) {
    if (err) *err = 1;
  }

  g_last_rx2 = rx2;
  return (rx2 & AS5047P_DATA_MASK);
}
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

// 3) 让 ODrive 进入 closed loop + torque passthrough
odrive_clear_errors(&hfdcan1, JOINT_NODE_ID);
HAL_Delay(20);

odrive_set_axis_state(&hfdcan1, JOINT_NODE_ID, AXIS_STATE_CLOSED_LOOP);
HAL_Delay(100);

odrive_set_controller_mode(&hfdcan1, JOINT_NODE_ID, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
HAL_Delay(100);

// ===== DEBUG: 固定扭矩测试（确认 ODrive 已闭环）=====
for (int i = 0; i < 200; i++) {
    odrive_set_input_torque(&hfdcan1, JOINT_NODE_ID, 0.05f);
    HAL_Delay(5);
}
odrive_set_input_torque(&hfdcan1, JOINT_NODE_ID, 0.0f);
HAL_Delay(50);

// 4) 读一次 second encoder，当做 zero（你之后可以换成“按键/命令触发”）
uint8_t e = 0;
int32_t raw0 = (int32_t)as5047p_read_reg14(0, AS5047P_REG_ANGLECOM, &e);
if (e == 0) {
    joint1_zero_here(raw0);
} else {
    const char *msg = "[WARN] encoder zero failed\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 50);
}

// 5) start 1kHz tick
HAL_TIM_Base_Start_IT(&htim2);

const char *boot = "\r\n[H725] Joint1 POC: SPI1 AS5047P + CAN1 MiniODrive torque @1kHz\r\n";
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

    // 1) read encoder raw
    enc_raw[0] = as5047p_read_reg14(0, AS5047P_REG_ANGLECOM, &enc_err[0]);

    // 2) control (send torque)
    joint1_control_1khz((int32_t)enc_raw[0]);

    // 3) print @10Hz
    if (++print_ctr >= PRINT_DIV) {
        print_ctr = 0;

        int n = snprintf(uart_buf, sizeof(uart_buf),
            "[t=%lu] raw=%u zero=%ld q=%ldmrad qd=%ldmrad/s tau=%ldmNm err=%u\r\n",
            (unsigned long)g_tick_1khz,
            (unsigned)enc_raw[0],
            (long)g_joint1.raw_zero,
            (long)(g_joint1.q * 1000.0f),
            (long)(g_joint1.qd * 1000.0f),
            (long)(g_joint1.tau * 1000.0f),
            (unsigned)enc_err[0]);

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
