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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef *cs_port;
  uint16_t      cs_pin;
} EncoderCS;

typedef struct {
    // encoder
    int32_t raw;        // absolute raw
    int32_t raw_zero;   // zero offset (raw at q=0)
    int32_t raw_rel;    // raw - raw_zero

    float q;            // rad
    float q_prev;
    float qd;           // rad/s
    float qd_f;

    // command
    float q_des;        // rad
    float qd_des;       // rad/s
    float kp;
    float kd;
    float tau_ff;
    float tau;
    float tau_max;

    uint8_t zero_valid;
} Joint1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== Encoder count (future 6) =====
#define ENCODER_COUNT  1

// ===== Print rate control =====
#define PRINT_DIV      10    // 1kHz / 10 = 100Hz print

// ===== AS5047P protocol =====
#define AS5047P_CMD_READ       0x4000u
#define AS5047P_REG_ANGLECOM   0x3FFFu
#define AS5047P_DATA_MASK      0x3FFFu
#define AS5047P_ERR_MASK       0x4000u

// ===== Joint raw boundary (given) =====
#define RAW_MIN  5500
#define RAW_MAX  10500

// ===== Map raw_rel -> rad (POC) =====
// 你现在还没给真实机械角度，所以先用 ±1 rad 的“软定义”跑通闭环
#define JOINT_Q_MAX_RAD 1.0f

// ===== Soft limit =====
#define MARGIN_RAW 200
#define K_SOFT     0.002f     // Nm/raw 先保守
#define TAU_MAX_DEFAULT 0.30f // Nm 先保守，确认方向后再加

// ===== ODrive CAN =====
#define JOINT_NODE_ID  6      // 你给的 node_id
#define ODRIVE_CMD_SET_AXIS_STATE       0x07
#define ODRIVE_CMD_SET_CONTROLLER_MODE  0x0B
#define ODRIVE_CMD_SET_INPUT_TORQUE     0x0E
#define ODRIVE_CMD_CLEAR_ERRORS         0x18

// Axis / controller enums (ODrive)
#define AXIS_STATE_CLOSED_LOOP_CONTROL  8
#define CONTROL_MODE_TORQUE             1
#define INPUT_MODE_PASSTHROUGH          1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UNUSED(x) ((void)(x))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// ---- 1kHz scheduler tick ----
volatile uint32_t g_tick_1khz = 0;
volatile uint8_t  g_tick_flag = 0;

// ---- Debug UART buffer ----
static char uart_buf[256];
static uint16_t g_last_rx2 = 0;

// ---- Encoder CS table ----
static EncoderCS enc_cs[ENCODER_COUNT] = {
  {GPIOE, GPIO_PIN_15}, // PE15 as CSn (your setup)
};

// ---- Latest encoder data ----
static uint16_t enc_raw[ENCODER_COUNT];
static uint8_t  enc_err[ENCODER_COUNT];

// ---- Joint state ----
static Joint1 g_joint1 = {
    .q_des   = 0.0f,
    .qd_des  = 0.0f,
    .kp      = 1.0f,     // POC: start small
    .kd      = 0.0f,     // later add damping
    .tau_ff  = 0.0f,
    .tau_max = TAU_MAX_DEFAULT,
    .zero_valid = 0,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void joint1_zero_calibrate(void);
static void joint1_control_1khz(void);

static void odrive_clear_errors(FDCAN_HandleTypeDef* hcan, uint8_t node_id);
static void odrive_set_axis_state(FDCAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t state);
static void odrive_set_controller_mode(FDCAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t control_mode, uint32_t input_mode);
static void odrive_set_input_torque(FDCAN_HandleTypeDef* hcan, uint8_t node_id, float tau_nm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// ---------------- GPIO CS helpers ----------------
static inline void CS_Low(const EncoderCS *cs)  { HAL_GPIO_WritePin(cs->cs_port, cs->cs_pin, GPIO_PIN_RESET); }
static inline void CS_High(const EncoderCS *cs) { HAL_GPIO_WritePin(cs->cs_port, cs->cs_pin, GPIO_PIN_SET); }

// ---------------- small math helpers ----------------
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ---------------- AS5047P SPI helpers ----------------
// Ensure 16-bit word has EVEN parity (AS5047P uses parity bit at bit15)
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

static HAL_StatusTypeDef spi1_txrx_u16(uint16_t tx, uint16_t *rx)
{
  uint8_t txb[2] = {(uint8_t)(tx >> 8), (uint8_t)(tx & 0xFF)};
  uint8_t rxb[2] = {0, 0};

  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, txb, rxb, 2, 2);
  if (st == HAL_OK && rx) {
    *rx = ((uint16_t)rxb[0] << 8) | (uint16_t)rxb[1];
  }
  return st;
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

// ---------------- raw->rad mapping (with zero offset) ----------------
static inline float raw_rel_to_q_rad(int32_t raw_rel)
{
    // 把 (raw_rel) 映射到 [-1..1] 再乘 JOINT_Q_MAX_RAD
    // 这里用你给的绝对边界来估计半行程
    const float half_range = (float)((RAW_MAX - RAW_MIN) / 2); // 2500
    float norm = (float)raw_rel / half_range;                  // roughly [-1..1]
    norm = clampf(norm, -1.2f, +1.2f);                         // 防止偶发超界
    return norm * JOINT_Q_MAX_RAD;
}

// ---------------- CAN helpers ----------------
static inline uint16_t odrive_can_id(uint8_t node_id, uint16_t cmd_id)
{
    return (uint16_t)(((uint16_t)node_id << 5) | (cmd_id & 0x1F));
}

static HAL_StatusTypeDef can_send_4b(FDCAN_HandleTypeDef* hcan, uint16_t std_id, const uint8_t d[4])
{
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = std_id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_4;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx, (uint8_t*)d);
}

static void can_send_u32(FDCAN_HandleTypeDef* hcan, uint16_t std_id, uint32_t u32)
{
    uint8_t d[4];
    d[0] = (uint8_t)(u32 >> 0);
    d[1] = (uint8_t)(u32 >> 8);
    d[2] = (uint8_t)(u32 >> 16);
    d[3] = (uint8_t)(u32 >> 24);
    (void)can_send_4b(hcan, std_id, d);
}

static void can_send_f32(FDCAN_HandleTypeDef* hcan, uint16_t std_id, float v)
{
    union { float f; uint32_t u; } x;
    x.f = v;
    can_send_u32(hcan, std_id, x.u);
}

// ---------------- ODrive commands ----------------
static void odrive_clear_errors(FDCAN_HandleTypeDef* hcan, uint8_t node_id)
{
    uint16_t id = odrive_can_id(node_id, ODRIVE_CMD_CLEAR_ERRORS);
    can_send_u32(hcan, id, 0);
}

static void odrive_set_axis_state(FDCAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t state)
{
    uint16_t id = odrive_can_id(node_id, ODRIVE_CMD_SET_AXIS_STATE);
    can_send_u32(hcan, id, state);
}

static void odrive_set_controller_mode(FDCAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t control_mode, uint32_t input_mode)
{
    // payload: control_mode (u32) + input_mode(u32) -> 8 bytes
    // 为了简化：分两帧不行（ODrive期望8字节）
    // 所以这里用 DLC=8 发送
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = odrive_can_id(node_id, ODRIVE_CMD_SET_CONTROLLER_MODE);
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;

    uint8_t d[8];
    d[0] = (uint8_t)(control_mode >> 0);
    d[1] = (uint8_t)(control_mode >> 8);
    d[2] = (uint8_t)(control_mode >> 16);
    d[3] = (uint8_t)(control_mode >> 24);
    d[4] = (uint8_t)(input_mode >> 0);
    d[5] = (uint8_t)(input_mode >> 8);
    d[6] = (uint8_t)(input_mode >> 16);
    d[7] = (uint8_t)(input_mode >> 24);

    (void)HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx, d);
}

static void odrive_set_input_torque(FDCAN_HandleTypeDef* hcan, uint8_t node_id, float tau_nm)
{
    uint16_t id = odrive_can_id(node_id, ODRIVE_CMD_SET_INPUT_TORQUE);
    can_send_f32(hcan, id, tau_nm);
}

// ---------------- joint functions ----------------
static void joint1_zero_calibrate(void)
{
    // 上电后采样多次取平均当 zero（避免启动瞬间噪声）
    const int N = 100;
    int64_t sum = 0;
    int ok = 0;

    for (int i = 0; i < N; i++) {
        uint8_t e = 0;
        uint16_t r = as5047p_read_reg14(0, AS5047P_REG_ANGLECOM, &e);
        if (e == 0) { sum += (int32_t)r; ok++; }
        HAL_Delay(2);
    }

    if (ok > (N/2)) {
        g_joint1.raw_zero = (int32_t)(sum / ok);
        g_joint1.zero_valid = 1;
        g_joint1.q = 0.0f;
        g_joint1.q_prev = 0.0f;
        g_joint1.qd = 0.0f;
        g_joint1.qd_f = 0.0f;
        g_joint1.q_des = 0.0f;      // 保持零位（即保持上电位置）
    }
}

static void joint1_control_1khz(void)
{
    // 1) read encoder
    g_joint1.raw = (int32_t)enc_raw[0];
    if (!g_joint1.zero_valid) {
        // 没标零就先不输出力矩
      //  odrive_set_input_torque(&hfdcan1, JOINT_NODE_ID, 0.0f);
        return;
    }

    // 2) raw_rel
    g_joint1.raw_rel = g_joint1.raw - g_joint1.raw_zero;

    // 3) q / qd
    g_joint1.q = raw_rel_to_q_rad(g_joint1.raw_rel);

    const float dt = 0.001f;
    float qd = (g_joint1.q - g_joint1.q_prev) / dt;
    g_joint1.q_prev = g_joint1.q;

    const float alpha = 0.1f;
    g_joint1.qd_f = g_joint1.qd_f + alpha * (qd - g_joint1.qd_f);
    g_joint1.q = g_joint1.q;      // keep
    g_joint1.qd = g_joint1.qd_f;

    // 4) MIT-like torque (P/PD + ff)
    float tau = g_joint1.kp * (g_joint1.q_des - g_joint1.q)
              + g_joint1.kd * (g_joint1.qd_des - g_joint1.qd)
              + g_joint1.tau_ff;

    // 5) soft limit based on absolute raw (更可靠)
    if (g_joint1.raw < (RAW_MIN + MARGIN_RAW)) {
        float x = (float)((RAW_MIN + MARGIN_RAW) - g_joint1.raw);
        tau += K_SOFT * x;
    } else if (g_joint1.raw > (RAW_MAX - MARGIN_RAW)) {
        float x = (float)(g_joint1.raw - (RAW_MAX - MARGIN_RAW));
        tau -= K_SOFT * x;
    }

    // 6) clamp
    tau = clampf(tau, -g_joint1.tau_max, +g_joint1.tau_max);
    g_joint1.tau = tau;

    // 7) send torque
 //   odrive_set_input_torque(&hfdcan1, JOINT_NODE_ID, tau);
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


  // ensure CS high
  for (int i = 0; i < ENCODER_COUNT; i++) CS_High(&enc_cs[i]);

  // power enable (your board)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(50);

  // CAN init/start (assumed inside bsp_can_init)

  bsp_can_init();
  HAL_Delay(50);

  // IMU init (optional)
 // (void)BMI088_init();

	// ODrive: clear errors + closed loop + torque passthrough
	odrive_clear_errors(&hfdcan1, JOINT_NODE_ID);
	HAL_Delay(10);

	odrive_set_axis_state(&hfdcan1, JOINT_NODE_ID, AXIS_STATE_CLOSED_LOOP_CONTROL);
	HAL_Delay(50);

	odrive_set_controller_mode(&hfdcan1, JOINT_NODE_ID, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
	HAL_Delay(20);

	// zero calibration (set current as 0)
	joint1_zero_calibrate();

	// start 1kHz tick
	HAL_TIM_Base_Start_IT(&htim2);

	const char *boot = "\r\n[H725] Joint POC: SPI1 AS5047P + CAN1 ODrive torque loop @1kHz\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)boot, (uint16_t)strlen(boot), 50);
	static volatile uint8_t g_uart_busy = 0;

	static void uart1_print(const char* s, uint16_t len)
	{
	    if (g_uart_busy) return;
	    g_uart_busy = 1;
	    HAL_UART_Transmit(&huart1, (uint8_t*)s, len, 200);
	    g_uart_busy = 0;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last_tick = 0;
  uint32_t print_div_ctr = 0;

  while (1)
  {
	  if (!g_tick_flag) continue;
	  g_tick_flag = 0;

	  if (g_tick_1khz == last_tick) continue;
	  last_tick = g_tick_1khz;

	   // 1) read encoder at 1kHz
	   enc_raw[0] = as5047p_read_reg14(0, AS5047P_REG_ANGLECOM, &enc_err[0]);

	   // 2) run joint control (send torque)
	   joint1_control_1khz();

	   // 3) debug print @100Hz
	   if (++print_div_ctr >= PRINT_DIV) {
		   print_div_ctr = 0;

		   int32_t q_mrad  = (int32_t)(g_joint1.q  * 1000.0f);
		   int32_t qd_mrad = (int32_t)(g_joint1.qd * 1000.0f);
		   int32_t tau_mNm = (int32_t)(g_joint1.tau * 1000.0f);

		   int n = 0;
		   n += snprintf(uart_buf+n, sizeof(uart_buf)-n,
				   "q=%ldmrad qd=%ldmrad/s tau=%ldmNm\r\n", q_mrad, qd_mrad, tau_mNm);



		   HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n, 10);
	   }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

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
