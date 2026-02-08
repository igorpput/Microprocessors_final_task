/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - ULTRA STABLE (PI + UART 0..2000)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "led_config.h"
#include "aio.h"
#include "bh1750_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float measurement;
  float reference;
  float control;
} SWV_TypeDef;

typedef struct {
  float kp;
  float ki;
  float Ts;
  float integrator;
  float out_min;
  float out_max;
} PI_Controller_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOSED_LOOP
#define CNT_MAX 900
#define DUTY_FILTER_ALPHA 0.1f    // Output filter: 0.1 = 10% new value, 90% old value
#define MAX_DUTY_CHANGE 5.0f      // Max change per control cycle (%)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buffer[1];
uint8_t cmd_buffer[32];
uint8_t cmd_idx = 0;
_Bool waiting_for_value = 0;

float duty = 0.0f;
float duty_filtered = 0.0f;  // Filtered/smoothed duty cycle
float luminance = 0.0f;
float luminance_ref = 0.0f;

uint32_t cnt = 0;
const uint32_t cnt_dec = 10;
SWV_TypeDef swv;

// Sensor read counter - reads sensor every 100 control cycles (~11ms at 9kHz)
uint32_t sensor_read_counter = 0;
const uint32_t sensor_read_period = 100;

// PI controller instance
PI_Controller_t pi;

#ifdef CNT_MAX
uint8_t txBuffer[2][CNT_MAX * sizeof(swv)];
_Bool txActiveBuffer = 0;
_Bool txFlag = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void process_uart_command(uint8_t c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Rx callback
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart3)
  {
    process_uart_command(rx_buffer[0]);
    HAL_UART_Receive_IT(&huart3, rx_buffer, 1);
  }
}

/**
  * @brief  Process UART command
  * Keys:
  *   L -> set lux (0..2000) then Enter
  *   I -> status
  *   ? -> help
  *   ESC -> cancel input
  */
void process_uart_command(uint8_t c)
{
  if(waiting_for_value)
  {
    // Digits
    if(c >= '0' && c <= '9')
    {
      if(cmd_idx < 4)   // allow up to 4 digits (0..2000)
      {
        cmd_buffer[cmd_idx++] = c;
        HAL_UART_Transmit(&huart3, &c, 1, 100);
      }
    }
    // Backspace/Delete
    else if(c == '\b' || c == 127)
    {
      if(cmd_idx > 0)
      {
        cmd_idx--;
        uint8_t bs_seq[] = "\b \b";
        HAL_UART_Transmit(&huart3, bs_seq, sizeof(bs_seq)-1, 100);
      }
    }
    // Enter -> parse value (supports 1..4 digits)
    else if(c == '\r' || c == '\n')
    {
      if(cmd_idx > 0)
      {
        cmd_buffer[cmd_idx] = '\0';
        int value = 0;
        sscanf((char*)cmd_buffer, "%d", &value);

        if(value >= 0 && value <= 2000)
        {
          luminance_ref = (float)value;

          uint8_t ack[48];
          int len = sprintf((char*)ack, "\r\n[OK] Ref Lux = %d\r\n", value);
          HAL_UART_Transmit(&huart3, ack, len, 100);
        }
        else
        {
          uint8_t error[] = "\r\n[ERR] Range 0-2000\r\n";
          HAL_UART_Transmit(&huart3, error, strlen((char*)error), 100);
        }
      }
      else
      {
        uint8_t msg[] = "\r\n[ERR] No value\r\n";
        HAL_UART_Transmit(&huart3, msg, strlen((char*)msg), 100);
      }

      waiting_for_value = 0;
      cmd_idx = 0;
    }
    // ESC -> cancel
    else if(c == 27)
    {
      waiting_for_value = 0;
      cmd_idx = 0;
      uint8_t cancel[] = "\r\nCancelled\r\n";
      HAL_UART_Transmit(&huart3, cancel, strlen((char*)cancel), 100);
    }
    else
    {
      // ignore
    }
  }
  else if(c == 'L' || c == 'l')
  {
    waiting_for_value = 1;
    cmd_idx = 0;
    uint8_t prompt[] = "\r\nEnter lux (0-2000) then press Enter: ";
    HAL_UART_Transmit(&huart3, prompt, strlen((char*)prompt), 100);
  }
  else if(c == 'I' || c == 'i')
  {
    uint8_t tx_buffer[128];
    int resp_len = sprintf((char*)tx_buffer,
                           "Lux=%.1f  Ref=%.1f  Duty=%.1f%%  Err=%.1f\r\n",
                           luminance, luminance_ref, duty_filtered, luminance_ref - luminance);
    HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 100);
  }
  else if(c == '?')
  {
    uint8_t help[] =
      "\r\nKeys:\r\n"
      "  L -> set lux (0-2000) + Enter\r\n"
      "  I -> status\r\n"
      "  ? -> help\r\n"
      "  ESC -> cancel input\r\n";
    HAL_UART_Transmit(&huart3, help, strlen((char*)help), 100);
  }
  else if(c == '\r' || c == '\n')
  {
    // Ignore
  }
}

/**
  * @brief  Period elapsed callback
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2)
  {
    HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_SET);
    HAL_ADC_Start_IT(&hadc1);
  }
}

/**
  * @brief  ADC conversion complete callback - CONTROL LOOP (PI)
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc == &hadc1)
  {
    // *** READ SENSOR PERIODICALLY ***
    sensor_read_counter++;
    if(sensor_read_counter >= sensor_read_period)
    {
      sensor_read_counter = 0;
      luminance = BH1750_ReadIlluminance_lux(&hbh1750);
    }

#ifdef CLOSED_LOOP
    if(luminance_ref > 0.0f)
    {
      // --- PI controller ---
      float error = luminance_ref - luminance;

      // Integrator
      pi.integrator += (pi.ki * pi.Ts * error);

      // Anti-windup clamp integrator
      if(pi.integrator > pi.out_max) pi.integrator = pi.out_max;
      if(pi.integrator < pi.out_min) pi.integrator = pi.out_min;

      // PI output
      float duty_pi = (pi.kp * error) + pi.integrator;

      // Clamp PI output
      if(duty_pi < 0.0f) duty_pi = 0.0f;
      else if(duty_pi > 100.0f) duty_pi = 100.0f;

      // *** RATE LIMITING: Limit how fast duty can change ***
      float duty_change = duty_pi - duty_filtered;
      if(duty_change > MAX_DUTY_CHANGE) duty_change = MAX_DUTY_CHANGE;
      else if(duty_change < -MAX_DUTY_CHANGE) duty_change = -MAX_DUTY_CHANGE;

      // *** LOW-PASS FILTER: Smooth the output ***
      duty_filtered = duty_filtered + DUTY_FILTER_ALPHA * duty_change;

      // Final clamp
      if(duty_filtered < 0.0f) duty_filtered = 0.0f;
      else if(duty_filtered > 100.0f) duty_filtered = 100.0f;

      duty = duty_filtered;
      LED_PWM_WriteDuty(&hld1, duty);
    }
    else
    {
      // No setpoint - smooth ramp down to 0
      duty_filtered -= DUTY_FILTER_ALPHA * duty_filtered;
      if(duty_filtered < 0.5f) duty_filtered = 0.0f;

      duty = duty_filtered;
      LED_PWM_WriteDuty(&hld1, duty);
    }
#endif

    HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_RESET);

    // Logging
    if(cnt % cnt_dec == 0)
    {
      swv.measurement = luminance;
      swv.reference   = luminance_ref;
      swv.control     = duty;

#ifdef CNT_MAX
      memcpy(&txBuffer[txActiveBuffer][(cnt / cnt_dec) * sizeof(swv)], &swv, sizeof(swv));
#endif
    }

#ifdef CNT_MAX
    cnt = (cnt < cnt_dec * CNT_MAX - 1) ? (cnt + 1) : 0;
#else
    cnt++;
#endif
  }
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  // Initialize components
  BH1750_Init(&hbh1750);
  LED_PWM_Init(&hld1);

  // PI init (your gains)
  pi.kp = 0.7f;
  pi.ki = 0.6f;
  pi.Ts = 0.001f;        // TEMP: 1ms. (If TIM2 is not 1kHz, we will adjust)
  pi.integrator = 0.0f;
  pi.out_min = 0.0f;
  pi.out_max = 100.0f;

  // Start UART
  HAL_UART_Receive_IT(&huart3, rx_buffer, 1);

  // Initial sensor reading
  luminance = BH1750_ReadIlluminance_lux(&hbh1750);

  // Start control loop
  HAL_TIM_Base_Start_IT(&htim2);

  // Send startup message
  uint8_t msg[] =
    "\r\n*** LED BRIGHTNESS CONTROL - STABLE (PI) ***\r\n"
    "Setpoint range: 0..2000 lux\r\n"
    "Keys:\r\n"
    "  L -> set lux (0-2000) + Enter\r\n"
    "  I -> status\r\n"
    "  ? -> help\r\n";
  HAL_UART_Transmit(&huart3, msg, strlen((char*)msg), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(100);
  }
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

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
