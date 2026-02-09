/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - ULTRA STABLE (PI + UART 0..900)
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

/* ==== PI Controller (instead of PID) ==== */
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
#define DUTY_FILTER_ALPHA 0.1f
#define MAX_DUTY_CHANGE 5.0f

/* ===== LUX RANGE ===== */
#define LUX_MAX 900

/* ===== PI Gains ===== */
#define PI_KP   0.7f
#define PI_KI   0.6f

/* Control loop Ts (assuming ~9kHz) */
#define CONTROL_TS  (1.0f / 9000.0f)
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
float duty_filtered = 0.0f;
float luminance = 0.0f;
float luminance_ref = 0.0f;

uint32_t cnt = 0;
const uint32_t cnt_dec = 10;
SWV_TypeDef swv;

uint32_t sensor_read_counter = 0;
const uint32_t sensor_read_period = 100;

#ifdef CNT_MAX
uint8_t txBuffer[2][CNT_MAX * sizeof(swv)];
_Bool txActiveBuffer = 0;
_Bool txFlag = 0;
#endif

PI_Controller_t hpi1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void process_uart_command(uint8_t c);

static void PI_Init(PI_Controller_t *pi, float kp, float ki, float Ts, float out_min, float out_max);
static float PI_Update(PI_Controller_t *pi, float ref, float meas);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void PI_Init(PI_Controller_t *pi, float kp, float ki, float Ts, float out_min, float out_max)
{
  pi->kp = kp;
  pi->ki = ki;
  pi->Ts = Ts;
  pi->integrator = 0.0f;
  pi->out_min = out_min;
  pi->out_max = out_max;
}

static float PI_Update(PI_Controller_t *pi, float ref, float meas)
{
  float error = ref - meas;

  pi->integrator += (pi->ki * pi->Ts * error);

  float out = (pi->kp * error) + pi->integrator;

  if(out > pi->out_max) {
    out = pi->out_max;
    if(pi->integrator > pi->out_max) pi->integrator = pi->out_max;
  }
  else if(out < pi->out_min) {
    out = pi->out_min;
    if(pi->integrator < pi->out_min) pi->integrator = pi->out_min;
  }

  return out;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart3)
  {
    process_uart_command(rx_buffer[0]);
    HAL_UART_Receive_IT(&huart3, rx_buffer, 1);
  }
}

void process_uart_command(uint8_t c)
{
  if(waiting_for_value)
  {
    if(c >= '0' && c <= '9')
    {
      if(cmd_idx < 3)   // still 3 digits
      {
        cmd_buffer[cmd_idx++] = c;
        HAL_UART_Transmit(&huart3, &c, 1, 100);

        if(cmd_idx == 3)
        {
          cmd_buffer[3] = '\0';
          int value = 0;
          sscanf((char*)cmd_buffer, "%d", &value);

          if(value >= 0 && value <= LUX_MAX)
          {
            luminance_ref = (float)value;

            uint8_t ack[32];
            int len = sprintf((char*)ack, " -> Lux set to %d\r\n", value);
            HAL_UART_Transmit(&huart3, ack, len, 100);
          }
          else
          {
            uint8_t error[64];
            int len = sprintf((char*)error, " -> ERROR: Range 000-%03d\r\n", LUX_MAX);
            HAL_UART_Transmit(&huart3, error, len, 100);
          }

          waiting_for_value = 0;
          cmd_idx = 0;
        }
      }
    }
    else if(c == 27 || c == '\b' || c == 127)
    {
      waiting_for_value = 0;
      cmd_idx = 0;
      uint8_t cancel[] = "\r\nCancelled\r\n";
      HAL_UART_Transmit(&huart3, cancel, strlen((char*)cancel), 100);
    }
  }
  else if(c == 'R' || c == 'r')
  {
    waiting_for_value = 1;
    cmd_idx = 0;
    uint8_t prompt[64];
    int len = sprintf((char*)prompt, "Enter lux (000-%03d): ", LUX_MAX);
    HAL_UART_Transmit(&huart3, prompt, len, 100);
  }
  else if(c == '?' || c == 's' || c == 'S')
  {
    uint8_t tx_buffer[128];
    int resp_len = sprintf((char*)tx_buffer, "L:%.1f Lref:%.1f D:%.1f%% Err:%.1f\r\n",
                          luminance, luminance_ref, duty_filtered, luminance_ref - luminance);
    HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 100);
  }
  else if(c == '\r' || c == '\n')
  {
    // Ignore
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2)
  {
    HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_SET);
    HAL_ADC_Start_IT(&hadc1);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc == &hadc1)
  {
    sensor_read_counter++;
    if(sensor_read_counter >= sensor_read_period)
    {
      sensor_read_counter = 0;
      luminance = BH1750_ReadIlluminance_lux(&hbh1750);
    }

#ifdef CLOSED_LOOP
    if(luminance_ref > 0.0f)
    {
      float duty_pi = PI_Update(&hpi1, luminance_ref, luminance);

      if(duty_pi < 0.0f) duty_pi = 0.0f;
      else if(duty_pi > 100.0f) duty_pi = 100.0f;

      float duty_change = duty_pi - duty_filtered;
      if(duty_change > MAX_DUTY_CHANGE) duty_change = MAX_DUTY_CHANGE;
      else if(duty_change < -MAX_DUTY_CHANGE) duty_change = -MAX_DUTY_CHANGE;

      duty_filtered = duty_filtered + DUTY_FILTER_ALPHA * duty_change;

      if(duty_filtered < 0.0f) duty_filtered = 0.0f;
      else if(duty_filtered > 100.0f) duty_filtered = 100.0f;

      duty = duty_filtered;
      LED_PWM_WriteDuty(&hld1, duty);
    }
    else
    {
      duty_filtered -= DUTY_FILTER_ALPHA * duty_filtered;
      if(duty_filtered < 0.5f) duty_filtered = 0.0f;

      duty = duty_filtered;
      LED_PWM_WriteDuty(&hld1, duty);
    }
#endif

    HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_RESET);

    if(cnt % cnt_dec == 0)
    {
      swv.measurement = luminance;
      swv.reference = luminance_ref;
      swv.control = duty;

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

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  BH1750_Init(&hbh1750);
  LED_PWM_Init(&hld1);

  PI_Init(&hpi1, PI_KP, PI_KI, CONTROL_TS, 0.0f, 100.0f);

  HAL_UART_Receive_IT(&huart3, rx_buffer, 1);

  luminance = BH1750_ReadIlluminance_lux(&hbh1750);

  HAL_TIM_Base_Start_IT(&htim2);


  uint8_t msg[] =
      "\r\n*** LED BRIGHTNESS CONTROL ***\r\n"
      "Press R to set lux (000-900)\r\n"
      "Press S for status\r\n";
  HAL_UART_Transmit(&huart3, msg, strlen((char*)msg), 1000);

  /* USER CODE END 2 */

  while (1)
  {
    HAL_Delay(100);
  }
}

/* Clock + Error_Handler  */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
