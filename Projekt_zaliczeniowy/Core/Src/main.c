/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Scalona wersja - Regulator Pozycyjny + Zakłócenie ADC
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "led_config.h"
#include "bh1750_config.h"
#include "pid_config.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// --- Zmienne regulacji (LED1 - TIM9 CH2) ---
float target_lux = 0.0f;
float current_lux = 0.0f;
float pid_pwm_out = 0.0f;     // Bezpośrednie wyjście z regulatora PID (pozycyjne)

// --- Zmienne zakłócenia (LED2 - TIM9 CH1) ---
extern ADC_HandleTypeDef hadc1;
extern LED_PWM_Handle_TypeDef hld2;
uint32_t adc_raw_val = 0;
float disturbance_pwm = 0.0f;

// --- Flagi i bufory ---
volatile uint8_t pid_trigger = 0;
char rx_buffer[32];
int rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t Compute_CRC8(char* data, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  return (HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY) == HAL_OK) ? len : -1;
}

int _read(int file, char *ptr, int len)
{
  uint8_t ch = 0;
  if (HAL_UART_Receive(&huart3, &ch, 1, HAL_MAX_DELAY) == HAL_OK)
  {
    // Echo wyłączone dla czystości komunikacji z Matlabem
    if (ch == '\r') ch = '\n';
    *ptr = ch;
    return 1;
  }
  return -1;
}
/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration --------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  BH1750_Init(&hbh1750A);

  LED_PWM_Init(&hld1);
  LED_PWM_WriteDuty(&hld1, 0.0f);

  LED_PWM_Init(&hld2);
  LED_PWM_WriteDuty(&hld2, 0.0f);

  PID_Config_Init();

  // Konfiguracja timera na 150ms
  __HAL_TIM_SET_PRESCALER(&htim7, 9599);
  __HAL_TIM_SET_AUTORELOAD(&htim7, 1499);
  __HAL_TIM_SET_COUNTER(&htim7, 0);
  HAL_TIM_Base_Start_IT(&htim7);

  memset(rx_buffer, 0, sizeof(rx_buffer));
  rx_index = 0;
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  uint8_t rx_byte = 0;

  while (1)
  {
      // ---OBSŁUGA KOMEND UART ---
      if (HAL_UART_Receive(&huart3, &rx_byte, 1, 0) == HAL_OK)
      {
          if (rx_byte == '\r' || rx_byte == '\n')
          {
              if (rx_index > 0)
              {
                  rx_buffer[rx_index] = 0;
                  float val = 0.0f;
                  int valid = 0;

                  if (rx_buffer[0] == 's' || rx_buffer[0] == 'S') {
                      val = (float)atof(&rx_buffer[1]);
                      valid = 1;
                  } else if (rx_buffer[0] >= '0' && rx_buffer[0] <= '9') {
                      val = (float)atof(rx_buffer);
                      valid = 1;
                  }

                  if (valid) {
                      if (val < 0) val = 0;
                      target_lux = val;
                  }
                  rx_index = 0;
              }
          }
          else if (rx_byte == 8 || rx_byte == 127)
          {
              if (rx_index > 0) rx_index--;
          }
          else if (rx_index < 30)
          {
              rx_buffer[rx_index++] = rx_byte;
          }
      }

      // OBSŁUGA ZAKŁÓCENIA (Płynna reakcja na ADC) ---
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
      {
          adc_raw_val = HAL_ADC_GetValue(&hadc1);
          disturbance_pwm = ((float)adc_raw_val / 4095.0f) * 100.0f;
          if(disturbance_pwm > 100.0f) disturbance_pwm = 100.0f;
          LED_PWM_WriteDuty(&hld2, disturbance_pwm);
      }
      HAL_ADC_Stop(&hadc1);

      // --- PĘTLA REGULACJI PID (Wyzwalana co 150ms) ---
      if (pid_trigger == 1)
      {
          pid_trigger = 0;

          // Pomiar
          current_lux = BH1750_ReadIlluminance_lux(&hbh1750A);

          // Obliczenie PID (Metoda Pozycyjna)
          float error = target_lux - current_lux;
          pid_pwm_out = arm_pid_f32(&PID1, error);

          // Saturacja (Ograniczenie wyjścia 0-100%)
          if (pid_pwm_out > 100.0f) pid_pwm_out = 100.0f;
          if (pid_pwm_out < 0.0f) pid_pwm_out = 0.0f;

          // Wysterowanie LED 1
          LED_PWM_WriteDuty(&hld1, pid_pwm_out);

          // Diagnostyka MATLAB (Format: Ref, Meas, PWM)
          char msg_buffer[128];
          int msg_len = sprintf(msg_buffer, "Ref:%.2f, Meas:%.2f, PWM:%.2f",
                                target_lux, current_lux, pid_pwm_out);

          uint8_t crc_val = Compute_CRC8(msg_buffer, msg_len);
          printf("%s*%02X\r\n", msg_buffer, crc_val);
      }
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        pid_trigger = 1;
    }
}

uint8_t Compute_CRC8(char* data, int len) {
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
