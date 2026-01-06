/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h> // dla atof i abs
#include <string.h>
#include "led_config.h"
#include "bh1750_config.h"
#include "pid_config.h"
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
float target_lux = 0.0f;
float current_lux = 0.0f;
float current_pwm = 0.0f;

// Flaga wyzwalająca PID (volatile, bo zmieniana w przerwaniu!)
volatile uint8_t pid_trigger = 0;

// Bufor UART
char rx_buffer[32];
int rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_FlushStandard(void)
{
    uint8_t dummy;
    // Czytamy tak długo, aż funkcja zwróci, że nie ma więcej danych (TIMEOUT)
    // To opróżnia bufor bez używania skomplikowanych makr/rejestrów
    while(HAL_UART_Receive(&huart3, &dummy, 1, 0) == HAL_OK);
}

// Funkcja czytająca liczbę (z echem)
float UART_ReadFloat(void)
{
    char buffer[32];
    int idx = 0;
    uint8_t ch;
    memset(buffer, 0, sizeof(buffer));

    // Czyścimy bufor przed wpisywaniem
    UART_FlushStandard();

    while (1)
    {
        // Tutaj czekamy na znak (HAL_MAX_DELAY = czekaj w nieskończoność)
        if (HAL_UART_Receive(&huart3, &ch, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            // Enter (\r lub \n) kończy wpisywanie
            if (ch == '\r' || ch == '\n')
            {
                HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 10);
                break;
            }

            // Backspace (usuwanie)
            if (ch == 8 || ch == 127)
            {
                if (idx > 0) {
                    idx--;
                    buffer[idx] = 0;
                    HAL_UART_Transmit(&huart3, (uint8_t*)"\b \b", 3, 10);
                }
                continue;
            }

            // Tylko cyfry, kropka i minus
            if (idx < 30)
            {
                if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-')
                {
                    buffer[idx++] = ch;
                    HAL_UART_Transmit(&huart3, &ch, 1, 10);
                }
            }
        }
    }
    return (float)atof(buffer);
}/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  return (HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY) == HAL_OK) ? len : -1;
}

int _read(int file, char *ptr, int len)
{
  uint8_t ch = 0;
  // Odbierz 1 bajt z UART3
  if (HAL_UART_Receive(&huart3, &ch, 1, HAL_MAX_DELAY) == HAL_OK)
  {
    // Echo: odeślij znak z powrotem do konsoli, abyś widział co piszesz
    HAL_UART_Transmit(&huart3, &ch, 1, HAL_MAX_DELAY);

    // Zamiana Carriage Return (\r) na Line Feed (\n) dla potrzeb scanf
    if (ch == '\r')
    {
      ch = '\n';
      uint8_t lf = '\n';
      HAL_UART_Transmit(&huart3, &lf, 1, HAL_MAX_DELAY); // Wyślij też \n dla czytelności w konsoli
    }

    *ptr = ch;
    return 1; // Zwróć informację o przeczytaniu jednego znaku
  }
  return -1;
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
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  BH1750_Init(&hbh1750A);
    LED_PWM_Init(&hld1);
    LED_PWM_WriteDuty(&hld1, 0.0f);

    PID_Config_Init();

    // --- KONFIGURACJA TIMERA 7 NA 150 MS ---
    // Zakładając zegar 96 MHz (z Twojego pliku main.c):
    // 96 MHz / (9599 + 1) = 10 kHz (Taktowanie licznika co 0.1 ms)
    // 10 kHz / (1499 + 1) = 6.66 Hz -> Okres ok. 150 ms
    __HAL_TIM_SET_PRESCALER(&htim7, 9599);
    __HAL_TIM_SET_AUTORELOAD(&htim7, 1499);

    // Wyzerowanie licznika i start w trybie przerwań (IT)
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    HAL_TIM_Base_Start_IT(&htim7);

    printf("\r\n=== REGULATOR PID (Przerwania TIM7) ===\r\n");
    printf("Probkowanie: stale, sprzetowe 150ms.\r\n");
    printf("Komendy: s150 [Enter]\r\nCMD> ");

    // Reset bufora
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    uint8_t rx_byte = 0;

      while (1)
      {
          // --- 1. OBSŁUGA KOMEND (UART) ---
          // To dzieje się "w wolnym czasie", nie blokuje regulatora
          if (HAL_UART_Receive(&huart3, &rx_byte, 1, 0) == HAL_OK)
          {
              if (rx_byte == '\r' || rx_byte == '\n') // ENTER
              {
                  if (rx_index > 0)
                  {
                      rx_buffer[rx_index] = 0;
                      if (rx_buffer[0] == 's' || rx_buffer[0] == 'S')
                      {
                          float val = atof(&rx_buffer[1]);
                          if (val < 0) val = 0;
                          target_lux = val;
                          printf("\r\n[PID] Nowy Cel: %.1f lux\r\nCMD> ", target_lux);
                      }
                      rx_index = 0;
                  }
              }
              else if (rx_byte == 8 || rx_byte == 127) // Backspace
              {
                  if (rx_index > 0) {
                      rx_index--;
                      HAL_UART_Transmit(&huart3, (uint8_t*)"\b \b", 3, 10);
                  }
              }
              else
              {
                  if (rx_index < 30) {
                      rx_buffer[rx_index++] = rx_byte;
                      HAL_UART_Transmit(&huart3, &rx_byte, 1, 10);
                  }
              }
          }

          // --- 2. REGULACJA (Wyzwalana przez Timer) ---
          if (pid_trigger == 1)
          {
              // Czyścimy flagę - czekamy na kolejne przerwanie
              pid_trigger = 0;

              // A. Pomiar
              current_lux = BH1750_ReadIlluminance_lux(&hbh1750A);

              // B. Obliczenie PID
              float error = target_lux - current_lux;
              float correction = arm_pid_f32(&PID1, error);

              // C. Aktualizacja PWM (Metoda przyrostowa)
              current_pwm += correction;

              if (current_pwm > 100.0f) current_pwm = 100.0f;
              if (current_pwm < 0.0f) current_pwm = 0.0f;

              LED_PWM_WriteDuty(&hld1, current_pwm);

              // D. Diagnostyka (Dla spełnienia pkt 5 założeń)
              // Ref: Cel, Meas: Pomiar, PWM: Sterowanie
              printf("Ref:%.1f, Meas:%.1f, PWM:%.1f%%\r\n", target_lux, current_lux, current_pwm);
          }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        pid_trigger = 1; // Ustaw flagę "Czas na PID!"
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
