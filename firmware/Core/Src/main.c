/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 레이저 변위센서를 이용한 곡률 측정 시스템 (STM32F411CEU6)
  ******************************************************************************
  * - PC GUI에서 'M' 명령 수신 시 서보모터를 -30도 ~ +30도로 회전하며 연속 스캔
  * - 30ms(약 33Hz) 주기로 레이저 변위센서(BD-100) 거리 샘플링 후 UART 전송
  * - 스캔 종료 후 3점(-30/0/+30도) 기반 평탄도(Bow), 곡률(Curvature) 연산
  * - 대기 상태에서는 100ms 주기로 실시간 거리값(RTD) 전송
  *
  * UART 프로토콜 (9600 8N1, ASCII, \r\n 종단)
  *   PC  -> MCU : 'M'                        스캔 시작
  *   MCU -> PC  : RTD,<거리 um>              대기 중 실시간 거리
  *   MCU -> PC  : SCAN,<서보 펄스 us>,<거리 um>  스캔 데이터
  *   MCU -> PC  : RES,<P1 um>,<P2 um>,<P3 um>,<Bow um>,<곡률 x1e6 (1/mm)>
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 센서 영점 조정값 (평평한 기준 시편으로 측정한 Bow 오차) */
#define BOW_OFFSET      (8.00241089f)

/* 서보모터 펄스 (단위: us, TIM1 1MHz / 50Hz) */
#define PULSE_CENTER    1475            /* 0도   */
#define PULSE_MIN       1142            /* -30도 */
#define PULSE_MAX       1808            /* +30도 */
#define PULSE_STEP      3               /* 스캔 1스텝 (333us = 30도 -> 111스텝) */

/* 타이밍 (단위: ms) */
#define HOME_WAIT_MS    2000            /* 원점(-30도) 이동 후 안정화 대기 */
#define SCAN_PERIOD_MS  30              /* 스캔 샘플링 주기 (약 33Hz)     */
#define RTD_PERIOD_MS   100             /* 대기 중 실시간 거리 전송 주기   */

/* ADC 오버샘플링 횟수 (분해능 13.43um -> 4.75um) */
#define OVERSAMPLE_COUNT 8

/* 스캔 상태 */
#define SCAN_IDLE       0               /* 대기: 실시간 거리 전송 */
#define SCAN_HOMING     1               /* 원점(-30도) 복귀       */
#define SCAN_RUNNING    2               /* -30도 -> +30도 연속 스캔 */
#define SCAN_CALC       3               /* 평탄도 / 곡률 연산     */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* 스캔 상태 머신 (UART 수신 인터럽트에서 변경되므로 volatile) */
volatile uint8_t  g_scan_state = SCAN_IDLE;
volatile uint32_t g_scan_timer = 0;
uint32_t g_rtd_timer = 0;
uint32_t g_current_pulse = PULSE_CENTER;

/* UART 1바이트 수신 버퍼 */
uint8_t g_uart_rx_byte = 0;

/* 3개 지점 거리값 [0]: 좌(-30도), [1]: 중앙(0도), [2]: 우(+30도) */
volatile float distance_storage[3] = {0.0f, 0.0f, 0.0f};

/* 연산 결과 */
volatile float distance_mm = 0.0f;
volatile float bow_value_mm = 0.0f;
volatile float curvature_val = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
float measure_distance(void);
static void servo_set_pulse(uint32_t pulse);
static void store_reference_point(uint32_t pulse, float dist);
static void calculate_bow_curvature(void);
static void send_line(const char *buf, int len);
static void send_rtd(float dist);
static void send_scan(uint32_t pulse, float dist);
static void send_result(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 서보모터 위치 설정 */
static void servo_set_pulse(uint32_t pulse)
{
  g_current_pulse = pulse;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_current_pulse);
}

/* 스캔 중 -30 / 0 / +30도 지점의 거리값을 연산용으로 저장 */
static void store_reference_point(uint32_t pulse, float dist)
{
  if (pulse == PULSE_MIN)         distance_storage[0] = dist;
  else if (pulse == PULSE_CENTER) distance_storage[1] = dist;
  else if (pulse == PULSE_MAX)    distance_storage[2] = dist;
}

/* 평탄도(Bow) 및 곡률(Curvature) 연산
 *  - Bow  = (y1' + y3') / 2 - y2'          (좌/우 거리를 수직 깊이로 변환)
 *  - k    = 1 / R = 4A / (a * b * c)       (세 점 외접원)
 */
static void calculate_bow_curvature(void)
{
  const float cos30 = cosf(30.0f * M_PI / 180.0f);
  const float sin30 = sinf(30.0f * M_PI / 180.0f);

  /* 평탄도(Bow) */
  float y_left     = distance_storage[0] * cos30;
  float y_right    = distance_storage[2] * cos30;
  float y_side_avg = (y_left + y_right) / 2.0f;
  bow_value_mm = y_side_avg - distance_storage[1] - BOW_OFFSET;

  /* (각도, 거리) -> (x, y) 좌표 변환 */
  float x1 = -distance_storage[0] * sin30;
  float y1 =  distance_storage[0] * cos30;
  float x2 =  0.0f;
  float y2 =  distance_storage[1];
  float x3 =  distance_storage[2] * sin30;
  float y3 =  distance_storage[2] * cos30;

  /* 세 변의 길이와 삼각형 면적 */
  float a = sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
  float b = sqrtf(powf(x2 - x3, 2) + powf(y2 - y3, 2));
  float c = sqrtf(powf(x3 - x1, 2) + powf(y3 - y1, 2));
  float area = 0.5f * fabsf(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

  /* 곡률 k = 4A / abc (직선이면 0) */
  if (a * b * c != 0.0f) curvature_val = (4.0f * area) / (a * b * c);
  else                   curvature_val = 0.0f;

  /* 볼록/오목 방향을 부호로 표시 */
  if (bow_value_mm < 0.0f) curvature_val = -curvature_val;
}

/* UART 전송 (printf float 미사용: 정수 um 단위로 변환해서 전송) */
static void send_line(const char *buf, int len)
{
  if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
}

static void send_rtd(float dist)
{
  char rtd_buffer[30];
  int32_t rtd_int = (int32_t)(dist * 1000.0f);
  int len = snprintf(rtd_buffer, sizeof(rtd_buffer), "RTD,%ld\r\n", rtd_int);
  send_line(rtd_buffer, len);
}

static void send_scan(uint32_t pulse, float dist)
{
  char scan_buffer[50];
  int32_t dist_int = (int32_t)(dist * 1000.0f);
  int len = snprintf(scan_buffer, sizeof(scan_buffer), "SCAN,%lu,%ld\r\n", pulse, dist_int);
  send_line(scan_buffer, len);
}

static void send_result(void)
{
  char res_buffer[80];
  int len = snprintf(res_buffer, sizeof(res_buffer), "RES,%ld,%ld,%ld,%ld,%ld\r\n",
                     (int32_t)(distance_storage[0] * 1000.0f),
                     (int32_t)(distance_storage[1] * 1000.0f),
                     (int32_t)(distance_storage[2] * 1000.0f),
                     (int32_t)(bow_value_mm * 1000.0f),
                     (int32_t)(curvature_val * 1000000.0f));
  send_line(res_buffer, len);
}

/* UART 수신 완료 콜백: 'M' 수신 시 스캔 시작 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (g_uart_rx_byte == 'M' && g_scan_state == SCAN_IDLE)
    {
      g_scan_state = SCAN_HOMING;
      g_scan_timer = 0;
    }
    HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);
  }
}

/* UART 오류(오버런 등) 발생 시 수신 재시작 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 서보 PWM 시작 후 중앙(0도) 위치에서 대기 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  servo_set_pulse(PULSE_CENTER);
  HAL_Delay(500);

  /* PC 명령 수신 대기 (1바이트 인터럽트 수신) */
  HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);

  g_rtd_timer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_tick = HAL_GetTick();

    if (g_scan_state == SCAN_IDLE)
    {
      /* 대기 상태: 100ms 주기로 실시간 거리값 전송 */
      if (current_tick - g_rtd_timer >= RTD_PERIOD_MS)
      {
        g_rtd_timer = current_tick;
        distance_mm = measure_distance();
        send_rtd(distance_mm);
      }
    }
    else if (g_scan_state == SCAN_HOMING)
    {
      /* 1. 초기화: 원점(-30도)으로 이동 후 안정화 대기 */
      if (g_scan_timer == 0)
      {
        servo_set_pulse(PULSE_MIN);
        g_scan_timer = current_tick;
      }
      if (current_tick - g_scan_timer > HOME_WAIT_MS)
      {
        g_scan_state = SCAN_RUNNING;
        g_scan_timer = current_tick;
      }
    }
    else if (g_scan_state == SCAN_RUNNING)
    {
      /* 2. 연속 스캔: 30ms마다 측정 -> 전송 -> 다음 각도로 이동 */
      if (current_tick - g_scan_timer >= SCAN_PERIOD_MS)
      {
        g_scan_timer = current_tick;
        float scan_dist = measure_distance();

        send_scan(g_current_pulse, scan_dist);
        store_reference_point(g_current_pulse, scan_dist);

        if (g_current_pulse >= PULSE_MAX)
        {
          g_scan_state = SCAN_CALC;
        }
        else
        {
          servo_set_pulse(g_current_pulse + PULSE_STEP);
        }
      }
    }
    else if (g_scan_state == SCAN_CALC)
    {
      /* 3. 데이터 연산 -> 결과 전송 -> 중앙 복귀 */
      calculate_bow_curvature();
      send_result();

      servo_set_pulse(PULSE_CENTER);
      g_rtd_timer = current_tick;
      g_scan_state = SCAN_IDLE;
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 레이저 변위센서 거리 측정 (8회 오버샘플링 평균)
 *  - BD-100 출력 1~5V 를 저항 분압(2.2k / 3.3k)으로 ADC 입력 범위에 맞춤
 *  - 거리(mm) = -10 * 센서전압 + 127.1  (측정거리 70 ~ 130mm)
 */
float measure_distance(void)
{
  uint32_t adc_sum = 0;

  for (int i = 0; i < OVERSAMPLE_COUNT; i++)
  {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      adc_sum += HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
  }

  float avg_adc_value = (float)adc_sum / (float)OVERSAMPLE_COUNT;
  float measured_voltage = avg_adc_value * (3.0f / 4095.0f);
  float sensor_voltage = measured_voltage * (2.2f + 3.3f) / 3.3f;
  return (-10.0f * sensor_voltage) + 127.1f;
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
