/* USER CODE BEGIN Header */
/**
  * @brief          : Neuro-Driver Firmware (Simple Polling Version)
  * @note           : DMA/Interrupt 제거됨. 가장 기본 모드.
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =========================
 * Tunables
 * ========================= */
#define MOTOR_PWM_MAX            999
#define SERVO_MIN_US             1100
#define SERVO_MAX_US             1900
#define SERVO_CENTER_US          1500
#define TELEMETRY_PERIOD_MS      50

/* 핸들러 선언 */
TIM_HandleTypeDef htim1; // 엔코더
TIM_HandleTypeDef htim2; // DC 모터
TIM_HandleTypeDef htim3; // 서보
UART_HandleTypeDef huart2;

/* 함수 원형 */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void Force_Hardware_Config(void);

int main(void)
{
  /* 1. 초기화 */
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* 2. 핀 강제 설정 (PA0, PA1, PA4 등) */
  Force_Hardware_Config();

  /* 3. 장치 시작 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);       // DC Motor
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);       // Servo
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // Encoder
  __HAL_TIM_SET_COUNTER(&htim1, 0);

  /* 4. 변수 초기화 */
  uint8_t rx_data;       // 1바이트 수신용
  uint8_t buffer[64];    // 문자열 조립용 버퍼
  uint8_t buf_index = 0; // 버퍼 인덱스

  uint32_t last_telemetry_time = 0;
  uint32_t last_command_time = HAL_GetTick();
  uint16_t prev_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

  /* 5. 메인 루프 */
    while (1)
    {
        /* ====================================================
         * [1] UART 수신 (Polling 방식 - 개선됨)
         * ==================================================== */
        if (HAL_UART_Receive(&huart2, &rx_data, 1, 1) == HAL_OK)
        {
            /* 1. 캐리지 리턴(\r)은 무시 (중요!) */
            if (rx_data == '\r') {
                continue;
            }

            /* 2. 개행 문자(\n)를 만나면 패킷 처리 시작 */
            if (rx_data == '\n')
            {
                buffer[buf_index] = 0; // 문자열 끝 맺음 (Null Terminate)

                int speed_cmd = 0;
                int angle_us  = SERVO_CENTER_US;

                /* 3. 파싱 시도 */
                // 예: "100,1500"
                if (sscanf((char*)buffer, "%d,%d", &speed_cmd, &angle_us) == 2)
                {
                    last_command_time = HAL_GetTick();

                    // [디버그] 명령 수신 확인용 LED (LD2) 토글
                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

                    /* --- DC 모터 제어 --- */
                    // 제한폭 설정 (Safety)
                    if (speed_cmd > MOTOR_PWM_MAX) speed_cmd = MOTOR_PWM_MAX;
                    if (speed_cmd < -MOTOR_PWM_MAX) speed_cmd = -MOTOR_PWM_MAX;

                    if (speed_cmd > 0) {
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // 전진
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)speed_cmd);
                    } else if (speed_cmd < 0) {
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // 후진
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)abs(speed_cmd));
                    } else {
                        // 정지 시 브레이크 (둘 다 HIGH or 둘 다 LOW) -> 여기선 Free Wheel(LOW/LOW)
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                    }

                    /* --- 서보 모터 제어 --- */
                    if (angle_us < SERVO_MIN_US) angle_us = SERVO_MIN_US;
                    if (angle_us > SERVO_MAX_US) angle_us = SERVO_MAX_US;
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)angle_us);

                    // [디버그] 수신된 명령을 다시 PC로 보내서 확인 (Echo)
                    // char debug_msg[32];
                    // snprintf(debug_msg, sizeof(debug_msg), "ACK:%d,%d\n", speed_cmd, angle_us);
                    // HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 10);
                }
                else
                {
                    // 파싱 실패 시 디버깅 메시지 전송 (옵션)
                    // char err_msg[] = "ERR:PARSE_FAIL\n";
                    // HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), 10);
                }

                // 버퍼 및 인덱스 초기화
                buf_index = 0;
                memset(buffer, 0, sizeof(buffer));
            }
            else
            {
                /* 일반 문자 저장 */
                if (buf_index < 60) {
                    buffer[buf_index++] = rx_data;
                } else {
                    // 버퍼가 꽉 차면 강제로 초기화 (오버플로우 방지)
                    buf_index = 0;
                }
            }
        }

        /* =========================
         * [2] 안전장치 (Failsafe)
         * ========================= */
        if (HAL_GetTick() - last_command_time > 500)
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            // LED 끄기 (연결 끊김 표시)
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }

        /* =========================
         * [3] Telemetry 전송
         * ========================= */
        if (HAL_GetTick() - last_telemetry_time >= TELEMETRY_PERIOD_MS)
        {
            // 엔코더 값 읽기 (Typecasting 주의)
            // int16_t로 캐스팅해야 65535 -> 0 넘어갈 때 음수로 계산됨
            uint16_t current_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
            int16_t delta = (int16_t)(current_cnt - prev_cnt);

            prev_cnt = current_cnt;

            // PPS (Pulse Per Second) 계산
            // 50ms마다 측정하므로 x20 (1000/50)
            int32_t speed_pps = (int32_t)delta * (1000 / TELEMETRY_PERIOD_MS);

            char tx_buffer[32];
            // ENC:속도\n 형식으로 전송
            snprintf(tx_buffer, sizeof(tx_buffer), "ENC:%ld\n", (long)speed_pps);
            HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, (uint16_t)strlen(tx_buffer), 10);

            last_telemetry_time = HAL_GetTick();
        }
    }
}
/* --- 초기화 및 설정 함수들 --- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_TIM1_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim1, &sConfig);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
}

static void MX_TIM2_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
}

static void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CENTER_US;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
}

/* ★ 핀 강제 설정 유지 ★ */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  /* PA2, PA3 강제 활성화 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void Force_Hardware_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* TIM2_CH1 -> PA0 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* TIM3_CH3 -> PB0 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* TIM1 Encoder -> PA8/PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
