/* USER CODE BEGIN Header */
/**
  * @brief          : 인터럽트해보자. 
  * @note           : 
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> // abs() 함수용
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int speed;
	int angle;
} MotorCommand_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_PWM_MAX             999
#define SERVO_MIN_US              650
#define SERVO_MAX_US              2350
#define SERVO_CENTER_US           1500
#define TELEMETRY_PERIOD_MS       50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for Task_Comm */
osThreadId_t Task_CommHandle;
const osThreadAttr_t Task_Comm_attributes = {
  .name = "Task_Comm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Motor */
osThreadId_t Task_MotorHandle;
const osThreadAttr_t Task_Motor_attributes = {
  .name = "Task_Motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Safety */
osThreadId_t Task_SafetyHandle;
const osThreadAttr_t Task_Safety_attributes = {
  .name = "Task_Safety",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */
// 태스크 간 공유 변수 (volatile 필수)
volatile int speed_cmd = 0;
volatile int angle_us = SERVO_CENTER_US;
volatile uint32_t last_command_time = 0;

// UART 수신 버퍼
uint8_t rx_data;
uint8_t buffer[64];
uint8_t buf_index = 0;

// 큐 핸들 선언
osMessageQueueId_t myQueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // ★ [긴급 추가] 타이머 3번 전원 강제 공급
    __HAL_RCC_TIM3_CLK_ENABLE();

    // 1. 핀 설정 강제 적용
    Force_Hardware_Config();

    // 2. PWM 타이머 시작
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // DC Motor
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Servo

    // 큐 생성 (크기: 16개, 데이터 크기: 상자 크기만큼)
    // 큐 생성 옮겨야 함
    // 3. 변수 초기화
    last_command_time = HAL_GetTick();

    // 인터럽트 수신 시작( 한 글자 들어오면 인터럽트 발생해라는 느낌)
    // 이것도 옮겨야함

    /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  myQueueHandle = osMessageQueueNew(16, sizeof(MotorCommand_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_Comm */
  Task_CommHandle = osThreadNew(StartDefaultTask, NULL, &Task_Comm_attributes);

  /* creation of Task_Motor */
  Task_MotorHandle = osThreadNew(StartTask02, NULL, &Task_Motor_attributes);

  /* creation of Task_Safety */
  Task_SafetyHandle = osThreadNew(StartTask03, NULL, &Task_Safety_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ★ [필수 추가] printf를 UART로 쏘기 위한 함수
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
// 기존 코드에서 가져온 핀 강제 설정 함수
void Force_Hardware_Config(void)
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
}

// uart 인터럽트 콜백 함수 (데이터 오면 여기로 점프)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		//디버깅. 인터럽트가 지금 잘 안되는 중.
		// 일단 데이터 오는지 확인해야 하니깐 데이터 오면 무조건 깜빡이기
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		static uint8_t buffer[64]; //static으로 선언 ..
		static uint8_t buf_index = 0;

		if(rx_data == '\n' || rx_data == '\r')
		{
			if(buf_index > 0)
			{
				buffer[buf_index] = 0;

				int temp_speed = 0;
				int temp_angle = 1500;

				//ㅠㅏ싱 성공하면 큐에 넣음
				if(sscanf((char*)buffer, "%d,%d", &temp_speed, &temp_angle) == 2)
				{
					MotorCommand_t send_msg;
					send_msg.speed = temp_speed;
					send_msg.angle = temp_angle;

					//큐에 넣기
					osMessageQueuePut(myQueueHandle, &send_msg, 0, 0);

					last_command_time = HAL_GetTick();
					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				}
				buf_index = 0;
				memset(buffer, 0, sizeof(buffer));
			}
		}
		else
		{
			if(buf_index < 60){
				buffer[buf_index++] = rx_data;
			}
		}

		// 다음 인터럽트 장전
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}

//dp에러 발생 시(노이즈 같은 거) 자동 복구 함수
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		// 에러 발생하면 빨간불(LD2) 대신 끄거나 다른 표시 가능
		// 핵심 : 다시 수신 모드로 복귀시켜야 함
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the Task_Comm thread.
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  /* Infinite loop */
  for(;;)
  {
    // [Polling] 데이터 수신 확인
//    if (HAL_UART_Receive(&huart2, &rx_data, 1, 1) == HAL_OK)
//    {
//        // 1. 엔터키 감지 (\r 또는 \n 둘 다 처리)
//        if (rx_data == '\n' || rx_data == '\r')
//        {
//            if (buf_index > 0) // 버퍼에 데이터가 있을 때만 실행
//            {
//                buffer[buf_index] = 0; // 문자열 끝 맺음 (Null termination)
//
//                int temp_speed = 0;
//                int temp_angle = 1500;
//
//                // 2. 파싱 및 명령어 적용
//                if (sscanf((char*)buffer, "%d,%d", &temp_speed, &temp_angle) == 2)
//                {
////                    speed_cmd = temp_speed;
////                    angle_us = temp_angle;
//
//                	//큐에 넣기
//                	MotorCommand_t send_msg;
//                	send_msg.speed = temp_speed;
//                	send_msg.angle = temp_angle;
//
//                	// 큐에 발송
//                	osMessageQueuePut(myQueueHandle, &send_msg, 0, 0);
//
//                    last_command_time = HAL_GetTick(); // 안전장치 타이머 리셋
//
//                    // ★ 성공 확인용: 명령 알아들으면 LED 깜빡!
//                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//                }
//
//                // 버퍼 초기화
//                buf_index = 0;
//                memset(buffer, 0, sizeof(buffer));
//            }
//        }
//        else
//        {
//            // 3. 일반 문자 저장 (숫자, 쉼표 등)
//            if (buf_index < 60) {
//                buffer[buf_index++] = rx_data;
//            }
//        }
//    }
//    else
    {
        // 데이터 없으면 대기 (RTOS 스케줄링 양보)
    	// 이제 폴링 방식에서 인터럽트로 바꿀거라서 주석때림
        osDelay(1000);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task_Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	// 받을 빈 상자 준비
	MotorCommand_t rcv_msg;

  int current_speed = 0;
  int current_angle = 1500;

  //성능 측정용 변수
  uint32_t last_wake_time = osKernelGetTickCount();
  uint32_t current_time = 0;
  uint32_t time_diff = 0;
  //통계용
  uint32_t max_jitter = 0;
  uint32_t min_jitter = 9999;
  uint32_t loop_count = 0;

  /* Infinite loop */
  for(;;)
  {
	  //(1) 측정 시작
	  current_time = osKernelGetTickCount();
	  time_diff = current_time - last_wake_time;
	  last_wake_time = current_time;

	  //(2) 통계 집계 (처음 10번은 무시)
	  if(loop_count > 10){
		  if(time_diff > max_jitter) max_jitter = time_diff;
		  if(time_diff < min_jitter) min_jitter = time_diff;
	  }
	  loop_count++;

	  //(3) 데이터 출력 - 100번(약 2초)마다 리포트 출력
	  // "현재주기, 최대지연, 최소지연" csv 로 뽑음
	  if(loop_count % 100 == 0){
		  printf("PERF: %lu, MAX: %lu, MIN: %lu\r\n", time_diff, max_jitter, min_jitter);
		  //측정값 다시 초기화 (구간별 측정위해)
		  max_jitter = 0;
		  min_jitter = 9999;
	  }
    // 1. 전역 변수 읽기 (통신 태스크가 받아온 값)
//    current_speed = speed_cmd;
//    current_angle = angle_us;
	  // 큐에서 데이터 꺼내기
	  // 데이터가 없으면 여기서 멈춰서 (Block) 기다림 (osWaitForever)
	  // 데이터가 오면 꺠어나서 아래 코드를 실행
	  osStatus_t status = osMessageQueueGet(myQueueHandle, &rcv_msg, NULL, 0);

	  if(status == osOK){
		  current_speed = rcv_msg.speed;
		  current_angle = rcv_msg.angle;
	  }

    // 2. DC 모터 방향 및 속도 제어
    if (current_speed > MOTOR_PWM_MAX) current_speed = MOTOR_PWM_MAX;
    if (current_speed < -MOTOR_PWM_MAX) current_speed = -MOTOR_PWM_MAX;

    if (current_speed > 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // 전진
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)current_speed);
    } else if (current_speed < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // 후진
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)abs(current_speed));
    } else {
        // 정지
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }

    // 3. 서보 모터 제어
    if (current_angle < SERVO_MIN_US) current_angle = SERVO_MIN_US;
    if (current_angle > SERVO_MAX_US) current_angle = SERVO_MAX_US;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)current_angle);

    last_wake_time = current_time;
    osDelayUntil(last_wake_time + 20);

    // 100Hz 주기
//    osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task_Safety thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    // 500ms 동안 명령 없으면 정지
    if (HAL_GetTick() - last_command_time > 500)
    {
//        speed_cmd = 0; // 다음 루프에서 모터 태스크가 멈추게 함

    	// 큐에 "속도 0" 명령을 강제로 집어넣음
    	MotorCommand_t stop_msg;
    	stop_msg.speed = 0;
    	stop_msg.angle = 1500; // 조향 중립

    	//큐에 넣기 (우선순위 높여서 보내도 되지만, 일단 기본적으로)
    	// 큐가 꽉 차있어도(0) 강제로 넣지는 않음 (어차피 꽉 찼으면 명령이 온 거니까)
    	osMessageQueuePut(myQueueHandle, &stop_msg, 0, 0);

        // 즉시 하드웨어 차단 (이중 안전)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }

    osDelay(50);
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
