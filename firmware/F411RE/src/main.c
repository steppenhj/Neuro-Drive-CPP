/* USER CODE BEGIN Header */
/**
  * @brief  Neuro-Drive MCU 펌웨어: UART 명령 수신 -> 모터/서보 제어 + 안전 감시
  * @note   태스크 구성 (우선순위: 안전 > 제어 > 통신)
  *         - UART ISR  : 1바이트 수신, 줄 조립, 큐 투입 [이벤트]
  *         - Task_Motor(H) : 큐 소비 -> FF+FB 제어 -> PWM [100Hz]
  *         - Task_Comm(N)  : ENC 텔레메트리, PONG (huart2 TX 전용) [100Hz]
  *         - Task_Safety(RT) : 500ms 명령 타임아웃 감시, 최악 지연 ~550ms [20Hz]
  * @note   클럭: HSI 16MHz / 8 * 100 /2 = 100MHz (FLASH 3WS)
  *         APB1=50MHz지만 타이머 클럭은 x2 규칙으로 100MHz
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h" //FreeRTOS를 쓰겠다는 선언임

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
	int speed;   //속도 (PWM)
	int angle;   //각도 (Servo)
} MotorCommand_t;

//PID관련 구조체
typedef struct {
	float kp; //비례 게인
	float ki; //적분 게인
	float kd; //미분 게인
	float integral; //누적 오차 (I항용)
	float prev_error; //이전 오차 (D항용)
	float output_min;  //출력 하한
	float output_max; //출력 상한
} PID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_PWM_MAX             999
#define SERVO_MIN_US              650
#define SERVO_MAX_US              2350
#define SERVO_CENTER_US           1500
#define TELEMETRY_PERIOD_MS       50

// 4/23 PID 제어 다시 시도
#define PWM_TO_ENC_SCALE 0.1f
// PWM999->엔코더 100이면 0.1, 엔코더 50이면 0.05

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for Task_Comm */
//RTOS 핸들 선언
osThreadId_t Task_CommHandle; // 통신 담당
const osThreadAttr_t Task_Comm_attributes = {
  .name = "Task_Comm",
  .stack_size = 128 * 4, //스택 크기
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Motor */
osThreadId_t Task_MotorHandle; // 모터 담당
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
// mutex 없이 volatile로 충분한 근거:
// (1) 각 변수의 쓰는 쪽이 하나뿐 (last_command_time=ISR, current_speed_rpm=Task_Motor)
// (2) Cortex-M4에서 정렬된 32비트 이하 접근은 원자적 -> 찢어진 값 불가
// volatile의 역할은 "컴파일러가 레지스터에 캐시해두고 안 읽는 것"을 막는 것
volatile int speed_cmd = 0;
volatile int angle_us = SERVO_CENTER_US;
volatile uint32_t last_command_time = 0;

// UART 수신 버퍼
uint8_t rx_data;
// uint8_t buffer[64];
// uint8_t buf_index = 0;

// PING 수신 플래그 (ISR이 세팅, Task_Comm이 소비)
volatile uint8_t ping_received = 0;

// 큐 핸들 선언
osMessageQueueId_t myQueueHandle;

//엔코더 설정 전역변수
volatile int16_t current_speed_rpm = 0; //계산된 속도(부호 있는 것)
uint16_t last_encoder_count = 0; //이전 카운터 값 저장용
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
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
//main()함수는 초기화만 담당하고, osKernelStart()를 만나는 순간 RTOS로 넘겨주고 끝남
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();  //1. 하드웨어 초기화

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); // 2. 심장 박동 설정느낌 system clock

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); //핀 설정
  MX_USART2_UART_Init(); //통신 설정
  MX_TIM2_Init(); //타이머 설정
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // [긴급 추가] 타이머 3번 전원 강제 공급
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
  // 큐 만들기: 크기 16개, MotorCommand_t 구조체만큼
  myQueueHandle = osMessageQueueNew(16, sizeof(MotorCommand_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_Comm */
  // 스레드 생성
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
  //이제부터 main함수는 멈추고, 태스크들이 움직임 -> RTOS 시작
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  // 엔코더 모드 TI12 = A/B 양 채널 엣지 모두 카운트(4체배).
  // 90도 위상차로 방향까지 하드웨어가 판별 -> CPU 개입 없음
  // 16비트 프리러닝 카운터: 넘침은 Task_Motor에서 int16_t 캐스팅으로 처리
  /* USER CODE END TIM1_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  // DC모터 PWM: 100MHz/1400/1000 = 71.4Hz, 듀티 0~999
  // Period 999가 상위 계층 계약의 근원 (C++의 throttle*999)
  // 71Hz는 낮은 편 -> 소음/전류리플 관점에선 kHz대로 올리는 게 개선 방향
  /* USER CODE END TIM2_Init 1 */

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1399;
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
  // 서보 PWM: 100MHz/100 = 1MHz -> 1틱 = 1us, 20000틱 = 20ms(50Hz)
  // 프리스케일러를 이렇게 고르면 compare 값 = 펄스폭(us) 그대로
  // (SERVO_MIN 650 = 650us) -> 단위변환 코드가 사라짐
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
  // 115200 8N1 : 1프레임 10비트 -> 1바이트 = 약 87us
  // 이 간격이 "1바이트 인터럽트 방식"이 감당 가능한 근거
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

// [필수 추가] printf를 UART로 쏘기 위한 함수
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

// PID함수
// 7/23 
// PID (현재 kp=3.0, ki=kd=0 -> 실질적으론 P 제어임 => 솔직하게 이 부분은 너무 어렵다)
// dt=0.01f 하드코딩: Task_Motor가 osDelayUntil로 100Hz 고정이라 가능
// 한계: anti-windup이 출력 클램프 방식뿐 (ki 사용 시 적분항 별도 클램프 필요)
float PID_Calculate(PID_t *pid, float error){
	// P항: 지금 오차에 비례
	float p = pid->kp * error;

	// I항: 오차를 누적 (dt = 0.01s 고정, 100Hz 루프)
	pid->integral += error * 0.01f;
	float i = pid->ki * pid->integral;

	// D항: 오차의 변화율
	float d = pid->kd * (error - pid->prev_error) / 0.01f;
	pid->prev_error = error;

	float output = p + i + d;

	// 출력 범위 제한 (Anti-windup)
	if(output > pid->output_max) output = pid->output_max;
	if(output < pid->output_min) output = pid->output_min;

	return output;
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
	//하드웨어가 데이터 받으면 이 함수로 오는 거임
	if(huart->Instance == USART2)
	{
		//디버깅. 인터럽트가 지금 잘 안되는 중.
		// 일단 데이터 오는지 확인해야 하니깐 데이터 오면 무조건 깜빡이기
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    // 7/23 위 내용은 다 해결되었지만 디버깅 흔적 남겨두기
    // static인 이유: ISR 호출 사이에 "조립 중인 줄"의 상태를 보존해야 함
    // (1바이트마다 호출되므로 지역변수면 매번 초기화되어 줄 조립 불가)
		static uint8_t buffer[64]; //static으로 선언했다-> 함수가 끝나도 값이 유지돼야 함
		static uint8_t buf_index = 0;

		// 문장 끝 확인
		if(rx_data == '\n' || rx_data == '\r')
		{
			if(buf_index > 0)
			{
				buffer[buf_index] = 0;

				//************ Phase 6 들어가기 전 *(******
				//**************** Ping Pong 추가해서 시간 측정*********
        //***********4/24중요
        // PONG이 현재 ISR에 있어, UART 자체에 두가지 문제가 발생함
        // 1) ISR에서 blocking UART 100ms 대기
        // 2) Race Condition
        // 해결하기 위해 PONG을 ISR에서 Task로 옮김
        //  */
				// if(strcmp((char*)buffer, "PING")==0)
				// {
				// 	HAL_UART_Transmit(&huart2, (uint8_t*)"PONG\r\n", 6, 100);
				// 	buf_index = 0;
				// 	memset(buffer, 0, sizeof(buffer));
				// 	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
				// 	return; //모터 명령 파싱으로 안 넘어감
				// }
        if(strcmp((char*)buffer, "PING")==0)
        {
          ping_received = 1;
          buf_index = 0;
          memset(buffer, 0, sizeof(buffer));
          HAL_UART_Receive_IT(&huart2, &rx_data, 1);
          return;
        }

				int temp_speed = 0;
				int temp_angle = 1500;

        // 7/23 아래 내용 개선 여지 작성
        // 개선 여지: sscanf는 무거워서 ISR 최소화 원칙에 어긋남
        // (PONG을 태스크로 옮긴 것과 같은 원칙 -> 줄 단위로 큐에 넘기고 파싱은 태스크에서)
				//파싱 성공하면 큐에 넣음
				if(sscanf((char*)buffer, "%d,%d", &temp_speed, &temp_angle) == 2)
				{
					MotorCommand_t send_msg;
					send_msg.speed = temp_speed;
					send_msg.angle = temp_angle;

					//큐에 넣기 (비동기 전송)
					// 0은 대기시간 0, 즉 큐가 꽉 찼으면 미련 없이 버린다는 뜻
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
    // 7/23 추가설명 아래에 붙임
    // 재장전 필수: 빼먹으면 수신이 영구 정지
    // 노이즈 에러로 여기 못 와도 ErrorCallback이 재장전해주는 이중 보험
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
//젤 먼저 실행
//데이터가 오면 나한테 알려줘라고 하드웨어에 명령하고 즉시 끝나는 거임.
// 실제로 데이터를 받을 때까지 기다리진 않는다. (Non-blocking)
{
  /* USER CODE BEGIN 5 */
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  //2번 uart 장치 사용
  //rx_data변수에 데이터 담기
  // 1바이트만 와도 알리기.
  // IT의 의미는 interrupt
  //"300,1500\n"이 왔다고 가정하면
  // '3'이 제일 먼저 도착하고 rx_data에 3 저장
  // 1개 왔으니깐 interrupt를 검.
  // 하던 일 멈추고 콜백 함수 HAL_UART_RxCpltCallback으로 점프

  /* Infinite loop */
  for(;;)
  {
    // 7/23
    // huart2 TX는 이 태스크(Task_Comm)가 단독 소유 (ENC + PONG 모두 여기서만 송신)
    // -> ISR에서 PONG으로 옮겨온 이유이자, TX 경쟁이 구조적으로 없는 근거

    //4/24 PING 요청이 들어와 있으면 PONG 응답
    // ISR 대신 Task에서 송신하여 huart2 Race Condition 방지
    if(ping_received)
    {
      ping_received=0; //플래그 해제 (다음 PING 수신 대비임)
      HAL_UART_Transmit(&huart2, (uint8_t*)"PONG\r\n", 6, 10);
    }

    //엔코더 값 송신
    char enc_buf[32];
    int len = snprintf(enc_buf, sizeof(enc_buf), "ENC:%d\r\n", current_speed_rpm);
    HAL_UART_Transmit(&huart2, (uint8_t*)enc_buf, len, 10);

    osDelay(10);
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
    // {
    //     // 데이터 없으면 대기 (RTOS 스케줄링 양보)
    // 	// 이제 폴링 방식에서 인터럽트로 바꿀거라서 주석때림

    // 	//****엔코더 때문에 바뀜
    // 	char enc_buf[32];
    // 	int len = snprintf(enc_buf, sizeof(enc_buf), "ENC:%d\r\n", current_speed_rpm);
    // 	HAL_UART_Transmit(&huart2, (uint8_t*)enc_buf, len, 100);
    //     osDelay(50); // 할 거 없으니깐 대기 (자는거임) -> 엔코더 받는 역할해보자
    // }
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
//모터 태스크
{
  /* USER CODE BEGIN StartTask02 */
  // 받을 빈 상자 준비
  MotorCommand_t rcv_msg;
  int current_angle = 1500;
  int target_pwm = 0; //Python이 보낸 원본 PWM (Feedforward용)
  float target_enc = 0.0f;  // 엔코더 단위로 변환된 목표
  float error = 0.0f;
  int final_pwm = 0;

  PID_t pid_speed = {
		  .kp = 3.0f,  // 4/23 단위 바뀌니깐 게인 재조정 필요함. + 7/22 그리고 사실 PID는 제대로 못 했다고 볼 수 있다. 어렵다.
		  .ki = 0.0f,
		  .kd = 0.0f,
		  .integral = 0.0f,
		  .prev_error = 0.0f,
		  .output_min = -500.0f, // PID는 "보정량"만 담당하므로 999->500 축소
		  .output_max = 500.0f
  };



  // 성능 측정용 변수
  uint32_t last_wake_time = osKernelGetTickCount();
  uint32_t current_time = 0;
  uint32_t time_diff = 0;

  // 통계용
  uint32_t max_jitter = 0;
  uint32_t min_jitter = 9999;
  uint32_t loop_count = 0;

  // 1. 엔코더 하드웨어 시작
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  // 2. 초기값 설정
  last_encoder_count = __HAL_TIM_GET_COUNTER(&htim1);

  /* Infinite loop */
  for(;;)
  {
      // (1) 주기적 실행 보장을 위한 시간 측정
      current_time = osKernelGetTickCount();
      time_diff = current_time - last_wake_time;

      // (2) 통계 집계
      if(loop_count > 10){
          if(time_diff > max_jitter) max_jitter = time_diff;
          if(time_diff < min_jitter) min_jitter = time_diff;
      }
      loop_count++;

      // ---------------------------------------------------------
      // ★ 엔코더 속도 계산 로직
      // ---------------------------------------------------------
      uint16_t current_count = __HAL_TIM_GET_COUNTER(&htim1);
      int16_t diff = (int16_t)(current_count - last_encoder_count);
      // 카운터가 65535 -> 0으로 넘어가도 unsigned 뺄셈 후 int16_t 캐스팅이 자동으로
      // 올바른 부호 있는 변화량을 만들어냄
      //
      // TIM1 엔코더 카운터는 하드웨어적으로 16비트(0~65535 순환)라서,
      // 뺄셈 결과를 정확히 16비트로 해석해야 랩어라운드가 자동 처리됨
      // 예를 들면, 바퀴 +6틱, 카운터 한 바퀴 돈 경우:
      // last = 65533, current = 3
      // 3 - 65533 = -65530 -> 16비트로 자르면 6 -> int16_t로 읽으면 +6
      // 만약 여기에서 int로 계산하면 -65530이라는 엉뚱한 값이 나옴
      // 그런데 이 차는 10ms에 수십 틱 수준이라 여유가 수백배임.
      // 정리하면, int는 플랫폼 의존 크기, int16_t는 표준이 보장하는 정확히 16비트

      current_speed_rpm = diff;
      last_encoder_count = current_count;


      // ---------------------------------------------------------

      //*************엔코더 값 보내기
      // Task_Motor 루프 안, 엔코더 계산 바로 아래에 추가
      static uint8_t enc_tick = 0;
      if(++enc_tick >= 5) { // 100Hz 루프에서 5번마다 = 20Hz 전송
          current_speed_rpm = diff;  //전역변수에 저장. 프린트 안하기
          enc_tick = 0;
      }

      // (3) 데이터 수신 및 처리 (순서 수정됨)
      // 먼저 큐를 확인
      // 7/23 아래 
      // 타임아웃 0 = 논블로킹: 새 명령 없으면 "직전 목표를 유지"한 채 제어 계속
      // -> 통신이 끊겨도 모터는 마지막 명령으로 계속 달린다는 뜻
      // -> 이게 Task_Safety(워치독)가 반드시 필요한 이유
      osStatus_t status = osMessageQueueGet(myQueueHandle, &rcv_msg, NULL, 0);

      // 데이터가 "있으면" (osOK)
      if(status == osOK){
          // 1. 변수 갱신
//          current_speed = rcv_msg.speed;
//          current_angle = rcv_msg.angle;
    	  //위 방식이 Open-Loop임 (명령을 그대로 PWM에 떄려 박음)

    	  //목표: Closed-Loop (엔코더 피드백으로 보정)
    	  target_pwm = rcv_msg.speed; // 목표값
    	  current_angle = rcv_msg.angle;
//
//    	  error = target_speed - current_speed_rpm; //오차
//    	  current_speed = (int)PID_Calculate(&pid_speed, error);
//    	  pwm_output = current_speed;
//    	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_output);
//
//          // 2.  로그 출력 (명령 받았을 때만 찍힘)
//          printf("CMD_RECV: Speed=%d, Angle=%d\r\n", current_speed, current_angle);
      }

      // 단위 통일: PWM 목표 -> 엔코더 목표로 변환
      target_enc = (float)target_pwm * PWM_TO_ENC_SCALE;

      // 이제 error가 "엔코더 단위"로서 의미가 있음
      error = target_enc - (float)current_speed_rpm;
      //PID는 "보정량(ΔPWM)"만 계산
      float pid_correction = PID_Calculate(&pid_speed, error);

      //** Feedforward(원본 PWM) + Feedback(PID보정) 결합 */
      final_pwm = target_pwm + (int)pid_correction;


      // ---------------------------------------------------------
      // 모터 구동 로직
      // ---------------------------------------------------------

      // DC 모터 제어
      if (final_pwm > MOTOR_PWM_MAX) final_pwm = MOTOR_PWM_MAX;
      if (final_pwm < -MOTOR_PWM_MAX) final_pwm = -MOTOR_PWM_MAX;

      //전진
      if (final_pwm > 0) {
    	  // IN1=HIGH, IN2=LOW (정방향 회전)
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
          //PWM값 설정 (속도 조절)
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)final_pwm);
      }
      //후진
      else if (final_pwm < 0) {
    	  //IN1=LOW, IN2=HIGH (역방향 회전)
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)abs(final_pwm));
      } else {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      }

      // 서보 모터 제어, 각도 제한도 해줌.
      if (current_angle < SERVO_MIN_US) current_angle = SERVO_MIN_US;
      if (current_angle > SERVO_MAX_US) current_angle = SERVO_MAX_US;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)current_angle);

      //*******
      //중요함. OTA 성공 검증용 LED 깜빡이기
      // *******
      static uint16_t led_counter = 0;
      if(++led_counter >= 100) {
    	  //100Hz 루프에서 100번 = 1초
    	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    	  led_counter = 0;
      }
      // 주기 제어 (100Hz)
      // 7/23 아래 주석 추가
      // osDelayUntil = 절대 시각 기준이라 작업 시간과 무관하게 드리프트 없음
      // (osDelay였으면 "작업시간+10ms"로 주기가 계속 밀림. C++의 sleep_until과 동일 사상)
      last_wake_time = current_time;
      osDelayUntil(last_wake_time + 10);
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
    // 7/23 아래 주석 추가
    // 워치독: 500ms 타임아웃 + 50ms 감시 주기 -> 최악 반응 지연 ~550ms
    // 이중 조치: (1) 큐로 정식 정지(서보 중립 포함) (2) 레지스터 직접 차단(즉시)
    // 우선순위 Realtime = 안전 감시는 어떤 작업에도 선점당하지 않는다
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
