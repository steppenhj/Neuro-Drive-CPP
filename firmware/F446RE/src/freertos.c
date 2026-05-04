/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
//큐 생성
	myQueueHandle = osMessageQueueNew(16, sizeof(MotorCommand_t), NULL);
  /* USER CODE END Init */

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

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the Task_Comm thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  char enc_buf[32];
  /* Infinite loop */

  for(;;)
  {
    //PING응답 (ISR대신 Task 에서 송신 -> Race Condition 방지)
	 if(ping_received)
	 {
		 ping_received = 0;
		 HAL_UART_Transmit(&huart2, (uint8_t*)"PONG\r\n", 6, 10);
	 }

	 //엔코더 값 송신
	 int len = snprintf(enc_buf, sizeof(enc_buf), "ENC:%d\r\n", current_speed_rpm);
	 HAL_UART_Transmit(&huart2, (uint8_t*)enc_buf, len, 10);

	 osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
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
	MotorCommand_t rcv_msg;
	int current_angle = SERVO_CENTER_US;
	int target_pwm = 0;
	float target_enc = 0.0f;
	float error = 0.0f;
	int final_pwm = 0;

	PID_t pid_speed = {
			.kp = 3.0f,
			.ki = 0.0f,
			.kd = 0.0f,
			.integral = 0.0f,
			.prev_error = 0.0f,
			.output_min = -500.0f,
			.output_max = 500.0f
	};

	//엔코더 카운터는 main.c에서 이미 시작됨
	extern uint16_t last_encoder_count;

	uint32_t last_wake_time = osKernelGetTickCount();


  /* Infinite loop */
  for(;;)
  {
    //엔코더 속도 계산 (10ms당 펄스 수)
	  uint16_t current_count = __HAL_TIM_GET_COUNTER(&htim1);
	  int16_t diff = (int16_t)(current_count - last_encoder_count);
	  current_speed_rpm = diff;
	  last_encoder_count = current_count;

	  //큐에서 새 명령 확인 (없으면 이전 target 유지)
//	  osStatus_t status = osMessageQueueGet(myQueueHandle, &rcv_msg, NULL, 0);
//	  if(status == osOK)
//	  {
//		  target_pwm = rcv_msg.speed;
//		  current_angle = rcv_msg.angle;
//	  }

	  // 변경 코드: 큐에 쌓인 찌꺼기(과거 명령)를 모두 소진하고 '최신' 명령만 덮어씀
	  while(osMessageQueueGet(myQueueHandle, &rcv_msg, NULL, 0) == osOK)
	  {
	       target_pwm = rcv_msg.speed;
	       current_angle = rcv_msg.angle;
	  }

	  // FF + FB 결합
	  target_enc = (float)target_pwm * PWM_TO_ENC_SCALE;
	  error = target_enc - (float)current_speed_rpm;
	  float pid_correction = PID_Calculate(&pid_speed, error);
	  final_pwm = target_pwm + (int)pid_correction;

	  //클리핑
	  if(final_pwm > MOTOR_PWM_MAX) final_pwm = MOTOR_PWM_MAX;
	  if(final_pwm < -MOTOR_PWM_MAX) final_pwm = -MOTOR_PWM_MAX;

	  //모터 방향 + PWM (DC 모터 = htim3, PB0)
	  if(final_pwm > 0)
	  {
		  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)final_pwm);
	  }
	  else if(final_pwm < 0)
      {
          HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)abs(final_pwm));
      }
      else
      {
          HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      }

	  // 서보 각도 제한 + 출력 (서보 = htim2, PA0)
	  if(current_angle < SERVO_MIN_US) current_angle = SERVO_MIN_US;
	  if(current_angle > SERVO_MAX_US) current_angle = SERVO_MAX_US;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)current_angle);

      // 1초마다 LED 토글 (생존 신호)
      static uint16_t led_counter = 0;
      if (++led_counter >= 100)
      {
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          led_counter = 0;
      }

      // 100Hz 주기 보장
      last_wake_time += 10;
      osDelayUntil(last_wake_time);
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
    //500ms 동안 명령 없으면 비상 정지
	  if(HAL_GetTick() - last_command_time > 500)
	  {
		  //큐에 정지 명령 푸시
		  MotorCommand_t stop_msg;
		  stop_msg.speed = 0;
		  stop_msg.angle = SERVO_CENTER_US;
		  osMessageQueuePut(myQueueHandle, &stop_msg, 0, 0);

		  //즉시 하드웨어 차단 (이중 안전) - DC 모터 = htim3
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }

      osDelay(50);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

