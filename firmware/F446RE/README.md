# F446RE Firmware (Phase 6)

## 사양
- MCU: STM32F446RET6 (Nucleo-64)
- Clock: HSI 16MHz → PLL → SYSCLK 100MHz
- APB1: 25MHz peripheral / 50MHz timer
- APB2: 50MHz peripheral / 100MHz timer

## 페리페럴
- USART2 (PA2/PA3): RPi 통신, 115200 8N1, IT 모드
- TIM1 (PA8/PA9): 엔코더 TI12 모드
- TIM2_CH1 (PA0): DC 모터 PWM, 1kHz (Prescaler 49, Period 999)
- TIM3_CH3 (PB0): 서보 PWM, 50Hz (Prescaler 49, Period 19999)
- PA1, PA4: 모터 방향 (MOTOR_IN1, MOTOR_IN2)
- PA5: LD2 (상태 LED)

## FreeRTOS 태스크
| 태스크 | 우선순위 | 주기 | 역할 |
|---|---|---|---|
| Task_Comm | Normal | 10ms | UART 송신 (엔코더, PONG) |
| Task_Motor | High | 10ms | PID 제어 + PWM 출력 |
| Task_Safety | Realtime | 50ms | 500ms Watchdog |

## TODO
- [ ] PWM 주파수 튜닝 (현재 1kHz, 가청 소음)
- [ ] CAN1 페리페럴 추가
- [ ] MCP2562 또는 TJA1050 트랜시버 연결