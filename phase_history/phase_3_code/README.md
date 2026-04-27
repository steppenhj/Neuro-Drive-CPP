# Phase 3 — FreeRTOS + UART 인터럽트 + 큐

STM32에 FreeRTOS를 도입해 태스크 분리, UART 폴링을 인터럽트로 교체하고 태스크 간 통신에 메시지 큐를 사용한다. RPi 측도 C++ 멀티스레드 구조로 전환한다.

## 아키텍처

### STM32 (FreeRTOS)

```
HAL_UART_RxCpltCallback (ISR)
        |
  osMessageQueuePut()
        |
   [myQueueHandle]
        |
  Task_Motor (High) ──── TIM2(DC모터) / TIM3(서보)
  Task_Safety (Realtime) ── 500ms Watchdog
  Task_Comm (Normal) ──── UART 인터럽트 장전, 대기
```

### RPi (C++ 멀티스레드)

```
Thread 1: udp_receiver_task  (Blocking UDP 수신 → SharedMemory 갱신)
Thread 2: control_loop_task  (100Hz 고정 주기 → UART 직렬 전송)
```

## 주요 파일

| 파일 | 역할 |
|------|------|
| `main.c` | STM32 FreeRTOS 펌웨어. ISR에서 큐로 명령 전달, 3개 태스크 구조 |
| `control_core_threaded.cpp` | RPi C++ 제어 코어. UDP 수신 스레드 + 100Hz 제어 루프 스레드, mutex + atomic Watchdog |
| `app3.py` | Flask/SocketIO 서버 (Phase 2와 유사) |

## 핵심 기술

- **FreeRTOS 3-task 구조**
  - `Task_Safety` (Realtime): 500ms Watchdog 감시, 큐에 정지 명령 삽입
  - `Task_Motor` (High): 큐에서 명령 수신 후 모터·서보 제어, 100Hz `osDelayUntil` 주기
  - `Task_Comm` (Normal): UART 인터럽트 장전 후 대기
- **UART 인터럽트 수신**: `HAL_UART_Receive_IT()` → `HAL_UART_RxCpltCallback` → `osMessageQueuePut()`
- **UART 에러 자동 복구**: `HAL_UART_ErrorCallback`에서 수신 인터럽트 재장전
- **Jitter 측정**: Task_Motor 루프에서 실제 실행 주기 측정 (max/min 통계 100회마다 출력)
- **RPi 측 100Hz 정밀 타이밍**: `std::this_thread::sleep_until(next_tick)` 으로 드리프트 방지

## Phase 2 대비 변경점

| 항목 | Phase 2 | Phase 3 |
|------|---------|---------|
| STM32 수신 방식 | 폴링 (`HAL_UART_Receive`) | 인터럽트 + 큐 |
| STM32 구조 | 단일 루프 | FreeRTOS 3-task |
| RPi 제어 | Python subprocess | C++ 멀티스레드 |

## Phase 4와의 차이

- 속도 제어가 Open-Loop (명령 PWM을 그대로 출력)
- Phase 4에서 PID 피드백 제어와 RTH 기능 추가
