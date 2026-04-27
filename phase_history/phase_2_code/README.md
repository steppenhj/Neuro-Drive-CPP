# Phase 2 — STM32 추가 (폴링 방식)

STM32F411RE를 추가해 모터 PWM 및 엔코더 계측을 하드웨어로 분리한 버전.
STM32의 UART 수신은 메인 루프에서 폴링으로 처리한다.

## 아키텍처

```
[브라우저] --WebSocket--> [app2.py (Flask/RPi)] --UART Serial--> [main.c (STM32)]
                                                                       |
                                                               TIM2(DC모터PWM)
                                                               TIM3(서보PWM)
                                                               TIM1(엔코더)
```

## 주요 파일

| 파일 | 역할 |
|------|------|
| `main.c` | STM32 펌웨어. 메인 루프에서 `HAL_UART_Receive()` 폴링으로 `speed,angle\n` 수신 후 모터·서보 제어. 50ms마다 `ENC:속도\n` 텔레메트리 송신 |
| `app2.py` | Flask/SocketIO 서버. STM32에 UART로 명령 전송. GStreamer 카메라 스트리밍. 코너링 부스터 로직 포함 |
| `index2.html` | Phase 1 UI 기반에 카메라 영상 피드 추가 |

## 핵심 기술

- **HAL TIM 하드웨어 PWM**: TIM2(DC모터, 60Hz), TIM3(서보, 50Hz)
- **TIM1 엔코더 모드**: 쿼드러처 엔코더로 바퀴 속도 측정 (50ms 주기 → PPS 계산)
- **UART 폴링 수신**: `HAL_UART_Receive()` 1ms 타임아웃으로 1바이트씩 수집, `\n` 감지 후 `sscanf` 파싱
- **Failsafe (500ms)**: 마지막 명령 후 500ms 경과 시 PWM 강제 0
- **코너링 부스터**: steering 값에 비례해 throttle 최대 1.8배 증폭 (전압 부족 보상)

## Phase 1 대비 변경점

| 항목 | Phase 1 | Phase 2 |
|------|---------|---------|
| 모터 제어 | RPi + PCA9685 (I2C) | STM32 하드웨어 PWM |
| 속도 측정 | 불가 | STM32 엔코더 |
| 통신 | UDP (RPi 내부) | UART Serial (RPi ↔ STM32) |

## Phase 3와의 차이

- 폴링 방식이라 UART 수신 중 다른 작업이 블로킹됨
- Phase 3에서 FreeRTOS + UART 인터럽트 + 큐 구조로 전환
