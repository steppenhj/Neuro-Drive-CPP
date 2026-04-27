# Phase 1 — RPi 단독 주행

RPi 하나로 전체 시스템을 구성한 초기 버전. STM32 없이 RPi가 모터 하드웨어를 직접 제어한다.

## 아키텍처

```
[브라우저] --WebSocket--> [app1.py (Flask)] --UDP--> [drive_server.cpp]
                                                           |
                                                      PCA9685 (I2C)  +  Servo (gpiod)
```

## 주요 파일

| 파일 | 역할 |
|------|------|
| `drive_server.cpp` | C++ 하드웨어 서버. UDP 수신 후 PCA9685(I2C)로 DC 모터, gpiod 소프트웨어 PWM으로 서보 제어 |
| `app1.py` | Flask/SocketIO 웹 서버. 엔진 토글 시 `drive_server` 를 subprocess로 실행, 제어 명령을 UDP로 전달 |
| `index1.html` | 조이스틱(nipplejs) UI. 엔진 ON/OFF 버튼, CPU 온도 텔레메트리, Ping-Pong 레이턴시 측정 포함 |

## 핵심 기술

- **PCA9685 (I2C)**: DC 모터 2채널 PWM 제어
- **gpiod 소프트웨어 PWM**: 서보 모터 (`atomic<int>` + 별도 스레드로 20ms 주기 생성)
- **UDP 소켓**: Python ↔ C++ 프로세스 간 `{throttle(float), steering(float)}` 이진 전송
- **Watchdog (500ms)**: `SO_RCVTIMEO` 타임아웃으로 신호 끊기면 자동 정지

## Phase 2와의 차이

- STM32가 없어서 모터 PWM 정밀도가 낮고, RPi의 OS 스케줄러에 전적으로 의존함
- 엔코더 피드백 없음 → 속도를 측정할 수 없음
- Phase 2에서 STM32를 추가해 하드웨어 PWM 및 엔코더 계측을 분리
