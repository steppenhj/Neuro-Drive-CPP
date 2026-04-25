# Neuro-Drive: 분산 UGV 제어 시스템 (RPi 5 + STM32)

🇺🇸 [English](README.en.md)

![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)
![STM32](https://img.shields.io/badge/MCU-STM32F411RE-03234B?logo=stmicroelectronics&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/MPU-Raspberry_Pi_5-C51A4A?logo=raspberrypi&logoColor=white)

Ackermann 조향 UGV를 위한 이기종 MPU/MCU 분산 제어 시스템.  
이 프로젝트의 핵심은 "RC카 주행"이 아닌, **물리적으로 분리된 두 프로세서 간의 실시간 제어, 통신 파이프라인 설계, Fail-Safe 메커니즘**에 있습니다.

---

## 프로젝트 발전 과정

이 프로젝트는 6단계에 걸쳐 발전했으며, 각 단계는 이전 단계에서 드러난 문제를 해결하는 방식으로 진행되었습니다.  
Phase 2와 3 사이의 **전환점** — 필드 테스트 중 발생한 하드웨어 장애 — 은 전체 아키텍처를 재설계하는 계기가 되었습니다.  
각 Phase를 클릭하면 상세 내용을 확인할 수 있습니다.

| Phase | 주제 | 해결한 핵심 문제 | 상태 |
|:-----:|------|--------------------|:----:|
| **1** | [단일 Linux 제어](https://steppenhj.github.io/#phase1) | 단일 RPi 보드에서 Python ↔ C++ IPC 구현 | ✅ 완료 |
| **2** | [STM32 분산 아키텍처](https://steppenhj.github.io/#phase2) | Linux의 Hard Real-Time 불가 → Brain(RPi) / Reflex(MCU)로 분리 | ✅ 완료 |
| ⚠️ | [**전환점 — 운용 중 장애 발생**](https://steppenhj.github.io/#turning_point) | 필드 테스트 중 7.4V 배터리 선이 RPi 3.3V GPIO에 접촉 → 전체 아키텍처 전면 재설계 | — |
| **3** | [RTOS 및 Interrupt 기반 제어](https://steppenhj.github.io/#phase3) | Bare-metal Polling의 100Hz 데드라인 미달 → FreeRTOS + ISR + Queue, 2-DOF PID | ✅ 완료 |
| **4** | [Return-to-Home 안전 시스템](https://steppenhj.github.io/#phase4) | Watchdog과 자율 동작의 충돌 → Keep-Alive 패턴, 8B → 12B 프로토콜 확장 | ✅ 완료 |
| **5** | [OTA Firmware 업데이트](https://steppenhj.github.io/#phase5) | 물리적 재플래싱 부담 → 커스텀 Bootloader, CRC 핸드셰이크, 섹터 관리 | ✅ 완료 |
| **6** | [CAN Bus 및 Multi-ECU](https://steppenhj.github.io/#phase6) | 단일 UART 병목 → 3노드 CAN 2.0 분산 제어 | 🟡 진행 중 |

---

## 아키텍처

![System Architecture](assets/omd_diagram.png)

**MPU (Raspberry Pi 5 / Linux)** — Web UI, WebSocket 서버, UDP 릴레이, 상위 레벨 모드 관리 (RTH, Keep-Alive).  
**MCU (STM32F411RE / FreeRTOS)** — UART ISR, RTOS 태스크 스케줄링, PWM 생성, 엔코더 읽기, Watchdog 타이머.

### 데이터 흐름

```
Browser ──WebSocket──▶ Python (Flask-SocketIO)
                          │
                          ├──UDP (12B: throttle, steering, mode)──▶ C++ Control Core
                          │                                              │
                          │                                         UART (115200)
                          │                                              │
                          │                                              ▼
                          │                                      STM32 (FreeRTOS)
                          │                                        ├─ UART RX ISR → Queue
                          │                                        ├─ Motor Task (PWM)
                          │                                        ├─ Encoder Task (TIM1)
                          │                                        └─ Safety Task (Watchdog)
                          │
                          └──UDP (Encoder telemetry)◀── C++ ◀──UART── STM32
```

---

## RTH 시연 영상

### Phase 4 — Return-to-Home

https://github.com/user-attachments/assets/2779ef3e-39d6-4a21-8bef-ed63d195250f

---

## 주요 기능

**실패로부터 검증된 아키텍처** — 현재 설계는 Phase 2 필드 테스트 중 발생한 치명적 하드웨어 장애의 산물입니다. 탈락한 7.4V 배터리 선이 RPi의 3.3V GPIO에 접촉하는 사고 이후, 결정론적 C++ 제어 루프, RTOS 기반 Fail-Safe 로직, 물리적 절연이 도입되었습니다. 

**분산 제어** — RPi는 네트워킹과 모드 로직을, STM32는 Hard Real-Time 모터 제어를 담당합니다. 두 프로세서는 서로의 역할을 대체할 수 없습니다.

**FreeRTOS 태스크 아키텍처** — UART 수신(ISR + Queue), 모터 제어, 엔코더 읽기, 안전 모니터링이 독립적인 RTOS 태스크로 실행되며, Mutex로 공유 상태를 보호합니다.

**2-DOF 제어 (Feedback + Feedforward)** — STM32는 100Hz P-controller로 외란을 억제(Feedback)하고, Python은 조향 각도에 비례한 코너링 부스트를 적용(Feedforward)합니다. 단일 루프 PID보다 빠르고 안정적인 응답을 제공합니다.

**Watchdog & Fail-Safe** — 500ms 이내에 유효한 패킷이 수신되지 않으면, MCU가 RPi와 독립적으로 모든 모터를 자율 정지합니다.

**Return-to-Home (RTH)** — 엔코더 기반 경로 기록(LIFO 스택)과 자율 역주행. Watchdog 안전 기능과 자율 동작 간의 충돌을 Keep-Alive 패턴으로 해결하고, 프로토콜을 8B에서 12B로 확장해 전체 스택에 모드를 전파합니다.

**OTA Firmware 업데이트** — 핸드셰이크 프로토콜, CRC 검증 이미지 전송, 섹터 단위 Flash 관리, 올바른 Bootloader→Application 점프 시퀀스를 갖춘 커스텀 UART Bootloader.

**MBSE 문서화** — IBM Rhapsody를 사용해 유스케이스, 객체 모델, 시퀀스, 상태차트 다이어그램으로 시스템 동작을 공식 문서화.

---

## 기술 스택

| 레이어 | 기술 |
|--------|------|
| Web UI | HTML/CSS/JS, nipplejs (joystick), Socket.IO |
| 서버 | Python 3.11, Flask-SocketIO, eventlet |
| Control Core | C++17, UDP socket, threads, mutex |
| Firmware | C (STM32 HAL), FreeRTOS, UART ISR, TIM/PWM, 커스텀 Bootloader |
| 통신 | WebSocket, UDP (struct pack), UART (115200 baud), CAN 2.0, SPI |
| 설계 도구 | IBM Rhapsody \| StarUML (MBSE), STM32CubeIDE |
| 하드웨어 | Raspberry Pi 5, STM32 Nucleo-F411RE, L298N, Ackermann 섀시 |

---

## 시작하기

### 사전 준비
- Raspberry Pi 5
- STM32 Nucleo-F411RE + STM32CubeIDE
- Python 3.11

### 실행

```bash
# 1. STM32CubeIDE로 STM32 Firmware 빌드 및 플래시
#    (Bootloader 먼저, 이후 Application 이미지)

# 2. RPi에서 C++ Control Core 빌드
cd rpi/
g++ -o drive_server control_core_oop.cpp -pthread

# 3. 웹 서버 시작
cd rpi/web/
python3 app.py

# 4. 브라우저에서 접속 → http://<rpi-ip>:5000
```

### OTA Firmware 업데이트 (Phase 5)

```bash
# 프로그래머 없이 UART로 새 Application 이미지 플래시
cd rpi/web/
python3 ota_flasher.py parkhaejin_car.bin
```

---

## 작성자

**박해진 (Haejin Park)**  
경북대학교
