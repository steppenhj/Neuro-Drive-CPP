# 🚜 Neuro-Drive: Heterogeneous UGV Control Stack (RPi + STM32)

<div align="center">

![Python](https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/MPU-Raspberry_Pi_5-C51A4A?logo=raspberrypi&logoColor=white)
![STM32](https://img.shields.io/badge/MCU-STM32_Nucleo_F411RE-03234B?logo=stmicroelectronics&logoColor=white)

</div>

---

## 📌 Overview
**Neuro-Drive**는 실제 차량/UGV에서 흔한 구조를 축소한 **이기종(heterogeneous) 제어 아키텍처** 프로젝트입니다.

- **MPU (Raspberry Pi / Linux)**: UI·네트워크·상위 제어(계산/모드 전환)·로깅
- **MCU (STM32 / RTOS)**: 하드 리얼타임 제어(PWM/TIM)·UART 인터럽트 수신·Fail-safe

핵심 목표는 “RC카 구동”이 아니라 **분산 제어 + 실시간성 + 안전 정지 로직**을 구현하고, 이를 로그/시나리오로 증명하는 것입니다.

---

## 🧱 System Architecture

### Data Path (현재 구현)
- **PC/Client → RPi**: WebSocket(Flask-SocketIO) 또는 UDP 입력
- **RPi → STM32**: Serial(`/dev/ttyACM0`)로 `speed,angle\n` 송신  
  > RPi 측은 USB-CDC(가상 COM)로 접근하지만, MCU 측은 **USART2(UART)** 로 수신(Interrupt 기반).
- **STM32**: UART RX ISR → packet parse → **RTOS Queue** → Motor task에서 TIM/PWM 갱신
- **Fail-safe**: 명령 타임아웃 시 MCU 단독으로 모터 정지(2중 차단)

---

## 🗺️ Roadmap

### ✅ Phase 1: Single-Board Baseline (Done)
- [x] Flask 기반 Web UI 및 원격 제어
- [x] UDP/WebSocket 입력 처리
- [x] (Legacy) PCA9685 기반 PWM 구동

### 🚧 Phase 2: Distributed Control (Current)
- [x] RPi(C++/Python) ↔ STM32(Embedded C) 분산 구조 구축
- [x] STM32 **UART RX Interrupt** 기반 수신 + 파싱
- [x] RTOS Task 분리(Comm / Motor / Safety) 및 **Queue 기반 전달**
- [x] TIM/PWM 기반 DC Motor + Servo 제어
- [x] **Fail-safe**: 통신 두절 시 자동 정지(타임아웃)
- [ ] UART 패킷 강건화(헤더/길이/CRC) + 에러 카운터
- [ ] (Optional) CAN 확장(진단/텔레메트리 용도)

### 🔜 Phase 3: Autonomous Assist
- [ ] 카메라 기반 차선/장애물 감지(단계적)
- [ ] 로깅/리플레이 기반 재현 가능한 테스트 하네스
- [ ] (Optional) ROS 2 도입

---

## 🛠 Hardware

| Unit | Device | Role |
| --- | --- | --- |
| MPU | Raspberry Pi 5 | Network/UI, High-level control, logging |
| MCU | STM32 Nucleo F411RE | Hard real-time motor control, safety |

Drive:
- Ackermann chassis (Front steering / Rear drive)
- Motor Driver: **L298N (prototype)**  
  > 향후 MOSFET 기반 드라이버로 교체 계획(보호/진단 강화)
- Camera: RPi Camera v3 (Wide)

---

## 💻 Software

### RPi (Linux)
- Python: Flask + SocketIO (UI / streaming / logging)
- C++: UDP receiver + 100Hz control loop + serial TX(`/dev/ttyACM0`)
- Multi-threading + mutex-based shared data

### STM32 (RTOS)
- UART RX Interrupt + error callback
- RTOS Queue (command mailbox)
- Motor task: TIM/PWM output
- Safety task: timeout stop + HW cut-off

---

## ✅ Key Engineering Points (현재까지 증명 가능한 항목)
- 분산 제어(MPU/MCU 역할 분리)
- UART 인터럽트 기반 수신 파이프라인 + RTOS Queue 설계
- 주기 제어(100Hz) 및 지터 측정 로그(확장 가능)
- 통신 두절 Fail-safe (MCU 단독 안전정지)

---

## 📁 Repo Structure (예시)
- `rpi/` : Python server, C++ control core
- `firmware/` : STM32CubeIDE project

---

## 🔧 Quick Start
```bash
# Clone
git clone https://github.com/steppenhj/Neuro-Drive-CPP.git
cd Neuro-Drive-CPP
