# 🚜 Neuro-Drive: Scaled Autonomous UGV Project

<div align="center">

![Python](https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white)
![C++](https://img.shields.io/badge/Language-C++17-00599C?logo=c%2B%2B&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/MPU-Raspberry_Pi_5-C51A4A?logo=raspberrypi&logoColor=white)
![STM32](https://img.shields.io/badge/MCU-STM32_Nucleo_F411RE-03234B?logo=stmicroelectronics&logoColor=white)
![ROS 2](https://img.shields.io/badge/Middleware-ROS_2_Humble-22314E?logo=ros&logoColor=white)

</div>

---

## 📖 Overview
**Neuro-Drive**는 실제 자동차 및 방산 무인 차량(UGV)의 아키텍처를 축소 모사한 **이기종 컴퓨팅(Heterogeneous Computing) 기반 자율주행 프로젝트**입니다.

단순한 RC카 구동을 넘어, **Linux 기반의 Mission Computer(MPU)**와 **RTOS/Firmware 기반의 Vehicle Control Unit(MCU)**을 연동하여 시스템의 안정성과 실시간성을 확보하는 것을 목표로 합니다. 최종적으로는 컴퓨터 비전과 센서 퓨전을 활용한 **피아식별(IFF) 및 자율 임무 수행**을 지향합니다.

---

## 🗺️ Project Roadmap

본 프로젝트는 현업의 자율주행 시스템 개발 절차를 따르는 **4단계 마일스톤**으로 진행됩니다.

### ✅ Phase 1: Baseline (Completed)
> **Goal: 라즈베리파이 단독 제어 및 기구학적 특성 파악**
- [x] Ackermann Steering Geometry 분석 및 제어
- [x] Flask 기반의 Web 인터페이스 및 저지연(Low-latency) 원격 제어
- [x] PCA9685 PWM Driver 제어

### 🚧 Phase 2: Distributed Architecture (Current Focus)
> **Goal: MPU(RPi)와 MCU(STM32)의 역할 분리 및 실시간 제어기(VCU) 구축**
- [ ] **Hardware Interface:** 라즈베리파이(Brain) ↔ STM32(Spinal Cord) 간 UART/CAN 통신 프로토콜 설계
- [ ] **Failsafe Logic:** 상위 제어기(MPU) 통신 두절 시 MCU 단독 비상 정지 로직 구현
- [ ] **PID Control:** STM32 타이머 인터럽트를 활용한 정밀 모터 속도/위치 제어

### 🔜 Phase 3: Autonomous Navigation
> **Goal: 인지(Perception) - 판단(Decision) - 제어(Control) 파이프라인 구축**
- [ ] **Middleware:** ROS 2 (Robot Operating System) 도입 및 노드 통신 구축
- [ ] **Vision:** OpenCV 및 Lane Detection 알고리즘 적용
- [ ] **Control:** Pure Pursuit 등 경로 추종 알고리즘 구현

### 🔮 Phase 4: Mission & Sensor Fusion (Defense Application)
> **Goal: 객체 인식 기반의 피아식별(IFF) 및 동적 임무 수행**
- [ ] **AI:** YOLOv8 (Nano) 기반 실시간 객체 탐지 (NPU/GPU 가속)
- [ ] **Sensor Fusion:** Vision(형상 인식) + IR/RF Sensor(신호 식별) 융합
- [ ] **Scenario:**
    - 🟥 **적군(Enemy) 탐지:** 추적(Tracking) 또는 충돌(Collision) 시나리오
    - 🟩 **아군(Friend) 탐지:** 회피(Avoidance) 또는 정지(Stop) 시나리오

---

## 🛠 Hardware Specifications

### 1. Dual-Core Architecture
실제 차량처럼 **고성능 연산부(MPU)**와 **실시간 제어부(MCU)**를 분리한 아키텍처를 채택했습니다.

| Unit | Device | Role in System |
| :--- | :--- | :--- |
| **Main Computer**<br>(MPU) | **Raspberry Pi 5**<br>(8GB RAM) | **[Brain]**<br>고해상도 영상 처리(YOLO), SLAM, 경로 생성, 상위 판단 로직 수행. |
| **Control Unit**<br>(MCU) | **STM32 Nucleo**<br>(F411RE) | **[Reflex]**<br>Hard Real-time이 요구되는 모터 PWM 생성, 엔코더 피드백, 센서 데이터 수집. |

### 2. Drive System (Robot Body)

| Component | Model / Specs | Description |
| :--- | :--- | :--- |
| **Platform** | Ackermann Chassis | 실제 차량과 동일한 **전륜 조향/후륜 구동(RWD)** 방식. 비홀로노믹 제약 조건 학습. |
| **Motor Driver** | L298N / Waveshare | MCU(STM32)의 GPIO 신호를 받아 DC 모터 및 서보 모터 구동. |
| **Camera** | **RPi Cam v3 (Wide)** | **120° 광각**을 통해 차선 및 주변 환경 정보 수집. (Phase 3~4) |
| **IFF Sensor** | IR Receiver (VS1838B) | 적외선 신호 패턴 분석을 통한 피아식별 보조 센서. (Phase 4) |
| **Power** | Li-Po 7.4V (35C) | 고방전율 배터리로 급격한 부하 변동에 대응하며 시스템 전원 공급. |

---

## 💻 Software Stack

### High-Level (Raspberry Pi 5)
- **OS:** Raspberry Pi OS (Bookworm 64-bit) / Ubuntu 22.04 LTS
- **Middleware:** ROS 2 (Humble/Jazzy)
- **Languages:** Python 3.11, C++17
- **Vision/AI:** OpenCV, PyTorch (YOLOv8), NCNN

### Low-Level (STM32)
- **IDE:** STM32CubeIDE (v1.16.x Recommended)
- **Language:** Embedded C
- **Communication:** UART (Custom Packet Structure), I2C, SPI
- **Control:** PID Algorithm, Odometry Calculation

---

## 🚀 Key Features
1. **Hybrid Control System:** Web 수동 제어와 자율주행 모드 실시간 전환 (Handover)
2. **Robust Communication:** 자체 설계한 패킷 프로토콜(Header-Data-Checksum)로 노이즈 강건성 확보
3. **Safety First:** Watchdog Timer 및 Dead Man's Switch를 통한 2중 안전 장치
4. **Sensor Fusion IFF:** 단순 영상 인식을 넘어선 **Vision + Sensor 융합** 기반의 신뢰성 높은 피아식별 구현

---

## 🔧 Installation

```bash
# Clone Repository
git clone [https://github.com/steppenhj/Neuro-Drive-CPP.git](https://github.com/steppenhj/Neuro-Drive-CPP.git)
cd Neuro-Drive-C++

# (Optional) Setup Virtual Environment
python -m venv venv
source venv/bin/activate