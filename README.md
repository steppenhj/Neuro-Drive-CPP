# 🚗 Neuro-Drive: Raspberry Pi 5 Autonomous RC Car

![Python](https://img.shields.io/badge/Python-3.11-blue?logo=python) ![Flask](https://img.shields.io/badge/Flask-Web_Server-green?logo=flask) ![Raspberry Pi](https://img.shields.io/badge/Hardware-Raspberry_Pi_5-C51A4A?logo=raspberrypi) ![C++](https://img.shields.io/badge/Language-C++-00599C?logo=c%2B%2B)

**Raspberry Pi 5**와 **Ackermann Steering Chassis**를 활용한 자율주행 RC카 프로젝트입니다.  
현재 **Web 기반의 실시간 원격 제어(Remote Control)** 시스템이 구축되어 있으며, 향후 C++ 기반의 고성능 자율주행 알고리즘을 탑재할 예정입니다.

## 🛠 Hardware Specs
- **Main Computer:** Raspberry Pi 5 (8GB RAM)
- **Motor Driver:** Waveshare Motor Driver HAT (PCA9685 + TB6612)
- **Chassis:** Ackermann Steering Geometry (전륜 조향, 후륜 구동)
- **Power:** 7.4V Li-Po Battery (XT60 Connector)
- **Camera:** Raspberry Pi Camera Module V3 Wide (Scheduled)

## 💻 Software Stack
- **Backend:** Python (Flask), C++ (Low-level Motor Control)
- **Frontend:** HTML5, CSS3, JavaScript (Touch Interface)
- **Communication:** HTTP (REST API), WebSocket (Planned)
- **OS:** Raspberry Pi OS (Bookworm 64-bit)

## 🚀 Key Features
1. **Web-based Controller:** 별도 앱 설치 없이 스마트폰 브라우저로 접속하여 제어
2. **Real-time Latency:** 내부망(Wi-Fi) 기준 지연 시간 최소화
3. **Safety Logic:** Dead Man's Switch 적용 (손을 떼면 즉시 정지)
4. **Touch Interface:** 모바일 터치 이벤트 최적화 (확대/메뉴 팝업 방지)

## 🔧 Installation & Run

### 1. Clone Repository
```bash
git clone [https://github.com/steppenhj/Neuro-Drive-C-.git](https://github.com/steppenhj/Neuro-Drive-C-.git)
cd Neuro-Drive-C++