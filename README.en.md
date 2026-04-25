🇰🇷 [한국어](README.md)

# Neuro-Drive: Distributed UGV Control System (RPi 5 + STM32)

![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)
![STM32](https://img.shields.io/badge/MCU-STM32F411RE-03234B?logo=stmicroelectronics&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/MPU-Raspberry_Pi_5-C51A4A?logo=raspberrypi&logoColor=white)

Heterogeneous MPU/MCU distributed control system for an Ackermann-steering UGV.  
The focus is not on "driving an RC car" — it is on **real-time control, communication pipeline design, and fail-safe mechanisms** across two physically separate processors.

---

## Project Evolution

This project grew through 6 phases, each solving a problem the previous phase exposed.  
The **Turning Point** between Phase 2 and 3 — a hardware failure during field testing — reshaped the entire architecture.  
Click any phase for the detailed write-up.

| Phase | Focus | Key Problem Solved | Status |
|:-----:|-------|--------------------|:------:|
| **1** | [Monolithic Linux Control](https://steppenhj.github.io/#phase1) | Python ↔ C++ IPC on a single RPi board | ✅ Done |
| **2** | [STM32 Distributed Architecture](https://steppenhj.github.io/#phase2) | Linux cannot guarantee hard real-time → split into Brain (RPi) / Reflex (MCU) | ✅ Done |
| ⚠️ | [**Turning Point — In-Operation Failure**](https://steppenhj.github.io/#turning_point) | 7.4V battery line contacted RPi 3.3V GPIO during field test → full architecture pivot | — |
| **3** | [RTOS & Interrupt-Driven Control](https://steppenhj.github.io/#phase3) | Bare-metal polling missed 100Hz deadlines → FreeRTOS + ISR + Queue, 2-DOF PID | ✅ Done |
| **4** | [Return-to-Home Safety System](https://steppenhj.github.io/#phase4) | Watchdog vs. autonomous operation conflict → Keep-Alive pattern, 8B → 12B protocol | ✅ Done |
| **5** | [OTA Firmware Update](https://steppenhj.github.io/#phase5) | Physical reflashing overhead → custom bootloader, CRC handshake, sector management | ✅ Done |
| **6** | [CAN Bus & Multi-ECU](https://steppenhj.github.io/#phase6) | Single UART bottleneck → 3-node CAN 2.0 distributed control | 🟡 In Progress |

---

## Architecture

![System Architecture](assets/omd_diagram.png)

**MPU (Raspberry Pi 5 / Linux)** — Web UI, WebSocket server, UDP relay, high-level mode management (RTH, Keep-Alive).  
**MCU (STM32F411RE / FreeRTOS)** — UART ISR, RTOS task scheduling, PWM generation, encoder reading, watchdog timer.

### Data Path

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

## Demo

### Phase 4 — Return-to-Home

https://github.com/user-attachments/assets/2779ef3e-39d6-4a21-8bef-ed63d195250f

---

## Key Features

**Battle-Tested Architecture** — The current design is the result of a critical hardware failure during Phase 2 field testing, when a detached 7.4V battery line contacted the RPi's 3.3V GPIO. The post-incident redesign introduced a deterministic C++ control loop, RTOS-based fail-safe logic, and physical isolation. 

**Distributed Control** — RPi handles networking and mode logic; STM32 handles hard real-time motor control. Neither can do the other's job.

**FreeRTOS Task Architecture** — UART reception (ISR + Queue), motor control, encoder reading, and safety monitoring run as independent RTOS tasks with mutex-protected shared state.

**2-DOF Control (Feedback + Feedforward)** — STM32 runs a 100 Hz P-controller for disturbance rejection (feedback); Python applies a cornering boost proportional to steering angle (feedforward). Combined output is faster and more stable than single-loop PID.

**Watchdog & Fail-Safe** — If no valid packet arrives within 500 ms, the MCU autonomously stops all motors, independent of the RPi.

**Return-to-Home (RTH)** — Encoder-based path recording (LIFO stack) and autonomous reverse traversal. Protocol extended from 8 B to 12 B to propagate mode across the full stack. Keep-Alive pattern resolves the inherent conflict between watchdog safety and autonomous operation.

**OTA Firmware Update** — Custom UART bootloader with handshake protocol, CRC-validated image transfer, sector-level Flash management, and correct bootloader-to-application jump sequence.

**MBSE Documentation** — Use case, object model, sequence, and statechart diagrams created with IBM Rhapsody to formally document system behavior.

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Web UI | HTML/CSS/JS, nipplejs (joystick), Socket.IO |
| Server | Python 3.11, Flask-SocketIO, eventlet |
| Control Core | C++17, UDP socket, threads, mutex |
| Firmware | C (STM32 HAL), FreeRTOS, UART ISR, TIM/PWM, custom bootloader |
| Communication | WebSocket, UDP (struct pack), UART (115200 baud), CAN 2.0, SPI |
| Design | IBM Rhapsody | StarUML (MBSE), STM32CubeIDE |
| Hardware | Raspberry Pi 5, STM32 Nucleo-F411RE, L298N, Ackermann chassis |

---

## Getting Started

### Prerequisites
- Raspberry Pi 5
- STM32 Nucleo-F411RE + STM32CubeIDE
- Python 3.11

### Run

```bash
# 1. Build and flash STM32 firmware via CubeIDE
#    (Bootloader first, then application image)

# 2. Build C++ control core on RPi
cd rpi/
g++ -o drive_server control_core_oop.cpp -pthread

# 3. Start web server
cd rpi/web/
python3 app.py

# 4. Open browser → http://<rpi-ip>:5000
```

### OTA Firmware Update (Phase 5)

```bash
# Flash a new application image over UART without a programmer
cd rpi/web/
python3 ota_flasher.py parkhaejin_car.bin
```

---

## Author

**박해진 (Haejin Park)**  
Kyungpook National University