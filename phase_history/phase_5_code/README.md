# Phase 5 — OTA (Over-The-Air) 펌웨어 업데이트

STM32에 커스텀 부트로더를 추가해 UART를 통해 USB 없이 펌웨어를 원격 업데이트할 수 있다. 앱 펌웨어에는 PING/PONG 레이턴시 측정과 Feedforward+PID 속도 제어가 추가됐다.

## 아키텍처

### Flash 메모리 레이아웃 (STM32F411RE 512KB)

```
0x08000000  Sector 0 (16KB) ── 부트로더 (OTA.c)
0x08004000  Sector 1~7      ── Application (main.c)
```

### OTA 업데이트 흐름

```
[ota_flasher.py]          [OTA.c 부트로더]
    GPIO NRST reset ──>   부팅 (LED ON)
    "UPDATE\n"       ──>  3초 대기
                    <──   "READY\r\n"
    [4B: file_size]  ──>
                    <──   "ACK\r\n"
                          Flash Erase (Sector 1~7)
                    <──   "ACK\r\n"
    [256B chunk×N]   ──>  Flash Write (각 청크마다)
                    <──   "ACK\r\n"  (반복)
    [4B: CRC32]      ──>  CRC 검증
                    <──   "DONE\r\n" / "NACK\r\n"
                          NVIC_SystemReset → 부트로더 → JumpToApp
```

## 주요 파일

| 파일 | 역할 |
|------|------|
| `OTA.c` | STM32 커스텀 부트로더. Flash Sector 0에 위치. UPDATE 신호 대기 → 에라이즈 → 청크 수신 → CRC 검증 → 앱으로 점프 |
| `main.c` | STM32 앱 펌웨어 (Phase 4 기반). PING/PONG 레이턴시 측정, Feedforward+PID 속도 제어, OTA 검증용 1Hz LED 점멸 추가 |
| `ota_flasher.py` | RPi OTA 전송 스크립트. `OTAFlasher` 클래스로 GPIO 리셋, 핸드셰이크, 청크 전송, CRC32 검증 처리 |
| `app.py` | Flask/SocketIO 서버 (웹 UI에서 OTA 트리거 가능) |

## 핵심 기술

- **커스텀 부트로더**
  - `JumpToApp()`: 앱 영역 Stack Pointer 유효성 검증 → 인터럽트 비활성화 → `SCB->VTOR` 재설정 → `__set_MSP()` → Reset Handler 점프
  - `WaitForUpdateSignal()`: 3초 내 `"UPDATE\n"` 문자열 수신 시 업데이트 모드 진입
  - `EraseAppFlash()`: Sector 1~7 삭제 (Sector 0 부트로더 영역은 절대 삭제 안 함)
  - `CalculateFlashCRC()`: CRC32 소프트웨어 구현, Python `zlib.crc32()`와 동일 알고리즘
- **OTAFlasher 클래스 (Python)**
  - 256바이트 청크 단위 전송 (마지막 청크 `0xFF` 패딩)
  - GPIO subprocess로 NRST 핀 하드웨어 리셋 (eventlet 충돌 방지용 subprocess 분리)
  - `UPDATE\n` 재전송 최대 5회 retry
- **PING/PONG 레이턴시 측정 (앱 펌웨어)**
  - `"PING\n"` 수신 시 ISR에서 플래그만 세팅 → `Task_Comm`에서 `"PONG\r\n"` 송신
  - ISR에서 직접 송신 시 huart2 Race Condition 발생 문제를 Task로 이전하여 해결
- **Feedforward + PID 속도 제어**
  - `PWM_TO_ENC_SCALE = 0.1f` 로 PWM 단위를 엔코더 단위로 변환
  - `final_pwm = target_pwm(Feedforward) + PID_correction(Feedback)` 결합

## Phase 4 대비 변경점

| 항목 | Phase 4 | Phase 5 |
|------|---------|---------|
| 펌웨어 업데이트 | USB 직접 연결 | UART OTA (부트로더) |
| 속도 제어 | PID only | Feedforward + PID |
| 레이턴시 측정 | 없음 | PING/PONG (ISR→Task 이전) |
| Flash 구조 | 단일 앱 영역 | 부트로더(Sector 0) + 앱(Sector 1~7) |
