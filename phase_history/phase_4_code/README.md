# Phase 4 — RTH (Return to Home) 기능 추가

RPi C++ 코드를 OOP로 리팩토링하고, 엔코더 피드백을 이용한 RTH 기능과 PID 속도 제어를 추가한다.

## 아키텍처

### RPi (OOP C++)

```
SharedContext (공유 자원, mutex/atomic)
    ├── UdpReceiver (Thread 1): UDP 수신 → throttle/steering/mode 갱신
    └── VehicleController (Thread 2, 100Hz):
            ├── 일반 모드(0): 명령 → UART 전송
            ├── 기록 모드(1): 이동 경로 {enc_diff, servo_angle} 벡터에 저장
            └── 복귀 모드(2): 경로 역순 재생, 거리 기반 감속
```

### STM32 (FreeRTOS + PID)

```
UART ISR → Queue → Task_Motor
                       ├── TIM1 엔코더 diff 계산 (100Hz)
                       ├── PID: error = target_speed - current_speed_rpm
                       └── PWM 출력 → 모터 구동
```

## 주요 파일

| 파일 | 역할 |
|------|------|
| `control_core_oop.cpp` | RPi C++ OOP 제어 코어. `UdpReceiver`, `VehicleController`, `SharedContext` 클래스. RTH 경로 기록·복귀 로직 |
| `main.c` | STM32 펌웨어. Phase 3 구조 유지 + PID 구조체·함수 추가, 엔코더 피드백 제어 |

## 핵심 기술

- **RTH (Return to Home)**
  - 기록 모드: 엔코더 diff와 서보 각도를 `vector<pair<int,int>>`에 누적 저장 (노이즈 필터 `|enc| > 2`)
  - 복귀 모드: 벡터를 역순으로 꺼내며 반대 방향으로 동일 거리 주행, 남은 거리에 비례 감속 (500~600 PWM)
- **PID 속도 제어 (STM32)**
  - 구조체: `PID_t {kp, ki, kd, integral, prev_error, output_min, output_max}`
  - 100Hz 루프에서 `error = target - encoder_diff` 계산
  - Anti-windup 포함 (출력 클리핑)
- **OOP 리팩토링 (RPi)**
  - `SharedContext`: mutex + atomic으로 스레드 안전 보장
  - 소멸자에서 `join()` + `close()` 자동 처리 (RAII)
- **엔코더 피드백 루프**: STM32 `ENC:값\n` → RPi VehicleController `read()` → `ctx.last_encoder_diff` → RTH 거리 계산
- **UDP 포트 분리**: 명령 수신 5555 / 엔코더 피드백 전달 5556

## Phase 3 대비 변경점

| 항목 | Phase 3 | Phase 4 |
|------|---------|---------|
| RPi 구조 | 함수형 스레드 | OOP 클래스 |
| 속도 제어 | Open-Loop | PID Closed-Loop |
| 특수 기능 | 없음 | RTH (경로 기록·복귀) |
| UDP 패킷 | `{throttle, steering}` | `{throttle, steering, mode}` |

## Phase 5와의 차이

- 펌웨어 업데이트 시 매번 USB 연결 필요
- Phase 5에서 무선 OTA 업데이트 기능 추가
