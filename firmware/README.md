# Neuro-Drive Firmware

## 디렉토리 구조
- `bootloader/` — F411RE용 OTA 부트로더 (Sector 0)
- `F411RE/` — Phase 1~5 메인 펌웨어 (UART, OTA 지원)
- `F446RE/` — Phase 6 메인 펌웨어 (CAN 전환 진행 중)

## 현재 상태 (2026-05-03)
- F411RE: 운영 중, OTA 가능
- F446RE: F411RE 펌웨어 이식 완료, 기본 동작 검증 OK
  - TODO: PWM 주파수 튜닝 (모터 소음/토크)
  - TODO: CAN 페리페럴 추가
- Bootloader: F411RE 전용. F446RE용 미작성.

## 빌드 방법
각 보드 폴더의 .ioc 파일을 STM32CubeIDE에서 열고 코드 생성.
src/ 폴더의 파일들을 USER CODE 블록에 복사.