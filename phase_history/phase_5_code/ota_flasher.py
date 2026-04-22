#!/usr/bin/env python3
"""
ota_flasher.py - OTA Firmware Flasher
STM32F411RE 부트로더와 UART 통신하여 .bin 펌웨어를 전송하는 스크립트

사용방법:
    1) 단독 실행 (CLI 테스트):
        python3 ota_flasher.py firmware.bin

    2) app.py에서 모듈로 import
        from ota_flasher import OTAFlasher
        flasher = OTAFlasher("/dev/ttyACM0")
        flasher.flash("firmware.bin", progress_callback=my_callback)

UART handshake Protocol:
    RPi -> STM32 : "UPDATE\n"
    STM32 -> RPi : "READY\r\n"
    RPi -> STM32 : [4B: total_size (little-endian)]
    STM32 -> RPi : "ACK\r\n"
    (Flash Erase)
    STM32 -> RPi : "ACK\r\n";
    RPi -> STM32 : [256B: chunk 0]
    STM32 -> RPi : "ACK\r\n"
    위 반복
    RPi -> STM32: [4B: CRC32 (little-endian)]
    STM32 -> RPi: "DONE\r\n" 또는 "NACK\r\n"
"""

import serial
import struct
import zlib
import time
import sys
import os

#설정 상수
CHUNK_SIZE = 256 #부트로더와 약속한 청크 크기
BAUD_RATE = 115200 #부트로더 UART 설정 맞추기
READY_TIMEOUT = 10 #"READY" 대기(초) - 리셋 후 부트로더 진입 시간 포함
ERASE_TIMEOUT = 30 #Flash Erase 대기 (초) - Sector 1~7 삭제는 시간 걸림
ACK_TIMEOUT = 5 #일반 ACK 대기 (초)
UPDATE_SIGNAL_RETRIES = 5 #UPDATE\n" 재전송 횟수

#GPIO 이용한 하드웨어 리셋 (선택사항이긴 한데 해보자. 점퍼선 하나 추가하면 됨)
NRST_GPIO_PIN = 17 #점퍼선으로 핀 잘 보고 연결하자.

class OTAFlasher:
    """STM32 부트로더와 UART 통신하여 펌웨어를 Flash하는 클래스."""

    def __init__(self, port="/dev/ttyACM0", baudrate=BAUD_RATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    #Public API
    def flash(self, bin_path, progress_callback=None):
        """
        펌웨어 .bin 파일을 stm32에 전송.

        Args:
            bin_path: .bin파일 경로
            progress_callback: 진행률 콜백 함수
                호출 형태: callback(percent, message)
                percent: 0~100정수
                message: 현재 상태 문자열

        Returns:
            True, Fasle

        Raises:
            FileNotFoundError
            OTAError: 통신 오류 발생
        """
        # 0.파일 검증
        if not os.path.exists(bin_path):
            raise FileNotFoundError(f"펌웨어 파일 없음: {bin_path}")
        
        firmware = open(bin_path, "rb").read()
        total_size = len(firmware)

        if total_size == 0 or total_size > (496 * 1024):
            raise OTAError(f"잘못된 파일 크기: {total_size} bytes (최대 496KB)")
        
        self._log(progress_callback, 0, f"펌웨어 로드 완료: {total_size} bytes")

        try:
            # 1. 시리얼 포트 열기
            self._open_serial()
            self._log(progress_callback, 2, f"시리얼 포트 열림: {self.port}")

            # 2. (선택임 없애도됨) 하드웨어 리셋
            if NRST_GPIO_PIN is not None:
                self._hardware_reset()
                self._log(progress_callback, 5, "STM32 하드웨어 리셋 완료")
            else:
                self._log(progress_callback, 5, "수동 리셋 대기 (Stm32 리셋 버튼 누르면 됨)")

            # 3. "UPDATE\n" 전송 -> "READY" 대기
            self._send_update_signal()
            self._log(progress_callback, 10, "부트로더 연결 완료 (READY수신)")

            # 4. 파일 크기 전송 -> ACK
            self._send_file_size(total_size)
            self._log(progress_callback, 12, f"파일 크기 전송 완료: {total_size} bytes")

            # 5. Flash Erase 대기 -> ACK
            self._wait_for_erase()
            self._log(progress_callback, 20, "Flash Erase 완")

            # 6. 청크 전송
            self._send_chunks(firmware, total_size, progress_callback)

            # 7. CRC전송 -> DONE 확인
            crc = zlib.crc32(firmware) & 0xFFFFFFFF
            self._send_crc(crc)
            self._log(progress_callback, 100, "펌웨어 업데이트 성공. stm32 재부팅 중")

            return True
        
        except OTAError as e:
            self._log(progress_callback, -1, f"OTA실패: {e}")
            return False

        finally:
            self._close_serial()

    
    # 내부 통신 함수
    def _open_serial(self):
        """시리얼 포트 열기"""
        try: 
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=5
            )
            #버퍼 비우기
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(0.1)
        except serial.SerialException as e:
            raise OTAError(f"시리얼 포트 열기 실패: {e}")
        
    def _close_serial(self):
        """시리얼 포트 닫기"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None

    def _hardware_reset(self):
        """GPIO를 이용한 STM32 하드웨어 리셋 (NRST 핀 LOW->HIGH).
        """
        #이 부분이 좀 어려움.
        # try:
        #     import RPi.GPIO as GPIO
        #     GPIO.setmode(GPIO.BCM)
        #     GPIO.setup(NRST_GPIO_PIN, GPIO.OUT)

        #     #NRST를 LOW로 -> 리셋
        #     GPIO.output(NRST_GPIO_PIN, GPIO.LOW)
        #     time.sleep(0.1)

        #     #NRST를 HIGH로 -> 부트로더 시작
        #     GPIO.output(NRST_GPIO_PIN, GPIO.HIGH)
        #     time.sleep(0.5) #부트로더 초기화 대기

        #     #********************
        #     #추가: GPIO 해제 (다른 프로세스와 충돌 방지)
        #     GPIO.cleanup(NRST_GPIO_PIN)

        # except ImportError:
        #     raise OTAError("RPi.GPIO 모듈 없음. 하드웨어 리셋 불가")
        """subprocess로 STM32 하드웨어 리셋 (eventlet 충돌 회피)"""
        import subprocess
        subprocess.run([
            'python3', '-c',
            'import RPi.GPIO as GPIO; import time; '
            'GPIO.setmode(GPIO.BCM); '
            'GPIO.setup(17, GPIO.OUT); '
            'GPIO.output(17, GPIO.LOW); '
            'time.sleep(0.1); '
            'GPIO.output(17, GPIO.HIGH); '
            'time.sleep(0.5); '
            'GPIO.cleanup(17)'
        ], timeout=5)

    def _send_update_signal(self):
        """'UPDATE\\n' 전송하고 'READY' 응답 대기"""
        for attempt in range(UPDATE_SIGNAL_RETRIES):
            #버퍼 비우기
            self.ser.reset_input_buffer()

            #"UPDATE\n" 전송
            self.ser.write(b"UPDATE\n")
            self.ser.flush()

            #"READY" 대기
            response = self._read_line(READY_TIMEOUT)
            if response and "READY" in response:
                return
        
            print(f" 재시도 {attempt + 1}/{UPDATE_SIGNAL_RETRIES}..")
            time.sleep(1)

        raise OTAError(
            f"부트로더 응답 없음 ({UPDATE_SIGNAL_RETRIES}ghl tleh)"
            "STM32가 부트로더 모드인지 확인하기"
        )
    
    def _send_file_size(self, total_size):
        """파일 크기 전송 (4바이트, little-endian) -> ACK 대기"""
        self.ser.write(struct.pack("<I", total_size))
        self.ser.flush()

        response = self._read_line(ACK_TIMEOUT)
        if not response or "ACK" not in response:
            if response and "ERR" in response:
                raise OTAError(f"파일 크기 거부: {response}")
            raise OTAError(f"파일 크기 ACK실패. 응답: {response}")
        

    def _wait_for_erase(self):
        """Flash Erase 완료 대기 (Sector 1~7 삭제). 여기서 오래 걸릴 수 있음"""
        response = self._read_line(ERASE_TIMEOUT)
        if not response or "ACK" not in response:
            if response and "ERR" in response:
                raise OTAError(f"Flash Erase 실패: {response}")
            raise OTAError(f"Flash Erase 타임아웃 ({ERASE_TIMEOUT})")
        
    def _send_chunks(self, firmware, total_size, progress_callback):
        """256바이트 청크 단위로 펌웨어 전송."""
        sent = 0
        chunk_count = (total_size + CHUNK_SIZE - 1) // CHUNK_SIZE

        for i in range(chunk_count):
            #청크 추출
            offset = i * CHUNK_SIZE
            chunk = firmware[offset:offset + CHUNK_SIZE]

            #마지막 청크가 256바이트 미만이면 0xFF로 패딩
            if len(chunk) < CHUNK_SIZE:
                chunk = chunk + b'\xFF' * (CHUNK_SIZE - len(chunk))

            #전송
            self.ser.write(chunk)
            self.ser.flush()

            #ACK 대기
            response = self._read_line(ACK_TIMEOUT)
            if not response or "ACK" not in response:
                if response and "ERR" in response:
                    raise OTAError(f"청크 {i} 쓰기 실패: {response}")
                raise OTAError(f"청크 {i} ACK 타임아웃. 응답: {response}")
            
            #진행률 계산 (20~95% 구간 매핑)
            sent += min(CHUNK_SIZE, total_size-offset)
            percent = 20 + int((sent/total_size) * 75)
            self._log(progress_callback, percent, f"전송 중: {sent}/{total_size} bytes ({i+1}/{chunk_count} chunks)")

    
    def _send_crc(self, crc):
        """CRC32전송 -> DONE/NACK 확인"""
        self.ser.write(struct.pack("<I", crc))
        self.ser.flush()

        response = self._read_line(ACK_TIMEOUT)
        if not response:
            raise OTAError("CRC 응답 타임아웃")
        
        if "DONE" in response:
            return #성공
        
        if "NACK" in response:
            raise OTAError(
                f"CRC불일치 (전송: 0x{crc:08X})"
                "stm32가 재부팅됨. 다시 시도하자"
            )
        
        raise OTAError(f"예상 못한 CRC 응답: {response}")
    

#######################
    # 유틸리티
    ###################
    def _read_line(self, timeout):
        """
        시리얼에서 한 줄 읽기 (\\r\\n 또는 \\n 종료)
        타임아웃 내에 읽지 못하면 None 반환.
        """
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout
        
        try:
            raw = self.ser.readline()
            if raw:
                return raw.decode("utf-8", errors="ignore").strip()
            return None
        except serial.SerialException as e:
            raise OTAError(f"시리얼 읽기 오류:{e}")
        finally:
            self.ser.timeout = old_timeout

    
    @staticmethod
    def _log(callback, percent, message):
        """진행률 로그. 콜백 있으면 호출, 없으면 stdout 출력"""
        print(f" [{percent:3d}%] {message}")
        if callback:
            callback(percent, message)


class OTAError(Exception):
    """OTA 업데이트 관련 예외"""
    pass

#===============================================================
# CLI 실행 (테스트 용)
#======================================
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("사용법: python3 ota_flasher.py <firmware.bin> [serial_port]")
        print("  예시: python3 ota_flasher.py parkhaejin_car.bin /dev/ttyACM0")
        sys.exit(1)

    bin_file = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"

    print("=" * 50)
    print("  Neuro-Drive OTA Flasher")
    print("=" * 50)
    print(f"  파일: {bin_file}")
    print(f"  포트: {port}")
    print()

    if NRST_GPIO_PIN is None:
        print("  ⚠️  NRST GPIO 미설정 — STM32 리셋 버튼을 눌러주세요.")
        print("  (3초 안에 부트로더가 UPDATE 신호를 기다립니다)")
        print()

    flasher = OTAFlasher(port)
    success = flasher.flash(bin_file)

    if success:
        print("\n  ✅ 펌웨어 업데이트 완료!")
    else:
        print("\n  ❌ 펌웨어 업데이트 실패.")
        sys.exit(1)