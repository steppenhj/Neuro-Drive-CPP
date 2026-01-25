import serial
import time

# STM32 Nucleo는 보통 ttyACM0 입니다.
# 만약 USB 허브 등을 쓴다면 ttyUSB0일 수도 있습니다. (ls /dev/tty* 로 확인)
PORT = '/dev/ttyACM1'
BAUD = 115200

print(f"Opening {PORT}...")

try:
    # dsrdtr=True 옵션은 Nucleo 보드 연결 시 리셋 문제를 방지해줍니다.
    ser = serial.Serial(PORT, BAUD, timeout=1, dsrdtr=True)
    ser.dtr = False
    time.sleep(2) # 연결 후 STM32가 안정화될 때까지 대기

    print("Sending command...")
    
    while True:
        # 이 명령을 보내면 STM32의 LED가 깜빡여야 정상입니다.
        msg = "100,1500" 
        print(f"Tx: {msg}")
        
        # 인코딩 + 개행문자(혹시 모르니) 없이 보냄 -> STM32는 DMA Idle로 끊어 읽음
        ser.write(msg.encode())
        ser.flush() # 버퍼 비우기 (즉시 전송)
        
        time.sleep(1) # 1초 간격 전송

except PermissionError:
    print(f"[Error] {PORT}에 접근 권한이 없습니다.")
    print(f"해결법: sudo chmod 666 {PORT}")
except Exception as e:
    print(f"[Error] {e}")