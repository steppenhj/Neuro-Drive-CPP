import serial
import time
import threading

# 포트 설정 (아까 확인한 ttyACM0)
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flushInput()
    print(f"✅ STM32 Connected on {SERIAL_PORT}")
except Exception as e:
    print(f"❌ Connection Failed: {e}")
    exit()

# [수신 스레드] 엔코더 값 확인용
def read_serial():
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"   [STM32 Says] {line}") # 엔코더 값 출력
        except:
            pass
        time.sleep(0.01)

# 스레드 시작
t = threading.Thread(target=read_serial)
t.daemon = True
t.start()

print("------------------------------------------------")
print("   🚀 NEURO-DRIVER HARDWARE DIAGNOSTIC TOOL 🚀   ")
print("------------------------------------------------")
print("1. 엔코더 값을 눈으로 확인하세요. (바퀴를 손으로 돌려보세요)")
print("2. 3초 뒤에 모터를 살짝 돌려봅니다. (Test Run)")
print("------------------------------------------------")

time.sleep(3)

try:
    # 1단계: 살짝 전진 (속도 300)
    print("\n[Command] Forward Speed 300 (GO!)")
    for i in range(10): # 1초 동안 명령 전송 (Failsafe 방지)
        cmd = "300,1500\n" # 속도 300, 조향 중립
        ser.write(cmd.encode())
        time.sleep(0.1)

    # 2단계: 정지
    print("[Command] STOP")
    ser.write("0,1500\n".encode())
    time.sleep(1)

    # 3단계: 살짝 후진 (속도 -300)
    print("[Command] Backward Speed -300 (BACK!)")
    for i in range(10): 
        cmd = "-300,1500\n"
        ser.write(cmd.encode())
        time.sleep(0.1)

    # 4단계: 최종 정지
    print("[Command] STOP & FINISH")
    ser.write("0,1500\n".encode())

except KeyboardInterrupt:
    print("\nTesting Aborted.")
    ser.write("0,1500\n".encode())

print("------------------------------------------------")
print("테스트 종료. 로그를 분석하세요.")