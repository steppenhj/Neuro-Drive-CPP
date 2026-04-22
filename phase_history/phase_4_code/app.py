import sys
import time
import socket #네트워크 통신
import struct  # C++ 구조체와 통신하기 위해 필요
import eventlet #비동기 처리 라이브러리
import os
from eventlet.semaphore import Semaphore

import serial

#표준 라이브러리를 비동기 방식에 맞게 바꾸는 거임
eventlet.monkey_patch()

from flask import Flask, render_template  #웹 서버용
from flask_socketio import SocketIO, emit #웹소켓용

# ==========================================
# 1. 설정 (Configuration)
# ==========================================
# 시리얼 설정 삭제됨
# C++ 과 통신할 UDP 설정
UDP_IP = "127.0.0.1" #라즈베리파이 내부라는 뜻 (localhost)
UDP_PORT = 5555

#웹 서버(Flask)와 웹소켓(SocketIO) 객체 생성
app = Flask(__name__, static_folder='static', template_folder='templates')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# UDP 소켓 생성 (Non-blocking)
#AF_INET: 인터넷 주소 체계 (IPv4) 쓰기
#SOCK_DGRAM: "Datagram" 방식, 즉 UDP를 사용. (TCP는 SOCK_STREAM)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 아래 변수는 index.html에 요청에 의해서 바뀜
#처음에는 false로 설정해둠(껐다는 의미). 이것도 하나의 안전장치임.
engine_running = False 

#rth watchdog문제 해결
rth_active = False

rth_mode_current = 0  # 전역 변수 추가

#2/26추가. 엔코더 받기
feedback_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
feedback_sock.bind(('127.0.0.1', 5556))
feedback_sock.settimeout(1.0)

def encoder_monitor_task():
    while True:
        try:
            data, _ = feedback_sock.recvfrom(64)
            line = data.decode('utf-8', errors='ignore').strip()
            if line.startswith("ENC:"):
                speed = int(line.split(":")[1])
                socketio.emit('encoder_update', {'speed': speed})
        except:
            pass
        socketio.sleep(0)

socketio.start_background_task(encoder_monitor_task)

#rth watchdog문제
def rth_keep_alive_task():
    while True:
        if rth_active:
            send_udp_command(0.0, 0.0, 2)
        socketio.sleep(0.1)

socketio.start_background_task(rth_keep_alive_task)

@socketio.on('rth_command')
def handle_rth(data):
    global rth_active, rth_mode_current
    mode = int(data.get('mode', 0))
    rth_mode_current = mode
    rth_active = (mode == 2)
    send_udp_command(0.0, 0.0, mode)
    emit('rth_update', {'mode': mode})

# ==========================================
# 2. 시스템 기능
# ==========================================
def sys_log(msg, type="INFO"):
    #현재 시간 구하기
    timestamp = time.strftime("%H:%M:%S")
    formatted_msg = f"[{timestamp}] {msg}"
    #터미널에 출력
    print(f"[{type}] {formatted_msg}")
    #웹화면에도 로그 띄우기 위해 전송
    socketio.emit('system_log', {'type': type, 'log': formatted_msg})

# C++ Core로 명령 전송 (UDP)
def send_udp_command(throttle, steering, mode=0):
    try:
        # C++ 구조체: struct Packet { float th; float st; };
        # Python: struct.pack('ff', ...) -> float 2개 (8바이트) 패킹
        #이진수 binary로 압축하는 걸로 보면 됨

        #***이제 PID제어 -> RTH 기능을 추가함. RTH 모드를 패킷에 추가해야 함
        packet = struct.pack('ffi', float(throttle), float(steering), int(mode))
        
        #sock.sendto: 만들어진 패킷을 5555번 포트로 휙 던진다(UDP)
        sock.sendto(packet, (UDP_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP Send Error: {e}")

# ==========================================
# 3. 백그라운드 태스크
# (이게 뭐 그냥 추가한 건데 이걸로 race condition도 생겨보고
# 좋은 경험이라고 판단해서 남겨둠)
# ==========================================
# read_serial_task 삭제됨 (파이썬은 더 이상 시리얼을 읽지 않음)

def status_monitor_task():
    print("[Task] Monitor Started")
    while True:
        try:
            #리눅스는 하드웨어 정보를 파일처럼 읽을 수 있다
            #CPU온도가 적힌 파일을 열어서 읽는다.
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = float(f.read()) / 1000.0  #1000으로 나눠서 섭씨
            
            #웹화면에 온도와 엔진 상태 보내기
            socketio.emit('status_update', {'cpu_temp': temp, 'engine': engine_running})
        except Exception:
            pass

        #1초 쉬기. (time.sleep 대신 socketio.sleep을 써야 다른 통신이 안 막힘)
        socketio.sleep(1.0)

# ==========================================
# 4. 라우팅 & 소켓 핸들러
# ==========================================
@app.route('/')
def index():
    return render_template('index.html')  #index.html파일을 보여줘라

@socketio.on('connect')
def handle_connect():
    #웹 접속하면 실행됨
    sys_log("Client Connected", "SUCCESS")
    emit('engine_update', {'running': engine_running})

@socketio.on('toggle_engine')
# index.html에서 요청 받음.
# engine_running이라는 변수를 바꿔줌.
def handle_engine():
    global engine_running
    #True -> False, False -> True로 뒤집는 토글
    engine_running = not engine_running
    status = "STARTED" if engine_running else "STOPPED" 
    sys_log(f"Engine {status}", "SUCCESS" if engine_running else "WARN")
    
    #변경된 상태를 다시 웹에 알려줌 (버튼 색 바꿔야 해서)
    emit('engine_update', {'running': engine_running})
    
    # 엔진 끄면 즉시 정지 명령 전송, C++로
    # rth모드 젤 끝에 0추가
    if not engine_running: 
        send_udp_command(0.0, 0.0, 0)



#js에서 보낸 control_command 받으면 handle_control_command 함수 실행
@socketio.on('control_command')
def handle_control_command(data):

    #엔진 변수 false이면 바로 return 해버림
    if not engine_running: return 
    if rth_active: return #rth복귀중 조이스틱 무시
    try:
        # 1. 원본 데이터 받기. JSON 데이터 ({throttle:0.5, ...})을 뜯어내기
        raw_throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))
        
        # throttle 변수를 계산용으로 복사
        final_throttle = raw_throttle
        
        # ========================================================
        # [NEW] 코너링 기동성 확보 로직 (Joystick Geometry Fix)
        # 알고리즘이다. 결국 성공. 조향 시에 속도 빨라짐.
        # 그러나 추후에 PID 제어 성공하면
        # 이건 없어질 수도 있다.
        # ========================================================
        
        # (1) 조향이 절반 이상(0.5) 꺾였고, 스로틀 입력이 조금이라도 있다면?
        if abs(steering) > 0.5 and abs(raw_throttle) > 0.05:
            
            # (2) 부스트 배율도 더 과감하게 (0.8 -> 2.0)
            boost_factor = 1.0 + (abs(steering) * 2.0)
            final_throttle = raw_throttle * boost_factor
            
            # (3) ★ "최소 기동 토크" 강제 주입
            # 조향 시 마찰을 이기기 위한 최소 PWM 비율 (0.4 = 40%)
            min_torque = 0.40 
            
            if final_throttle > 0:
                final_throttle = max(final_throttle, min_torque)
            elif final_throttle < 0:
                final_throttle = min(final_throttle, -min_torque)

        # 3. 안전장치: -1.0 ~ 1.0 범위 강제
        final_throttle = max(-1.0, min(1.0, final_throttle))

        # [디버깅] 
        # 디버깅 이유는 조향 시에 차량 속도가 너무 느려져서
        # boost 변수 값을 만들어서 조향 시에 속도 올려줄려고 확인한 것임
        # print(f"[DEBUG] S: {steering:.2f} | T_Raw: {raw_throttle:.2f} -> T_Boost: {final_throttle:.2f}")

        # 4. C++ (UDP)로 전송
        # 0추가 : RHT모드
        send_udp_command(final_throttle, steering, rth_mode_current)

    except Exception as e:
        print(f"Control Error: {e}")

if __name__ == '__main__':
    print(f"Starting Neuro-Driver Python Server -> Sending to UDP {UDP_PORT}...")
    
    # 모니터링 태스크만 실행 (시리얼 태스크 제거됨)
    #아까 정의한 온도 감시 태스크를 뒷단(background)에서 실행시킴
    socketio.start_background_task(status_monitor_task)
    
    #서버 시작 (0.0.0.0은 모든 IP에서 접속 허용)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)