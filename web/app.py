import sys
import time
import socket
import struct  # C++ 구조체와 통신하기 위해 필요
import eventlet
import os
from eventlet.semaphore import Semaphore
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

# ==========================================
# 1. 설정 (Configuration)
# ==========================================
# 시리얼 설정 삭제됨
# C++ Core와 통신할 UDP 설정
UDP_IP = "127.0.0.1"
UDP_PORT = 5555

app = Flask(__name__, static_folder='static', template_folder='templates')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# UDP 소켓 생성 (Non-blocking)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

engine_running = False 

# ==========================================
# 2. 시스템 기능
# ==========================================
def sys_log(msg, type="INFO"):
    timestamp = time.strftime("%H:%M:%S")
    formatted_msg = f"[{timestamp}] {msg}"
    print(f"[{type}] {formatted_msg}")
    socketio.emit('system_log', {'type': type, 'log': formatted_msg})

# C++ Core로 명령 전송 (UDP)
def send_udp_command(throttle, steering):
    try:
        # C++ 구조체: struct Packet { float th; float st; };
        # Python: struct.pack('ff', ...) -> float 2개 (8바이트) 패킹
        packet = struct.pack('ff', float(throttle), float(steering))
        sock.sendto(packet, (UDP_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP Send Error: {e}")

# ==========================================
# 3. 백그라운드 태스크
# ==========================================
# read_serial_task 삭제됨 (파이썬은 더 이상 시리얼을 읽지 않음)

def status_monitor_task():
    print("[Task] Monitor Started")
    while True:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = float(f.read()) / 1000.0
            socketio.emit('status_update', {'cpu_temp': temp, 'engine': engine_running})
        except Exception:
            pass
        socketio.sleep(1.0)

# ==========================================
# 4. 라우팅 & 소켓 핸들러
# ==========================================
@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    sys_log("Client Connected", "SUCCESS")
    emit('engine_update', {'running': engine_running})

@socketio.on('toggle_engine')
def handle_engine():
    global engine_running
    engine_running = not engine_running
    status = "STARTED" if engine_running else "STOPPED"
    sys_log(f"Engine {status}", "SUCCESS" if engine_running else "WARN")
    emit('engine_update', {'running': engine_running})
    
    # 엔진 끄면 즉시 정지 명령 전송
    if not engine_running: 
        send_udp_command(0.0, 0.0)

@socketio.on('control_command')
def handle_control_command(data):
    if not engine_running: return
    try:
        # 1. 원본 데이터 받기
        raw_throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))
        
        # throttle 변수를 계산용으로 복사
        final_throttle = raw_throttle
        
        # ========================================================
        # [NEW] 코너링 기동성 확보 로직 (Joystick Geometry Fix)
        # ========================================================
        
        # (1) 조향이 절반 이상(0.5) 꺾였고, 스로틀 입력이 조금이라도 있다면?
        if abs(steering) > 0.5 and abs(raw_throttle) > 0.05:
            
            # (2) 부스트 배율도 더 과감하게 (0.8 -> 2.0)
            boost_factor = 1.0 + (abs(steering) * 2.0)
            final_throttle = raw_throttle * boost_factor
            
            # (3) ★ 핵심: "최소 기동 토크" 강제 주입
            # 조향 시 마찰을 이기기 위한 최소 PWM 비율 (0.4 = 40%)
            min_torque = 0.40 
            
            if final_throttle > 0:
                final_throttle = max(final_throttle, min_torque)
            elif final_throttle < 0:
                final_throttle = min(final_throttle, -min_torque)

        # 3. 안전장치: -1.0 ~ 1.0 범위 강제
        final_throttle = max(-1.0, min(1.0, final_throttle))

        # [디버깅] 
        print(f"[DEBUG] S: {steering:.2f} | T_Raw: {raw_throttle:.2f} -> T_Boost: {final_throttle:.2f}")

        # 4. C++ (UDP)로 전송
        send_udp_command(final_throttle, steering)

    except Exception as e:
        print(f"Control Error: {e}")

if __name__ == '__main__':
    print(f"Starting Neuro-Driver Python Server -> Sending to UDP {UDP_PORT}...")
    
    # 모니터링 태스크만 실행 (시리얼 태스크 제거됨)
    socketio.start_background_task(status_monitor_task)
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)