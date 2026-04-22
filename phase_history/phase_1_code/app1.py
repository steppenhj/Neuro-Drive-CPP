import eventlet
eventlet.monkey_patch()

import socket
import struct
import subprocess
import os
import signal
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'

socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

UDP_IP = "127.0.0.1"
UDP_PORT = 8080
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# [NEW] C++ 프로세스를 담을 전역 변수
cpp_process = None

car_state = {
    'speed': 0.0,
    'steering': 0.0,
    'cpu_temp': 0.0,
    'engine_status': False  # 엔진 상태 (True=켜짐)
}

def get_cpu_temp():
    try:
        temp = subprocess.check_output("vcgencmd measure_temp", shell=True)
        return float(temp.decode("utf-8").replace("temp=", "").replace("'C\n", ""))
    except:
        return 0.0

def sys_log(message, type="INFO"):
    log_data = f"[{type}] {message}"
    print(log_data)
    socketio.emit('system_log', {'log': log_data, 'type': type})

def telemetry_loop():
    while True:
        car_state['cpu_temp'] = get_cpu_temp()
        # 엔진 상태도 같이 방송
        car_state['engine_status'] = (cpp_process is not None)
        socketio.emit('car_telemetry', car_state)
        socketio.sleep(1)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    sys_log(f"Client Connected: {request.remote_addr}", "SUCCESS")
    # 접속하자마자 현재 엔진 상태 알려줌
    emit('engine_update', {'running': (cpp_process is not None)})

# [NEW] 엔진 시동 토글 (켜기/끄기)
@socketio.on('toggle_engine')
def handle_toggle_engine():
    global cpp_process
    
    if cpp_process is None:
        try:
            # [PRO 방식] 현재 app.py 파일의 위치를 기준으로 drive_server 경로를 정확히 찾습니다.
            # 가정: app.py는 web/ 폴더에 있고, drive_server는 그 상위 폴더에 있음
            current_dir = os.path.dirname(os.path.abspath(__file__))
            exe_path = os.path.join(current_dir, "..", "drive_server") # 상위 폴더(..)의 실행파일
            
            # 파일이 실제로 있는지 체크 (디버깅용)
            if not os.path.exists(exe_path):
                 sys_log(f"File not found: {exe_path}", "ERROR")
                 return

            # 찾은 절대 경로로 실행
            cpp_process = subprocess.Popen([exe_path]) 
            
            sys_log("C++ Drive Engine STARTED", "SUCCESS")
            emit('engine_update', {'running': True})
        except Exception as e:
            sys_log(f"Failed to start engine: {e}", "ERROR")
    else:
        # 2. 엔진 끄기 (Stop)
        try:
            # SIGINT(Ctrl+C) 신호를 보내서 깔끔하게 종료 유도
            cpp_process.send_signal(signal.SIGINT)
            cpp_process.wait(timeout=2) # 2초 대기
            cpp_process = None
            sys_log("C++ Drive Engine STOPPED", "WARN")
            emit('engine_update', {'running': False})
        except:
            # 말 안 들으면 강제 종료 (Kill)
            if cpp_process: cpp_process.kill()
            cpp_process = None
            sys_log("Engine Force Killed", "DANGER")
            emit('engine_update', {'running': False})

@socketio.on('control_command')
def handle_control_command(data):
    if cpp_process is None: return # 엔진 꺼져있으면 명령 무시

    throttle = float(data.get('throttle', 0))
    steering = float(data.get('steering', 0))
    car_state['speed'] = throttle * 20 
    send_udp_command(throttle, steering)

# [NEW] 레이턴시 측정을 위한 Ping-Pong 핸들러
@socketio.on('ping_event')
def handle_ping_event(data):
    # 클라이언트가 보낸 타임스탬프를 그대로 반사(Echo)
    emit('pong_event', data)

def send_udp_command(throttle, steering):
    payload = struct.pack('ff', throttle, steering)
    sock.sendto(payload, (UDP_IP, UDP_PORT))

if __name__ == '__main__':
    print("Starting Neuro-Drive System...")
    socketio.start_background_task(telemetry_loop)
    # 0.0.0.0으로 열어야 외부 접속 가능
    socketio.run(app, host='0.0.0.0', port=5000)