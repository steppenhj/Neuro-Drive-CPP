import eventlet
eventlet.monkey_patch()

import socket
import struct
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'

socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# --- C++ 통신 설정 (UDP) ---
UDP_IP = "127.0.0.1" # 로컬 호스트
UDP_PORT = 8080
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print(f"[Server] Client Connected: {request.remote_addr}")
    emit('server_status', {'msg': 'Connected to Pi'})

@socketio.on('disconnect')
def handle_disconnect():
    print("[Server] Client Disconnected")
    # 연결 끊김 시 정지 신호 전송 (안전 장치)
    send_udp_command(0.0, 0.0)

@socketio.on('control_command')
def handle_control_command(data):
    try:
        throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))
        
        # C++로 전송
        send_udp_command(throttle, steering)
        
    except Exception as e:
        print(f"Error: {e}")

def send_udp_command(throttle, steering):
    # C++의 struct ControlData { float throttle; float steering; } 에 맞춰서 바이너리 패킹
    # 'ff'는 float 2개를 의미
    payload = struct.pack('ff', throttle, steering)
    sock.sendto(payload, (UDP_IP, UDP_PORT))

@socketio.on('ping')
def handle_ping():
    return True

# 추후에 사용할 관제센터용 *** 관제 센터를 사용할 때부터 handle_control_command함수를 수정해야 함. ***
# @socketio.on('control_command')
# def handle_control_command(data):
#     try:
#         throttle = float(data.get('throttle', 0))
#         steering = float(data.get('steering', 0))
        
#         # 1. C++ 하드웨어로 전송 (기존 코드)
#         send_udp_command(throttle, steering)

#         # 2. [추가!] 관제센터(monitor.html)로 데이터 방송
#         #    broadcast=True 옵션이 중요함 (나 말고 접속한 모든 사람에게 전송)
#         emit('car_status', {'throttle': throttle, 'steering': steering}, broadcast=True)
        
#     except Exception as e:
#         print(f"Error: {e}")

if __name__ == '__main__':
    print("Starting Web Controller...")
    socketio.run(app, host='0.0.0.0', port=5000)