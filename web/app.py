import eventlet
# eventlet은 flask-socketio의 비동기 성능을 위해 권장됩니다 (pip install eventlet 필요)
eventlet.monkey_patch()

import json
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'

# async_mode='eventlet' 명시 권장 (라즈베리파이 성능 최적화)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

@app.route('/')
def index():
    return render_template('index.html')

# -- [이벤트 핸들러] --

@socketio.on('connect')
def handle_connect():
    # request 객체에서 IP 가져오기
    client_ip = request.remote_addr
    print(f"[Server] Client Connected: {client_ip}")
    emit('server_status', {'msg': 'Connected to Pi'})

@socketio.on('disconnect')
def handle_disconnect():
    print("[Server] Client Disconnected")
    # TODO: 안전 장치 - 연결 끊김 시 모터 정지 명령 필수!
    # stop_motor()

@socketio.on('control_command')
def handle_control_command(data): # 함수 이름 변경
    """
    JS 데이터: {'throttle': '0.50', 'steering': '-1.00'}
    """
    try:
        # data.get() 사용 및 오타 수정
        throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))

        # 디버깅용 (실제 주행시에는 주석 처리 권장 - I/O 부하 감소)
        print(f"[Recv] T: {throttle:.2f} | S: {steering:.2f}")

        # 여기에 하드웨어 제어 코드 추가
        # 예: send_udp_to_cpp(throttle, steering)
        
    except Exception as e:
        print(f"Error Parsing data: {e}")

@socketio.on('ping')
def handle_ping():
    # 클라이언트의 latency 측정을 위해 빈 응답이라도 반환해야 함
    return True

if __name__ == '__main__':
    print("Starting Web Controller Server...")
    # host='0.0.0.0'으로 해야 외부(스마트폰) 접속 가능
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)