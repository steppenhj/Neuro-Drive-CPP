# 자 여기에서는 이제 데이터를 받아서 cpp로 보내기만 하면 된다.
# 계산 할 필요 없고. UDP만 잘 짜보자.

import sys
import time
import socket #serial 대신에 socket
import struct #cpp로 보낼 데이터 패킹용
import cv2
import eventlet
import os
import datetime
import csv
from flask import Flask, render_template, Response
from flask_socketio import SocektIO, emit

eventlet.monkey_patch()

# 1. 설정 configuration  로컬 주소 등등 막 뭐 많다.
CPP_IP = "127.0.0.1"
CPP_PORT = 5555

app = Flask(__name__, static_folder='static', template_folder='templates')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

#시리얼 객체 제거
engine_running = False 
camera = None

# UDP 소켓 초기화
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#데이터 수집 관련 변수
recording = False
base_dir = "captured_data"
current_session_dir = None
csv_file = None
csv_writer = None
current_throttle = 0.0
current_steering = 0.0

if not os.path.exists(base_dir):
    os.makedirs(base_dir)

# 2. 카메라 및 녹화
def get_camera():
    global camera
    if camera is None:
        try:
            pipeline = (
                "libcamerasrc ! "
                "video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "appsink drop=true sync=false"
            )
            camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        except Exception:
            camera = None
    return camera

def generate_frames():
    global recording, csv_writer, current_throttle, current_steering, camera, current_session_dir
    while True:
        cam = get_camera()
        if cam is None or not cam.isOpened():
            socketio.sleep(1.0)
            if cam is not None: cam.release()
            camera = None
            continue

        success, frame = cam.read()
        if not success:
            socketio.sleep(0.1)
            continue

        #녹화 로직
        if recording and csv_writer and current_session_dir:
            try:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%h%M%S_%f")
                img_filename = f"img_{timestamp}.jpg"
                full_path = os.path.join(current_session_dir, img_filename)
                cv2.imwrite(full_path, frame)
                csv_writer.writerow([img_filename, current_steering, current_throttle])
            except Exception:
                pass

        try:
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ret:
                yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except Exception:
            pass
        eventlet.sleep(0.01)


#3. 시스템 기능 (로그만 유지, 시리얼 연결 제거)
def sys_log(msg, type="INFO"):
    timestamp = time.strftime("%H:%M:%S")
    formatted_msg = f"[{timestamp}] {msg}"
    print(f"[{type}] {formatted_msg}")
    socketio.emit('system_log', {'type': type, 'log': formatted_msg})

# connect_serial(), send_to_stm32() 함수는 삭제
# C++로 데이터 전송 함수 추가
def send_to_cpp(throttle, steering):
    try:
        #float 2ro (4byte * 2 = 8byte) 패킹
        packet = struct.pack('ff', float(throttle), float(steering))
        udp_sock.sendto(packet, (CPP_IP, CPP_PORT))
    except Exception as e:
        print(f"UDP Error: {e}")

#4. 백그라운드 태스크 (시리얼 수신 제거)
# 제거 - read_serial_task() -> cpp
def status_monitor_task():
    while True:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = float(f.read()) / 1000.0
            socketio.emit('status_update', {'cpu_temp': temp, 'engine': engine_running})
        except Exception:
            pass
        socketio.sleep(1.0)

#5. 라우팅 & 소켓 핸들러
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def handle_connect():
    sys_log("Client Connected", "SUCCESS")
    emit('engine_update', {'running': engine_running})

@socketio.on('ping_event')
def handle_ping(data):
    emit('pong_event', data)

@socketio.on('toggle_engine')
def handle_engine():
    global engine_running
    engine_running = not engine_running
    status = "STARTED" if engine_running else "STOPPED"
    sys_log(f"Engine {status}", "SUCCESS" if engine_running else "WARN")
    emit('engine_update', {'running': engine_running})

    #엔진 끄면 C++에게 (0, 0) 전송
    if not engine_running:
        send_to_cpp(0.0, 0.0)

@socketio.on('toggle_recording')
def handle_recording():
    global recording, csv_file, csv_writer, current_session_dir
    recording = not recording

    if recording:
        session_name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        current_session_dir = os.path.join(base_dir, session_name)
        if not os.path.exists(current_session_dir):
            os.makedirs(current_session_dir)
        log_path = os.path.join(current_session_dir, "data_log.csv")
        csv_file = open(log_path, 'w', newline'')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['image', 'steering', 'throttle'])
        sys_log(f"Recording Session: {session_name}", "SAVE")
    else:
        if csv_file:
            csv_file.close()
            csv_file = None
            csv_writer = None
        current_steering = None
        sys_log("Recording Stopped", "INFO")

    emit('recording_update', {'recording': recording})

@socketio.on('control_command')
def handle_control_command(data):
    global current_throttle, current_steering

    if not engine_running: return

    # 계산 로직 삭제 -> Raw 데이터 파싱만 진행
    try:
        throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))

        current_throttle = throttle
        current_steering = steering

        #cpp로 토스 (계산을 cpp에서)
        send_to_cpp(throttle, steering)

    except Exception:
        pass

if __name__ == '__main__':
    print("Starting Neuro-Drive Gateway (Python)...")
    socketio.start_background_task(status_monitor_task)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)