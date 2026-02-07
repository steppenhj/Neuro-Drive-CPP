import sys
import time
import serial
import cv2
import eventlet
import os
import datetime
import csv
from eventlet.semaphore import Semaphore
eventlet.monkey_patch()

from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit

serial_lock = Semaphore(1)

# ==========================================
# 1. 설정 (Configuration)
# ==========================================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
STEERING_FACTOR = 900
CENTER_PWM = 1500   

app = Flask(__name__, static_folder='static', template_folder='templates')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

stm32_serial = None
engine_running = False 
camera = None

# [NEW] 데이터 수집 관련 변수
recording = False
base_dir = "captured_data"      # 최상위 폴더
current_session_dir = None      # 현재 시도(Run) 저장 폴더
csv_file = None
csv_writer = None
current_throttle = 0.0 
current_steering = 0.0

# 기본 폴더 생성
if not os.path.exists(base_dir):
    os.makedirs(base_dir)

# ==========================================
# 2. 카메라 및 녹화 (폴더 분리 로직 적용)
# ==========================================
def get_camera():
    global camera
    if camera is None:
        try:
            # GStreamer 파이프라인 (YUY2 -> BGR)
            pipeline = (
                "libcamerasrc ! "
                "video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "appsink drop=true sync=false"
            )
            camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if camera.isOpened():
                print("[INFO] Camera Connected via GStreamer")
            else:
                print("[ERROR] GStreamer Pipeline Failed to Open")
                camera = None
        except Exception as e:
            print(f"[ERROR] Camera Init Exception: {e}")
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
        
        # [NEW] 녹화 로직: 현재 세션 폴더에 저장
        if recording and csv_writer and current_session_dir:
            try:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                img_filename = f"img_{timestamp}.jpg"
                
                # [중요] 이미지를 '현재 세션 폴더' 안에 저장
                full_path = os.path.join(current_session_dir, img_filename)
                
                cv2.imwrite(full_path, frame)
                
                # CSV에는 파일명만 기록 (같은 폴더에 있으므로)
                csv_writer.writerow([img_filename, current_steering, current_throttle])
                
                socketio.sleep(0.1) 
            except Exception as e:
                print(f"[ERROR] Save Failed: {e}")

        try:
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception as e:
            pass
            
        eventlet.sleep(0.01)

# ==========================================
# 3. 시스템 기능
# ==========================================
def sys_log(msg, type="INFO"):
    timestamp = time.strftime("%H:%M:%S")
    formatted_msg = f"[{timestamp}] {msg}"
    print(f"[{type}] {formatted_msg}")
    socketio.emit('system_log', {'type': type, 'log': formatted_msg})
    # 받아주고

def connect_serial():
    global stm32_serial
    try:
        stm32_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        stm32_serial.flushInput()
        sys_log(f"Connected to STM32 on {SERIAL_PORT}", "SUCCESS")
        return True
    except Exception as e:
        sys_log(f"Serial Connection Failed: {e}", "ERROR")
        return False

def send_to_stm32(speed, angle):
    with serial_lock: 
        if stm32_serial and stm32_serial.is_open:
            try:
                command = f"{speed},{angle}\n"
                stm32_serial.write(command.encode())
            except Exception as e:
                print(f"Serial Write Error: {e}")

# ==========================================
# 4. 백그라운드 태스크
# ==========================================
def read_serial_task():
    global stm32_serial
    print("[Task] Serial Listener Started")
    while True:
        if stm32_serial and stm32_serial.is_open:
            try:
                if stm32_serial.in_waiting > 0:
                    try:
                        line = stm32_serial.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("ENC:"):
                            val_str = line.split(":")[1]
                            socketio.emit('encoder_update', {'speed': int(val_str)})
                    except ValueError:
                        pass
            except Exception as e:
                print(f"Serial Read Error: {e}")
        socketio.sleep(0.01)

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
# 5. 라우팅 & 소켓 핸들러
# ==========================================
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
    if not engine_running: send_to_stm32(0, CENTER_PWM)

# [NEW] 녹화 토글 핸들러 (폴더 생성 로직 포함)
@socketio.on('toggle_recording')
def handle_recording():
    global recording, csv_file, csv_writer, current_session_dir
    recording = not recording
    
    if recording:
        # 1. 현재 시간으로 폴더 이름 생성 (예: 20260128_230010)
        session_name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        current_session_dir = os.path.join(base_dir, session_name)
        
        # 2. 폴더 생성
        if not os.path.exists(current_session_dir):
            os.makedirs(current_session_dir)
            
        # 3. 해당 폴더 안에 로그 파일 생성
        log_path = os.path.join(current_session_dir, "data_log.csv")
        csv_file = open(log_path, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['image', 'steering', 'throttle'])
        
        sys_log(f"Recording Session: {session_name}", "SAVE")
    else:
        # 녹화 종료
        if csv_file:
            csv_file.close()
            csv_file = None
            csv_writer = None
        current_session_dir = None # 세션 초기화
        sys_log("Recording Stopped", "INFO")
        
    emit('recording_update', {'recording': recording})

@socketio.on('control_command')
def handle_control_command(data):
    global current_throttle, current_steering
    
    if not engine_running: return
    try:
        throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))
        
        current_throttle = throttle
        current_steering = steering
        
        if abs(steering) > 0.1:
            boost_factor = 1.0 + (abs(steering) * 0.8)
            throttle = throttle * boost_factor
        
        pwm_speed = int(throttle * 999)
        pwm_speed = max(-999, min(999, pwm_speed))

        pwm_angle = int(CENTER_PWM + (steering * STEERING_FACTOR))
        pwm_angle = max(600, min(2400, pwm_angle))
        
        send_to_stm32(pwm_speed, pwm_angle)
    except Exception:
        pass

if __name__ == '__main__':
    print("Starting Neuro-Driver Async Server with Session Recording...")
    if connect_serial():
        socketio.start_background_task(read_serial_task)
        socketio.start_background_task(status_monitor_task)
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)