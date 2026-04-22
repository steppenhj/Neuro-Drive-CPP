import sys
import time
import serial
import cv2
import eventlet
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

# ==========================================
# 2. 카메라 설정 (라즈베리파이 5 GStreamer 호환)
# ==========================================
def get_camera():
    global camera
    if camera is None:
        try:
            # [수정] 라즈베리파이 5용 GStreamer 파이프라인
            # libcamerasrc를 통해 영상을 받아와서 OpenCV가 이해하는 포맷으로 변환
            # [수정된 파이프라인] YUY2 포맷을 명시하여 ISP를 강제 구동
# [수정된 파이프라인] 카메라가 선호하는 I420 포맷 사용
            pipeline = (
                "libcamerasrc ! "
                "video/x-raw, format=I420, width=640, height=480, framerate=30/1 ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "appsink drop=true sync=false"
            )
            
            # GStreamer 백엔드로 카메라 열기
            camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not camera.isOpened():
                print("[ERROR] Camera failed to open via GStreamer.")
                camera = None
            else:
                print("[SUCCESS] Camera opened via GStreamer pipeline.")
                
        except Exception as e:
            print(f"[ERROR] Camera Init Failed: {e}")
            camera = None
            
    return camera

def generate_frames():
    while True:
        cam = get_camera()
        
        # 카메라가 없거나 연결 실패 시, 검은 화면 대신 빈 데이터 전송 (조종은 되게 함)
        if cam is None or not cam.isOpened():
            socketio.sleep(1.0) # 1초 대기 후 재시도
            continue

        success, frame = cam.read()
        if not success:
            socketio.sleep(0.1)
            continue
        
        # 이미지 압축 (화질 70%)
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        eventlet.sleep(0.01)

# ==========================================
# 3. 시스템 기능
# ==========================================
def sys_log(msg, type="INFO"):
    timestamp = time.strftime("%H:%M:%S")
    formatted_msg = f"[{timestamp}] {msg}"
    print(f"[{type}] {formatted_msg}")
    socketio.emit('system_log', {'type': type, 'log': formatted_msg})

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

@socketio.on('control_command')
def handle_control_command(data):
    if not engine_running: return
    try:
        throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))
        
        # ▼▼▼ [추가된 부분] 코너링 부스터 (Cornering Booster) ▼▼▼
        # 설명: 조향각(steering)이 클수록 throttle을 증폭시킵니다.
        # steering 절대값이 1.0(최대)이면 힘을 1.8배까지 올림 (7.4V 전압 부족 보상)
        if abs(steering) > 0.1:
            boost_factor = 1.0 + (abs(steering) * 0.8) # 최대 1.8배
            throttle = throttle * boost_factor
        # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

        pwm_speed = int(throttle * 999)
        
        # 하드웨어 한계인 999를 넘지 않도록 안전장치 (Clipping)
        pwm_speed = max(-999, min(999, pwm_speed))

        pwm_angle = int(CENTER_PWM + (steering * STEERING_FACTOR))
        # 펌웨어에서 제한하겠지만 여기서도 한 번 더 안전장치
        pwm_angle = max(600, min(2400, pwm_angle)) 
        
        send_to_stm32(pwm_speed, pwm_angle)
    except Exception:
        pass

# ==========================================
# 6. 메인 실행
# ==========================================
if __name__ == '__main__':
    print("Starting Neuro-Driver Async Server with GStreamer...")
    if connect_serial():
        socketio.start_background_task(read_serial_task)
        socketio.start_background_task(status_monitor_task)
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)