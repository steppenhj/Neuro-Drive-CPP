import sys
import time
import serial
import eventlet # [중요] 비동기 라이브러리
# eventlet 버그 방지를 위한 패치
from eventlet.semaphore import Semaphore
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO, emit
# [수정 2] 전역 변수에 락(Lock) 생성
serial_lock = Semaphore(1) # <-- 추가

# ==========================================
# 1. 설정 (Configuration)
# ==========================================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# 조향 팩터 (자네 상황에 맞게 조절)
STEERING_FACTOR = 500 
CENTER_PWM = 1500   

app = Flask(__name__, static_folder='static', template_folder='templates')
# [중요] async_mode를 eventlet으로 명시하여 성능 극대화
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

stm32_serial = None
engine_running = False 

# ==========================================
# 2. 시스템 기능
# ==========================================
def sys_log(msg, type="INFO"):
    timestamp = time.strftime("%H:%M:%S")
    formatted_msg = f"[{timestamp}] {msg}"
    print(f"[{type}] {formatted_msg}")
    socketio.emit('system_log', {'type': type, 'log': formatted_msg})

def connect_serial():
    global stm32_serial
    try:
        stm32_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) # 타임아웃 단축
        stm32_serial.flushInput()
        sys_log(f"Connected to STM32 on {SERIAL_PORT}", "SUCCESS")
        return True
    except Exception as e:
        sys_log(f"Serial Connection Failed: {e}", "ERROR")
        return False

# [수정 3] send_to_stm32 함수에 락 적용
def send_to_stm32(speed, angle):
    # 락을 걸어서 동시에 여러 스레드가 쓰지 못하게 막음
    with serial_lock: 
        if stm32_serial and stm32_serial.is_open:
            try:
                command = f"{speed},{angle}\n"
                stm32_serial.write(command.encode())
            except Exception as e:
                print(f"Serial Write Error: {e}")

# ==========================================
# 3. 백그라운드 태스크 (비동기 처리)
# ==========================================

# [태스크 1] STM32 데이터 수신 전담 (엔코더)
def read_serial_task():
    global stm32_serial
    print("[Task] Serial Listener Started")
    
    while True:
        if stm32_serial and stm32_serial.is_open:
            try:
                if stm32_serial.in_waiting > 0:
                    try:
                        line = stm32_serial.readline().decode('utf-8', errors='ignore').strip()
                        
                        # [디버깅] 엔코더 값이 들어오는지 터미널로 확인
                        # print(f"Raw: {line}") 
                        
                        if line.startswith("ENC:"):
                            val_str = line.split(":")[1]
                            raw_val = int(val_str)
                            
                            # 엔코더 값만 빠르게 전송
                            socketio.emit('encoder_update', {'speed': raw_val})
                    except ValueError:
                        pass
            except Exception as e:
                print(f"Serial Read Error: {e}")
        
        socketio.sleep(0.01) # [중요] time.sleep 대신 socketio.sleep 사용

# [태스크 2] 상태 모니터링 전담 (온도, 시스템 상태) - 1초에 1번만 실행
def status_monitor_task():
    print("[Task] Monitor Started")
    while True:
        try:
            # CPU 온도 읽기
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = float(f.read()) / 1000.0
            
            # 온도 및 상태 전송
            socketio.emit('status_update', {
                'cpu_temp': temp,
                'engine': engine_running
            })
        except Exception as e:
            print(f"Monitor Error: {e}")
            
        socketio.sleep(1.0) # 1초 대기

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

@socketio.on('ping_event')
def handle_ping(data):
    # 즉시 응답 (지연 시간 체크용)
    emit('pong_event', data)

@socketio.on('toggle_engine')
def handle_engine():
    global engine_running
    engine_running = not engine_running
    
    if engine_running:
        sys_log("Engine STARTED", "SUCCESS")
        emit('engine_update', {'running': True})
    else:
        sys_log("Engine STOPPED", "WARN")
        emit('engine_update', {'running': False})
        send_to_stm32(0, CENTER_PWM)

@socketio.on('control_command')
def handle_control_command(data):
    if not engine_running:
        return

    try:
        throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))
        
        pwm_speed = int(throttle * 999)
        pwm_angle = int(CENTER_PWM + (steering * STEERING_FACTOR))
        pwm_angle = max(700, min(2300, pwm_angle))

        send_to_stm32(pwm_speed, pwm_angle)

    except Exception as e:
        print(f"[ERROR] {e}")

# ==========================================
# 5. 메인 실행
# ==========================================
if __name__ == '__main__':
    print("Starting Neuro-Driver Async Server...")
    
    if connect_serial():
        # 백그라운드 태스크 시작
        socketio.start_background_task(read_serial_task)
        socketio.start_background_task(status_monitor_task)
    
    # [중요] flask run 대신 socketio.run 사용
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)