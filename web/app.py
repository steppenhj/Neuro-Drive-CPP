import sys
import time
import serial
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO

# ==========================================
# 1. 설정 (Configuration)
# ==========================================
# [중요] STM32가 연결된 포트 이름 (아까 찾으신 걸로 확인!)
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200

app = Flask(__name__, static_folder='static', template_folder='templates')
socketio = SocketIO(app, cors_allowed_origins="*")

# 시리얼 객체 (처음엔 None)
stm32_serial = None

# ==========================================
# 2. 시스템 기능 (System Functions)
# ==========================================
def sys_log(msg, type="INFO"):
    """
    서버 터미널과 웹 UI 양쪽에 로그를 남기는 함수
    """
    timestamp = time.strftime("%H:%M:%S")
    formatted_msg = f"[{timestamp}] {msg}"
    print(f"[{type}] {formatted_msg}")
    
    # 웹페이지의 로그 터미널로도 전송
    socketio.emit('system_log', {'type': type, 'log': formatted_msg})

def connect_serial():
    """
    STM32와 시리얼 연결 시도
    """
    global stm32_serial
    try:
        stm32_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        sys_log(f"Connected to STM32 on {SERIAL_PORT}", "SUCCESS")
        return True
    except Exception as e:
        sys_log(f"Serial Connection Failed: {e}", "ERROR")
        return False

# ==========================================
# 3. 플라스크 라우팅 (Web Routing)
# ==========================================
@app.route('/')
def index():
    return render_template('index.html')

# ==========================================
# 4. 소켓 통신 핸들러 (Socket Handlers)
# ==========================================
@socketio.on('connect')
def handle_connect():
    sys_log("Client Connected", "SUCCESS")
    # 연결되자마자 시리얼 포트 확인
    if stm32_serial is None or not stm32_serial.is_open:
        connect_serial()

@socketio.on('ping_event')
def handle_ping(data):
    # 레이턴시 측정을 위한 핑퐁 (index.html과 짝꿍)
    socketio.emit('pong_event', data)

@socketio.on('toggle_engine')
def handle_engine():
    # 시동 버튼 누름 (여기서는 단순 로그만, 나중에 기능 확장 가능)
    sys_log("Engine Toggle Switch Activated", "INFO")
    # 일단 켜진 것으로 응답
    socketio.emit('engine_update', {'running': True})

# ★★★ 핵심 수정 부분: 변수명을 throttle, steering으로 변경 ★★★
@socketio.on('control_command')
def handle_control_command(data):
    """
    index.html에서 보내준 {throttle, steering} 데이터를 받아서 처리
    """
    
    # 1. 데이터 추출 (안전하게 float 변환)
    try:
        throttle = float(data.get('throttle', 0))
        steering = float(data.get('steering', 0))
        
        # [디버깅 로그] 입력값이 제대로 들어오는지 눈으로 확인!
        print(f"[DEBUG] Input -> Throttle: {throttle:.2f}, Steering: {steering:.2f}")

    except ValueError:
        return

    command = None
    
    # 2. 명령 판단 로직 (Threshold: 0.3)
    # Nipple.js Vector: 위쪽(+y), 아래쪽(-y) ... 하지만 index.html에서 어떻게 오는지 로그 보고 판단 필요
    # 보통 nipple.js vector.y는 위가 양수(+)입니다.
    
    THRESHOLD = 0.3

    # (1) 주행 판단 (전진/후진 우선)
    if throttle > THRESHOLD:
        command = 'w'   # 전진
    elif throttle < -THRESHOLD:
        command = 'x'   # 후진
    
    # (2) 조향 판단 (주행 명령이 없을 때 회전 명령 처리)
    # 만약 '전진하면서 우회전' 같은 복합 명령을 하려면 STM32 코드가 더 똑똑해야 함.
    # 지금은 단순하게 주행이 없으면 조향을 체크하는 순서로 갑니다.
    elif steering < -THRESHOLD:
        command = 'a'   # 좌회전
    elif steering > THRESHOLD:
        command = 'd'   # 우회전
        
    # (3) 아무것도 안 밀었으면 정지
    else:
        command = 's'   # 정지

    # 3. STM32로 전송
    if command:
        try:
            if stm32_serial and stm32_serial.is_open:
                stm32_serial.write(command.encode())
                # 최종 전송 로그
                sys_log(f"Sent to STM32: {command} (T:{throttle:.2f}, S:{steering:.2f})", "INFO")
            else:
                # 연결 끊겨있으면 재연결 시도
                sys_log("Serial disconnected, retrying...", "WARN")
                connect_serial()
                
        except Exception as e:
            sys_log(f"Tx Error: {e}", "ERROR")

# ==========================================
# 5. 메인 실행 (Main Execution)
# ==========================================
if __name__ == '__main__':
    print("Starting Neuro-Driver Bridge Server...")
    # 시리얼 연결 시도
    connect_serial()
    # 서버 시작
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)