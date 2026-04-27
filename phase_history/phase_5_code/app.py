import sys
import time
import socket #네트워크 통신
import struct  # C++ 구조체와 통신하기 위해 필요
import eventlet #비동기 처리 라이브러리
import os
import subprocess #phase5의 OTA 기능 위해서 추가함
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
#OTA 객체 추가
app = Flask(__name__, static_folder='static', template_folder='templates')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet', max_http_buffer_size=1024*1024)

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

# OTA 상태 변수
ota_in_progress = False

#2/26추가. 엔코더 받기
feedback_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
feedback_sock.bind(('127.0.0.1', 5556))
feedback_sock.settimeout(1.0)

def encoder_monitor_task():
    while True:
        try:
            data, _ = feedback_sock.recvfrom(64)
            line = data.decode('utf-8', errors='ignore').strip()

            #Ping-Pong추가
            if line.startswith("PONG"):
                print("[디버깅-핑퐁] Pong received in app.py") #여기에서 수신 안되는 가능성이 좀 있다ㅏ.
                socketio.emit('pong_response')
                continue

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

#******************
# Phase 6 전 Ping-Pong추가
@socketio.on('ping_request')
def handle_ping():
    send_udp_command(0.0, 0.0, 99) #mode=99가 ping 신호

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
        #*******************************************
        # 4/23 드디어 원인을 알았고, 이 코드는 없애도 된다
        # PID의 단위 불일치가 원인이었고, 
        # Firmware 수정할 예정이다.
        #PID는 어느 정도 했는데 이거는 일단 놔두는 게 좋다고 판단함
        # ========================================================
        # (1) 조향이 절반 이상(0.5) 꺾였고, 스로틀 입력이 조금이라도 있다면?
        # if abs(steering) > 0.5 and abs(raw_throttle) > 0.05:
        #     # (2) 부스트 배율도 더 과감하게 (0.8 -> 2.0)
        #     boost_factor = 1.0 + (abs(steering) * 2.0)
        #     final_throttle = raw_throttle * boost_factor
        #     # (3) ★ "최소 기동 토크" 강제 주입
        #     # 조향 시 마찰을 이기기 위한 최소 PWM 비율 (0.4 = 40%)
        #     min_torque = 0.40 
        #     if final_throttle > 0:
        #         final_throttle = max(final_throttle, min_torque)
        #     elif final_throttle < 0:
        #         final_throttle = min(final_throttle, -min_torque)
        # # 3. 안전장치: -1.0 ~ 1.0 범위 강제
        # final_throttle = max(-1.0, min(1.0, final_throttle))
        #**********4/23*************
        #*******boost는 없앤다, PID로
        final_throttle = raw_throttle
        final_throttle = max(-1.0, min(1.0, final_throttle)) #안전 클리핑은 유지
        # [디버깅] 
        # 디버깅 이유는 조향 시에 차량 속도가 너무 느려져서
        # boost 변수 값을 만들어서 조향 시에 속도 올려줄려고 확인한 것임
        # print(f"[DEBUG] S: {steering:.2f} | T_Raw: {raw_throttle:.2f} -> T_Boost: {final_throttle:.2f}")

        # 4. C++ (UDP)로 전송
        # 0추가 : RHT모드
        send_udp_command(final_throttle, steering, rth_mode_current)

    except Exception as e:
        print(f"Control Error: {e}")

#=================================================================
# 5. OTA 펌웨어 업데이트 핸들러
# ===================================================================
@socketio.on('firmware_upload')
def handle_firmware_upload(data):
    """
    브라우저에서 .bin 파일을 받아 OTA 업데이트 실행.

    흐름:
    1. 엔진 정지 + 조이스틱 차단
    2. C++ 프로세스 종료 (UART 포트 해제)
    3. ota_flasher.py로 펌웨어 전송
    4. 완료 후 C++ 재시작

    OTA 성공하긴 했는데 잠시 수정이 필요함. 
    현재 즉각적인 업데이트가 안되고 있음.
    """
    global ota_in_progress, engine_running

    #이미 OTA 진행 중이면 무시
    if ota_in_progress:
        emit('ota_status', {'percent': -1, 'message': 'OTA가 이미 진행 중임'})
        return

    ota_in_progress = True
    engine_running = False
    emit('engine_update', {'running': False})

    #백그라운드 태스크로 실행 (이벤트 루프 안 막힘)
    socketio.start_background_task(run_ota, data)

    # try:
    #     # 1. 바이너리 데이터 추출
    #     bin_data = data.get('firmware')
    #     filename = data.get('filename', 'firmware.bin')

    #     if not bin_data:
    #         emit('ota_status', {'percent': -1, 'message': '파일 데이터 없음'})
    #         return 

    #     # 2. 임시 파일로 저장
    #     save_path = os.path.join(os.path.dirname(__file__), f'_temp_{filename}')
    #     with open(save_path, 'wb') as f:
    #         f.write(bin_data)

    #     file_size = os.path.getsize(save_path)
    #     emit('ota_status', {'percent': 0, 'message': f'파일 수신 완료: {file_size} bytes'})
    #     sys_log(f"OTA 시작: {filename} ({file_size} bytes)", "WARN")

    #     # 3. C++ 프로세스 종료 (UART 포트 해제를 위함)
    #     emit('ota_status', {'percent': 1, 'message': 'C++ 제어 프로세스 종료중'})
    #     subprocess.run(['pkill', '-f', 'control_core'], capture_output=True)
    #     subprocess.run(['pkill', '-f', 'drive_server'], capture_output=True)
    #     time.sleep(1) #프로세스 종료 + 시리얼 포트 해제 대기

    #     #4. OTA Flasher 실행 -> 이게 좀 좋은 것 같다
    #     from ota_flasher import OTAFlasher, OTAError

    #     def progress_callback(percent, message):
    #         """진행률을 WebSocekt으로 브라우저에 전달"""
    #         socketio.emit('ota_status', {'percent': percent, 'message': message})

    #     flasher = OTAFlasher("/dev/ttyACM0")
    #     success = flasher.flash(save_path, progress_callback=progress_callback)

    #     #5. 임시 파일 삭제
    #     if os.path.exists(save_path):
    #         os.remove(save_path)

    #     if success:
    #         # 6. C++ 프로세스 재시작
    #         emit('ota_status', {'percent': 100, 'message': '펌웨어 업데이트 완료, 시스템 재시작 시작'})
    #         sys_log("OTA성공, C++ 재시작 시작", "SUCCESS")

    #         time.sleep(3) #STM32 재부팅 대기(부트로더 3초 +App점프)

    #         #C++ 백그라운드로 재시작
    #         cpp_path = os.path.join(os.path.dirname(__file__), '..', 'drive_server')
    #         if os.path.exists(cpp_path):
    #             subprocess.Popen([cpp_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #             sys_log("C++ 제어 프로세스 재시작 완료", 'SUCCESS')
    #         else:
    #             sys_log(f"C++ 실행 파일 없음: {cpp_path}", "WARN")

    #         socketio.emit('ota_status', {'percent': 100, 'message': '시스템 준비 완료. START 버튼 누르면 됨'})
    #     else:
    #         sys_log("OTA 실패", "ERROR")
    #         socketio.emit('ota_status', {'percent': -1, 'message': 'OTA 실패임. 다시 시도해보자'})

    # except Exception as e:
    #     sys_log(f"OTA오류: {e}", "ERROR")
    #     socketio.emit('ota_status', {'percent': -1, 'message': f'OTA 오류: {e}'})

    # finally:
    #     ota_in_progress = False

#기존 하나의 큰 함수를 두 개로 쪼개기. 
#run_dta가 백그라운드에서 처리
def run_ota(data):
    """OTA를 백그라운드에서 실행"""
    global ota_in_progress

    try:
        bin_data = data.get('firmware')
        filename = data.get('filename', 'firmware.bin')

        if not bin_data:
            socketio.emit('ota_status', {'percent': -1, 'message': '파일 데이터 없음'})
            return

        save_path = os.path.join(os.path.dirname(__file__), f'_temp_{filename}')
        with open(save_path, 'wb') as f:
            f.write(bin_data)

        file_size = os.path.getsize(save_path)
        socketio.emit('ota_status', {'percent': 0, 'message': f'파일 수신 완료: {file_size} bytes'})
        sys_log(f"OTA 시작: {filename} ({file_size} bytes)", "WARN")

        socketio.emit('ota_status', {'percent': 1, 'message': 'C++ 제어 프로세스 종료중'})
        subprocess.run(['pkill', '-f', 'control_core'], capture_output=True)
        subprocess.run(['pkill', '-f', 'drive_server'], capture_output=True)
        time.sleep(1)

        from ota_flasher import OTAFlasher, OTAError

        def progress_callback(percent, message):
            socketio.emit('ota_status', {'percent': percent, 'message': message})

        flasher = OTAFlasher("/dev/ttyACM0")
        success = flasher.flash(save_path, progress_callback=progress_callback)

        if os.path.exists(save_path):
            os.remove(save_path)

        if success:
            socketio.emit('ota_status', {'percent': 100, 'message': '펌웨어 업데이트 완료, 시스템 재시작 시작'})
            sys_log("OTA성공, C++ 재시작 시작", "SUCCESS")
            time.sleep(3)

            cpp_path = os.path.join(os.path.dirname(__file__), '..', 'drive_server')
            if os.path.exists(cpp_path):
                subprocess.Popen([cpp_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                sys_log("C++ 제어 프로세스 재시작 완료", "SUCCESS")
            else:
                sys_log(f"C++ 실행 파일 없음: {cpp_path}", "WARN")

            socketio.emit('ota_status', {'percent': 100, 'message': '시스템 준비 완료. START 버튼 누르면 됨'})
        else:
            sys_log("OTA 실패", "ERROR")
            socketio.emit('ota_status', {'percent': -1, 'message': 'OTA 실패임. 다시 시도해보자'})

    except Exception as e:
        sys_log(f"OTA오류: {e}", "ERROR")
        socketio.emit('ota_status', {'percent': -1, 'message': f'OTA 오류: {e}'})

    finally:
        ota_in_progress = False        

if __name__ == '__main__':
    print(f"Starting Neuro-Driver Python Server -> Sending to UDP {UDP_PORT}...")
    
    # 모니터링 태스크만 실행 (시리얼 태스크 제거됨)
    #아까 정의한 온도 감시 태스크를 뒷단(background)에서 실행시킴
    socketio.start_background_task(status_monitor_task)
    
    #서버 시작 (0.0.0.0은 모든 IP에서 접속 허용)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)