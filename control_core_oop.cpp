/**
* @file control_core_oop.cpp
* @brief 객체지향으로 리팩토링
* @details
*  -UdpReceiver Class: Handles Network Communication
* -VehicleController Class: Handles Serial & Control Logic
* - SharedContext Struct: Thread-Safe data exchange
*/

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;

// 1. 공통 데이터 객체
// 스레드 간 데이터를 주고받기 위한 "공용 칠판" 느낌
struct SharedContext {
    float throttle = 0.0f;
    float steering = 0.0f;
    std::mutex data_mutex; //자원 보호용 자물쇠
    std::atomic<int64_t> last_rx_us{0}; //WatchDog 타이머용 (마지막 수신 시간)
    std::atomic<bool> keep_running{true}; //프로그램 종료 플래그

    //현재 시간 (us) 가져오기 유틸리티
    static int64_t now_us(){
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }
};

// 2. UDP 수신 클래스 (Receiver)
class UdpReceiver{
private:
    int sockfd;
    int port;
    SharedContext& ctx; //참조변수 (공유 자원)
    std::thread receiverThread;

    void receiveLoop(){
        cout << "[UdpReceiver] Thread Started on Port " << port << endl;

        sockaddr_in cliaddr{};
        socklen_t len = sizeof(cliaddr);

        //파이썬과 약속한 데이터 구조체
        struct Packet { float th; float st; } pkt;

        while(ctx.keep_running){
            //Blocking Receive
            int n = recvfrom(sockfd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&cliaddr, &len);

            if (n>0){
                //자물쇠 잠그고 데이터 업데이트
                std::lock_guard<std::mutex> lock(ctx.data_mutex);
                ctx.throttle = pkt.th;
                ctx.steering = pkt.st;

                //WatchDog 시간 갱신 (atomic이라 mutex 필요 없음)
                ctx.last_rx_us.store(SharedContext::now_us(), std::memory_order_relaxed);
            }
        }
    }

public:
    UdpReceiver(int _port, SharedContext& _ctx) :port(_port), ctx(_ctx){
        //생성자: 소켓 열기
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in servaddr{};
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(port);

        // 이미 사용 중인 포트 재사용 가능하게 설정 (좀비 프로세스 방지)
        int opt = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &opt, sizeof(opt));

        // 바인딩 실패 시 에러 출력하고 프로그램 종료
        if(bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0){
            perror("[Critical Error] UDP Bind Failed (Port 5555 is busy?)");
            exit(EXIT_FAILURE);
        }

        //타임아웃 1초 설정 (스레드가 종료 프래그를 확인하는 용도)
        struct timeval tv = {1, 0};
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    }

    ~UdpReceiver() {
        //소멸자: 자원해제
        if(receiverThread.joinable()) receiverThread.join();
        close(sockfd);
        cout << "[UdpReceiver] Closed." << endl;
    }

    void start(){
        //스레드 시작
        receiverThread = std::thread(&UdpReceiver::receiveLoop, this);
    }
};

// 3. 차량 제어 클래스 (Controller)
class VehicleController {
private:
    int serial_fd;
    string device_name;
    SharedContext& ctx;
    std::thread controlThread;
    const int WATCHDOG_MS = 500;

    // 시리얼 포트 설정 함수 (내부용)
    bool openSerial(){
        serial_fd = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if(serial_fd == -1) return false;

        termios options{};
        tcgetattr(serial_fd, &options);
        
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD | CS8);
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);

        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        
        tcsetattr(serial_fd, TCSANOW, &options);
        return true;
    }

    void controlLoop() {
        cout << "[VehicleController] Loop Started (100Hz)." << endl;
        char buffer[64];

        using clock = std::chrono::steady_clock;
        auto next_tick = clock::now();

        while(ctx.keep_running){
            //10ms 주기 설정
            next_tick += std::chrono::milliseconds(10);

            //1. WatchDog 체크
            int64_t age_us = SharedContext::now_us() - ctx.last_rx_us.load();
            bool timeout = (age_us > (int64_t)WATCHDOG_MS * 1000);

            float current_th = 0.0f;
            float current_st = 0.0f;

            //2. 데이터 읽기
            {
                std::lock_guard<std::mutex> lock(ctx.data_mutex);
                current_th = ctx.throttle;
                current_st = ctx.steering;
            }

            //3. 안전장치 발동
            if(timeout){
                current_th = 0.0f;
                current_st = 0.0f;
            }

            //4. PWM 변환 및 전송
            int pwm_speed = (int)(current_th * 999.0f);
            int pwm_angle = 1500 + (int)(current_st * 900.0f);

            int len = snprintf(buffer, sizeof(buffer), "%d,%d\n", pwm_speed, pwm_angle);
            if(serial_fd != -1){
                write(serial_fd, buffer, len);
            }

            //5. 정밀 주기 대기
            std::this_thread::sleep_until(next_tick);
        }
    }

public:
    VehicleController(string dev, SharedContext& _ctx) : device_name(dev), ctx(_ctx), serial_fd(-1){
        if(!openSerial()){
            cerr << "[Error] Failed to open Seral port: " << dev << endl;
        } else{
            cout << "[VehicleController] Serial Port Opened: " << dev << endl;
        }
    }

    ~VehicleController() {
        if(controlThread.joinable()) controlThread.join();
        if(serial_fd != -1) close(serial_fd);
        cout << "[VehicleController] Closed." << endl;
    }

    void start(){
        controlThread = std::thread(&VehicleController::controlLoop, this);
    }
};

//4. 메인 함수
int main(){
    cout << "===OOP Version 시작===" << endl;

    // 1. 공유 자원 생성 (Stack에 생성)
    SharedContext sharedData;
    sharedData.last_rx_us = SharedContext::now_us(); //초기화

    // 2. 객체 생성 (의조선 주입 - Dependency Injection)
    // UdpReciver 는 5555포트를 듣고, sharedData에 쓴다
    UdpReceiver receiver(5555, sharedData);

    // VehicleController는 시리얼로 쏘고, sharedData를 읽는다
    VehicleController car("/dev/ttyACM0", sharedData);

    // 3. 시스템 가동
    receiver.start();
    car.start();

    // 4. 메인 스레드 대기 (종료 명령 대기)
    cout << "Press ENTER to stop the system..." << endl;
    cin.get(); // 엔터 키 입력 대기

    // 5. 종료 절차
    cout << "Stopping System...." << endl;
    sharedData.keep_running = false; //플래그를 내리면 각 스레드가 루프를 탈출함

    //객체들이 소멸될 떄(destructor) 자동으로 join()하고 자원해제
    return 0;
}