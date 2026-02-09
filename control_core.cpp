/**
 * @file control_core.cpp
 * @brief Multi-threaded Control Architecture (Safety Enhanced)
 * @details 
 * Thread 1: UDP Receiver (Blocking I/O)
 * Thread 2: Control Loop (Fixed Frequency 100Hz)
 * Feature: Watchdog Timer (Auto-stop on signal loss)
 */

#include <iostream>
#include <stdio.h>
#include <thread> // 멀티스레드
#include <mutex>  // 자원 보호
#include <chrono> // 시간 제어
#include <atomic> // 플래그용
#include <cstdint>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;

// 1. 공유 자원
struct SharedMemory{
    float throttle = 0.0f;
    float steering = 0.0f;
    bool is_active = false;
};

SharedMemory shared_data; 
mutex data_lock; 

// 프로그램 종료 플래그
atomic<bool> keep_running(true);

// ★ 안전장치 설정 (0.5초 동안 데이터 없으면 정지)
const int WATCHDOG_MS = 500;
atomic<int64_t> last_rx_us{0};

// 현재 시간(마이크로초) 가져오기
static inline int64_t now_us(){
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

// 2. 시리얼 설정 함수
int open_serial(const char* device){
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1) return -1;
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// 3. 스레드1: UDP 수신 (데이터 받으면 시간 기록)
void udp_receiver_task(){
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in servaddr{}, cliaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(5555);

    bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));

    // 수신 타임아웃 1초
    struct timeval tv = {1, 0};
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    cout << "[Thread 1] UDP Receiver Started." << endl;

    struct Packet { float th; float st; } pkt;
    socklen_t len = sizeof(cliaddr);

    while(keep_running){
        int n = recvfrom(sockfd, &pkt, sizeof(pkt), 0, (struct sockaddr *)&cliaddr, &len);

        if(n > 0){
            lock_guard<mutex> lock(data_lock);
            shared_data.throttle = pkt.th;
            shared_data.steering = pkt.st;
            shared_data.is_active = true;
            
            // ★ 데이터 수신 시각 갱신 (Watchdog Reset)
            last_rx_us.store(now_us(), std::memory_order_relaxed);
        }
    }
    close(sockfd);
}

// 4. 스레드2: 제어 루프 (100Hz + Watchdog 기능)
void control_loop_task(int serial_fd){
    cout << "[Thread 2] Control Loop Started (100Hz)." << endl;
    char buffer[64];

    // 정밀 타이밍용 변수
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();

    while(keep_running){
        // 10ms (100Hz) 주기 설정
        next_tick += std::chrono::milliseconds(10);

        // ★ Watchdog 체크: 마지막 수신 후 500ms 지났는지 확인
        int64_t age_us = now_us() - last_rx_us.load(std::memory_order_relaxed);
        bool timeout = (age_us > (int64_t)WATCHDOG_MS * 1000);

        float th_local = 0;
        float st_local = 0;

        {
            lock_guard<mutex> lock(data_lock);
            th_local = shared_data.throttle;
            st_local = shared_data.steering;
        }

        // ★ 타임아웃이면 강제 정지 (안전장치)
        if(timeout){
            th_local = 0.0f;
            st_local = 0.0f;
            // (옵션) 타임아웃 로그가 너무 많이 뜨면 주석 처리 가능
            // cout << "[WARN] Watchdog Timeout! Stopping..." << endl;
        }

        // 제어 연산
        int pwm_speed = (int)(th_local * 999.0f);
        int pwm_angle = 1500 + (int)(st_local * 900.0f);

        // 시리얼 전송
        int len = snprintf(buffer, sizeof(buffer), "%d,%d\n", pwm_speed, pwm_angle);
        write(serial_fd, buffer, len);

        // 남은 시간만큼 대기 (정밀 주기 유지)
        std::this_thread::sleep_until(next_tick);
    }
}

int main(){
    // ★ 포트 이름 확인 (/dev/ttyACM0 또는 /dev/ttyUSB0)
    int serial_fd = open_serial("/dev/ttyACM0"); 
    if(serial_fd < 0){
        cerr << "Error: Serial Open Failed. Check Permission or Port Name." << endl;
        return -1;
    }

    // 초기화: 시작하자마자 멈추지 않게 현재 시간 기록
    last_rx_us.store(now_us(), std::memory_order_relaxed);

    thread receiver(udp_receiver_task);
    thread controller(control_loop_task, serial_fd);

    cout << "[Main] System Running. Press Ctrl+C to stop." << endl;

    receiver.join();
    controller.join();

    close(serial_fd);
    return 0;
}