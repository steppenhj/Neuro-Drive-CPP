/**
 * @file control_core.cpp
 * @brief Multi-threaded 
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
#include <atomic> // 플래그용 - 쪼개지지 않는 변수
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

SharedMemory shared_data;  //칠판 - 데이터 저장소 느낌
mutex data_lock;   // 칠판 지우개 (누가 쓰고 있을 때 못 건드리게 함)


// 프로그램 종료 플래그 - atomic -> 스레드가 A가 값 바꿀 때, 스레드 B가 찰나의 순간에 옛 값 읽는 걸 방지 (bool은 안됨)
atomic<bool> keep_running(true);

// ★ 안전장치 설정 (0.5초 동안 데이터 없으면 정지)
const int WATCHDOG_MS = 500;
atomic<int64_t> last_rx_us{0};

// 현재 시간(마이크로초) 가져오기
static inline int64_t now_us(){
    using namespace std::chrono;

    // steady_clock -> "절대 뒤로 가지 않는 시계"
    // system_clock을 쓰다가 시간이 1초 뒤로 점프하면 Watchdog이 오작동해서 차가 멈출 수도 있다.
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
    // 소켓 만들기 => 우편함이라고 생각하면 됨
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in servaddr{}, cliaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;

    //주소 5555포트로 설정
    servaddr.sin_port = htons(5555);

    //바인딩. 문패 다는 느낌.
    bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));

    // 수신 타임아웃 1초
    struct timeval tv = {1, 0};
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    cout << "[Thread 1] UDP Receiver Started." << endl;

    //파이썬이 보낸 'ff'랑 똑같은 모양의 그릇
    struct Packet { float th; float st; } pkt;
    socklen_t len = sizeof(cliaddr);

    while(keep_running){
        // recvfrom=> 올 때까지 대기 - Blocking
        // CPU를 거의 안 쓰고 데이터가 오는 걸 기다림. 효율적
        int n = recvfrom(sockfd, &pkt, sizeof(pkt), 0, (struct sockaddr *)&cliaddr, &len);

        if(n > 0){
            // n>0이 데이터가 잘 도착했다는 뜻
            lock_guard<mutex> lock(data_lock); //자물쇠 잠가주기

            //공유 변수에 데이터 덮어쓰기
            shared_data.throttle = pkt.th;
            shared_data.steering = pkt.st;
            shared_data.is_active = true;
            
            // 데이터 수신 시각 갱신 (Watchdog Reset)
            // memory_order_relaxed -> 순서 너무 안 따지고 그냥 빠르게 저장하는 느낌
            last_rx_us.store(now_us(), std::memory_order_relaxed);
        } //중괄호 닫으면 자물쇠가 풀림.
        //mutex 쓰는 이유 아래에 정리
        // receiver 스레드는 데이터를 쓰고 있고
        // 동시에 controller 스레드가 데이터를 읽고 있음. 
        // 쓰는 도중에 읽으면 안되잖아. read - write

        //간단하게 아래처럼 정리 가능
        // 1. recvfrom으로 받는다
        // 2. lock으로 잠근다
        // 3. shared_data에 옮겨 적는다.
        
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
            th_local = shared_data.throttle; //복사본
            st_local = shared_data.steering;
        } //바로 잠금 해제. 길게 잡으면 receiver스레드가 기다려야 함

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
        //pwm_speed, pwm_angle이 숫자이고 이걸 문자열 buffer로 바꿔줌.
        //pwm_speed(예 300), pwm_angle(예 1500) 이건 메모리의 2진수 숫자
        //buffer는 "300,1500\n" 같은 내가 볼 수 있는 아스키로 바꿈. 개행문자를 마침표로 사용

        write(serial_fd, buffer, len);
        //리눅스 커널 -> UART 장치. system call

        //아래와 같이 정리
        //1. write 함수가 실행되면, 운영체제(커널)가 buffer에 있는 글자를 가져감
        //2. 커널은 이 글자를 usb 케이블을 통해 전기 신호로 바꿈
        //3. stm32의 RX 핀으로 전기 신호가 쏟아짐

        // 남은 시간만큼 대기 (정밀 주기 유지)
        std::this_thread::sleep_until(next_tick);
    }
}

int main(){
    //시리얼 포트 열기 -> 포트 이름 확인 (/dev/ttyACM0 또는 /dev/ttyUSB0)
    //전자임
    int serial_fd = open_serial("/dev/ttyACM0"); 
    if(serial_fd < 0){
        cerr << "Error: Serial Open Failed. Check Permission or Port Name." << endl;
        return -1;
    }

    // 초기화: 시작하자마자 멈추지 않게 현재 시간 기록
    last_rx_us.store(now_us(), std::memory_order_relaxed);


    // 스레드 생성
    thread receiver(udp_receiver_task); // 일꾼1 -> 귀
    thread controller(control_loop_task, serial_fd); // 일꾼 2 -> 머리

    cout << "[Main] System Running. Press Ctrl+C to stop." << endl;

    //메인 스레드는 여기서 대기
    // join(): 일꾼들이 퇴근할 때까지 기다리기
    // join() 없으면 main함수가 바로 끝나고, 프로세스가 종료되면서
    // 일하고 있던 스레드들도 강제로 죽음. (프로그램 즉시 종료)
    receiver.join();
    controller.join();

    close(serial_fd);
    return 0;
}

// 중요한 거 아래에 적자
// 1. UDP 수신부: recvfrom으로 받아서 mutex로 잠그고 shared_data에 적는다
// 2. 제어 루프: 100Hz 마다 깨어나서, shared_data값을 복사(local)해 온 뒤 
// 안전장치(watchdog)를 확인하고 시리얼(write)로 쓴다.
// 3. 안전장치: 0.5초 동안 파이썬 연락이 없으면 차를 세운다.