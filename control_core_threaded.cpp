/**
 * @file control_core_threaded.cpp
 * @brief Multi-threaded Control Architecture
 * @details Thread 1: UDP Receiver (Blocking I/O)
 * Thread 2: Control Loop (Fixed Frequency 100Hz)
 */

 #include <iostream>
 #include <thread> //멀티스레드 헤더
 #include <mutex> // 자원 보호용 자물쇠
 #include <chrono> // 시간제어
 #include <atomic> //플래그용
 #include <cstring>
 #include <cmath>
 #include <unistd.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>
 #include <fcntl.h>
 #include <termios.h>

 using namespace std;

 // 1. 공유 자원 shared resource
 struct SharedMemory{
    float throttle = 0.0f;
    float steering = 0.0f;
    bool is_active = false; //데이터 수신 여부
 };

 SharedMemory shared_data; // 두 스레드가 같이 쓸 변수
 mutex data_lock; // 자물쇠 (Mutex)

 //프로그램 종료 플래그
 atomic<bool> keep_running(true);

 // 2. 유틸리티 함수 (시리얼 설정)
 int open_serial(const char* device){
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1) return -1;
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
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

    // 3. 스레드1: UDP 수신 (Receiver 역할)
    void udp_receiver_task(){
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in servaddr{}, cliaddr{};
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(5555);

        bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));

        //수신 타임아웃 1초  (너무 오래 안 멈추게)
        struct timeval tv = {1, 0};
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

        cout << "[Thread 1] UDP Receiver Started." << endl;

        struct Packet { float th; float st; } pkt;
        socklen_t len = sizeof(cliaddr);

        while(keep_running){
            //여기서 대기(blocking)함. 데이터 올 때까지 잠
            int n = recvfrom(sockfd, &pkt, sizeof(pkt), 0, (strcut sockaddr *)&cliaddr, &len);

            if(n > 0){
                //자물쇠 잠금 (critical section시작이라고 보자)
                lock_guard<mutex> lock(data_lock);

                shared_data.throttle = pkt.th;
                shared_data.steering = pkt.st;
                shared_data.is_active = true;
                // 자물쇠는 lock_guard가 사라질 때(블록 끝) 자동으로 풀림
            }
        }
        close(sockfd);
    }

    //4. 스레드2 : 제어 루프 (Controller 역할)
    void control_loop_task(int serial_fd){
        cout << "[Thread 2] Control Loop Started (100Hz)." << endl;
        char buffer[64];

        while(keep_running){
            auto start_time = chrono::steady_clock::now();

            float th_local = 0;
            float st_local = 0;

            // 1. 데이터 ㅇ릭기 (빠르게 읽고 자물쇠 풂)
            {
                lock_guard<mutex> lock(data_lock);
                th_local = shared_data.throttle;
                st_local = shared_data.steering;

                // Watchdog: 데이터가 안 들어오는지 체크 로직은 여기에 추가할 수 있다
            }

            // 2. 제어 연산 (자물쇠 없이 자유롭게 계산)
            int pwm_speed = (int)(th_local * 999.0f);
            int pwm_angle = 1500 + (int)(st_local * 900.0f);

            // 3. 시리얼 전송 --- 형태 유지해야지~~ 띄어쓰기도 없이
            int len = snprintf(buffer, sizeof(buffer), "%d,%d\n", pwm_speed, pwm_angle);
            write(serial_fd, buffer, len);

            // 4. 주기 맞추기 (100Hz = 10ms)
            this_thread::sleep_until(start_time + chrono::milliseconds(10));
        }
    }

    
    //5. 메인 함수
 int main(){
    int serial_fd = open_serial("/dev/ttyACM0");
    if(serial_fd < 0){
        cerr << "Error: Serial Open Failed" << endl;
        return -1;
    }

    //  thread생성 및 실행
    thread receiver(udp_receiver_task);
    thread controller(control_loop_task, serial_fd);

    //메인 스레드는 여기서 대기(여기서 나중에 시스템 관리 등 할 수 있음*)
    cout << "[Main] System Running. Press Ctrl+C to stop." << endl;

    // 실제로 는 signal handler로 종료 처리해야 하지만 일단 join으로 대기
    receiver.join();
    controller.join();

    close(serial_fd);
    return 0;
 }