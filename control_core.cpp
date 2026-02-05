/**
 * @file control_core.cpp
 * @brief Neuro-Drive Main Control Process
 * @details Receives UDP from Python, Calculates Logic, Sends UART to STM32
 */

 #include <iostream>
 #include <string>
 #include <cstring>
 #include <cmath>
 #include <unistd.h>
 #include <fcntl.h>
 #include <termios.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>

 //설정 상수
 #define UDP_PORT 5555
 #define SERIAL_PORT "/dev/ttyACM0"
 #define BAUDRATE B115200

 #define CENTER_PWM 1500
 #define STEERING_FACTOR 900 // 조향 감도

 //Python과 맞춘 데이터 구조체 (8bytes)
 struct ControlPacket{
    float throttle; // -1.0 ~ 1.0
    float steering; // -1.0 ~ 1.0
 };

 // 시리얼 포트 설정 함수
 //이 부분은 llm한테 받은 그대로 쓰기. 어려움. 
 int open_serial(const char* device){
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd==-1){
        perror("open_serial: Unalbe to open device");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);

    options.c_cflag |= (CLOCAL | CREAD); // 수신 가능, 로컬 연결
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; // 8Data bits


    //Raw 모드 설정 (이게 없으면 바이너리 전송 시에 문제될 수도)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
 }

 int main(){
    std::cout << "[Neuro-Drive] C++ Control Core Starting.." << std::endl;

    // 1. 시리얼 포트 열기.
    int serial_fd = open_serial(SERIAL_PORT);
    if(serial_fd < 0) return -1;
    std::cout << "[Info] STM32 Serial Connected." << std::endl;

    //2. UDP 소켓 생성 (Receiver)
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0){
        perror("socket creation failed");
        return -1;
    }

    struct sockaddr_in servaddr, cliaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(UDP_PORT);

    if(bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0){
        perror("bind failed");
        return -1;
    }

    std::cout << "[Info] Waiting for UDP Data on port" << UDP_PORT << "..." << std::endl;

    ControlPacket packet;
    char buffer[1024]; // 시리ㄹ얼 송신 버퍼
    socklen_t len = sizeof(cliaddr);

    // Watchdog용 타임아웃 설정 (UDP 수신 대기 시)
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000; // 500ms
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    while(true){
        //3. 데이터 수신(blocking with timeout)
        int n = recvfrom(sockfd, &packet, sizeof(packet), 0, (struct sockaddr *)&cliaddr, &len);

        if(n==sizeof(ControlPacket)){
            // [Logic migration] 파이썬에 잇던 로직을 여기에서 수행
            float throttle = packet.throttle;
            float steering = packet.steering;

            //Boost Factor Logic
            // 조향 많이할수록 출력 높여서 stall 방지.
            // 일단 넣고 추후에 필요 없다 싶으면 제거하면 됨
            if(std::abs(steering) > 0.1f){
                float boost_factor = 1.0f + (std::abs(steering) * 0.8f);
                throttle = throttle * boost_factor;
            }

            // PWM Mapping
            int pwm_speed = (int)(throttle * 999.0f);
            //Clamp(-999 ~ 999)
            if(pwm_speed > 999) pwm_speed = 999;
            if(pwm_speed < -999) pwm_speed = -999;

            int pwm_angle = CENTER_PWM + (int)(steering * STEERING_FACTOR);
            //Clamp(600 ~ 2400)
            if (pwm_angle > 2400) pwm_angle =2400;
            if(pwm_angle < 600) pwm_angle = 600;

            // 4. stm32로 전송("speed,angle\n" 포맷. 일단 십진수 쓰는데 나중에 16진수 쓸까 고민중이긴 함.)
            int len = snprintf(buffer, sizeof(buffer), "%d,%d\n", pwm_speed, pwm_angle);
            write(serial_fd, buffer, len);

            //debug (너무 많으면 주석 처리하자. 일단 처음에도 주석처리임. 버그있으면 이거 풀어서 확인하면됨)
            // printf("In(%.2f, %.2f) -> Out(%d %d)\n", packet.throttle, packet.steering, pwm_speed, pwm_angle);
        }
        else{
            // Timeout or Error (500ms 동안 파이썬에서 데이터가 안 오면)
            // 안전을 위해 정지 명령 전송
            // snprintf(buffer, sizeof(buffer), "0,1500\n");
            // write(serial_fd, buffer, strlen(buffer));
            // std::cout << "[Warn] No Signal... Stopping." << std::endl; 
        }
    }

    close(serial_fd);
    close(sockfd);
    return 0;
 }