// drive_server_safe.cpp
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <gpiod.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <cerrno> // 에러 코드 확인용 추가

// --- 설정 ---
#define PCA9685_ADDR 0x40
#define CHIP_PATH "/dev/gpiochip4"
#define SERVO_PIN 18
#define UDP_PORT 8080
#define TIMEOUT_US 500000 // 0.5초 (500ms) 타임아웃 설정

using namespace std;

// --- 1. DC 모터 드라이버 (기존 유지) ---
class MotorDriver {
private:
    int file;
    const int LED0_ON_L = 0x06;
    const int MODE1 = 0x00;
    const int PRE_SCALE = 0xFE;

    void write8(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        write(file, buffer, 2);
    }

    uint8_t read8(uint8_t reg) {
        write(file, &reg, 1);
        uint8_t result;
        read(file, &result, 1);
        return result;
    }

    void setPWM(int channel, int on, int off) {
        write8(LED0_ON_L + 4 * channel, on & 0xFF);
        write8(LED0_ON_L + 4 * channel + 1, on >> 8);
        write8(LED0_ON_L + 4 * channel + 2, off & 0xFF);
        write8(LED0_ON_L + 4 * channel + 3, off >> 8);
    }

public:
    MotorDriver() {
        const char *filename = "/dev/i2c-1";
        if ((file = open(filename, O_RDWR)) < 0) {
            cerr << "I2C Open Failed" << endl; exit(1);
        }
        if (ioctl(file, I2C_SLAVE, PCA9685_ADDR) < 0) {
            cerr << "I2C Connect Failed" << endl; exit(1);
        }
        write8(MODE1, 0x00);
        uint8_t oldmode = read8(MODE1);
        uint8_t newmode = (oldmode & 0x7F) | 0x10;
        write8(MODE1, newmode);
        write8(PRE_SCALE, 121); // 50Hz
        write8(MODE1, oldmode);
        this_thread::sleep_for(chrono::milliseconds(5));
        write8(MODE1, oldmode | 0xA1);
    }

    void setMotorA(int speed) {
        speed = max(-100, min(100, speed));
        int pwm_val = abs(speed) * 4095 / 100;
        if (speed > 0) { setPWM(1, 0, 4096); setPWM(2, 4096, 0); }
        else { setPWM(1, 4096, 0); setPWM(2, 0, 4096); }
        setPWM(0, 0, pwm_val);
    }

    void setMotorB(int speed) {
        speed = max(-100, min(100, speed));
        int pwm_val = abs(speed) * 4095 / 100;
        if (speed > 0) { setPWM(3, 0, 4096); setPWM(4, 4096, 0); }
        else { setPWM(3, 4096, 0); setPWM(4, 0, 4096); }
        setPWM(5, 0, pwm_val);
    }

    // 비상 정지용 함수 추가
    void stopAll() {
        setMotorA(0);
        setMotorB(0);
    }

    ~MotorDriver() { 
        stopAll();
        close(file); 
    }
};

// --- 2. 서보 모터 드라이버 (기존 유지) ---
class ServoDriver {
private:
    struct gpiod_chip *chip;
    struct gpiod_line_request *request;
    atomic<int> current_angle;
    atomic<bool> running;
    thread pwm_thread;

    void pwm_loop() {
        while (running) {
            int angle = current_angle.load();
            int pulse_us = 500 + (angle * 2000 / 180);
            
            gpiod_line_request_set_value(request, SERVO_PIN, GPIOD_LINE_VALUE_ACTIVE);
            this_thread::sleep_for(chrono::microseconds(pulse_us));
            
            gpiod_line_request_set_value(request, SERVO_PIN, GPIOD_LINE_VALUE_INACTIVE);
            this_thread::sleep_for(chrono::microseconds(20000 - pulse_us));
        }
    }

public:
    ServoDriver() : current_angle(90), running(true) {
        chip = gpiod_chip_open(CHIP_PATH);
        auto settings = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        
        auto line_cfg = gpiod_line_config_new();
        unsigned int offset = SERVO_PIN;
        gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
        
        auto req_cfg = gpiod_request_config_new();
        gpiod_request_config_set_consumer(req_cfg, "servo_driver");
        
        request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
        pwm_thread = thread(&ServoDriver::pwm_loop, this);

        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
    }

    void setAngle(int angle) {
        current_angle.store(max(0, min(180, angle)));
    }

    ~ServoDriver() {
        running = false;
        if (pwm_thread.joinable()) pwm_thread.join();
        if (request) gpiod_line_request_release(request);
        if (chip) gpiod_chip_close(chip);
    }
};

// --- 3. 데이터 구조체 ---
struct ControlData {
    float throttle;
    float steering;
};

int main() {
    cout << ">>> Neuro-Drive Hardware Server Started (Safe Mode) <<<" << endl;

    MotorDriver motors;
    ServoDriver servo;

    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return 1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        return 1;
    }

    // --- [중요] 타임아웃 설정 (Safety Mechanism) ---
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = TIMEOUT_US; // 0.5초
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) < 0) {
        perror("Setsockopt failed");
        return 1;
    }

    cout << ">>> Listening on UDP Port " << UDP_PORT << " with " << TIMEOUT_US/1000 << "ms Watchdog <<<" << endl;

    ControlData data;
    socklen_t len = sizeof(cliaddr);
    bool is_stopped = false; // 불필요한 중복 명령 방지용 플래그
    
    while (true) {
        // 데이터 수신 (이제 타임아웃 발생 시 -1 반환)
        int n = recvfrom(sockfd, &data, sizeof(data), 0, (struct sockaddr *)&cliaddr, &len);
        
        if (n > 0) {
            // [정상 수신 시]
            is_stopped = false;
            
            int motor_speed = static_cast<int>(data.throttle * 100);
            motors.setMotorA(motor_speed);
            motors.setMotorB(motor_speed);

            int servo_angle = static_cast<int>((data.steering + 1.0) * 90.0);
            servo.setAngle(servo_angle);
        } 
        else {
            // [수신 실패 또는 타임아웃]
            if ((errno == EAGAIN || errno == EWOULDBLOCK) && !is_stopped) {
                // 타임아웃 발생! 즉시 정지
                cout << "[Safety Warning] No Signal! Stopping Car..." << endl;
                motors.stopAll();
                
                // 선택 사항: 조향도 중앙으로 정렬하려면 아래 주석 해제
                // servo.setAngle(90); 
                
                is_stopped = true; // 정지 상태 기록
            }
        }
    }

    close(sockfd);
    return 0;
}