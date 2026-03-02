#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <chrono>

// PCA9685 기본 주소
#define PCA9685_ADDR 0x40
#define MODE1 0x00
#define PRE_SCALE 0xFE
#define LED0_ON_L 0x06

using namespace std;

class MotorDriver {
private:
    int file;

    void write8(uint8_t reg, uint8_t value) {
        uint8_t buffer[2];
        buffer[0] = reg;
        buffer[1] = value;
        if (write(file, buffer, 2) != 2) {
            cerr << "I2C 쓰기 실패" << endl;
        }
    }

    uint8_t read8(uint8_t reg) {
        uint8_t result;
        if (write(file, &reg, 1) != 1) {
            cerr << "레지스터 주소 쓰기 실패" << endl;
        }
        if (read(file, &result, 1) != 1) {
            cerr << "I2C 읽기 실패" << endl;
        }
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
        // 라즈베리파이 5는 보통 /dev/i2c-1 사용
        const char *filename = "/dev/i2c-1";
        if ((file = open(filename, O_RDWR)) < 0) {
            cerr << "I2C 버스를 열 수 없습니다. (sudo 권한 확인)" << endl;
            exit(1);
        }

        if (ioctl(file, I2C_SLAVE, PCA9685_ADDR) < 0) {
            cerr << "장치 연결 실패 (주소 0x40 확인 요망)" << endl;
            exit(1);
        }

        // PCA9685 초기화
        write8(MODE1, 0x00);
        
        // 주파수 50Hz 설정
        uint8_t oldmode = read8(MODE1);
        uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
        write8(MODE1, newmode);
        write8(PRE_SCALE, 121);
        write8(MODE1, oldmode);
        this_thread::sleep_for(chrono::milliseconds(5));
        write8(MODE1, oldmode | 0xA1);
    }

    // 모터 설정 (속도: -100 ~ 100)
    void setMotorA(int speed) { 
        speed = max(-100, min(100, speed));
        int pwm_val = abs(speed) * 4095 / 100;

        if (speed > 0) {
            // [수정됨] 원래 Low였던 것을 High로, High였던 것을 Low로 변경
            setPWM(1, 0, 4096); // AIN1 Low
            setPWM(2, 4096, 0); // AIN2 High
        } else {
            // [수정됨] 반대로 변경
            setPWM(1, 4096, 0); // AIN1 High
            setPWM(2, 0, 4096); // AIN2 Low
        }
        setPWM(0, 0, pwm_val); // PWMA Speed
    }

    void setMotorB(int speed) { 
        speed = max(-100, min(100, speed));
        int pwm_val = abs(speed) * 4095 / 100;

        if (speed > 0) {
            // [수정됨] 원래 Low였던 것을 High로, High였던 것을 Low로 변경
            setPWM(3, 0, 4096); // BIN1 Low
            setPWM(4, 4096, 0); // BIN2 High
        } else {
            // [수정됨] 반대로 변경
            setPWM(3, 4096, 0); // BIN1 High
            setPWM(4, 0, 4096); // BIN2 Low
        }
        setPWM(5, 0, pwm_val); // PWMB Speed
    }
    
    void stop() {
        setMotorA(0);
        setMotorB(0);
    }
    
    ~MotorDriver() {
        stop();
        close(file);
    }
};

int main() {
    cout << "Neuro-Driver: 모터 테스트 시작..." << endl;
    
    MotorDriver driver;
    
    cout << ">>> 모터 전진 (속도 50%) <<<" << endl;
    driver.setMotorA(50);
    driver.setMotorB(50);
    
    this_thread::sleep_for(chrono::seconds(2));
    
    cout << ">>> 정지 <<<" << endl;
    driver.stop();
    
    return 0;
}