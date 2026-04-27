#include <iostream>
#include <gpiod.h>
#include <chrono>
#include <thread>
#include <vector>

// 라즈베리파이 5의 GPIO 칩 경로 (보통 /dev/gpiochip4)
#define CHIP_PATH "/dev/gpiochip4"
#define SERVO_PIN 18 // GPIO 18 (Pin 12)

using namespace std;

int main() {
    struct gpiod_chip *chip = NULL;
    struct gpiod_line_settings *settings = NULL;
    struct gpiod_line_config *line_cfg = NULL;
    struct gpiod_request_config *req_cfg = NULL;
    struct gpiod_line_request *request = NULL;
    int ret = 0;

    cout << "Neuro-Driver: 서보 모터 테스트 (libgpiod v2 API)" << endl;

    // 1. 칩 열기
    chip = gpiod_chip_open(CHIP_PATH);
    if (!chip) {
        cerr << "오류: 칩을 열 수 없습니다 (" << CHIP_PATH << ")" << endl;
        return 1;
    }

    // 2. 핀 설정 (출력 모드)
    settings = gpiod_line_settings_new();
    if (!settings) return 1;
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);

    // 3. 라인 구성 (어떤 핀을 쓸지)
    line_cfg = gpiod_line_config_new();
    if (!line_cfg) return 1;
    
    unsigned int offset = SERVO_PIN;
    ret = gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
    if (ret) {
        cerr << "오류: 라인 설정을 추가할 수 없습니다." << endl;
        return 1;
    }

    // 4. 요청 설정 (누가 쓰는지 이름표)
    req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "servo_test_v2");

    // 5. 하드웨어 점유 요청
    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!request) {
        cerr << "오류: GPIO 라인 요청 실패. (다른 프로세스가 사용 중일 수 있음)" << endl;
        return 1;
    }

    // 6. PWM 동작 함수 (람다)
    auto set_angle = [&](int angle, int duration_ms) {
        // 0도=500us, 180도=2500us
        int pulse_us = 500 + (angle * 2000 / 180); 
        int period_us = 20000; // 20ms (50Hz)
        int loops = duration_ms / 20;

        for (int i = 0; i < loops; i++) {
            // High
            gpiod_line_request_set_value(request, SERVO_PIN, GPIOD_LINE_VALUE_ACTIVE);
            std::this_thread::sleep_for(std::chrono::microseconds(pulse_us));
            
            // Low
            gpiod_line_request_set_value(request, SERVO_PIN, GPIOD_LINE_VALUE_INACTIVE);
            std::this_thread::sleep_for(std::chrono::microseconds(period_us - pulse_us));
        }
    };

    // 테스트 실행
    try {
        cout << ">>> 중앙 정렬 (90도) <<<" << endl;
        set_angle(90, 1000);

        cout << ">>> 왼쪽 (0도) <<<" << endl;
        set_angle(0, 1000);

        cout << ">>> 오른쪽 (180도) <<<" << endl;
        set_angle(180, 1000);

        cout << ">>> 중앙 복귀 (90도) <<<" << endl;
        set_angle(90, 500);
    } catch (...) {
        cerr << "실행 중 오류 발생" << endl;
    }

    // 7. 자원 해제 (메모리 정리)
    if (request) gpiod_line_request_release(request);
    if (req_cfg) gpiod_request_config_free(req_cfg);
    if (line_cfg) gpiod_line_config_free(line_cfg);
    if (settings) gpiod_line_settings_free(settings);
    if (chip) gpiod_chip_close(chip);

    cout << "테스트 완료." << endl;
    return 0;
}