#include <iostream>
#include <string>
#include "motor_driver.h"

int main(int argc, char* argv[]) {
    MotorDriver motor;

    motor.init();

    if(argc < 3){
        std::cout << "Usage: ./neuro_drive <left_pwm> <right_pwm>" << std::endl;
        return 1;
    }

    try{
        int left = std::stoi(argv[1]);
        int right = std::stoi(argv[2]);

        motor.setSpeed(left, right);
    } catch(const std::exception& e){
        std::cerr << "Error parsing arguments: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}