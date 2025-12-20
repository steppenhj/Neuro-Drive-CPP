#include "motor_driver.h"
#include <iostream>

MotorDriver::MotorDriver(){

}

MotorDriver::~MotorDriver(){
    stop();
}

void MotorDriver::init(){
    std::cout << "[System] Motor Driver Initialized (Ready)" << std::endl;
}

void MotorDriver::setSpeed(int leftSpeed, int rightSpeed) {
    std::cout << "[Hardware] Motor Logic -> Left: " << leftSpeed << " | Right: " << rightSpeed << std::endl; 
}

void MotorDriver::stop(){
    std::cout << "[System] Emergency Stop Triggered~~!" << std::endl;
}