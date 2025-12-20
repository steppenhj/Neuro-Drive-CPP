#pragma once

class MotorDriver {
public:
    MotorDriver();

    ~MotorDriver();

    void init();

    void setSpeed(int leftSpeed, int rightSpeed);

    void stop();

private:
    int pinLeftForward;
    int pinLeftBackward;
    int pinRightForward;
    int pinRightBackward;
};