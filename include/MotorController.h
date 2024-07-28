#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include <chrono>

class MotorController {
private:
    const int* motor_pins;
    static const int sequence[8][4];
    static const int reverse_sequence[8][4];

public:
    MotorController(const int* pins);
    int rotateMotorBySteps(int steps, int speed, bool direction);
    int rotateMotorForTime(int duration_ms, int speed, bool direction);
};

#endif // MOTOR_CONTROLLER_H
