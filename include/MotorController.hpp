#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include <chrono>

class MotorController {
private:
    const int* motor_pins;
    static const int sequence[8][4];
    static const int x_motor_pins[4];
    static const int y_motor_pins[4];

    int status;       // Current step position [0-7]
    int step_count;   // Total step count

public:
    MotorController(const int* pins);
    static MotorController selectMotor(bool motor);  // true for Y, false for X
    int rotateMotorBySteps(int steps, int speed, bool direction);
    int rotateMotorForTime(int duration_ms, int speed, bool direction);
    void startMotors();  // Initialize motor pins
    void stopMotors();   // Reset motor pins
    void resetStepCount();  // Reset step count to zero
    int getStepCount() const;  // Get the current step count
};

#endif // MOTOR_CONTROLLER_H
