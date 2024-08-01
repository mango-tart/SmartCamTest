#include "MotorController.hpp"

// Define the motor sequences as static constants within the class
const int MotorController::sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

const int MotorController::x_motor_pins[4] = {3, 4, 6, 9};
const int MotorController::y_motor_pins[4] = {10, 13, 15, 16};

MotorController::MotorController(const int* pins) : motor_pins(pins), status(0), step_count(0) {}

MotorController MotorController::selectMotor(bool motor) {
    return MotorController(motor ? y_motor_pins : x_motor_pins);
}

int MotorController::rotateMotorBySteps(int steps, int speed, bool direction) {
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < steps; i++) {
        if (direction) {
            status = (status + 1) % 8;
            step_count++;
        } else {
            status = (status == 0) ? 7 : (status - 1);
            step_count--;
        }
        for (int k = 0; k < 4; k++) {
            digitalWrite(motor_pins[k], sequence[status][k]);
        }
        usleep(speed);
    }

    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

int MotorController::rotateMotorForTime(int duration_ms, int speed, bool direction) {
    auto start = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() < duration_ms) {
        if (direction) {
            status = (status + 1) % 8;
            step_count++;
        } else {
            status = (status == 0) ? 7 : (status - 1);
            step_count--;
        }    
        for (int k = 0; k < 4; k++) {
            digitalWrite(motor_pins[k], sequence[status][k]);
        }
        usleep(speed);
    }

    return step_count;
}

void MotorController::startMotors() {
    for (int i = 0; i < 4; ++i) {
        pinMode(MotorController::x_motor_pins[i], 1);
        pinMode(MotorController::y_motor_pins[i], 1);  // Set pin to output mode
    }
}

void MotorController::stopMotors() {
    for (int i = 0; i < 4; ++i) {
        pinMode(MotorController::x_motor_pins[i], 0);
        pinMode(MotorController::y_motor_pins[i], 0);  // Reset pin to input mode to reduce power consumption
    }
}

void MotorController::resetStepCount() {
    step_count = 0;
}

int MotorController::getStepCount() const {
    return step_count;
}
