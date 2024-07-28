#include "MotorController.h"

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

const int MotorController::reverse_sequence[8][4] = {
    {0, 0, 0, 1},
    {0, 0, 1, 1},
    {0, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 0},
    {1, 1, 0, 0},
    {1, 0, 0, 0},
    {1, 0, 0, 1}
};

MotorController::MotorController(const int* pins) : motor_pins(pins) {}

int MotorController::rotateMotorBySteps(int steps, int speed, bool direction) {
    const int (*seq)[4] = direction ? sequence : reverse_sequence;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < 8; j++) {
            for (int k = 0; k < 4; k++) {
                digitalWrite(motor_pins[k], seq[j][k]);
            }
            usleep(speed);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

int MotorController::rotateMotorForTime(int duration_ms, int speed, bool direction) {
    const int (*seq)[4] = direction ? sequence : reverse_sequence;
    auto start = std::chrono::high_resolution_clock::now();
    int step_count = 0;

    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() < duration_ms) {
        for (int j = 0; j < 8; j++) {
            for (int k = 0; k < 4; k++) {
                digitalWrite(motor_pins[k], seq[j][k]);
            }
            usleep(speed);
        }
        step_count++;
    }

    return step_count;
}
