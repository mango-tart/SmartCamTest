#include "MotorController.h"
#include <thread>

int x_motor_pins[4] = {3, 4, 6, 9};
int y_motor_pins[4] = {10, 13, 15, 16};

void runMotor(MotorController& controller, bool direction, int duration, int speed, std::string name) {
    int steps = controller.rotateMotorForTime(duration, speed, direction);
    std::cout << name << " motor rotated " << steps << " steps." << std::endl;
}

int main() {
    wiringPiSetup();
    MotorController xController(x_motor_pins);
    MotorController yController(y_motor_pins);

    std::thread xMotorThread(runMotor, std::ref(xController), true, 1000, 1200, "X");
    std::thread yMotorThread(runMotor, std::ref(yController), true, 1000, 1200, "Y");

    xMotorThread.join();
    yMotorThread.join();

    return 0;
}
