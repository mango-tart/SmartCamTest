#include <iostream>
#include <thread>
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <chrono>
#include <termios.h>
#include <future>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include "MotorController.h"

std::atomic<bool> running(true);
MotorController xController = MotorController::selectMotor(false);
MotorController yController = MotorController::selectMotor(true);

class PIDController {
public:
    double kp, ki, kd;
    double previous_error;
    double integral;

    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0) {}

    double compute(double error) {
        integral += error;
        double derivative = error - previous_error;
        previous_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

void enableRawMode() {
    struct termios raw;
    tcgetattr(STDIN_FILENO, &raw);
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void disableRawMode() {
    struct termios raw;
    tcgetattr(STDIN_FILENO, &raw);
    raw.c_lflag |= (ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void resetMotor(MotorController& xController, MotorController& yController, int& xStep, int& yStep) {
    xController.startMotors();
    auto xFuture1 = std::async(std::launch::async, [&](){ return xController.rotateMotorBySteps(512*8, 1000, true); });
    auto yFuture1 = std::async(std::launch::async, [&](){ return yController.rotateMotorBySteps(135*8, 1000, true); });
    xFuture1.get();
    yFuture1.get();

    auto xFuture2 = std::async(std::launch::async, [&](){ return xController.rotateMotorBySteps(256*8, 1000, false); });
    auto yFuture2 = std::async(std::launch::async, [&](){ return yController.rotateMotorBySteps(85*8, 1000, false); });
    xFuture2.get();
    yFuture2.get();

    xController.resetStepCount();
    yController.resetStepCount();
    xStep = 0;
    yStep = 0;
}

void motorControl(MotorController& controller, bool direction, int duration_ms, int speed) {
    controller.rotateMotorForTime(duration_ms, speed, direction);
}

void faceDetect(PIDController& pidX, PIDController& pidY) {
    int fd = shm_open("/best_box", O_RDONLY, 0666);
    if (fd == -1) {
        std::cerr << "Unable to open shared memory." << std::endl;
        return;
    }

    float* data = (float*)mmap(NULL, 16, PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) {
        std::cerr << "Memory mapping failed." << std::endl;
        close(fd);
        return;
    }

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        float x1 = data[0];
        float y1 = data[1];
        float x2 = data[2];
        float y2 = data[3];

        if (x1 < 0) {
            continue;
        }

        float centerX = (x1 + x2) / 2.0;
        float centerY = (y1 + y2) / 2.0;

        double errorX = centerX - 0.5;
        double errorY = (centerY - 0.5)*0.75;
        if (fabs(errorX) < 0.04) errorX = 0;
        if (fabs(errorY) < 0.03) errorY = 0;
        double controlX = pidX.compute(errorX);
        double controlY = pidY.compute(errorY);

        std::cout << controlX << ' ' << controlY << std::endl;

        double pidXOutput = std::min(controlX, 1.25);
        double pidYOutput = std::min(controlY, 1.25);

        int speedX = static_cast<int>(1000 / pidXOutput);
        int speedY = static_cast<int>(1000 / pidYOutput);

        bool directionX = controlX < 0;
        bool directionY = controlY < 0;

        std::thread xMotorThread(motorControl, std::ref(xController), directionX, 198 - abs(speedX / 1000), abs(speedX));
        std::thread yMotorThread(motorControl, std::ref(yController), directionY, 198 - abs(speedY / 1000), abs(speedY));
        xMotorThread.detach();
        yMotorThread.detach();
    }

    munmap(data, 16);
    close(fd);
}

bool loadPIDConfig(const std::string& filename, double& kp, double& ki, double& kd) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open PID config file." << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, '=')) {
            std::string value;
            if (std::getline(iss, value)) {
                if (key == "kp") {
                    kp = std::stod(value);
                } else if (key == "ki") {
                    ki = std::stod(value);
                } else if (key == "kd") {
                    kd = std::stod(value);
                }
            }
        }
    }

    file.close();
    return true;
}

int main() {
    wiringPiSetup();

    double kp, ki, kd;
    if (!loadPIDConfig("pid_config.txt", kp, ki, kd)) {
        return -1;
    }

    PIDController pidX(kp, ki, kd);
    PIDController pidY(kp, ki, kd);

    int xStep = 0;
    int yStep = 0;

    resetMotor(xController, yController, xStep, yStep);

    std::thread faceDetectThread(faceDetect, std::ref(pidX), std::ref(pidY));

    enableRawMode();
    std::cout << "Press 'q' to quit..." << std::endl;

    while (running) {
        if (std::cin.get() == 'q') {
            running = false;
        }
    }

    disableRawMode();

    faceDetectThread.join();
    xController.stopMotors();
    return 0;
}
