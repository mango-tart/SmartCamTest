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
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <semaphore.h>
#include <vector>
#include <drogon/drogon.h>
#include "UltraFace.hpp"
#include "MotorController.hpp"

std::atomic<bool> running(true);
std::atomic<bool> newFrameForFaceDetectThread(false);
std::atomic<bool> newDataAvailable(false);
std::atomic<bool> faceDetectRunning(false);
std::atomic<int> motorControlMode(0); // 0: 休眠, 1: 自动追踪, 2: 手动控制
std::atomic<bool> upButtonPressed(false);
std::atomic<bool> downButtonPressed(false);
std::atomic<bool> leftButtonPressed(false);
std::atomic<bool> rightButtonPressed(false);

std::thread faceDetectThread;
MotorController xController = MotorController::selectMotor(false);
MotorController yController = MotorController::selectMotor(true);
sem_t* sem_newFrame;
sem_t* sem_processedFrame;
float* detectedBox;
int cam_shm_fd;
float faceLocationX = 0.0;
float faceLocationY = 0.0;
void* cam_shm_base;
int xStep = 0;
int yStep = 0;

class PIDController {
public:
    float kp, ki, kd;
    float previous_error;
    float integral;

    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0) {}

    float compute(float error) {
        integral += error;
        float derivative = error - previous_error;
        previous_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

void initSemaphores() {
    sem_unlink("sem_newFrame");
    sem_unlink("sem_processedFrame");
    sem_newFrame = sem_open("sem_newFrame", O_CREAT, 0666, 0);
    sem_processedFrame = sem_open("sem_processedFrame", O_CREAT, 0666, 0);
    if (sem_newFrame == SEM_FAILED || sem_processedFrame == SEM_FAILED) {
        std::cerr << "Failed to open semaphores." << std::endl;
        exit(EXIT_FAILURE);
    }
}

void cleanupSemaphores() {
    sem_close(sem_newFrame);
    sem_close(sem_processedFrame);
    sem_unlink("sem_newFrame");
    sem_unlink("sem_processedFrame");
}

void initSharedMemory() {
    cam_shm_fd = shm_open("/cam_frame", O_CREAT | O_RDWR, 0666);
    ftruncate(cam_shm_fd, 320 * 240 * 3);
    cam_shm_base = mmap(NULL, 320 * 240 * 3, PROT_READ | PROT_WRITE, MAP_SHARED, cam_shm_fd, 0);
}

void cleanupSharedMemory() {
    munmap(cam_shm_base, 320 * 240 * 3);
    close(cam_shm_fd);
    shm_unlink("/cam_frame");
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

void getCamFrame() {
    CURL* curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:8080/?action=snapshot");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    std::string readBuffer;
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

    while (running) {
        readBuffer.clear();
        CURLcode res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            std::vector<uchar> data(readBuffer.begin(), readBuffer.end());
            cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);
            if (!img.empty()) {
                cv::resize(img, img, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
                memcpy(cam_shm_base, img.data, 320 * 240 * 3);
                newFrameForFaceDetectThread = true;
            } else {
                std::cerr << "Failed to decode the image." << std::endl;
            }
        } else {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    curl_easy_cleanup(curl);
}

void faceDetectionTask() {
    UltraFace ultraface("/home/code/main/model/version-slim/slim-320-quant-ADMM-50.mnn", 320, 240, 4, 0.65);
    while (faceDetectRunning) {
        if (newFrameForFaceDetectThread) {
            newFrameForFaceDetectThread = false;
            cv::Mat frame(240, 320, CV_8UC3, cam_shm_base);
            std::vector<FaceInfo> face_info;
            ultraface.detect(frame, face_info);

            float max_width = 0;
            FaceInfo largest_face;
            for (auto &face : face_info) {
                float width = face.x2 - face.x1;
                if (width > max_width) {
                    max_width = width;
                    largest_face = face;
                }
            }

            if (max_width > 0) {
                faceLocationX = (largest_face.x1 + largest_face.x2) / 640.0;
                faceLocationY = (largest_face.y1 + largest_face.y2) / 480.0;
                newDataAvailable = true;
            }
        } else {
            usleep(10000);
        }
    }
}

void motorControlTask(PIDController& pidX, PIDController& pidY) {
    while (running) {
        if (motorControlMode == 0) {

        } else if (motorControlMode == 1) {
            if (newDataAvailable) {
                newDataAvailable = false;
                float errorX = (abs(faceLocationX - 0.5) > 0.04) * (faceLocationX - 0.5);
                float errorY = (abs(faceLocationY - 0.5) > 0.04) * (faceLocationY - 0.5) * 0.75;
                float controlX = pidX.compute(errorX);
                float controlY = pidY.compute(errorY);
                int delayX = abs(1000.0 / controlX);
                int delayY = abs(1000.0 / controlY);
                std::thread xMotorThread(&MotorController::rotateMotorForTime, &xController, 198-delayX/1000, delayX, (controlX<0));
                std::thread yMotorThread(&MotorController::rotateMotorForTime, &yController, 198-delayY/1000, delayY, (controlY<0));
                xMotorThread.detach();
                yMotorThread.detach();
            }
        } else if (motorControlMode == 2) {
            if (upButtonPressed) {
                upButtonPressed = false;
                std::thread yMotorThread(&MotorController::rotateMotorForTime, &yController, 195, 2000, true);
                yMotorThread.detach();
            } else if (downButtonPressed) {
                downButtonPressed = false;
                std::thread yMotorThread(&MotorController::rotateMotorForTime, &yController, 195, 2000, false);
                yMotorThread.detach();
            } else if (leftButtonPressed) {
                leftButtonPressed = false;
                std::thread xMotorThread(&MotorController::rotateMotorForTime, &xController, 195, 2000, true);
                xMotorThread.detach();
            } else if (rightButtonPressed) {
                rightButtonPressed = false;
                std::thread xMotorThread(&MotorController::rotateMotorForTime, &xController, 195, 2000, false);
                xMotorThread.detach();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void setMotorControlMode(int mode) {
    motorControlMode = mode;
}

void startDrogon() {
    drogon::app().addListener("0.0.0.0", 8081);
    drogon::app().registerHandler("/drogon/start_face_detect", [](const drogon::HttpRequestPtr& req,
                                                           std::function<void (const drogon::HttpResponsePtr &)> &&callback) {
        if (!faceDetectRunning) {
            faceDetectRunning = true;
            faceDetectThread = std::thread(faceDetectionTask);
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setBody("Face detection started.");
            callback(resp);
        } else {
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setBody("Face detection is already running.");
            callback(resp);
        }
    });
    drogon::app().registerHandler("/drogon/stop_face_detect", [](const drogon::HttpRequestPtr& req,
                                                          std::function<void (const drogon::HttpResponsePtr &)> &&callback) {
        if (faceDetectRunning) {
            faceDetectRunning = false;
            if (faceDetectThread.joinable()) {
                faceDetectThread.join();
            }
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setBody("Face detection stopped.");
            callback(resp);
        } else {
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setBody("Face detection is not running.");
            callback(resp);
        }
    });
    drogon::app().registerHandler("/drogon/set_motor_mode", [](const drogon::HttpRequestPtr& req,
                                                               std::function<void (const drogon::HttpResponsePtr &)> &&callback) {
        auto json = req->getJsonObject();
        if (json) {
            int mode = (*json)["mode"].asInt();
            setMotorControlMode(mode);
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setBody("Motor control mode set.");
            callback(resp);
        }
    });
    drogon::app().registerHandler("/drogon/set_button_state", [](const drogon::HttpRequestPtr& req,
                                                            std::function<void (const drogon::HttpResponsePtr &)> &&callback) {
        auto json = req->getJsonObject();
        if (json) {
            upButtonPressed = (*json)["up"].asBool();
            downButtonPressed = (*json)["down"].asBool();
            leftButtonPressed = (*json)["left"].asBool();
            rightButtonPressed = (*json)["right"].asBool();
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setBody("Button state updated.");
            callback(resp);
        }
    });
    drogon::app().run();
}

int main() {
    wiringPiSetup();
    initSemaphores();
    initSharedMemory();

    PIDController pidX(1.5, 0.0025, 0.0025);
    PIDController pidY(1.5, 0.0025, 0.0025);

    resetMotor(xController, yController, xStep, yStep);

    std::thread drogonThread(startDrogon);
    std::thread camFrameThread(getCamFrame);
    std::thread motorControlThread(motorControlTask, std::ref(pidX), std::ref(pidY));

    std::cout << "Press 'q' to quit..." << std::endl;
    while (running) {
        if (std::cin.get() == 'q') {
            running = false;
        }
    }

    camFrameThread.join();
    faceDetectThread.join();
    motorControlThread.join();
    drogonThread.join();

    xController.stopMotors();
    cleanupSemaphores();
    cleanupSharedMemory();

    return 0;
}
