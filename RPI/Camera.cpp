#include "Camera.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <thread>
#include <vector>

struct DetectedBlock {
    std::string color;  // red or green
    cv::Rect boundingBox;
    cv::Point center;
    double area;
    // Maybe add contour if: std::vector<cv::Point> contour;
};

// TODO: Info
// (Habs jetzt als Automaten modelliert, keine Ahnung wie man das von davor so schreiben könnte, dass da was funktionierendes,
// übersichliches rauskommt)
// State Machine
enum class State {
    IDLE,
    DRIVING_STRAIGHT,
    AVOIDING_RED,
    AVOIDING_GREEN,
    TURNING_LEFT,
    // TURNING_RIGHT, // Optional
    CENTERING,
    STOPPED
};

// Constants
// TODO: Color Detection (HSV Ranges - NEED TUNING!)
// (Min/Max Hue, Min/Max Saturation, Min/Max Value)
// Red (wrap around 0/180)
const cv::Scalar LOWER_RED1(0, 120, 70);
const cv::Scalar UPPER_RED1(10, 255, 255);
const cv::Scalar LOWER_RED2(170, 120, 70);
const cv::Scalar UPPER_RED2(180, 255, 255);
// Green
const cv::Scalar LOWER_GREEN(40, 50, 50);
const cv::Scalar UPPER_GREEN(90, 255, 255);

// TODO: Update I don't know the values
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

// TODO: Min and Max Value for how much a contour has to enclose to count as a block
const double MIN_CONTOUR_AREA = 500.0;
const double MAX_CONTOUR_AREA = 50000.0;

// Driving Control Parameters
const int FORWARD_SPEED = 120;    // Motor speed (0-255) when driving straight
const int AVOIDANCE_SPEED = 100;  // Motor speed when avoiding
const int TURN_SPEED = 100;       // Motor speed during 90-degree turns

// TODO: setup
const int SERVO_CENTER = 90;
const int SERVO_MAX_LEFT = 45;
const int SERVO_MAX_RIGHT = 135;
// How aggressively to steer towards avoidance target point: higher is more aggressive
const double STEERING_GAIN = 0.8;
// Define how far (horizont) a detected block center can be from direct center of camera view before car
// considers it "in the path" and triggers an avoidance
const double AVOIDANCE_THRESHOLD_X_RATIO = 0.3;
// Proximity filter based on size of detected block, prevent car from reacting to blocks that are too small
const int AVOIDANCE_DISTANCE_THRESHOLD_HEIGHT = static_cast<int>(FRAME_HEIGHT * 0.15);
// distance, by front ultrasonic sensor, at which car should stop driving straight and turn left
const int WALL_DETECT_FRONT_TURN = 35;
const double TURN_DURATION_SEC = 2.0;

// Global variales
// Current state of Car
State currentState = State::DRIVING_STRAIGHT;
std::chrono::steady_clock::time_point turnStartTime;
std::chrono::steady_clock::time_point lastSensorReadTime;
int avoidanceTargetX = -1;  // Target horizontal pixel position during avoidance
int sidesDriven = 0;        // How many sides of the square completed

// TODO for NN2, warp sensor data into a class or do something else, idk
// TODO I used following:
//
SensorData currentSensorData;

// --------------------------------------------------------------------------
// --------------------------------------------------------------------------

wro::Camera::Camera() {
    if (!camera.open("libcamerasrc ! videoconvert ! appsink", cv::CAP_GSTREAMER)) {
        std::cout << "Failed to open camera device\n";
        return;
    }
}

wro::Camera::Camera() {
    if (camera.isOpened()) {
        camera.release();
        std::cout << "Camera released" << std::endl;
    }
    return;
}

// Get list of detected blocks
std::vector<DetectedBlock> wro::Camera::detectedBlocks(const cv::Mat& frame) {
    cv::Mat hsv;
    cv::Mat blurred;

    // Gaussian Blur to reduce image noise
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
    // Convert to HSV color space, so that color can be detected better even with different lighting
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    // Create masks
    cv::Mat maskRed1, maskRed2, maskRed, maskGreen;
    // Isolate pixels that match first range for red
    cv::inRange(hsv, LOWER_RED1, UPPER_RED1, maskRed1);
    cv::inRange(hsv, LOWER_RED2, UPPER_RED2, maskRed2);
    cv::bitwise_or(maskRed1, maskRed2, maskRed);
    // Isolate pixels that match first range for green
    cv::inRange(hsv, LOWER_GREEN, UPPER_GREEN, maskGreen);

    // Found this somewhere to clean the mask images
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // Shrinks white areas slightly
    cv::morphologyEx(maskRed, maskRed, cv::MORPH_OPEN, kernel);
    // Expand white areas slightly (removes small pixels that are alone)
    cv::morphologyEx(maskRed, maskRed, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(maskGreen, maskGreen, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(maskGreen, maskGreen, cv::MORPH_CLOSE, kernel);

    std::vector<DetectedBlock> blocks;
    std::vector<std::vector<cv::Point>> contours;

    // Find contours for red (outlines)
    cv::findContours(maskRed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // For each outline
    for (const auto& contour : contours) {
        // Calculate area enclosed by contour
        double area = cv::contourArea(contour);
        // If valid area
        if (area > MIN_CONTOUR_AREA && area < MAX_CONTOUR_AREA) {
            DetectedBlock block;
            block.color = "red";
            // Get smallest standing rectangle as bounding box
            block.boundingBox = cv::boundingRect(contour);
            block.center = cv::Point(block.boundingBox.x + block.boundingBox.width / 2, block.boundingBox.y + block.boundingBox.height / 2);
            block.area = area;
            blocks.push_back(block);
        }
    }

    // Find contours for Green
    contours.clear();  // Clear contours from red
    cv::findContours(maskGreen, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& cnt : contours) {
        // Calculate area enclosed by contour
        double area = cv::contourArea(cnt);
        // If valid area
        if (area > MIN_CONTOUR_AREA && area < MAX_CONTOUR_AREA) {
            DetectedBlock block;
            block.color = "green";
            // Get smallest standing rectangle as bounding box
            block.boundingBox = cv::boundingRect(cnt);
            block.center = cv::Point(block.boundingBox.x + block.boundingBox.width / 2, block.boundingBox.y + block.boundingBox.height / 2);
            block.area = area;
            blocks.push_back(block);
        }
    }

    // Sort blocks by area - largest first (to see which is most important)
    std::sort(blocks.begin(), blocks.end(), [](const DetectedBlock& a, const DetectedBlock& b) { return a.area > b.area; });

    return blocks;
}

int wro::Camera::calculateSteeringAngle(int frameWidth, int targetXHorizontal, int currentXHorizontal) {
    // Calculate Error (difference in pixels)
    double error = static_cast<double>(targetXHorizontal - currentXHorizontal);
    double normedError = error / (static_cast<double>(frameWidth) / 2.0);

    // Calculate steering adjustment based on Error and Gain
    double steeringAdjustment = normedError * (SERVO_MAX_RIGHT - SERVO_CENTER) * STEERING_GAIN;

    // Calculate Target Servo Angle
    int targetAngle = static_cast<int>(round(SERVO_CENTER + steeringAdjustment));

    // Servo limits
    targetAngle = std::max(SERVO_MAX_LEFT, std::min(SERVO_MAX_RIGHT, targetAngle));

    return targetAngle;
}

void wro::Camera::run() {
    const int frameCenterX = FRAME_WIDTH / 2;
    const int avoidanceThresholdPixels = static_cast<int>(FRAME_WIDTH * AVOIDANCE_THRESHOLD_X_RATIO);

    cv::Mat frame;
    bool running = true;

    while (running) {
        if (!camera.read(frame)) {
            std::cerr << "Failed to read frame\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // TODO: Get distance sensor data here, to tired to implement this now
        // Read Sensors Periodically
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSensorReadTime).count() > 200) {
            // TODO: getSensorData() does not exist
            // And the class for SensorData also not
            currentSensorData = getSensorData();
            lastSensorReadTime = now;
        }

        // Get blocks
        std::vector<DetectedBlock> blocks = wro::Camera::detectedBlocks(frame);

        int motorSpeed = 0;
        int steeringAngle = SERVO_CENTER;
        std::string stateString = "UNKNOWN";  // For logging

        // Emergency Stop (Front Sensor)
        if (currentSensorData.valid && currentSensorData.front < 20) {
            if (currentState != State::STOPPED) {
                std::cout << "EMERGENCY STOP - Obstacle too close! State: " << (int)currentState << std::endl;
                currentState = State::STOPPED;
            }
        }

        switch (currentState) {
            case State::DRIVING_STRAIGHT:
                stateString = "DRIVING_STRAIGHT";
                motorSpeed = FORWARD_SPEED;
                steeringAngle = SERVO_CENTER;

                // 1. Check for corners (Front sensor)
                if (currentSensorData.valid && currentSensorData.front < WALL_DETECT_FRONT_TURN) {
                    std::cout << "Wall detected on front, turn left now" << std::endl;
                    currentState = State::TURNING_LEFT;
                    turnStartTime = std::chrono::steady_clock::now();
                    motorSpeed = TURN_SPEED;
                    steeringAngle = SERVO_MAX_LEFT;
                }
                // 2. Check for blocks to avoid
                else if (!blocks.empty()) {
                    const auto& closest_block = blocks[0];  // Sorted by area
                    bool is_close_enough = closest_block.boundingBox.height > AVOIDANCE_DISTANCE_THRESHOLD_HEIGHT;
                    bool is_in_path = std::abs(closest_block.center.x - frameCenterX) < avoidanceThresholdPixels;

                    if (is_close_enough && is_in_path) {
                        // Red block
                        if (closest_block.color == "red") {
                            std::cout << "Red block detected in path. Avoiding on RIGHT" << std::endl;
                            currentState = State::AVOIDING_RED;
                            // Aim a bit to the right of blocks right edge
                            avoidanceTargetX =
                                closest_block.boundingBox.x + closest_block.boundingBox.width + static_cast<int>(FRAME_WIDTH * 0.2);
                            motorSpeed = AVOIDANCE_SPEED;
                        } else {  // Green block
                            std::cout << "Green block detected in path. Avoiding on LEFT" << std::endl;
                            currentState = State::AVOIDING_GREEN;
                            // Aim a bit to the left of blocks left edge
                            avoidanceTargetX = closest_block.boundingBox.x - static_cast<int>(FRAME_WIDTH * 0.2);
                            motorSpeed = AVOIDANCE_SPEED;
                        }
                        // Clamp target in frame bounds
                        avoidanceTargetX = std::max(0, std::min(FRAME_WIDTH - 1, avoidanceTargetX));
                        std::cout << "Avoidance Target X: " << avoidanceTargetX << std::endl;
                    }
                }
                break;

            case State::AVOIDING_RED:  // Pass Right
                stateString = "AVOIDING_RED";
                motorSpeed = AVOIDANCE_SPEED;
                {
                    bool found_red = false;
                    for (const auto& block : blocks) {
                        if (block.color == "red") {
                            // Assume the largest/first red block is to be avoided
                            cv::rectangle(frame, block.boundingBox, cv::Scalar(0, 0, 255), 2);
                            steeringAngle = wro::Camera::calculateSteeringAngle(FRAME_WIDTH, avoidanceTargetX, frameCenterX);

                            // Check if block center has passed to the left
                            if (block.center.x < frameCenterX - static_cast<int>(FRAME_WIDTH * 0.1)) {
                                std::cout << "Red block seems passed CENTERING now" << std::endl;
                                currentState = State::CENTERING;
                                avoidanceTargetX = -1;  // Clear target
                            }
                            found_red = true;
                            break;  // Only process first block
                        }
                    }
                    if (!found_red) {
                        std::cout << "Lost red block while avoiding: CENTERING" << std::endl;
                        currentState = State::CENTERING;
                        steeringAngle = SERVO_CENTER;
                    }
                }
                break;

            case State::AVOIDING_GREEN:  // Pass Left
                stateString = "AVOIDING_GREEN";
                motorSpeed = AVOIDANCE_SPEED;
                {
                    bool found_green = false;
                    for (const auto& block : blocks) {
                        if (block.color == "green") {
                            cv::rectangle(frame, block.boundingBox, cv::Scalar(0, 255, 0), 2);
                            steeringAngle = wro::Camera::calculateSteeringAngle(FRAME_WIDTH, avoidanceTargetX, frameCenterX);

                            // Check if block center has passed to the right
                            if (block.center.x > frameCenterX + static_cast<int>(FRAME_WIDTH * 0.1)) {
                                std::cout << "Green block seems passed CENTERING now" << std::endl;
                                currentState = State::CENTERING;
                                avoidanceTargetX = -1;  // Clear target
                            }
                            found_green = true;
                            break;  // Only process first block
                        }
                    }
                    if (!found_green) {
                        std::cout << "Lost green block while avoiding: CENTERING" << std::endl;
                        currentState = State::CENTERING;
                        steeringAngle = SERVO_CENTER;
                    }
                }
                break;

            case State::CENTERING:
                stateString = "CENTERING";
                motorSpeed = FORWARD_SPEED;
                steeringAngle = wro::Camera::calculateSteeringAngle(FRAME_WIDTH, frameCenterX, frameCenterX);
                // Transition back when steering is close to center
                if (std::abs(steeringAngle - SERVO_CENTER) < 5) {
                    std::cout << "Centering done now DRIVING_STRAIGHT" << std::endl;
                    currentState = State::DRIVING_STRAIGHT;
                }
                break;

            case State::TURNING_LEFT:
                stateString = "TURNING_LEFT (" + std::to_string(sidesDriven) + ")";
                motorSpeed = TURN_SPEED;
                steeringAngle = SERVO_MAX_LEFT;
                {
                    auto turn_elapsed =
                        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - turnStartTime);
                    if (turn_elapsed.count() > TURN_DURATION_SEC) {
                        std::cout << "Turn complete" << std::endl;
                        sidesDriven++;
                        // TODO: Change amount
                        if (sidesDriven >= 4) {
                            std::cout << "STOPPED, finished laps" << std::endl;
                            currentState = State::STOPPED;
                            motorSpeed = 0;
                            steeringAngle = SERVO_CENTER;
                        } else {
                            currentState = State::DRIVING_STRAIGHT;
                            steeringAngle = SERVO_CENTER;  // Straighten wheels
                            motorSpeed = FORWARD_SPEED;    // Start driving straight
                        }
                    }
                }
                break;

            case State::STOPPED:
                stateString = "STOPPED";
                motorSpeed = 0;
                steeringAngle = SERVO_CENTER;
                // TODO: Now it stays stopped unless manually restarted, maybe add logic with button
                break;

            default:  // Include IDLE or any other state
                stateString = "IDLE/UNKNOWN";
                motorSpeed = 0;
                steeringAngle = SERVO_CENTER;
                // Default to stopping
                currentState = State::STOPPED;
                break;
        }
    }

    // TODO: here turn and drive with motorSpeed and steeringAngle
}
