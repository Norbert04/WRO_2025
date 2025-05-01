#include "Camera.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <thread>
#include <vector>

#include "DistanceSensor.h"
#include "Robot.h"
#include "utils.h"

// TODO: Info
// (Habs jetzt als Automaten modelliert, keine Ahnung wie man das von davor so schreiben könnte, dass da was funktionierendes,
// übersichliches rauskommt)

// Constants
// TODO: Colour Detection (HSV Ranges - NEED TUNING!)
// (Min/Max Hue, Min/Max Saturation, Min/Max Value)
// Red (wrap around 0/180)
const cv::Scalar LOWER_RED1(0, 120, 70);
const cv::Scalar UPPER_RED1(10, 255, 255);
const cv::Scalar UPPER_RED2(180, 255, 255);
const cv::Scalar LOWER_RED2(170, 120, 70);
// Green
const cv::Scalar LOWER_GREEN(40, 50, 50);
const cv::Scalar UPPER_GREEN(90, 255, 255);

// TODO: Update I don't know the values
constexpr int FRAME_WIDTH = 640;
constexpr int FRAME_HEIGHT = 480;

// TODO: Min and Max Value for how much a contour has to enclose to count as a block
constexpr double MIN_CONTOUR_AREA = 500.0;
constexpr double MAX_CONTOUR_AREA = 50000.0;

// How aggressively to steer towards avoidance target point: higher is more aggressive
constexpr double STEERING_GAIN = 0.8;
// Define how far (horizont) a detected block centre can be from direct centre of camera view before car
// considers it "in the path" and triggers an avoidance
constexpr double AVOIDANCE_THRESHOLD_X_RATIO = 0.3;
// Proximity filter based on size of detected block, prevent car from reacting to blocks that are too small
constexpr int AVOIDANCE_DISTANCE_THRESHOLD_HEIGHT = static_cast<int>(FRAME_HEIGHT * 0.15);

// Global variables
std::chrono::steady_clock::time_point turnStartTime;
std::chrono::steady_clock::time_point lastSensorReadTime;
int avoidanceTargetX = -1;  // Target horizontal pixel position during avoidance
int sidesDriven = 0;        // How many sides of the square completed

// TODO for NN2, warp sensor data into a class or do something else, idk
// TODO I used following:
//
wro::DistanceSensor sensor;

// --------------------------------------------------------------------------
// --------------------------------------------------------------------------

wro::Camera::Camera(wro::Robot* robot)
	:robot(robot)
{
	if (!camera.open("libcamerasrc ! videoconvert ! appsink", cv::CAP_GSTREAMER))
	{
		std::cerr << "Failed to open camera device\n";
		return;
	}
}

wro::Camera::~Camera()
{
	if (camera.isOpened())
	{
		camera.release();
		DEBUG_PRINTLN("Camera released");
	}
}

// Get list of detected blocks
std::vector<DetectedBlock> wro::Camera::detectedBlocks(const cv::Mat& frame)
{
	cv::Mat hsv;
	cv::Mat blurred;

	// Gaussian Blur to reduce image noise
	cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
	// Convert to HSV colour space, so that colour can be detected better even with different lighting
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
	for (const auto& contour : contours)
	{
		// Calculate area enclosed by contour
		double area = cv::contourArea(contour);
		// If valid area
		if (area > MIN_CONTOUR_AREA && area < MAX_CONTOUR_AREA)
		{
			DetectedBlock block;
			block.colour = "red";
			// Get smallest standing rectangle as bounding box
			block.boundingBox = cv::boundingRect(contour);
			block.centre = cv::Point(block.boundingBox.x + block.boundingBox.width / 2, block.boundingBox.y + block.boundingBox.height / 2);
			block.area = area;
			blocks.push_back(block);
		}
	}

	// Find contours for Green
	contours.clear();  // Clear contours from red
	cv::findContours(maskGreen, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	for (const auto& cnt : contours)
	{
		// Calculate area enclosed by contour
		double area = cv::contourArea(cnt);
		// If valid area
		if (area > MIN_CONTOUR_AREA && area < MAX_CONTOUR_AREA)
		{
			DetectedBlock block;
			block.colour = "green";
			// Get smallest standing rectangle as bounding box
			block.boundingBox = cv::boundingRect(cnt);
			block.centre = cv::Point(block.boundingBox.x + block.boundingBox.width / 2, block.boundingBox.y + block.boundingBox.height / 2);
			block.area = area;
			blocks.push_back(block);
		}
	}

	// Sort blocks by area - largest first (to see which is most important)
	std::sort(blocks.begin(), blocks.end(), [](const DetectedBlock& a, const DetectedBlock& b) { return a.area > b.area; });

	return blocks;
}

int wro::Camera::calculateSteeringAngle(int frameWidth, int targetXHorizontal, int currentXHorizontal)
{
	// Calculate Error (difference in pixels)
	double error = static_cast<double>(targetXHorizontal - currentXHorizontal);
	double normedError = error / (static_cast<double>(frameWidth) / 2.0);

	// Calculate steering adjustment based on Error and Gain
	double steeringAdjustment = normedError * (wro::Robot::SERVO_MAX_RIGHT - wro::Robot::SERVO_CENTRE) * STEERING_GAIN;

	// Calculate Target Servo Angle
	int targetAngle = static_cast<int>(round(wro::Robot::SERVO_CENTRE + steeringAdjustment));

	// Servo limits
	targetAngle = std::max(wro::Robot::SERVO_MAX_LEFT, std::min(wro::Robot::SERVO_MAX_RIGHT, targetAngle));

	return targetAngle;
}

short wro::Camera::getSteeringAngle()
{
	const int frameCentreX = FRAME_WIDTH / 2;
	const int avoidanceThresholdPixels = static_cast<int>(FRAME_WIDTH * AVOIDANCE_THRESHOLD_X_RATIO);

	cv::Mat frame;

	if (!camera.read(frame))
	{
		std::cerr << "Failed to read frame\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return wro::Robot::SERVO_CENTRE;
	}

	// TODO: Get distance sensor data here, to tired to implement this now
	// Read Sensors Periodically
	auto now = std::chrono::steady_clock::now();

	// Get blocks
	std::vector<DetectedBlock> blocks = wro::Camera::detectedBlocks(frame);

	short steeringAngle = wro::Robot::SERVO_CENTRE;
	std::string stateString = "UNKNOWN";  // For logging

	switch (robot->currentState)
	{
	case State::DRIVING_STRAIGHT:
		stateString = "DRIVING_STRAIGHT";

		// Check for blocks to avoid
		if (!blocks.empty())
		{
			const auto& closest_block = blocks[0];  // Sorted by area
			bool is_close_enough = closest_block.boundingBox.height > AVOIDANCE_DISTANCE_THRESHOLD_HEIGHT;
			bool is_in_path = std::abs(closest_block.centre.x - frameCentreX) < avoidanceThresholdPixels;

			if (is_close_enough && is_in_path)
			{
				// Red block
				if (closest_block.colour == "red")
				{
					DEBUG_PRINTLN("Red block detected in path. Avoiding on RIGHT");
					robot->currentState = State::AVOIDING_RED;
					// Aim a bit to the right of blocks right edge
					avoidanceTargetX =
						closest_block.boundingBox.x + closest_block.boundingBox.width + static_cast<int>(FRAME_WIDTH * 0.2);
				}
				else
				{  // Green block
					DEBUG_PRINTLN("Green block detected in path. Avoiding on LEFT");
					robot->currentState = State::AVOIDING_GREEN;
					// Aim a bit to the left of blocks left edge
					avoidanceTargetX = closest_block.boundingBox.x - static_cast<int>(FRAME_WIDTH * 0.2);
				}
				// Clamp target in frame bounds
				avoidanceTargetX = std::max(0, std::min(FRAME_WIDTH - 1, avoidanceTargetX));
				DEBUG_PRINTLN("Avoidance Target X: " << avoidanceTargetX);
			}
		}
		break;

	case State::AVOIDING_RED:  // Pass Right
		stateString = "AVOIDING_RED";
		{
			bool found_red = false;
			for (const auto& block : blocks)
			{
				if (block.colour == "red")
				{
					// Assume the largest/first red block is to be avoided
					cv::rectangle(frame, block.boundingBox, cv::Scalar(0, 0, 255), 2);
					steeringAngle = wro::Camera::calculateSteeringAngle(FRAME_WIDTH, avoidanceTargetX, frameCentreX);

					// Check if block centre has passed to the left
					if (block.centre.x < frameCentreX - static_cast<int>(FRAME_WIDTH * 0.1))
					{
						DEBUG_PRINTLN("Red block seems passed CENTERING now");
						robot->currentState = State::CENTERING;
						avoidanceTargetX = -1;  // Clear target
					}
					found_red = true;
					break;  // Only process first block
				}
			}
			if (!found_red)
			{
				DEBUG_PRINTLN("Lost red block while avoiding: CENTERING");
				robot->currentState = State::CENTERING;
				steeringAngle = wro::Robot::SERVO_CENTRE;
			}
		}
		break;

	case State::AVOIDING_GREEN:  // Pass Left
		stateString = "AVOIDING_GREEN";
		{
			bool found_green = false;
			for (const DetectedBlock& block : blocks)
			{
				if (block.colour == "green")
				{
					cv::rectangle(frame, block.boundingBox, cv::Scalar(0, 255, 0), 2);
					steeringAngle = wro::Camera::calculateSteeringAngle(FRAME_WIDTH, avoidanceTargetX, frameCentreX);

					// Check if block centre has passed to the right
					if (block.centre.x > frameCentreX + static_cast<int>(FRAME_WIDTH * 0.1))
					{
						DEBUG_PRINTLN("Green block seems passed CENTERING now");
						robot->currentState = State::CENTERING;
						avoidanceTargetX = -1;  // Clear target
					}
					found_green = true;
					break;  // Only process first block
				}
			}
			if (!found_green)
			{
				DEBUG_PRINTLN("Lost green block while avoiding: CENTERING");
				robot->currentState = State::CENTERING;
				steeringAngle = wro::Robot::SERVO_CENTRE;
			}
		}
		break;

	case State::CENTERING:
		stateString = "CENTERING";
		steeringAngle = wro::Camera::calculateSteeringAngle(FRAME_WIDTH, frameCentreX, frameCentreX);
		// Transition back when steering is close to centre
		if (std::abs(steeringAngle - wro::Robot::SERVO_CENTRE) < 5)
		{
			DEBUG_PRINTLN("Centring done now DRIVING_STRAIGHT");
			robot->currentState = State::DRIVING_STRAIGHT;
		}
		break;

	case State::TURNING_LEFT:
		stateString = "TURNING_LEFT (" + std::to_string(sidesDriven) + ")";
		steeringAngle = wro::Robot::SERVO_MAX_LEFT;
		{
			auto turn_elapsed =
				std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - turnStartTime);
			if (turn_elapsed.count() > wro::Robot::TURN_DURATION_SEC)
			{
				DEBUG_PRINTLN("Turn complete");
				sidesDriven++;
				// TODO: Change amount
				if (sidesDriven >= 4)
				{
					DEBUG_PRINTLN("STOPPED, finished laps");
					robot->currentState = State::STOPPED;
					steeringAngle = wro::Robot::SERVO_CENTRE;
				}
				else
				{
					robot->currentState = State::DRIVING_STRAIGHT;
					steeringAngle = wro::Robot::SERVO_CENTRE;  // Straighten wheels
				}
			}
		}
		break;

	case State::STOPPED:
		stateString = "STOPPED";
		steeringAngle = wro::Robot::SERVO_CENTRE;
		// TODO: Now it stays stopped unless manually restarted, maybe add logic with button
		break;

	default:  // Include IDLE or any other state
		stateString = "IDLE/UNKNOWN";
		steeringAngle = wro::Robot::SERVO_CENTRE;
		// Default to stopping
		robot->currentState = State::STOPPED;
		break;
	}
	return steeringAngle;
}
