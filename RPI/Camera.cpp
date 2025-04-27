#include "Camera.h"

#include <iostream>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

// I have literally no idea if this will work

wro::Camera::Camera()
{
	if (!camera.open("libcamerasrc ! videoconvert ! appsink", cv::CAP_GSTREAMER))
	{
		std::cerr << "Failed to open camera device\n";
		return;
	}
}

void wro::Camera::detectAndStartTracking(const cv::Mat& hsv, const cv::Mat& frame)
{
	// Red & green HSV ranges
	cv::Scalar lower_red1(0, 120, 70), upper_red1(10, 255, 255);
	cv::Scalar lower_red2(170, 120, 70), upper_red2(180, 255, 255);
	cv::Scalar lower_green(40, 40, 40), upper_green(90, 255, 255);

	// Masks
	cv::Mat mask_red1;
	cv::Mat mask_red2;
	cv::Mat mask_red;
	cv::Mat mask_green;
	cv::Mat mask;

	cv::inRange(hsv, lower_red1, upper_red1, mask_red1);
	cv::inRange(hsv, lower_red2, upper_red2, mask_red2);
	cv::bitwise_or(mask_red1, mask_red2, mask_red);
	cv::inRange(hsv, lower_green, upper_green, mask_green);
	cv::bitwise_or(mask_red, mask_green, mask);

	std::vector<std::vector<cv::Point>> contours;
	// Find contours in mask
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// For each contour
	for (const auto& cnt : contours)
	{
		double area = contourArea(cnt);
		// If more than 1000 pixels, then count as object
		// TODO: test values
		if (area > 1000)
		{
			cv::Rect box = boundingRect(cnt);
			tracker = cv::TrackerKCF::create();
			tracker->init(frame, box);
			trackedBox = box;
			tracking = true;
			std::cout << "Started tracking obstacle\n";
			break; // Track one obstacle at a time
		}
	}
}

float wro::Camera::calculateSteeringFromTrackedObject(int frameWidth)
{
	int cx = trackedBox.x + trackedBox.width / 2;
	return float(cx - frameWidth / 2) / (frameWidth / 2); // -1 left to +1 right
}

void wro::Camera::runTest()
{
	cv::Mat frame;

	while (true)
	{
		// TODO replace with run()?
		if (!camera.read(frame))
		{
			std::cerr << "Failed to read frame\n";
		}

		cv::Mat blurred;
		// Result image
		cv::Mat hsv;
		cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
		cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

		if (!tracking)
		{
			detectAndStartTracking(hsv, frame);
		}
		else
		{
			bool ok = tracker->update(frame, trackedBox);
			if (ok)
			{
				// Draw tracking box
				cv::rectangle(frame, trackedBox, cv::Scalar(255, 0, 0), 2);

				float steer = calculateSteeringFromTrackedObject(frame.cols);

				// TODO: implement steering and return to center after passing object
				if (abs(steer) < 0.2)
				{
					std::cout << "Obstacle in front" << std::endl;
				}
				else if (steer < 0)
				{
					std::cout << "Obstacle on left, steer=" << steer << ")\n";
				}
				else
				{
					std::cout << "Obstacle on right, steer=" << steer << ")\n";
				}
			}
			else
			{
				std::cout << "Lost tracking\n";
				tracking = false;
			}
		}
	}
}

void wro::Camera::run()
{
	cv::Mat frame;
	if (!camera.read(frame))
	{
		std::cerr << "Failed to read frame\n";
		return;
	}

	cv::Mat blurred;
	// Result image
	cv::Mat hsv;
	cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
	cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

	if (!tracking)
	{
		detectAndStartTracking(hsv, frame);
	}
	else
	{
		bool ok = tracker->update(frame, trackedBox);
		if (ok)
		{
			// Draw tracking box
			cv::rectangle(frame, trackedBox, cv::Scalar(255, 0, 0), 2);

			float steer = calculateSteeringFromTrackedObject(frame.cols);

			// TODO: implement steering and return to center after passing object
			if (abs(steer) < 0.2)
			{
				std::cout << "Obstacle in front" << std::endl;
			}
			else if (steer < 0)
			{
				std::cout << "Obstacle on left, steer=" << steer << ")\n";
			}
			else
			{
				std::cout << "Obstacle on right, steer=" << steer << ")\n";
			}
		}
		else
		{
			std::cout << "Lost tracking\n";
			tracking = false;
		}
	}
}

std::optional<bool> wro::Camera::estimateDrivingDirection()
{
	// TODO implement
	return std::optional<bool>();
}
