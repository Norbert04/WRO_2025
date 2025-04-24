#pragma once

#include <optional>

#include <opencv2/opencv.hpp>

namespace wro
{
	class Camera
	{
	public:
		Camera();

		[[noreturn]] void runTest();
		void run();

		std::optional<bool> estimateDrivingDirection();

	private:
		cv::VideoCapture camera{};
		cv::Ptr<cv::Tracker> tracker;
		bool tracking = false;
		cv::Rect trackedBox;

		void detectAndStartTracking(const cv::Mat& hsv, const cv::Mat& frame);
		float calculateSteeringFromTrackedObject(int frameWidth);
	};
}