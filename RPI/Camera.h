#pragma once

#include <opencv2/opencv.hpp>

struct DetectedBlock
{
	std::string colour;  // red or green
	cv::Rect boundingBox;
	cv::Point centre;
	double area;
	// Maybe add contour if: std::vector<cv::Point> contour;
};

namespace wro
{
	class Robot;

	class Camera
	{
	public:
		Camera(wro::Robot* robot);
		~Camera();
		short getSteeringAngle();

	private:
		wro::Robot* robot;

		cv::VideoCapture camera{};

		std::vector<DetectedBlock> detectedBlocks(const cv::Mat& frame);
		int calculateSteeringAngle(int frameWidth, int targetXHorizontal, int currentXHorizontal);
	};
}
#include "Robot.h"