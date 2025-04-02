#pragma once

#include <opencv2/opencv.hpp>

namespace wro
{
	class Camera
	{
	public:
		Camera();
		~Camera();
		bool getImage(cv::Mat& image);

	private:
		cv::VideoCapture camera{};
	};
}