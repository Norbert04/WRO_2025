#include "Camera.h"

#include <iostream>

wro::Camera::Camera()
{
	if (!camera.open(0))
	{
		std::cerr << "Failed to open camera device\n";
		return;
	}
}

wro::Camera::~Camera()
{
}

bool wro::Camera::getImage(cv::Mat& image)
{
	return camera.read(image);
}
