#pragma once

#include <libcamera/libcamera.h>
#include <memory>
#include <vector>
#include <iostream>

namespace wro
{
	class Camera
	{
	public:
		Camera();
		~Camera();

	private:
		std::shared_ptr<libcamera::Camera> camera;
		libcamera::CameraManager manager;
		libcamera::CameraConfiguration config;
		FrameBufferAllocator* allocator;
		libcamera::StreamConfiguration& streamConfig;
	};
}