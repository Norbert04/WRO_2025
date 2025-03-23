#include "Camera.h"

wro::Camera::Camera()
	: manager()
{
	manager->start();
	std::vector<std::shared_ptr<libcamera::Camera>> cameras;
	if (cameras.empty())
	{
		std::cerr << "No camera could be detected\n";
	}
	camera = manager->get(cameras[0]->id());
	camera->acquire();
	config = camera->generateConfiguration({ libcamera::StreamRole::Raw });
	allocator = new libcamera::FrameBufferAllocator(camera);
	int res = allocator->allocate(config);
	if (res < 0)
	{
		std::cerr << "buffer allocation failed\n";
	}
	streamConfig = config.at(0);
	config->validate();
	camera->configure(config.get());
}

wro::Camera::~Camera()
{
	camera->release();
	camManager->stop();
}
