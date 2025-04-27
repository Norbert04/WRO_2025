#include <wiringPi.h>

#include "Camera.h"
#include "Robot.h"

int main()
{
	if (wiringPiSetup() != 0)
	{
		std::cerr << "WiringPi setup failed\n";
		return -1;
	}

	// wro::Robot robot = wro::Robot();

	wro::Camera cam{};
	cam.runTest();

	return 0;
}