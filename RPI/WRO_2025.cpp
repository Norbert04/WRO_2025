#include <chrono>
#include <thread>

#include <wiringPi.h>

#include "Camera.h"
#include "Robot.h"
#include "utils.h"

int main()
{
	constexpr int P_BUTTON_START = 27;

	if (wiringPiSetup() != 0)
	{
		std::cerr << "WiringPi setup failed\n";
		return -1;
	}

	pinMode(P_BUTTON_START, INPUT);
	pullUpDnControl(P_BUTTON_START, PUD_UP);

	wro::Robot robot = wro::Robot();

	while (digitalRead(BUTTON_PIN) == HIGH)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	DEBUG_PRINTLN("Button pressed, starting execution");
	{ // main

		wro::Camera cam{};
		cam.runTest();
	}

	return 0;
}