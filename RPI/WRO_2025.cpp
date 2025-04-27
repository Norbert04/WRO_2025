#include <chrono>
#include <thread>

#include <wiringPi.h>

#include "Camera.h"
#include "Robot.h"
#include "utils.h"

// #define TEST // use to skip start with button press

int main()
{
	constexpr int P_BUTTON_START = 27;

	if (wiringPiSetup() != 0)
	{
		std::cerr << "WiringPi setup failed\n";
		return -1;
	}

	wro::Robot robot = wro::Robot();

#ifndef TEST
	pinMode(P_BUTTON_START, INPUT);
	pullUpDnControl(P_BUTTON_START, PUD_UP);

	while (digitalRead(BUTTON_PIN) == HIGH)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	DEBUG_PRINTLN("Button pressed, starting execution");
#endif // !TEST
	{ // main

		wro::Camera cam{};
		cam.runTest();
	}

	return 0;
}