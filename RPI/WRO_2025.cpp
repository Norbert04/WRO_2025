#include <chrono>
#include <thread>

#include <wiringPi.h>

#include "Camera.h"
#include "Robot.h"
#include "utils.h"

// #define TEST // use to skip start with button press

bool finished(wro::Robot& robot);

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

	while (digitalRead(P_BUTTON_START) == HIGH)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	DEBUG_PRINTLN("Button pressed, starting execution");
#endif // !TEST

	// distance, by front ultrasonic sensor, at which car should stop driving straight and turn left
	constexpr int WALL_DETECT_FRONT_TURN = 35;
	wro::State currentState = wro::State::DRIVING_STRAIGHT;
	std::chrono::high_resolution_clock::time_point turnStartTime;

	while (!finished(robot)) // main
	{
		using namespace wro::directions;
		std::array<double, 4> currentDistances = robot.updateDistances();
		// 1. Check for corners (Front sensor)
		if (currentDistances[front] != -1 && currentDistances[front] < WALL_DETECT_FRONT_TURN)
		{
			DEBUG_PRINTLN("Wall detected on front, turn left now");
			currentState = wro::State::TURNING_LEFT;
			turnStartTime = std::chrono::high_resolution_clock::now();
			robot.setSpeed(wro::Robot::TURN_SPEED);
			robot.setSteeringAngle(wro::Robot::SERVO_MAX_LEFT);
		}




		robot.run();
	}

	return 0;
}

bool finished(wro::Robot& robot)
{
	// TODO check whether robot is back in starting section
	return false;
}
