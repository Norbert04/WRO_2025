#pragma once
#include <array>
#include <chrono>
#include <thread>

#include <wiringPi.h>

typedef unsigned char BYTE;

class DistanceSensor
{
public:
	DistanceSensor();
	std::array<double, 4> update();
	std::array<double, 4> getLastValues();

	static constexpr double NO_DISTANCE = -1.0;

private:
	static void measureDistance(BYTE pin, std::chrono::high_resolution_clock::time_point tTrigger, double& distance);

	std::array<double, 4> distances = // in cm
	{ NO_DISTANCE, NO_DISTANCE, NO_DISTANCE, NO_DISTANCE }; // 0-front, 1-left,...(counter clockwise)

	static constexpr unsigned int V_SOUND = 34330; // cm/s
	static constexpr BYTE P_TRIGGER = 22;
	static constexpr BYTE P_FRONT = 23;
	static constexpr BYTE P_BACK = 24;
	static constexpr BYTE P_LEFT = 25;
	static constexpr BYTE P_RIGHT = 26;
};