#include "DistanceSensor.h"

#include <thread>
#include <wiringPi.h>

#include "utils.h"

wro::DistanceSensor::DistanceSensor()
{
	pinMode(P_TRIGGER, OUTPUT);
	pinMode(P_FRONT, INPUT);
	pullUpDnControl(P_FRONT, PUD_DOWN);
	pinMode(P_BACK, INPUT);
	pullUpDnControl(P_BACK, PUD_DOWN);
	pinMode(P_LEFT, INPUT);
	pullUpDnControl(P_LEFT, PUD_DOWN);
	pinMode(P_RIGHT, INPUT);
	pullUpDnControl(P_RIGHT, PUD_DOWN);
}

std::array<double, 4> wro::DistanceSensor::update()
{
	std::chrono::high_resolution_clock::time_point tTrigger =
		std::chrono::high_resolution_clock::now();
	digitalWrite(P_TRIGGER, HIGH);
	std::array<std::thread, 4> threads;
	for (BYTE i = 0; i < 4; i++)
	{
		threads[i] = std::thread(measureDistance, P_FRONT + i, tTrigger, std::ref(distances[i]));
	}
	std::this_thread::sleep_until(tTrigger + std::chrono::microseconds(10)); // sensor is triggered by a 10us pulse
	digitalWrite(P_TRIGGER, LOW);
	for (BYTE i = 0; i < 4; i++)
	{
		if (threads[i].joinable())
			threads[i].join();
	}
	return distances;
}

std::array<double, 4> wro::DistanceSensor::getLastValues() const
{
	return distances;
}

void wro::DistanceSensor::measureDistance(
	PIN pin, std::chrono::high_resolution_clock::time_point tTrigger, double& distance)
{
	while (digitalRead(pin) == LOW &&
		(std::chrono::high_resolution_clock::now() - tTrigger) < std::chrono::milliseconds(15))
		std::this_thread::sleep_for(std::chrono::microseconds(10));
	const std::chrono::duration<double> t = std::chrono::high_resolution_clock::now() - tTrigger;
	distance = (t.count() * V_SOUND) / 2;
	if (distance > 400) // object is more than 4m away
		distance = NO_DISTANCE;
}
