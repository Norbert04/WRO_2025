#include "DistanceSensor.h"

#include <mutex>
#include <thread>
#include <vector>
#include <wiringPi.h>

#include "utils.h"

wro::DistanceSensor::DistanceSensor()
{
	pinMode(P_TRIGGER_FRONT, OUTPUT);
	pinMode(P_TRIGGER_BACK, OUTPUT);
	pinMode(P_TRIGGER_LEFT, OUTPUT);
	pinMode(P_TRIGGER_RIGHT, OUTPUT);
	digitalWrite(P_TRIGGER_FRONT, LOW);
	digitalWrite(P_TRIGGER_BACK, LOW);
	digitalWrite(P_TRIGGER_LEFT, LOW);
	digitalWrite(P_TRIGGER_RIGHT, LOW);

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
	std::array<double, 4> distances;
	std::vector<std::thread> threads;
	threads.emplace_back(measureDistance, P_TRIGGER_FRONT, P_FRONT, std::ref(distances[0]));
	threads.emplace_back(measureDistance, P_TRIGGER_BACK, P_BACK, std::ref(distances[1]));
	threads.emplace_back(measureDistance, P_TRIGGER_LEFT, P_LEFT, std::ref(distances[2]));
	threads.emplace_back(measureDistance, P_TRIGGER_RIGHT, P_RIGHT, std::ref(distances[3]));

	measureDistance(P_TRIGGER_FRONT, P_FRONT, std::ref(distances[0]));

	for (std::thread& thread : threads)
	{
		if (thread.joinable())
			thread.join();
	}
	return distances;
}

std::array<double, 4> wro::DistanceSensor::getLastValues() const
{
	return distances;
}
std::mutex m;
void wro::DistanceSensor::measureDistance(PIN pTrigger, PIN pEcho, double& distance)
{
	digitalWrite(pTrigger, HIGH);
	std::this_thread::sleep_for(std::chrono::microseconds(10)); // sensor is triggered by a 10us pulse
	digitalWrite(pTrigger, LOW);

	auto tTrigger = std::chrono::high_resolution_clock::now();
	auto start = std::chrono::high_resolution_clock::now();
	while (digitalRead(pEcho) == LOW &&
		(std::chrono::high_resolution_clock::now() - tTrigger) < std::chrono::milliseconds(15))
		start = std::chrono::high_resolution_clock::now();
	if (digitalRead(pEcho) == LOW)
	{
		distance = NO_DISTANCE;
		return;
	}
	auto end = std::chrono::high_resolution_clock::now();
	while (digitalRead(pEcho) == HIGH &&
		(std::chrono::high_resolution_clock::now() - start) < std::chrono::milliseconds(15))
		end = std::chrono::high_resolution_clock::now();
	const std::chrono::duration<double> t = end - start;
	m.lock();
	DEBUG_PRINTLN(t);
	m.unlock();
	distance = (t.count() * V_SOUND) / 2; //distance has to be travelled twice, time in nanoseconds
	if (distance > 400) // object is more than 4m away
		distance = NO_DISTANCE;
}
