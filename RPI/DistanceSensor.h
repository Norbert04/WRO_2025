#pragma once

#include <array>
#include <chrono>

typedef unsigned char BYTE;
typedef BYTE PIN;

namespace wro
{
	namespace directions
	{
		enum directions : BYTE
		{
			front,
			left,
			back,
			right
		};
	}

	class DistanceSensor
	{
	public:
		DistanceSensor();
		std::array<double, 4> update();
		std::array<double, 4> getLastValues() const;

		static constexpr double NO_DISTANCE = -1.0;

	private:
		static void measureDistance(PIN pin, std::chrono::high_resolution_clock::time_point tTrigger, double& distance);

		std::array<double, 4> distances = // in cm
		{ NO_DISTANCE, NO_DISTANCE, NO_DISTANCE, NO_DISTANCE }; // index as in directions

		static constexpr unsigned int V_SOUND = 34330; // cm/s
		static constexpr PIN P_TRIGGER = 22;
		static constexpr PIN P_FRONT = 23;
		static constexpr PIN P_BACK = 24;
		static constexpr PIN P_LEFT = 25;
		static constexpr PIN P_RIGHT = 27;
	};
}