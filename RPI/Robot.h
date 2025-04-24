#pragma once

#include <array>
#include <chrono>
#include <optional>

#include "Camera.h"
#include "DistanceSensor.h"
#include "utils.h"


typedef unsigned char BYTE;

namespace wro
{
	namespace robot
	{
		enum Pins :BYTE
		{
			P_TRIGGER = 22,
			P_FRONT = 23,
			P_BACK = 24,
			P_LEFT = 25,
			P_RIGHT = 26
		};

	}

	class Robot
	{
	public:
		Robot();
		~Robot();

		// movement
		void drive(float speed);// -1 < speed < 1
		void stopMovement();
		void steer(short angle);// angle of wheels in degree

		// distances
		std::array<double, 4> updateDistances();
		std::array<double, 4> getLastDistances() const;

		//camera
		void run(); // runs camera loop

		std::optional<bool> drivingDirection(); // true -> clockwise; false -> counterclockwise

		Point<unsigned int> estimatePosition(); // in mm starting in top-left edge
		double estimateAngle(); // in rad

	private:
		void updatePosition(); // should be called before changing angle or speed

		// TODO add correct data
		static constexpr double width = 1; // cm
		static constexpr double length = 1; // cm

		static constexpr double wheelbase = 1; // cm
		static constexpr double trackWidth = 1; // cm
		static constexpr double wheelDiameter = 6.6; // cm

		static constexpr short maxRPM = 326 * 0.5; // motorRPM * gearRatio

		Point<unsigned int> position = { 0,0 }; // in mm starting in top-left edge
		double angle = 0; // rad
		bool checkedDirection = false; // if direction is counterclockwise the starting angle must be 180 degrees

		std::chrono::time_point<std::chrono::high_resolution_clock> lastPositionUpdate;

		short steeringAngle = 90; // deg
		float speed = 0; // -1 - 1 as passed to drive function

		Camera camera;
		DistanceSensor distanceSensor;
	};
}