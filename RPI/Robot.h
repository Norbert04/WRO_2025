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

	// State Machine
	enum class State
	{
		IDLE,
		DRIVING_STRAIGHT,
		AVOIDING_RED,
		AVOIDING_GREEN,
		TURNING_LEFT,
		TURNING_RIGHT, // Optional
		CENTERING,
		STOPPED
	};

	class Robot
	{
	public:
		Robot();
		~Robot();

		// movement
		void drive(float speed);// -1 < speed < 1
		void stopMovement();
		void steer(short angle);// angle of wheels in degree

		void setSpeed(float speed);// -1 < speed < 1
		void setSteeringAngle(short angle);// angle of wheels in degree

		// distances
		std::array<double, 4> updateDistances();
		std::array<double, 4> getLastDistances() const;

		//camera
		void run(); // runs camera loop

		std::optional<bool> drivingDirection(); // true -> clockwise; false -> counter clockwise

		Point<unsigned int> estimatePosition(); // in mm starting in top-left edge
		double estimateAngle(); // in rad

		State currentState = State::IDLE;

		// TODO move/ structure constants
		// Driving Control Parameters
		static constexpr float FORWARD_SPEED = 0.75f;    // Motor speed when driving straight
		static constexpr float AVOIDANCE_SPEED = 0.5f;  // Motor speed when avoiding
		static constexpr float TURN_SPEED = 0.5f;       // Motor speed during 90-degree turns

		static constexpr double TURN_DURATION_SEC = 2.0;

		static constexpr int SERVO_CENTRE = 90;
		static constexpr int SERVO_MAX_LEFT = 45;
		static constexpr int SERVO_MAX_RIGHT = 135;

	private:
		void updatePosition(); // should be called before changing angle or speed

		static constexpr double width = 19.2; // cm
		static constexpr double length = 30; // cm

		static constexpr double wheelbase = 21.5; // cm
		static constexpr double trackWidth = 14.64; // cm
		static constexpr double wheelDiameter = 6.6; // cm

		static constexpr short maxRPM = 326 * 0.5; // motorRPM * gearRatio

		Point<unsigned int> position = { 0,0 }; // in mm starting in top-left edge
		double angle = 0; // rad
		bool checkedDirection = false; // if direction is counter clockwise the starting angle must be 180 degrees

		std::chrono::time_point<std::chrono::high_resolution_clock> lastPositionUpdate;

		short steeringAngle = 90; // deg
		float speed = 0; // -1 - 1 as passed to drive function

		Camera camera;
		DistanceSensor distanceSensor;
	};
}