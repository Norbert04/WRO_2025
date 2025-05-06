#include <chrono>
#include <cmath>
#include <thread>
#include <wiringPi.h>

#include "Camera.h"
#include "Robot.h"
#include "utils.h"

// #define TEST // use to skip start with button press

bool finished(wro::Robot& robot);
int obstacleChallenge(wro::Robot& robot);
int openChallenge(wro::Robot& robot);
double calculateOuterWallAdjustment(const std::array<double, 4> currentDistances,
	bool clockwise, int targetDistance, int& lastError, double kp, double kd);

int main()
{
	constexpr int P_BUTTON_START = 16;

	if (wiringPiSetupGpio() != 0)
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

#ifdef OPEN_CHALLENGE
	return openChallenge(robot);
#elif defined(OBSTACLE_CHALLENGE)
	return obstacleChallenge(robot);
#else
	return -1;
#endif // OPEN_CHALLENGE
}

bool finished(wro::Robot& robot)
{
	// TODO check whether robot is back in starting section
	return false;
}

int obstacleChallenge(wro::Robot& robot)
{
	// distance, by front ultrasonic sensor, at which car should stop driving straight and turn left
	constexpr int WALL_DETECT_FRONT_TURN = 35;
	robot.currentState = wro::State::DRIVING_STRAIGHT;

	while (!finished(robot)) // main
	{
		using namespace wro::directions;
		std::array<double, 4> currentDistances = robot.updateDistances();
		// Check for corners (Front sensor)
		if (currentDistances[front] != -1 && currentDistances[front] < WALL_DETECT_FRONT_TURN)
		{
			DEBUG_PRINTLN("Wall detected on front, turn left now");
			robot.currentState = wro::State::TURNING_LEFT;
			robot.turnStartTime = std::chrono::steady_clock::now();
			robot.setSpeed(wro::Robot::TURN_SPEED);
			robot.setSteeringAngle(wro::Robot::SERVO_MAX_LEFT);
		}

		// Emergency Stop (Front Sensor)
		if (currentDistances[front] != -1 && currentDistances[front] < 20)
		{
			if (robot.currentState != wro::State::STOPPED)
			{
				std::cout << "EMERGENCY STOP - Obstacle too close! State: " << static_cast<int>(robot.currentState) << "\n";
				robot.currentState = wro::State::STOPPED;
			}
		}

		switch (robot.currentState)
		{
		case wro::State::DRIVING_STRAIGHT:
		case wro::State::CENTERING:
			robot.setSpeed(wro::Robot::FORWARD_SPEED);
			break;

		case wro::State::AVOIDING_RED:  // Pass Right
		case wro::State::AVOIDING_GREEN:  // Pass Left
			robot.setSpeed(wro::Robot::AVOIDANCE_SPEED);
			break;

		case wro::State::TURNING_LEFT:
			robot.setSpeed(wro::Robot::TURN_SPEED);
			break;

		case wro::State::STOPPED:
			robot.setSpeed(0);
			break;

		default:  // Include IDLE or any other state
			robot.setSpeed(0);
			break;
		}

		robot.setSteeringAngle(robot.getCameraSteeringAngle());

		robot.run();
	}
	robot.stopMovement();
	return 0;
}

// when driving 
int openChallenge(wro::Robot& robot)
{
	constexpr int WALL_DETECT_FRONT_TURN = 35;

	wro::State currentState = wro::State::STARTING_STRAIGHT;
	std::chrono::steady_clock::time_point state_start_time;
	std::chrono::steady_clock::time_point last_sensor_read_time;
	int lastWallError = 0;
	int steeringAngle = robot.SERVO_CENTER;

	bool directionDetermined = false;
	bool isClockwise = false; // Determined when first wall is hit
	int lapsDone = 0;
	int turnsDone = 0;

	cv::Mat frame;
	bool running = true;
	bool started = false;

	while (running)
	{
		// TODO: make updateDistances work
		auto now = std::chrono::steady_clock::now();
		using namespace wro::directions;
		std::array<double, 4> currentDistances = robot.updateDistances();

		float motorSpeed = 0;
		int steeringAngle = robot.SERVO_CENTER;
		std::string currentStateString = "UNKNOWN";

		switch (currentState)
		{
		case wro::State::STARTING_STRAIGHT:
			currentStateString = "STARTING_STRAIGHT";
			motorSpeed = robot.OPENING_RACE_FORWARD_SPEED;
			steeringAngle = robot.SERVO_CENTER; // Go straight

			// Check if wall in front
			if (currentDistances[front] != -1 && currentDistances[front] < WALL_DETECT_FRONT_TURN)
			{
				DEBUG_PRINTLN("Front wall, checking side sensors...");

				// Read side sensors
				int distanceLeft = currentDistances[left];
				int distanceRight = currentDistances[right];

				// Determine direction
				// TODO: Tune values
				bool leftValid = (distanceLeft > 3 && distanceLeft < 200);
				bool rightValid = (distanceRight > 3 && distanceRight < 200);

				if (leftValid && rightValid)
				{
					// Both sensors look valid, compare them
					if (distanceRight > distanceLeft + 10)
					{ // Right is way more open -> Clockwise
						isClockwise = true;
					}
					else if (distanceLeft > distanceRight + 10)
					{ // Left is way more open -> Counter-Clockwise
						isClockwise = false;
					}
					else
					{
						// Distances too similar
						DEBUG_PRINTLN("WARN: Side distances similar (" << distanceLeft << "L, " << distanceRight << "R)");
						isClockwise = false;
					}
				}
				else if (rightValid)
				{
					// Only right sensor is valid -> Assume right is open -> Clockwise
					DEBUG_PRINTLN("INFO: Left sensor invalid (" << distanceLeft << "). Assuming Clockwise based on Right (" << distanceRight << ").");
					isClockwise = true;
				}
				else if (leftValid)
				{
					// Only left sensor is valid -> Assume left is open -> Counter-Clockwise
					DEBUG_PRINTLN("INFO: Right sensor invalid (" << distanceRight << "). Assuming Counter-Clockwise based on Left (" << distanceLeft << ").");
					isClockwise = false;
				}
				else
				{
					// Nothing is valid
					DEBUG_PRINTLN("WARN: Both side sensors invalid (" << distanceLeft << "L, " << distanceRight << "R)");
					isClockwise = false;
				}

				directionDetermined = true;
				DEBUG_PRINTLN("Determined Direction: " << (isClockwise ? "Clockwise" : "Counter-Clockwise"));


				currentState = wro::State::TURNING_CORNER;
				state_start_time = std::chrono::steady_clock::now(); // Start turn timer
				motorSpeed = robot.OPENING_RACE_TURN_SPEED;
				// Set steering based on the direction determined
				steeringAngle = isClockwise ? robot.SERVO_MAX_RIGHT : robot.SERVO_MAX_LEFT;
				lastWallError = 0; // Reset wall following state
			}
			break;

		case wro::State::TURNING_CORNER:
			// This state assumes directionDetermined is true 
			currentStateString = "TURNING_CORNER";
			motorSpeed = robot.OPENING_RACE_TURN_SPEED;
			// Set steering based on the direction
			steeringAngle = isClockwise ? robot.SERVO_MAX_RIGHT : robot.SERVO_MAX_LEFT;

			{
				auto turn_elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - state_start_time);
				if (turn_elapsed.count() > robot.TURN_DURATION_SEC)
				{
					DEBUG_PRINTLN("Turn " << (turnsDone + 1) << " done");
					turnsDone++;
					if (turnsDone >= 4)
					{
						lapsDone++;
						turnsDone = 0;
						DEBUG_PRINTLN("--- Lap " << lapsDone << " done ---");
						if (lapsDone >= 3)
						{
							DEBUG_PRINTLN("3 Laps Done, Stopping");
							currentState = wro::State::STOPPED;
							motorSpeed = 0;
							steeringAngle = robot.SERVO_CENTER;
						}
						else
						{
							currentState = wro::State::WALL_FOLLOWING_STRAIGHT;
							steeringAngle = robot.SERVO_CENTER; // Straighten
						}
					}
					else
					{
						currentState = wro::State::WALL_FOLLOWING_STRAIGHT;
						steeringAngle = robot.SERVO_CENTER; // Straighten
					}
					lastWallError = 0;
				}
			}
			break;


		case wro::State::WALL_FOLLOWING_STRAIGHT:
			currentStateString = "WALL_FOLLOWING";
			motorSpeed = robot.OPENING_RACE_FORWARD_SPEED;

			// Wall following logic needs direction 
			if (directionDetermined)
			{
				double steering_adjustment = calculateOuterWallAdjustment(currentDistances, isClockwise,
					robot.TARGET_OUTER_WALL_DISTANCE, lastWallError,
					robot.WALL_FOLLOW_KP, robot.WALL_FOLLOW_KD);
				steeringAngle = static_cast<int>(round(robot.SERVO_CENTER + steering_adjustment));
			}

			// Check for next turn 
			if (currentDistances[front] != -1 && currentDistances[front] < WALL_DETECT_FRONT_TURN)
			{
				DEBUG_PRINTLN("Wall detected, initiating turn " << (turnsDone + 1));
				currentState = wro::State::TURNING_CORNER;
				state_start_time = std::chrono::steady_clock::now();
				motorSpeed = robot.OPENING_RACE_TURN_SPEED;
				steeringAngle = isClockwise ? robot.SERVO_MAX_RIGHT : robot.SERVO_MAX_LEFT;
				lastWallError = 0;
			}
			break;

		case wro::State::STOPPED:
			currentStateString = "STOPPED";
			motorSpeed = 0;
			steeringAngle = robot.SERVO_CENTER;
			started = false;
			break;

		}
		int angle = std::max(robot.SERVO_MAX_LEFT, std::min(robot.SERVO_MAX_RIGHT, steeringAngle));
		robot.setSpeed(motorSpeed);
		robot.setSteeringAngle(angle);
	}
	robot.stopMovement();
	return 0;
}

// Wall Following Calculation 
double calculateOuterWallAdjustment(const std::array<double, 4> currentDistances,
	bool clockwise, int targetDistance, int& lastError, double kp, double kd)
{
	using namespace wro::directions;
	int currentDistance = clockwise ? currentDistances[right] : currentDistances[left];
	// TODO: tune the range
	if (currentDistance > 200 || currentDistance < 3) { lastError = 0; return 0.0; }
	int currentError = currentDistance - targetDistance;
	int errorDerivative = currentError - lastError;
	double adjustment = (kp * currentError) + (kd * errorDerivative);
	lastError = currentError;
	return clockwise ? adjustment : -adjustment; // Correct sign based on sensor used
}