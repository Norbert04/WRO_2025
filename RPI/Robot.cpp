#include "Robot.h"

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>

#include "Connection.h"
#include "utils.h"

wro::Robot::Robot()
	:camera(this), distanceSensor()
{
	stopMovement();
}

wro::Robot::~Robot()
{
	stopMovement();
	wro::Connection::End();
}

void wro::Robot::drive(float speed)
{
	assert(speed >= -1 && speed <= 1);
	wro::Connection::Get()->drive(speed);
	updatePosition();
	this->speed = speed;
}

void wro::Robot::stopMovement()
{
	wro::Connection::Get()->stopMovement();
	updatePosition();
	steeringAngle = 90;
	speed = 0;
}

void wro::Robot::steer(short angle)
{
	assert(angle >= SERVO_MAX_LEFT && angle <= SERVO_MAX_RIGHT);
	wro::Connection::Get()->steer(angle);
	updatePosition();
	steeringAngle = angle;
}

void wro::Robot::setSpeed(float speed)
{
	if (speed != this->speed)
		drive(speed);
}

void wro::Robot::setSteeringAngle(short angle)
{
	if (angle != steeringAngle)
		steer(angle);
}

std::array<double, 4> wro::Robot::updateDistances()
{
	return distanceSensor.update();
}

std::array<double, 4> wro::Robot::getLastDistances() const
{
	return distanceSensor.getLastValues();
}

void wro::Robot::run()
{
	camera.getSteeringAngle();
}

short wro::Robot::getCameraSteeringAngle()
{
	return camera.getSteeringAngle();
}

std::optional<bool> wro::Robot::drivingDirection()
{
	// TODO modify checkedDirection accordingly
	if (checkedDirection)
	{
		return direction;
	}
	// TODO check distance sensor
	return std::nullopt;
}

void wro::Robot::setDirection(bool direction)
{
	checkedDirection = true;
	this->direction = direction;
}

wro::Point<unsigned int> wro::Robot::estimatePosition()
{
	updatePosition();
	return position;
}

double wro::Robot::estimateAngle()
{
	updatePosition();
	return angle;
}

void wro::Robot::updatePosition()
{
	auto time = std::chrono::steady_clock::now();
	std::chrono::duration<double> deltaTime = time - lastPositionUpdate;
	unsigned int currentSpeed = speed * maxRPM * wheelDiameter * std::numbers::pi / 6; // mm/s
	if (steeringAngle == SERVO_CENTER)
	{
		position.x += std::cos(angle) * currentSpeed * deltaTime.count();
		position.y += std::sin(angle) * currentSpeed * deltaTime.count();
	}
	else
	{
		double curveRadius = wheelbase /
			(std::tan(std::abs(0.5 * std::numbers::pi - toRadians(steeringAngle))));
		double angularVelocity = currentSpeed / curveRadius;
		double deltaAngle = angularVelocity * deltaTime.count();
		position.x += std::cos(angle) * std::sin(deltaAngle) * curveRadius;
		position.y += std::sin(angle) * std::cos(deltaAngle) * curveRadius;
		angle += deltaAngle;
		// keep angle in range -pi to pi
		if (angle > std::numbers::pi)
			angle -= 2 * std::numbers::pi;
		if (angle < -std::numbers::pi)
			angle += 2 * std::numbers::pi;
	}
	if (!inCornerSection && abs(fmod(angle, std::numbers::pi / 2)) <= std::numbers::pi / 18)
	{ // robot is approximately parallel to side walls (+-10deg)
		std::array<double, 4> distances = updateDistances();
		if (distanceSensor.allDistancesValid())
		{
			// TODO correct position with distance sensors
		}
	}
	lastPositionUpdate = time;
}
