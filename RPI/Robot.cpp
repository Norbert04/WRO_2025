#include "Robot.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>

#include "Connection.h"

wro::Robot::Robot()
	:camera(), distanceSensor()
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
	assert(angle >= 40 && angle <= 130);
	wro::Connection::Get()->steer(angle);
	updatePosition();
	steeringAngle = angle;
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
	camera.run();
}

std::optional<bool> wro::Robot::drivingDirection()
{
	std::optional<bool> direction = camera.estimateDrivingDirection();
	// TODO modify checkedDirection accordingly
	if (direction)
	{
		return direction.value();
	}
	// TODO check distance sensor
	return std::nullopt;
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
	// TODO include distance sensor results in estimation
	auto time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> deltaTime = time - lastPositionUpdate;
	unsigned int currentSpeed = speed * maxRPM * wheelDiameter * std::numbers::pi / 6; // mm/s
	if (steeringAngle = 90)
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
	}
	lastPositionUpdate = time;
}
