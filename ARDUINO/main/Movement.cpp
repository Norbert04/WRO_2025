#include "Movement.h"

namespace wro::movement
{
	namespace
	{
		Servo motorSteering;
		int startingAngle = 90;
		int currentAngle = 90;
		int targetAngle = 90;
		float currentSpeed = 0;
		float startingSpeed = 0;
		float targetSpeed = 0;
		unsigned long tSteeringStart = 0;
		unsigned long tAccelerationStart = 0;
	}
}

void wro::movement::init()
{
	pinMode(P_DRV_FORWARD, OUTPUT);
	pinMode(P_DRV_BACKWARD, OUTPUT);
	pinMode(P_DRV_SPEED, OUTPUT);
	motorSteering.attach(P_STEERING);
	motorSteering.write(90);
}

void wro::movement::drive(float speed)
{
	startingSpeed = currentSpeed;
	targetSpeed = speed;
	tAccelerationStart = micros();
}

void wro::movement::stop()
{
	//drive(0);
	//motorSteering.write(90);
	digitalWrite(P_DRV_FORWARD, 0);
	digitalWrite(P_DRV_BACKWARD, 0);
	analogWrite(P_DRV_SPEED, 0);
	motorSteering.write(90);
	currentSpeed = 0;
	targetSpeed = 0;
	currentAngle = 90;
	targetAngle = 90;
}

void wro::movement::steer(int angle)
{
	startingAngle = currentAngle;
	targetAngle = angle;
	tSteeringStart = micros();
}

BYTE wro::movement::getAngle()
{
	return motorSteering.read();
}

void wro::movement::run()
{
	// driving
	if (currentSpeed != targetSpeed)
	{
		float speed = ((micros() - tAccelerationStart) *
			(targetSpeed - startingSpeed) / ACCELERATION_TIME) + startingSpeed;
		if (abs(targetSpeed - startingSpeed) < abs(currentSpeed - startingSpeed)) // limit speed to target speed
			speed = targetSpeed;
		digitalWrite(P_DRV_FORWARD, speed > 0);
		digitalWrite(P_DRV_BACKWARD, speed < 0);
		analogWrite(P_DRV_SPEED, abs(speed) * 255);
		currentSpeed = speed;
	}

	// steering
	if (currentAngle != targetAngle)
	{
		int angle = static_cast<int>((micros() - tSteeringStart) *
			static_cast<float>(targetAngle - startingAngle) / ACCELERATION_TIME) + startingAngle;
		if (abs(targetAngle - startingAngle) < abs(currentAngle - startingAngle)) // limit angle to target angle
			angle = targetAngle;
		motorSteering.write(angle);
		currentAngle = angle;
	}
}
