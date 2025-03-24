#include "Movement.h"

namespace wro::movement
{
	namespace
	{
		Servo motorSteering;
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
	digitalWrite(P_DRV_FORWARD, speed > 0);
	digitalWrite(P_DRV_BACKWARD, speed < 0);
	analogWrite(P_DRV_SPEED, abs(speed) * 255);
}

void wro::movement::stop()
{
	drive(0);
	motorSteering.write(90);
}

void wro::movement::steer(int angle)
{
	motorSteering.write(angle);
}

BYTE wro::movement::getAngle()
{
	return motorSteering.read();
}
