#include "Movement.h"

void wro::movement::init()
{
	pinMode(pDrvForward, OUTPUT);
	pinMode(pDrvBackward, OUTPUT);
	pinMode(pDrvSpeed, OUTPUT);
	motorSteering.attach(pSteering);
	motorSteering.write(90);
}

void wro::movement::drive(float speed)
{
	digitalWrite(pDrvForward, speed > 0);
	digitalWrite(pDrvBackward, speed < 0);
	analogWrite(pDrvSpeed, abs(speed) * 255);
}

void wro::movement::stop()
{
	drive(0);
	motorSteering.write(90);
}

void wro::movement::steer(int angle)
{
	motorSteering.write(angle)
}

BYTE wro::movement::getAngle()
{
	return motorSteering.read();
}
