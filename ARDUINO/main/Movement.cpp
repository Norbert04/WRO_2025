#include "Movement.h"

void wro::movement::init()
{
	pinMode(pDrvDirection, OUTPUT);
	pinMode(pDSpeed, OUTPUT);
	pinMode(pSteerDirection, OUTPUT);
	pinMode(pSSpeed, OUTPUT);
}

void wro::movement::drive(float speed)
{
	digitalWrite(pDrvDirection, speed < 0);
	analogWrite(pDSpeed, abs(speed) * 255);
}

void wro::movement::stop()
{
	analogWrite(pDSpeed, 0);
	analogWrite(pSSpeed, 0);
	digitalWrite(pSteerDirection, 0);
	digitalWrite(pDrvDirection, 0);
}

void wro::movement::steer(float angle, float speed)
{
	digitalWrite(pSteerDirection, speed < 0);
	analogWrite(pSSpeed, abs(speed) * 255);
}

float wro::movement::getAngle()
{
	return ((analogRead(pAngle) * operatingVoltage / analogueReadResolution) - 1.65) *
		0.5 * PI / 1.65;
}
