#include "Movement.h"

void wro::movement::init()
{
	pinMode(3, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
}

void wro::movement::drive(float speed)
{
	BYTE pin = speed > 0 ? 3 : 9;
	analogWrite(pin, abs(speed) * 255);
}

void wro::movement::stop()
{
	analogWrite(3, 0);
	analogWrite(9, 0);
	analogWrite(10, 0);
}

void wro::movement::steer(float angle, float speed)
{
	
}

float wro::movement::getAngle()
{
	return 0.0f;
}
