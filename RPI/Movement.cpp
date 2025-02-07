#include "Movement.h"

void wro::movement::drive(float speed)
{
	wro::Connection::Get()->drive(speed);
}

void wro::movement::stop()
{
	wro::Connection::Get()->stopMovement();
}

void wro::movement::steer(BYTE angle)
{
	wro::Connection::Get()->steer(angle);
}

float wro::movement::getAngle()
{
	return 0.0f;
}