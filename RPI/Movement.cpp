#include "Movement.h"

#include "Connection.h"

void wro::movement::drive(float speed)
{
	wro::Connection::Get()->drive(speed);
}

void wro::movement::stop()
{
	wro::Connection::Get()->stopMovement();
}

void wro::movement::steer(short angle)
{
	wro::Connection::Get()->steer(angle);
}