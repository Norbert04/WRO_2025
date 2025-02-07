#include "Movement.h"

void wro::movement::drive(const Connection& connection, float speed)
{
	connection.drive(speed);
}

void wro::movement::stop(const Connection& connection)
{
	drive(connection, 0);
}

void wro::movement::steer(const Connection& connection, BYTE angle)
{
	connection.steer(angle);
}

float wro::movement::getAngle(const Connection& connection)
{
	return 0.0f;
}