#pragma once

#include "Connection.h"

namespace wro
{
	namespace movement
	{
		void drive(const Connection& connection, float speed);// -1 < speed < 1
		void stop(const Connection& connection);
		void steer(const Connection& connection, float angle);// angle of wheels in radians
		float getAngle(const Connection& connection);
	}
}