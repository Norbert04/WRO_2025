#pragma once

#include "Connection.h"

namespace wro
{
	namespace movement
	{
		void drive(const Connection& connection, float speed);// -1 < speed < 1
		void stop(const Connection& connection);
		void steer(const Connection& connection, BYTE angle);// angle of wheels in degree
		float getAngle(const Connection& connection);
	}
}