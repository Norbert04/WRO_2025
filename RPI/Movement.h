#pragma once

#include "Connection.h"

namespace wro
{
	namespace movement
	{
		void drive(float speed);// -1 < speed < 1
		void stop();
		void steer(short angle);// angle of wheels in degree
	}
}