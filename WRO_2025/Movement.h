#pragma once

typedef unsigned char BYTE;

namespace wro
{
	namespace Movement
	{
		void drive(float speed);// -1 < speed < 1
		void stop();
		void steer(float angle);// angle of wheels in radians
	}
}