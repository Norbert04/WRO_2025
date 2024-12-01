#ifndef MOVEMENT_H
#define MOVEMENT_H

typedef unsigned char BYTE;

namespace wro
{
	namespace movement
	{
		void drive(float speed);// -1 < speed < 1
		void stop();
		void steer(float angle);// angle of wheels in radians
		float getAngle();
	}
}
#endif // !MOVEMENT_H