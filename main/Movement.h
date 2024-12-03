#ifndef MOVEMENT_H
#define MOVEMENT_H

typedef unsigned char BYTE;

namespace wro
{
	namespace movement
	{
		void init(); // should be called in setup function to set corect pin modes
		void drive(float speed);// -1 < speed < 1
		void stop();
		void steer(float angle, float speed = 0.5);// angle of wheels in radians, 0 < speed < 1
		float getAngle();
	}
}
#endif // !MOVEMENT_H