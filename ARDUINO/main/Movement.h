#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Servo.h>

typedef unsigned char BYTE;
typedef BYTE PIN;

constexpr double PI = 3.141592653589793;

namespace wro
{
	namespace movement
	{
		void init(); // should be called in setup function to set correct pin modes
		void drive(float speed); // -1 < speed < 1
		void stop();
		void steer(int angle); // angle of wheels in degree, 90deg is normal
		BYTE getAngle();

		constexpr PIN P_DRV_FORWARD = 12;
		constexpr PIN P_DRV_BACKWARD = 13;
		constexpr PIN P_DRV_SPEED = 3;
		constexpr PIN P_STEERING = 10;
		constexpr BYTE ANALOGUE_READ_RESOLUTION = 1023;
		constexpr float OPERATING_VOLTAGE = 5.0;
		
		Servo motorSteering;
	}
}
#endif // !MOVEMENT_H