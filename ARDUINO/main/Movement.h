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

		constexpr PIN pDrvForward = 12;
		constexpr PIN pDrvBackward = 13;
		constexpr PIN pDrvSpeed = 3;
		constexpr PIN pSteering = 10;
		constexpr BYTE analogueReadResolution = 1023;
		constexpr float operatingVoltage = 5.0;
		
		Servo motorSteering;
	}
}
#endif // !MOVEMENT_H