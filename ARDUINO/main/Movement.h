#ifndef MOVEMENT_H
#define MOVEMENT_H

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
		void steer(float angle, float speed = 0.5); // angle of wheels in radians, 0 < speed < 1
		float getAngle();

		constexpr PIN pDrvDirection = 9;
		constexpr PIN pSteerDirection = 13;
		constexpr PIN pDrvSpeed = 3;
		constexpr PIN pSteerSpeed = 10;
		constexpr PIN pAngle = 14;
		constexpr BYTE analogueReadResolution = 1023;
		constexpr float operatingVoltage = 5.0;
	}
}
#endif // !MOVEMENT_H