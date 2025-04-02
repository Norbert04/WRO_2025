#pragma once

typedef unsigned char BYTE;

namespace wro::robot
{
	enum Pins :BYTE
	{
		P_TRIGGER = 22,
		P_FRONT = 23,
		P_BACK = 24,
		P_LEFT = 25,
		P_RIGHT = 26
	};
	// TODO add correct data
	constexpr double width = 1; // cm
	constexpr double length = 1; // cm
}