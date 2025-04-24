#pragma once

namespace wro
{
	namespace movement
	{
		[[deprecated("use drive method defined in Robot class instead")]]
		void drive(float speed);// -1 < speed < 1
		[[deprecated("use stopMovement method defined in Robot class instead")]]
		void stop();
		[[deprecated("use steer method defined in Robot class instead")]]
		void steer(short angle);// angle of wheels in degree
	}
}