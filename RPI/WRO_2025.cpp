#include "Movement.h"

int main()
{
	wro::movement::drive(1);
	wro::movement::steer(20);
	wro::movement::stop();

	return 0;
}
