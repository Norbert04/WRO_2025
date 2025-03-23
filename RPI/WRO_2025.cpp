#include "Movement.h"
#include "Connection.h"

int main()
{
	wro::Connection::Get()->sendMessage("Hello");

	wro::movement::drive(1);
	wro::movement::steer(20);

	return 0;
}
