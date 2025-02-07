#include "Movement.h"
#include "Connection.h"

int main()
{
	wro::Connection::Get()->sendMessage("Hello");

	if (wro::Connection::Get()->waitForNext() == wro::connectionCode::message)
	{
		std::cout << wro::Connection::Get()->getMessage() << "\n";
	}

	return 0;
}
