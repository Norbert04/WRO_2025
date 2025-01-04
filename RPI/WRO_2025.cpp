#include "Movement.h"
#include "Connection.h"

int main()
{
	wro::Connection con{};
	std::cout << "";
	con.sendMessage("Hello");

	if (con.waitForNext() == wro::connectionCode::message)
	{
		std::cout << con.getMessage() << "\n";
	}

	return 0;
}
