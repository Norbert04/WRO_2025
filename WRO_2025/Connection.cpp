#include "Connection.h"

wro::Connection::Connection()
{
	if ((fileDescriptor = serialOpen("/dev/ttyUSB0", 115200)) < 0)
		std::cerr << "unable to open serial\n";
}

wro::Connection::~Connection()
{
	serialClose(fileDescriptor);
}

void wro::Connection::sendMessage(std::string message)
{
	if (valid())
	{

	}
}

void wro::Connection::sendDebug(std::string information)
{
	if (valid())
	{

	}
}

void wro::Connection::sendError(std::string error)
{
	if (valid())
	{

	}
}

bool wro::Connection::valid()
{
	return fileDescriptor >= 0;
}
