#include "Connection.h"

wro::Connection::Connection()
{
	Serial.begin(115200);
	while (!valid())
		continue;
}

wro::Connection::~Connection()
{
	Serial.end();
}

void wro::Connection::sendMessage(char* message) const
{

}

void wro::Connection::sendDebug(char* information) const
{

}

void wro::Connection::sendError(char* error) const
{

}

BYTE wro::Connection::waitForNext() const
{

}

char* wro::Connection::getMessage() const
{
	return new char*;
}

bool wro::Connection::valid() const
{
	return Serial;
}

char* wro::Connection::toSerial(float f) const
{
	char* result = new char[sizeof(float)];
	memcpy(&f, result, sizeof(float));
	return result;
}