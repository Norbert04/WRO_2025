#include "Connection.h"

static wro::Connection* wro::Connection::connection = nullptr;

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

wro::Connection* wro::Connection::Get()
{
	if (connection == nullptr)
		connection = new Connection();
	return connection;
}

void wro::Connection::End()
{
	delete connection;
	connection = nullptr;
}

void wro::Connection::sendMessage(char* message) const
{
	Serial.write(connectionCode::message);
	Serial.write(message);
}

void wro::Connection::sendDebug(char* information) const
{
	Serial.write(connectionCode::debug);
	Serial.write(information);
}

void wro::Connection::sendError(char* error) const
{
	Serial.write(connectionCode::error);
	Serial.write(error);
}

char* wro::Connection::getMessage() const
{
	const BYTE len = std::static_cast<BYTE>(Serial.read());
	char* result = new char[len];
	for (BYTE i = 0; i < len; i++)
		Serial.readBytes(result, len);
	return result;
}

bool wro::Connection::valid() const
{
	return Serial;
}

char* wro::Connection::toSerial(float f) const
{
}