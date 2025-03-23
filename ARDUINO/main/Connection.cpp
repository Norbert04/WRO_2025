#include "Connection.h"

wro::Connection::Connection()
	: eventHandlers()
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

void wro::Connection::setEventHandlers(EventHandlers handlers)
{
	eventHandlers = handlers;
}

void wro::Connection::handleEvent()
{
	const BYTE type = std::static_cast<BYTE>(Serial.read());
	switch (type)
	{
	case connectionCode::connect:
		eventHandlers.connect();
		break;
	case connectionCode::error:
		BYTE len = 0;
		char* error = getMessage(len);
		eventHandlers.error(error, len);
		break;
	case connectionCode::message:
		BYTE len = 0;
		char* msg = getMessage(len);
		eventHandlers.message(msg, len);
		break;
	case connectionCode::debug:
		BYTE len = 0;
		char* msg = getMessage(len);
		eventHandlers.debug(msg, len);
		break;
	case connectionCode::drive:
		float speed = 0.0;
		BYTE* buffer = new BYTE[sizeof(float)];
		if (Serial.available < sizeof(float))
		{
			delay(5);
			if (Serial.available < sizeof(float))
				sendMessage("Failed to receive serial data");
		}
		Serial.readBytes(buffer, sizeof(float));
		memcpy(buffer, &speed, sizeof(float));
		eventHandlers.drive(speed);
		delete[] buffer;
		break;
	case connectionCode::steer:
		int angle = 0;
		BYTE* buffer = new BYTE[sizeof(float)];
		if (Serial.available < sizeof(int))
		{
			delay(2);
			if (Serial.available < sizeof(int))
				sendMessage("Failed to receive serial data");
		}
		Serial.readBytes(buffer, sizeof(int));
		memcpy(buffer, &speed, sizeof(int));
		eventHandlers.steer(angle);
		delete[] buffer;
		break;
	case connectionCode::stopMovement:
		eventHandlers.stop();
		break;
	default:
		sendMessage("Failed to decode serial data");
		break;
	}
}

char* wro::Connection::getMessage(BYTE& len) const
{
	if (Serial.available == 0)
	{
		delay(2);
		if (Serial.available == 0)
			sendMessage("Failed to receive serial data");
	}
	len = std::static_cast<BYTE>(Serial.read());
	if (Serial.available < len && len <= 64)
	{
		delay(5);
		if (Serial.available < len)
			sendMessage("Failed to receive serial data");
	}
	char* result = new char[len];
	for (BYTE i = 0; i < len; i++)
		Serial.readBytes(result, len);
	return result;
}

bool wro::Connection::valid() const
{
	return Serial;
}

void wro::Connection::onConnect()
{
	Serial.write(connectionCode::connect);
}

void wro::Connection::onError(char* error, BYTE length)
{
	End();
}

void wro::Connection::onDebug(char* message, BYTE length)
{
}

void wro::Connection::onMessage(char* message, BYTE length)
{
}

char* wro::Connection::toSerial(float f) const
{
	char* result = new char[sizeof(float)];
	memcpy(&f, result, sizeof(float));
	return result;
}