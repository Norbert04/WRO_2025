#include "Connection.h"

static wro::Connection* wro::Connection::connection = nullptr;

wro::Connection::Connection()
	: eventHandlers()
{
	eventHandlers.connect = onConnect;
	eventHandlers.error = onError;
	eventHandlers.message = onMessage;
	eventHandlers.debug = onDebug;
	eventHandlers.drive = [](float) {};
	eventHandlers.steer = [](int) {};
	eventHandlers.stop = []() {};
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
	if (handlers.connect)
		eventHandlers.connect = handlers.connect;
	if (handlers.error)
		eventHandlers.error = handlers.error;
	if (handlers.message)
		eventHandlers.message = handlers.message;
	if (handlers.debug)
		eventHandlers.debug = handlers.debug;
	if (handlers.drive)
		eventHandlers.drive = handlers.drive;
	if (handlers.steer)
		eventHandlers.steer = handlers.steer;
	if (handlers.stop)
		eventHandlers.stop = handlers.stop;
}

void wro::Connection::handleEvent()
{
	while (Serial.available() > 0)
	{
		
		const BYTE type = (BYTE)(Serial.read());
		switch (type)
		{
		case connectionCode::connect:
			eventHandlers.connect();
			break;
			
		case connectionCode::error:
		case connectionCode::message:
		case connectionCode::debug:
		{
			BYTE len = 0;
			char* msg = getMessage(len);
			//TODO fix debug 
			//eventHandlers.debug(msg, len);
			break;
		}
		
		case connectionCode::drive:
		{
			float speed = 0.0;
			uint8_t buffer[sizeof(float)];  

			// Wait until enough bytes are available
			while (Serial.available() < sizeof(float));

			// Read bytes into buffer
			Serial.readBytes(buffer, sizeof(float));

			// Copy received bytes into speed variable
			memcpy(&speed, buffer, sizeof(float));

			eventHandlers.drive(speed);
			break;
		}
		
		case connectionCode::steer:
		{
			int angle = 0;
			uint8_t buffer[sizeof(int)];

			while (Serial.available() < sizeof(int));

			Serial.readBytes(buffer, sizeof(int));

			memcpy(&angle, buffer, sizeof(int));

			eventHandlers.steer(angle);
			break;
		}
		
		case connectionCode::stopMovement:
			eventHandlers.stop();
			break;
			
		default:
			sendMessage("Failed to decode serial data");
			break;
		}
	}
}

char* wro::Connection::getMessage(BYTE& length) const
{
    // Wait until at least 1 byte is available 
    while (Serial.available() == 0);

    length = (BYTE)(Serial.read());

    if (length == 0 || length > 64) {
        sendMessage("Invalid message length received");
        length = 0;
        return nullptr;
    }

    // Wait until all bytes are available
    while (Serial.available() < length);

    char* result = new char[length + 1]; 

    // Read the message into the buffer
    Serial.readBytes(result, length);

    result[length] = '\0';

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