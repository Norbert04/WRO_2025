#include "Connection.h"

wro::Connection::Connection()
{
	if ((fileDescriptor = serialOpen("/dev/ttyUSB0", 115200)) < 0)
		std::cerr << "unable to open serial\n";
	sleep(5); // wait for motor controller to finish setting up serial connection
	serialPutchar(fileDescriptor, connectionCode::connect);
	// TODO wait for response, if not responding try again or warn user
}

wro::Connection::~Connection()
{
	serialClose(fileDescriptor);
}

void wro::Connection::sendMessage(const std::string message) const
{
	if (message.size() < 256) // string must not exceed 255 characters
	{
		serialPutchar(fileDescriptor, connectionCode::message);
		serialPutchar(fileDescriptor, message.size());
		serialPuts(fileDescriptor, message.c_str());
#if defined(DEBUG) || defined(_DEBUG)
		std::cout << "sent message " << message << "\n";
#endif // DEBUG
	}
#if defined(DEBUG) || defined(_DEBUG)
	else
		std::cerr << "failed to send message\n";
#endif // defined(DEBUG) || defined(_DEBUG)
}

void wro::Connection::sendDebug(const std::string information) const
{
	if (information.size() < 256) // string must not exceed 255 characters
	{
		serialPutchar(fileDescriptor, connectionCode::debug);
		serialPutchar(fileDescriptor, information.size());
		serialPuts(fileDescriptor, information.c_str());
#if defined(DEBUG) || defined(_DEBUG)
		std::cout << "sent debug info " << information << "\n";
#endif // DEBUG
	}
#if defined(DEBUG) || defined(_DEBUG)
	else
		std::cerr << "failed to send debug info\n";
#endif // defined(DEBUG) || defined(_DEBUG)
}

void wro::Connection::sendError(const std::string error) const
{
	if (error.size() < 256) // string must not exceed 255 characters
	{
		serialPutchar(fileDescriptor, connectionCode::debug);
		serialPutchar(fileDescriptor, error.size());
		serialPuts(fileDescriptor, error.c_str());
#if defined(DEBUG) || defined(_DEBUG)
		std::cout << "sent error info " << error << "\n";
#endif // DEBUG
	}
#if defined(DEBUG) || defined(_DEBUG)
	else
		std::cerr << "failed to send error info\n";
#endif // defined(DEBUG) || defined(_DEBUG)
}

void wro::Connection::drive(float speed) const
{
	serialPutchar(fileDescriptor, connectionCode::drive);
	char* temp = toSerial(speed);
	for (BYTE i = 0; i < sizeof(float); i++)
		serialPutchar(fileDescriptor, temp[i]);
	delete[] temp;
}

void wro::Connection::steer(BYTE angle) const
{
	serialPutchar(fileDescriptor, connectionCode::steer);
	char* temp = toSerial(angle);
	for (BYTE i = 0; i < sizeof(float); i++)
		serialPutchar(fileDescriptor, temp[i]);
	delete[] temp;
}

BYTE wro::Connection::waitForNext() const
{
#if defined(DEBUG) || defined(_DEBUG)
	std::cout << "waiting for response\n";
#endif // defined(DEBUG) || defined(_DEBUG)

	while (serialDataAvail(fileDescriptor) <= 0)
		continue;
#if defined(DEBUG) || defined(_DEBUG)
	std::cout << "received some serial data\n";
#endif // defined(DEBUG) || defined(_DEBUG)

	return serialGetchar(fileDescriptor);
}

std::string wro::Connection::getMessage() const
{
#if defined(DEBUG) || defined(_DEBUG)
	std::cout << "reading string from serial\n";
#endif // defined(DEBUG) || defined(_DEBUG)

	std::string result;
	const BYTE len = serialGetchar(fileDescriptor);
	for (BYTE i = 0; i < len; i++)
		result.push_back(serialGetchar(fileDescriptor));

	return result;
}

bool wro::Connection::valid() const
{
	return fileDescriptor >= 0;
}

char* wro::Connection::toSerial(float f) const
{
	char* result = new char[sizeof(float)];
	std::memcpy(result, &f, sizeof(float));
	return result;
}
