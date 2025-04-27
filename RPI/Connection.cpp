#include "Connection.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <wiringSerial.h>

#include "utils.h"

wro::Connection* wro::Connection::connection{ nullptr };

wro::Connection::Connection()
{
	if ((fileDescriptor = serialOpen("/dev/ttyUSB0", 115200)) < 0)
		if ((fileDescriptor = serialOpen("/dev/ttyUSB1", 115200)) < 0)
			std::cerr << "unable to open serial\n";
	std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // wait for motor controller to finish setting up serial connection
	serialPutchar(fileDescriptor, connectionCode::connect);
	std::optional<BYTE> result = waitForNext(10);
	if (!result)
	{
		DEBUG_PRINTLN("waiting 5 more milliseconds for response");
		result = waitForNext(5);
	}
	if (result && *result == connectionCode::connect)
	{
		DEBUG_PRINTLN("creating connection succeeded");
	}
	else
		std::cerr << "creating connection failed\n";
}

wro::Connection::~Connection()
{
	serialClose(fileDescriptor);
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

void wro::Connection::sendMessage(const std::string message) const
{
	if (message.size() < 256) // string must not exceed 255 characters
	{
		serialPutchar(fileDescriptor, connectionCode::message);
		serialPutchar(fileDescriptor, message.size());
		serialPuts(fileDescriptor, message.c_str());
		DEBUG_PRINTLN("sent message " << message);
	}
	else
		DEBUG_PRINT_ERR("failed to send message\n");
}

void wro::Connection::sendDebug(const std::string information) const
{
	if (information.size() < 256) // string must not exceed 255 characters
	{
		serialPutchar(fileDescriptor, connectionCode::debug);
		serialPutchar(fileDescriptor, information.size());
		serialPuts(fileDescriptor, information.c_str());
		DEBUG_PRINTLN("sent debug info " << information);
	}
	else
		DEBUG_PRINT_ERR("failed to send debug info\n");
}

void wro::Connection::sendError(const std::string error) const
{
	if (error.size() < 256) // string must not exceed 255 characters
	{
		serialPutchar(fileDescriptor, connectionCode::debug);
		serialPutchar(fileDescriptor, error.size());
		serialPuts(fileDescriptor, error.c_str());
		DEBUG_PRINTLN("sent error info " << error);
	}
	else
		DEBUG_PRINT_ERR("failed to send error info\n");
}

void wro::Connection::drive(float speed) const
{
	serialPutchar(fileDescriptor, connectionCode::drive);
	char* temp = toSerial(speed);
	for (BYTE i = 0; i < sizeof(float); i++)
		serialPutchar(fileDescriptor, temp[i]);
	delete[] temp;
}

void wro::Connection::steer(short angle) const
{
	std::cout << "called steer" << std::endl;
	serialPutchar(fileDescriptor, connectionCode::steer);
	char* temp = toSerial(angle);
	for (BYTE i = 0; i < sizeof(short); i++)
		serialPutchar(fileDescriptor, temp[i]);
	delete[] temp;
}

void wro::Connection::stopMovement() const
{
	serialPutchar(fileDescriptor, connectionCode::stopMovement);
}

BYTE wro::Connection::waitForNext() const
{
	DEBUG_PRINTLN("waiting for response");

	while (serialDataAvail(fileDescriptor) <= 0)
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	DEBUG_PRINTLN("received some serial data");

	return serialGetchar(fileDescriptor);
}

std::optional<BYTE> wro::Connection::waitForNext(unsigned int ms) const
{
	DEBUG_PRINTLN("waiting for response");
	unsigned int t = 0;
	while (serialDataAvail(fileDescriptor) <= 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		t += 5;
		if (t >= ms)
			return std::nullopt;
	}
	DEBUG_PRINTLN("received some serial data");

	return serialGetchar(fileDescriptor);
}

std::string wro::Connection::getMessage() const
{
	DEBUG_PRINTLN("reading string from serial");

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

template<typename T>
char* wro::Connection::toSerial(T i) const
{
	char* result = new char[sizeof(T)];
	std::memcpy(result, &i, sizeof(T));
	return result;
}