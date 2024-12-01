#pragma once

#include <iostream>
#include <string>
#include <unistd.h>
#include <wiringSerial.h>
#include <cstring>

typedef unsigned char BYTE;

namespace wro
{
	namespace connectionCode
	{
		enum connectionCodes : BYTE
		{
			connect,
			error,
			message,
			debug,
			drive,
			steer,
			beep,
			gyroscope,
			charge
		};
	}

	class Connection
	{
	public:
		Connection();
		~Connection();
		Connection(const Connection& other) = delete; // rule of three
		Connection& operator=(const Connection& other) = delete; // rule of three

		void sendMessage(std::string message) const;
		void sendDebug(std::string information) const;
		void sendError(std::string error) const;

		void drive(float speed) const;
		void steer(float angle) const;

		BYTE waitForNext() const;

		std::string getMessage() const;

		bool valid() const;

	private:
		int fileDescriptor;

		char* toSerial(float f) const;
	};

}