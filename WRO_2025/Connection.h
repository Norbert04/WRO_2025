#pragma once
#include <array>
#include <iostream>
#include <string>
#include <wiringSerial.h>

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

		void sendMessage(std::string message);
		void sendDebug(std::string information);
		void sendError(std::string error);
		void drive(float speed) const;
		void steer(float angle) const;

		std::string getMessage() const;

		bool valid();

	private:
		int fileDescriptor;
	};

}