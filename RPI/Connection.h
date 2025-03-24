#pragma once

#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
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
			stopMovement
		};
	}

	class Connection
	{
	public:
		static Connection* Get();
		static void End();
		Connection(const Connection& other) = delete; // rule of three
		Connection& operator=(const Connection& other) = delete; // rule of three

		void sendMessage(std::string message) const;
		void sendDebug(std::string information) const;
		void sendError(std::string error) const;

		void drive(float speed) const;
		void steer(BYTE angle) const;
		void stopMovement() const;

		BYTE waitForNext() const;

		std::string getMessage() const;

		bool valid() const;

	private:
		Connection();
		~Connection();
		static Connection* connection;

		int fileDescriptor;

		char* toSerial(float f) const;
	};

}