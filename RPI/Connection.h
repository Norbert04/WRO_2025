#pragma once

#include <optional>
#include <string>

#include "Robot.h"

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
		Connection(const Connection& other) = delete;
		Connection& operator=(const Connection& other) = delete;

		void sendMessage(std::string message) const;
		void sendDebug(std::string information) const;
		void sendError(std::string error) const;

		void drive(float speed) const;
		void steer(short angle) const;
		void stopMovement() const;

		BYTE waitForNext() const;
		std::optional<BYTE> waitForNext(unsigned int ms) const;

		std::string getMessage() const;

		bool valid() const;

	private:
		Connection();
		~Connection();
		static Connection* connection;

		int fileDescriptor;

		template<typename T>
		char* toSerial(T i) const;
	};
}