#ifndef CONNECTION_H
#define CONNECTION_H

#include "Arduino.h"

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

		void sendMessage(char* message) const;
		void sendDebug(char* information) const;
		void sendError(char* error) const;

		char* getMessage() const;

		bool valid() const;

	private:
		Connection();
		~Connection();

		static Connection* connection;
	};
}

#endif // !CONNECTION_H
