#ifndef CONNECTION_H
#define CONNECTION_H

#include "Arduino.h"

typedef unsigned char BYTE;

namespace wro
{
	struct EventHandlers
	{
		void (*connect)();
		void (*error)(char* error, BYTE length);
		void (*message)(char* message, BYTE length);
		void (*debug)(char* message, BYTE length);
		void (*drive)(float speed);
		void (*steer)(int angle);
		void (*stop)();
	};

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

		void setEventHandlers(EventHandlers handlers); // pass elements as nullptr to not change the function

		void handleEvent();

		char* getMessage(BYTE& len) const;

		bool valid() const;

	private:
		Connection();
		~Connection();

		static Connection* connection;

		EventHandlers eventHandlers;

		static void onConnect();
		static void onError(char* error, BYTE length);
		static void onDebug(char* message, BYTE length);
		static void onMessage(char* message, BYTE length);
	};
}

#endif // !CONNECTION_H
