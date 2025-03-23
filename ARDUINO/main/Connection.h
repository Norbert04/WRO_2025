#ifndef CONNECTION_H
#define CONNECTION_H

#include "Arduino.h"

typedef unsigned char BYTE;

namespace wro
{
	struct EventHandlers
	{
		void (*connect)() = wro::Connection::onConnect;
		void (*error)(char* error, BYTE length)  = wro::Connection::onError;
		void (*message)(char* message, BYTE length) = wro::Connection::onMessage;
		void (*debug)(char* message, BYTE length) = wro::Connection::onDebug;
		void (*drive)(float speed) = [](float) {};
		void (*steer)(int angle) = [](float) {};
		void (*stop)() = []() {};
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

		void setEventHandlers(EventHandlers handlers);

		void handleEvent();

		char* getMessage() const;

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

		char* toSerial(float f) const;

		friend struct EventHandlers;
	};

}

#endif // !CONNECTION_H
