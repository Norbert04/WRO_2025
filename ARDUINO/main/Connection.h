#ifndef CONNECTION_H
#define CONNECTION_H

#include "Arduino.h"
#include "eventHandlers.h"

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
struct eventHandlers;

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

		char* getMessage(BYTE& len) const;

		bool valid() const;

	private:
		Connection();
		~Connection();

		static Connection* connection;

		wro::EventHandlers eventHandlers;

		static void onConnect();
		static void onError(char* error, BYTE length);
		static void onDebug(char* message, BYTE length);
		static void onMessage(char* message, BYTE length);

		char* toSerial(float f) const;

		friend struct EventHandlers;
	};

}

#endif // !CONNECTION_H
