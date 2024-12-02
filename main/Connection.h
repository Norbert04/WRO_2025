#ifndef CONNECTION_H
#define CONNECTION_H

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

		void sendMessage(char* message) const;
		void sendDebug(char* information) const;
		void sendError(char* error) const;

		char* getMessage() const;

		bool valid() const;

	private:
		char* toSerial(float f) const;
	};

}

#endif // !CONNECTION_H
