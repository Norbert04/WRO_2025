#ifndef EVENT_HANDLERS_H
#define EVENT_HANDLERS_H
typedef unsigned char BYTE;
namespace wro
{

	struct EventHandlers
	{
		void (*connect)() = [](){};
		void (*error)(char* error, BYTE length)  = [](char*,BYTE){};
		void (*message)(char* message, BYTE length) = [](char*,BYTE){};
		void (*debug)(char* message, BYTE length) = [](char*,BYTE){};
		void (*drive)(float speed) = [](float) {};
		void (*steer)(int angle) = [](float) {};
		void (*stop)() = []() {};
	};
}


#endif