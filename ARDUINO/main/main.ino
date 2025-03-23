#define _DEBUG // uncomment for debugging

#include "Connection.h"
#include "Movement.h"

void setup()
{
    wro::EventHandlers handlers;
    handlers.drive = wro::movement::drive;
    handlers.steer = wro::movement::steer;
    handlers.stop = wro::movement::stop;
    wro::Connection::Get()->setEventHandlers(handlers);

    pinMode(13, OUTPUT);
}

void loop()
{
    
}

void serialEvent()
{
    wro::Connection::Get()->handleEvent();
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);

}