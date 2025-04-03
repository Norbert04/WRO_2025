// #define _DEBUG // uncomment for debugging

#include "Connection.h"
#include "Movement.h"

void setup()
{
    pinMode(13, OUTPUT); 
    wro::EventHandlers handlers{ NULL };
    handlers.drive = wro::movement::drive;
    handlers.steer = wro::movement::steer;
    handlers.stop = wro::movement::stop;
    wro::Connection::Get()->setEventHandlers(handlers);
    wro::movement::init();
}

void loop()
{ 
      if(Serial.available() != 0) {
        wro::Connection::Get()->handleEvent();
      }
}