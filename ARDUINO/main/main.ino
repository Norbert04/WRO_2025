// #define _DEBUG // uncomment for debugging

#include "Connection.h"
#include "Movement.h"

void setup()
{
  pinMode(13, OUTPUT);
    wro::EventHandlers handlers;
    handlers.drive = &wro::movement::drive;
    handlers.steer = &wro::movement::steer;
    handlers.stop = &wro::movement::stop;
    wro::Connection::Get()->setEventHandlers(handlers);
    //Serial.begin(9600);
    wro::movement::init();
    wro::movement::drive(0.6);
}

void loop()
{ 
      if(Serial.available() != 0) {
        serialEvent();
      }
}

void serialEvent()
{
    
    wro::Connection::Get()->handleEvent();
}