#include <Arduino.h>
#include "AirPressure.h"

extern boolean DEBUG; // from main.cpp

bool clamp_status = false;


void setupAirValve()
{
    Serial.println("start setupAirValve");
    pinMode(CLAMP_PIN, OUTPUT);
    digitalWrite(CLAMP_PIN, clamp_status);
    Serial.println("end setupAirValve");
}


void actuateSolenoid()
{
    clamp_status = !clamp_status;
    digitalWrite(CLAMP_PIN, clamp_status);
    if ( DEBUG )
    {
        Serial.print("clamp_status: "); Serial.println(clamp_status);
    }
}


float readAirPressure()
{
    int press = 0;

    press = analogRead(AIR_PRESS); // returns a value 0-1023

    if ( press < 102 ) { press = 102; } // min v press out is 0.5v
    if ( press > 921 ) { press = 921; } // max v press out is 4.5v

    press = press - 102; // remove 0.5V offset

    //float psi = (press/819)*100; // normalize pressure reading then multiply * 100psi
    float current_psi = (press)*(0.1221001); // equivilent to the line above, 100/819 = 0.1221001-ish
    
    if ( DEBUG )
    {
        Serial.print("Air press analog read: "); Serial.println(press);
        Serial.print("Air pressure PSI: "); Serial.println(current_psi);
    }
    
    return current_psi;
}