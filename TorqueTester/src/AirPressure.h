#ifndef AIRPRESSURE_H
#define AIRPRESSURE_H

#define CLAMP_PIN 6 // RELAY 2 COM2
#define AIR_PRESS A5 // analog pin 5

void setupAirValve();
void actuateSolenoid();
float readAirPressure();

#endif