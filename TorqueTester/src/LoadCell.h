#ifndef LOADCELL_H
#define LOADCELL_H

#include <Arduino.h>

/*
 * The load cell amplifier used is an HX711 24-BIT ADC
 */

#define LOADCELL_DOUT_PIN 3 // PE5
#define LOADCELL_SCK_PIN 2  // PE4

void loadcellSetup();
long loadcellDetermineZero();
void loadcellSetCalibrationFactor(float cal);
float loadcellDetermineCalibrationFactor();
void loadcellTare();
float loadcellReadCurrentValue();

#endif