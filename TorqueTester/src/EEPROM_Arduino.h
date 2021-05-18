#ifndef EEPROM_ARDUINO_H
#define EEPROM_ARDUINO_H

#include <Arduino.h>

#define READ_EEPROM true
#define WRITE_EEPROM false

// sizeof(long)  = 4 bytes (arduino platform)
// sizeof(float) = 4 bytes (arduino platform)

// EEPROM MEMORY ADDRESS DEFINITIONS
#define EP_ADDR_SCALE_ZERO_BIAS_L          4  // sizeof(long)
#define EP_ADDR_SCALE_CALIBRATION_FACTOR_F 8  // sizeof(float)
#define EP_ADDR_SCALE_FORCE_TARGET_F       32 // sizeof(float)
#define EP_ADDR_SCALE_FORCE_MAXIMUM_F      36 // sizeof(float)
#define EP_ADDR_SCALE_FORCE_TOLERANCE_F    40 // sizeof(float)
#define EP_ADDR_SCALE_CALIBRATION_LOAD_F   50 // sizeof(float)

void setupEEPROM();
void AccessFloatEEPROM(boolean doread, float* f, int block);
void AccessIntegerEEPROM(boolean doread, int* f, int block);
void AccessLongEEPROM(boolean doread, long* f, int block);
void WriteEEPROM(boolean forcewrite);
void ReadEEPROM(boolean output);
void SENDUI_EEPROM(char* status);
void REPORT_EEPROM_CONTENTS();
void ZERO_EEPROM_CONTENTS();

#endif