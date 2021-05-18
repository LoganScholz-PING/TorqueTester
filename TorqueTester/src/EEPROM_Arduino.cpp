#include <EEPROM.h>
#include "EEPROM_Arduino.h"
#include "LoadCell.h"


long _last_eeprom_write = 0;
long _delta_eeprom_write_millis = 1200000;
boolean _eeprom_dirty = false;

/******* START EXTERNS FROM LoadCell.cpp ******/
extern long  scale_zero_bias;
extern float scale_calibration_factor;
extern float scale_calibration_load;
extern float scale_seek_target; // may not be used
extern float scale_force_target;
extern float scale_force_maximum;
extern float scale_force_tolerance;
/******** END EXTERNS FROM LoadCell.cpp *******/


void setupEEPROM()
{
    ReadEEPROM(false);

    scale_force_target = 100.0f;
    scale_force_maximum = 200.0f;
    scale_force_tolerance = 5.0f;
}

void AccessFloatEEPROM(boolean doread, float* f, int block)
{
    byte* bp = (byte*)f;

    for ( unsigned int i = 0; i < sizeof(float); ++i )
    {
        if (doread)
        {
            *bp++ = EEPROM.read(block + i);
        }
        else
        {
            EEPROM.write(block + i, *bp++);
        }
    }
}

void AccessIntegerEEPROM(boolean doread, int* f, int block)
{
    byte* bp = (byte*)f;

    for ( unsigned int i = 0; i < sizeof(int); ++i)
    {
        if ( doread )
        {
            *bp++ = EEPROM.read(block + i);
        }
        else
        {
            EEPROM.write(block + i, *bp++);
        }
    }
}

void AccessLongEEPROM(boolean doread, long* f, int block)
{
    byte* bp = (byte*)f;

    for ( unsigned int i = 0; i < sizeof(long); ++i)
    {
        if ( doread )
        {
            *bp++ = EEPROM.read(block + i);
        }
        else
        {
            EEPROM.write(block + i, *bp++);
        }
    }
}

void WriteEEPROM(boolean forcewrite)
{
    /* write the following memory locations from EEPROM:
     *
     * (long)  scale_zero_bias          @ ADDR=4
     * (float) scale_calibration_factor @ ADDR=8
     * (float) scale_force_target       @ ADDR=32
     * (float) scale_force_maximum      @ ADDR=36
     * (float) scale_force_tolerance    @ ADDR=40
     * (float) scale_calibration_load   @ ADDR=50
     * 
     */

    if ( (_eeprom_dirty) || (forcewrite) )
    {
        AccessLongEEPROM( WRITE_EEPROM, &scale_zero_bias,          EP_ADDR_SCALE_ZERO_BIAS_L);
        AccessFloatEEPROM(WRITE_EEPROM, &scale_calibration_factor, EP_ADDR_SCALE_CALIBRATION_FACTOR_F);
        AccessFloatEEPROM(WRITE_EEPROM, &scale_force_target,       EP_ADDR_SCALE_FORCE_TARGET_F);
        AccessFloatEEPROM(WRITE_EEPROM, &scale_force_maximum,      EP_ADDR_SCALE_FORCE_MAXIMUM_F);
        AccessFloatEEPROM(WRITE_EEPROM, &scale_force_tolerance,    EP_ADDR_SCALE_FORCE_TOLERANCE_F);
        AccessFloatEEPROM(WRITE_EEPROM, &scale_calibration_load,   EP_ADDR_SCALE_CALIBRATION_LOAD_F);
    
        _eeprom_dirty = false;
        _last_eeprom_write = millis();

        SENDUI_EEPROM("W");
    }

}

void ReadEEPROM(boolean output)
{
    /* read the following memory locations from EEPROM:
     *
     * (long)  scale_zero_bias          @ ADDR=4
     * (float) scale_calibration_factor @ ADDR=8
     * (float) scale_force_target       @ ADDR=32
     * (float) scale_force_maximum      @ ADDR=36
     * (float) scale_force_tolerance    @ ADDR=40
     * (float) scale_calibration_load   @ ADDR=50
     * 
     */
    AccessLongEEPROM( READ_EEPROM, &scale_zero_bias,          EP_ADDR_SCALE_ZERO_BIAS_L);
    AccessFloatEEPROM(READ_EEPROM, &scale_calibration_factor, EP_ADDR_SCALE_CALIBRATION_FACTOR_F);
    AccessFloatEEPROM(READ_EEPROM, &scale_force_target,       EP_ADDR_SCALE_FORCE_TARGET_F);
    AccessFloatEEPROM(READ_EEPROM, &scale_force_maximum,      EP_ADDR_SCALE_FORCE_MAXIMUM_F);
    AccessFloatEEPROM(READ_EEPROM, &scale_force_tolerance,    EP_ADDR_SCALE_FORCE_TOLERANCE_F);
    AccessFloatEEPROM(READ_EEPROM, &scale_calibration_load,   EP_ADDR_SCALE_CALIBRATION_LOAD_F);

    _eeprom_dirty = false;

    if (output)
    {
        SENDUI_EEPROM("R");
    }
    
}

void SENDUI_EEPROM(char* status)
{
    Serial.print("{,");
    Serial.print(status);
    Serial.print(":ZB=");
    Serial.print(scale_zero_bias);
    Serial.print(":CF=");
    Serial.print(scale_calibration_factor);
    Serial.println(",}"); 
}

void REPORT_EEPROM_CONTENTS()
{
    // start at address 0 and read EEPROM info to serial byte by byte
    // note we won't be storing data any higher than byte 80 (so far)
    Serial.println();
    for ( unsigned int i = 0; i < 80; ++i )
    {
        // this count can be slow so allow user to break out
        if ( Serial.available() )
        {
            char in = Serial.read();
            if (in == 'x')
            {
                break;
            }
        }

        Serial.print("EEPROM Address [");
        Serial.print(i);
        Serial.print("] Data = ");
        Serial.println(EEPROM.read(i));
    }
}

void ZERO_EEPROM_CONTENTS()
{
    Serial.println("WARNING: Zeroing EEPROM data (first 80 bytes)");
    for ( unsigned int i = 0; i < 80; ++i)
    {
        EEPROM.write(i, 0);
    }
    Serial.println("WARNING: Zeroing data complete");
}