#include "LoadCell.h"
#include "EEPROM_Arduino.h"
#include <HX711.h>

extern boolean DEBUG;         // from main.cpp
extern boolean _eeprom_dirty; // from EEPROM_Arduino.cpp

HX711 loadcell;

long  scale_zero_bias = 0;
float scale_calibration_factor = 1.0f; //45999.00 in testing
float scale_calibration_load = 50.0f;
float scale_seek_target = 0.0f; // may not be used
float scale_force_target = 100.0f;
float scale_force_maximum = 200.0f;
float scale_force_tolerance = 5.0f;

void loadcellSetup()
{
    Serial.println("start loadcellSetup");
    loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    loadcell.set_scale(scale_calibration_factor);
    Serial.println("end loadcellSetup");
}

long loadcellDetermineZero()
{
    scale_zero_bias = loadcell.read_average(5);
    if (DEBUG)
    {
        Serial.print("Zero Bias: "); Serial.println(scale_zero_bias); 
    }
    
    _eeprom_dirty = true; // we updated an EEPROM value (scale_zero_bias)
    return scale_zero_bias;
}


void loadcellSetCalibrationFactor(float cal)
{  
    if (DEBUG)
    {
        Serial.print("Reading BEFORE set cal factor: "); 
        Serial.print(loadcell.get_units(), 2); 
        Serial.println(" lbs");
    }
    
    loadcell.set_scale(cal);

    if (DEBUG)
    {
        Serial.print("Reading AFTER set cal factor: "); 
        Serial.print(loadcell.get_units(), 2); 
        Serial.println(" lbs");
    }
}

// The function below allows the user to set the calibration factor manually through serial

// *************************************************************
// *** NOTE: THIS IS MANUAL SERIAL CALIBRATION FACTOR METHOD ***
// ************ (PRETTY MUCH JUST FOR EMERGENCIES!) ************
// *************************************************************

float loadcellDetermineCalibrationFactor()
{
    /*
     * Physical Calibration Procedure:
     * 1. Tare the loadcell while no weight is applied
     * 2. Place a known weight on the loadcell then proceed to calibration
     * 3. Calibrate until the loadcell reads the correct value of the known weight
     * 4. End calibration procedure to save the data to EEPROM
     * 5. Remove the weight from the loadcell and re-tare it (if reading is not 0.0 lbs)
     * 6. If you had to re-tare, place known weight on loadcell and re-calibrate
     * 7. End calibration to save data, should be calibrated now
     */
    
    
    Serial.println(" **Starting Calibration Factor Determination** ");
    Serial.println(" ! - Send 'x' or 'X' to stop/finish calibration procedure");
    Serial.println(" ! - Send '+' to increment calibration factor +10");
    Serial.println(" ! - Send '-' to decrement calibration factor -10");
    Serial.println(" ! - Send 'a' to increment calibration factor +100");
    Serial.println(" ! - Send 'z' to decrement calibration factor -100");
    Serial.println(" ! - Send 'd' to increment calibration factor +1000");
    Serial.println(" ! - Send 'f' to decrement calibration factor -1000");
    
    scale_calibration_factor = 0;  

    loadcell.set_scale(scale_calibration_factor);

    Serial.print("Initial scale reading before calibration: ");
    Serial.print(loadcell.get_units(), 3);
    Serial.println(" lbs");

    loadcellDetermineZero();
    
    char cal_cmd;
    float old_calib = scale_calibration_factor;
    unsigned long timeout = millis();
    
    // exit the while loop when commanded by the user 
    // (received 'x') or a 180 second (3 min) timeout    
    while ( millis() - timeout <= 180000 )
    {       
        if ( Serial.available() )
        {
            cal_cmd = Serial.read();
            if (cal_cmd == '+')
                scale_calibration_factor += 10;
            else if(cal_cmd == '-')
                scale_calibration_factor -= 10;
            else if(cal_cmd == 'a')
                scale_calibration_factor += 100;
            else if(cal_cmd == 'z')
                scale_calibration_factor -= 100;
            else if(cal_cmd == 'd')
                scale_calibration_factor += 1000;
            else if(cal_cmd == 'f')
                scale_calibration_factor -= 1000;
            else if(cal_cmd == 'x')
                break;
        }

        // if the calibration factor has changed print the feedback out to serial
        if ( old_calib != scale_calibration_factor )
        {
            loadcell.set_scale(scale_calibration_factor);
            Serial.print("updated calibration_factor: "); 
            Serial.println(scale_calibration_factor);
            Serial.print("Scale reading: "); 
            Serial.print(loadcell.get_units(), 3); 
            Serial.println(" lbs");
            old_calib = scale_calibration_factor;
        }
    }

    Serial.println(" *Completed Load Cell Calibration* ");
    Serial.print("Final calibration value: "); 
    Serial.println(scale_calibration_factor);
    Serial.print("Final scale reading with updated cal factor: "); 
    Serial.print(loadcell.get_units(), 3); 
    Serial.println(" lbs");

    loadcellDetermineZero();

    _eeprom_dirty = true; // we updated an EEPROM value (scale_calibration_factor)
    WriteEEPROM(false); // forcewrite = false, _eeprom_dirty will be true
    
    return scale_calibration_factor;
}

void loadcellTare()
{
    if (DEBUG)
    {
        Serial.print("Load cell reading before tare: ");
        Serial.print(loadcell.get_units(), 2); Serial.println(" lbs");
    }
    
    loadcell.tare();
    
    if (DEBUG)
    {
        Serial.print("Load cell reading after tare: ");
        Serial.print(loadcell.get_units(), 2); Serial.println(" lbs");
    } 
}

float loadcellReadCurrentValue()
{
    float reading = loadcell.get_units();
    
    if (DEBUG)
    {
        Serial.print("Scale Reading: "); Serial.print(reading, 3); Serial.println(" lbs");
    }
    
    return reading;
}