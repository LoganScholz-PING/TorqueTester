#include "ServoMotorControl.h"
#include "printBinary.h"
#include <LS7366.h>
#include <NewTone.h>

extern boolean DEBUG; // from main.cpp
extern boolean motor_moving; // from main.cpp

/* === START SERVO MOTOR VARIABLE DEFINITIONS === */
boolean _motor_enable = false;
boolean _motor_direction = MOTOR_CCW;

// 4000 quadrature counts per 360 degree revolution
// 800 pulses per 360 degree revolution
// 5 counts per pulse which is 0.45 degrees of rotation
long pulses_per_revolution = 800;
double degrees_per_pulse = (360 / (double)pulses_per_revolution); // 0.45 @ 800ppr
//double pulses_per_degree = (pulses_per_revolution / 360);
unsigned long _max_freq = 35000;
int count_per_pulse = 4000 / pulses_per_revolution; // 5 counts per pulse @ 800 pulse / rot

float _current_speed = 0; // 0-1 controls percent of _max_freq
/* === END SERVO MOTOR VARIABLE DEFINITIONS === */ 

/* === START QUADRATURE VARIABLE DEFINITIONS === */
long _quad_count = 0;
LS7366 QuadCounter(QUAD_CHIPSELECT);
/* === END QUADRATURE VARIABLE DEFINITIONS === */


void servoMotorSetup()
{
    pinMode(mtrPulsePin, OUTPUT);
    pinMode(mtrDirectionPin, OUTPUT);
    pinMode(mtrEnablePin, OUTPUT);

    digitalWrite(mtrEnablePin, MOTOR_DISABLED);
    digitalWrite(mtrDirectionPin, MOTOR_CCW);

    servoMotorSetupQuadrature();
}

void servoMotorEnable(byte v)
{
    // Toggle state of the mtrEnablePin
    digitalWrite(mtrEnablePin, v);
    _motor_enable = (v == 0);
    if (_motor_enable == false)
    {
        // Turn off the PWM to the motor
        // When Motor is disabled
        noNewTone(mtrPulsePin);
        motor_moving = false;
    }
    //Serial.print("_motor_enable status: "); Serial.println(_motor_enable);
}

void servoMotorFrequency(float f)
{
    if ( f < 0 ) { f = 0; }
    if ( f > 1.0 ) { f = 1.0; }

    _current_speed = f;
    unsigned long x = (f * _max_freq);

    // NewTone(pin, freq, length)
    // if length not present, it lasts forever
    NewTone(mtrPulsePin, x);
}

void servoMotorDirection(boolean v)
{
    // TODO
    _motor_direction = v;
    digitalWrite(mtrDirectionPin, _motor_direction);
}

void servoMotorAdjustSpeed(unsigned long nw)
{
    // TODO
}


void runMotor(int cmd, float f)
{
  if (cmd == 1)
  {
    // cmd == 1 == start motor test
    if (DEBUG)
    {
        Serial.println("Running Motor");
    }
    
    servoMotorFrequency(f);   
    servoMotorEnable(MOTOR_ENABLED);
    
    if (DEBUG)
    {
        Serial.print("_motor_enable Status: "); Serial.println(_motor_enable);
    }
  }
  else if (cmd == 0)
  {
    // cmd == 0 == stop motor test 
    if (DEBUG)
    {
        Serial.println("Stopping motor test");
    }
    
    servoMotorEnable(MOTOR_DISABLED);
    
    if (DEBUG)
    {
        Serial.print("_motor_enable Status: "); Serial.println(_motor_enable);
    }
  }
  else
  {
    // unknown value passed to cmd
    Serial.println("(else) stopping motor");
    servoMotorEnable(MOTOR_DISABLED);
    Serial.print("_motor_enable Status: "); Serial.println(_motor_enable);
  }
}


int servoMotorMoveDegrees(int deg)
{
    if (DEBUG)
    {
        Serial.print("Value passed to servoMotorMoveDegrees(): "); Serial.println(deg);
    }
    
    /***************************************************************
     * 
     *  2 schools of thought here:
     *  1. If deg > 360, just spin all the way around the
     *     amount of degrees requested
     *  2. If deg > 360, interpret it such that extra revolutions
     *     are removed, eg: 520 degrees would be interpreted as
     *     160 degrees
     * 
     *  We'll implement #2 here, for #1 just divide deg by deg_per_pulse
     *  to get total pulses
     * 
     *****************************************************************/
    
    if (deg > 360)
    {
        // the math below fails if user passes a multiple of 360 in, 
        // so let's check for this condition first
        bool skip_math = false;
        if ( deg % 360 == 0 ) 
        { 
            deg = 360; 
            skip_math = true;            
        }       
        if ( skip_math == false )
        {
            int overage = deg / 360;
            deg = deg - (overage * 360);
        }
    }

    // convert degrees to pulses
    double pulses = (double)deg / degrees_per_pulse;
    double counts = count_per_pulse * pulses;
    
    // double modf (double x, double* intpart); is a C++ built-in
    // that stores the whole part of a float type variable (double
    // or single) in the intpart, and returns the fractional part
    double whole, error; 
    error = modf(pulses, &whole);
    
    int correction = error * deg; // milliseconds

    // using NewTone to activate the timer registers
    // period = 5 (200Hz, 2.5ms on/2.5ms off)
    // unsigned long time_on_ms = (5 * pulses) - correction;
    unsigned long time_on_ms = (5 * pulses); // uncorrected

    if (DEBUG)
    {
        Serial.print("deg after sanitization: "); Serial.println(deg);
        Serial.print("Total calculated pulses for input degrees: "); Serial.println(pulses);
        Serial.print("fractional part: "); Serial.println(error);
        Serial.print("Calculated time correction: "); Serial.print(correction); Serial.println(" milliseconds");
        Serial.print("Integer value of pulses: "); Serial.println((int)pulses);
        Serial.print("Rotating "); Serial.print(deg); Serial.println(" degrees");
    }

    // start from known state for the rotation
    digitalWrite(mtrPulsePin, LOW);
    servoMotorEnable(MOTOR_DISABLED);
    servoMotorEnable(MOTOR_ENABLED);

    if (DEBUG)
    {
        Serial.print("Rotation time: "); Serial.print(time_on_ms); Serial.println(" milliseconds");
    }

    NewTone(mtrPulsePin, 200, time_on_ms);

    return(counts);
}


void servoMotorSetupQuadrature()
{
    // 0x00 | 0x00 | 0x00 | 0x03 -- total: 0b0000 0011
    QuadCounter.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4); 
	//        0x00 | 0x00 | 0x00 -- total: 0b0000 0000
    QuadCounter.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4);
	//	QuadCounter.clear_counter();
	QuadCounter.clear_status_register();
	QuadCounter.write_data_register(4);
}

long servoMotorReadQuadratureCount()
{
    _quad_count = QuadCounter.read_counter();
    
    /* the output from the if statement below can get obnoxious
    if (DEBUG)
    {
        byte* b = (byte*)(&_quad_count);
        Serial.print("readQuad:"); 
        Serial.print(_quad_count); 
        Serial.print(" ");
        print_binary(*(b + 2));
        Serial.print(" ");
        print_binary(*(b + 3));
        Serial.print(" ");
        Serial.println();
    }
    */
    
    return _quad_count;
}

double servoMotorReadRotationAngle()
{
    
    // TODO: THIS FUNCTION DOES NOT WORK YET
    //  - NEED TO UPDATE IT TO BE RELATIVE TO HOME/EMERGENCY STOPS
    //    (AKA: record current quadrature count @ home position and 
    //    consider that 0 degrees)
    
    servoMotorReadQuadratureCount();

    long q = (_quad_count % pulses_per_revolution);

    if (DEBUG)
    {
        Serial.print("Read Quad:"); Serial.print(_quad_count); Serial.print(" || readRot Q:"); Serial.println(q);
    }
    
    double ang = (degrees_per_pulse * q);
    
    if (DEBUG)
    {
        Serial.print("readAng A:"); Serial.println(ang);
    }
    
    return ang;
} 