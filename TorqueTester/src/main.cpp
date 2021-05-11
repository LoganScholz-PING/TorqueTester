// Official Includes:
#include <Arduino.h>
#include <LS7366.h>
// NOTE for Nextion.h:
// you need to open NexConfig.h and set the "nexSerial"
// to the serial port you intend to use on the Arduino.
// In our case, we are using "Serial2" (pins 16 and 17)
// for serial communication on the mega
#include <Nextion.h>
#include <SPI.h> // nextion needs this
#include <SD.h> // nextion needs this
#include <SoftwareSerial.h> // nextion needs this
// Unofficial Includes:
#include "ServoMotorControl.h"
#include "LoadCell.h"
#include "Heartbeat.h"
#include "EEPROM_Arduino.h"
#include "relaypins.h"
#include "NEX_DISPLAY.h"


/* === START PROGRAM SPECIFIC DEFINES === */
#define SOP '<' // denotes start of serial data packet
#define EOP '>' // denotes end of serial data packet

#define pinOPTICALHOME 13 // PB7
//#define pinOPTICALEMERGENCY 14 // NOT ATTACHED YET
#define DEBUG_PIN 42      // PL7
boolean DEBUG = false;
/* === END PROGRAM SPECIFIC DEFINES === */



/* === START PROGRAM FLOW CONTROL VARIABLES === */
// serial comm vars
char ctrlChar;
char serialData[20];     // serial receive buffer
byte index;              // serialData indexer
boolean started = false; // serial data flow control
boolean ended   = false;   // serial data flow control

// optical stop vars
boolean optical_home_stop_chk      = false; // samples the digital state of the HOME optical stop
boolean optical_home_stop_hit      = false; // true for 10ms when rising edge detected on HOME optical stop
boolean optical_emergency_stop_chk = false; // samples the digital state of the EMERGENCY optical stop
boolean optical_emergency_stop_hit = false; // true for 10ms when rising edge detected on EMERGENCY optical stop
boolean prev_opt_home_reading      = false; // state control variable for determining rising edge of HOME stop
boolean prev_opt_emergency_reading = false; // state control variable for determining rising edge of EMERGENCY stop
long opt_home_debounce_time             = 0;     // 10ms debounce timer that starts upon detection of rising edge of HOME stop
long opt_emergency_debounce_time        = 0;     // 10ms debounce timer that starts upon detection of rising edge of EMERGENCY stop

// heartbeat vars
TimeSlice StatusSlice(2000);       // heartbeat object
TimeSlice ParamSlice(100);
long _heartbeat_interval    = 4000; // 4 second heartbeat
long _param_update_interval = 500;  // refresh parameters on the touch screen every 100ms
long _load_update_interval  = 50;   // read torque load every 50ms
unsigned long hb_timer      = 0;    // holds the heartbeat timer in main loop
unsigned long param_timer   = 0;    // holds the param timer in main loop

// Nextion Display vars
float highest_torque = 0;

// misc vars
int START_MOTOR      = 1;      // for passing to runMotor()
int STOP_MOTOR       = 0;      // for passing to runMotor()
long quad            = 0;      // for tracking motor movement
double start_counts  = 0;      // for tracking motor movement
double total_counts  = 0;      // for tracking motor movement
boolean motor_moving = false;  // for tracking if motor is moving

// State machine definitions
enum MACHINESTATE {
  Idle = 0, Running = 1, Alarm = 2, Homing = 3, Jogging = 4, SeekToLoad = 5,
  Loading = 7, Relase = 8, HM_CW_0 = 20, HM_CCW_1 = 21, HM_BACKOFF_2 = 22, 
  HM_SUCCESS = 23, HM_OFF_SWITCH = 24, CNT_BEGIN = 25, CNT_ACCEL = 28,
  CNT_RUNNING = 26, CNT_FINISH = 27, Undefined = 99
};

enum LIMITSTATE { LOW2HIGH = 0, HIGH2LOW = 1, UNKNOWN = 99 };

LIMITSTATE limitState = UNKNOWN;

MACHINESTATE _home_state    = Idle;
MACHINESTATE _count_state   = Idle;
MACHINESTATE _machine_state = Idle;
/* ==== END PROGRAM FLOW CONTROL VARIABLES ==== */




/* === START ServoMotorControl.cpp EXTERNS === */
extern boolean _motor_enable;
/* ==== END ServoMotorControl.cpp EXTERNS ==== */


/* === START NEX_DISPLAY.cpp EXTERNS === */
extern NexText ntTORQUE;
extern NexText ntAIR;
extern NexText ntMAXTORQUE;
extern NexText ntRESULT;
extern NexText ntSTATUS;
extern NexTouch *nex_listen_list[];
/* ==== END NEX_DISPLAY.cpp EXTERNS ==== */




/* Function Name: void checkSerial()
 * 
 * Returns: None
 *
 * Description:
 *  - inspects incoming serial data for "packetized" data. A proper 
 *    serial data packet starts with an '<' and ends with an '>'
 */
void checkSerial()
{
  // if serial is available, receive the
  // serial data packet (it better be formatted
  // correctly!!)
  while (Serial.available() > 0)
  {
    char inChar = Serial.read();
    // check if we receive the start character  
    // (SOP) of a serial packet
    if (inChar == SOP)
    {
      index = 0;
      serialData[index] = '\0'; // null character
      started = true;
      ended = false;
    }
    // check if we receive the end character
    // (EOP) of a serial packet
    else if (inChar == EOP)
    {
      ended = true;
      break;
    }
    else
    {
      if (index < 79)
      {
        serialData[index] = inChar;
        ++index;
        serialData[index] = '\0';
      }
    }
  }

  if (started && ended)
  {
    // packet start and end control characters
    // received, begin packet processing
    switch (serialData[0])
    {
      case '1':
        // DO STUFF
        break;
      default:
        // we received a packet where serialData[0] 
        // is unrecognized
        Serial.print("ERROR Invalid or uninterpretable command received on serial: ");
        Serial.println(serialData[0]);
        break;
    }

    // packet processing completed, reset packet
    // parameters to get ready for next packet
    started = false;
    ended = false;
    index = 0;
    serialData[index] = '\0';
  }
} // end void checkSerial()




/* *Function Name: void checkOpticalStops()
 * 
 * *Returns: None
 *
 * *Description:
 *  - Checks for a rising edge on either the HOME or EMERGENCY optical stops
 * 
 * **!TODO: Add emergency optical stop once hooked up!**
 * 
 */
void checkOpticalStops()
{
  // pinOPTICALSTOP is HIGH if beam is broken
  //optical_stop_chk = digitalRead(pinOPTICALSTOP); // slow
  optical_home_stop_chk = (_SFR_IO8(0X03) & B10000000); // more performant

  // if statement checks for rising edge condition
  // on optical_stop_chk (with a 10ms debounce)
  if ((optical_home_stop_chk) && 
      (prev_opt_home_reading != optical_home_stop_chk) && 
      (millis() - opt_home_debounce_time >= 10))
  {
    optical_home_stop_hit = true;
    ntSTATUS.setText("HOME STOP HIT");
    if (DEBUG)
    {
      Serial.println("*OPTICAL HOME STOP HIT*");
    }
    opt_home_debounce_time = millis();
  }

  // reset the boolean that tracks optical beam state after
  // 10ms (when optical endstop beam is broken this discrete
  // will stay true for 10ms)
  if (optical_home_stop_hit && (millis() - opt_home_debounce_time >= 10))
  {
    optical_home_stop_hit = false;
  }

  // if motor is enabled and the optical stop is hit
  // turn off the motor
  if (_motor_enable && optical_home_stop_hit)
  {
    servoMotorEnable(MOTOR_DISABLED);
    if (DEBUG)
    {
      Serial.println("*STOP MOTOR DUE TO OPTICAL HOME STOP*");
    }
  } 

  prev_opt_home_reading = optical_home_stop_chk;
} // void checkOpticalStop()




void ifMovingCheckCountFeedback()
{
  quad = servoMotorReadQuadratureCount();
  // TODO: need to initialize start_counts and total_counts
  //       when motor movement begins
  if ( (abs(quad-start_counts) >= abs(total_counts)) )
  {   
    servoMotorEnable(MOTOR_DISABLED);
    motor_moving = false;
    if (DEBUG)
    {
      Serial.println("Main loop has stopped motor movement"); 
      Serial.print("total_counts: "); Serial.println(total_counts);
      Serial.print("start_counts: "); Serial.println(start_counts);
      Serial.print("End counts (quad): "); Serial.println(abs(quad));
      Serial.print("Count delta: "); Serial.println(abs(quad-start_counts));
    }
  }
} // end void ifMovingCheckCountFeedback()




void updateEnvironment()
{
  //char* cp; // for tracking state machine current state, not implemented
  //double pos = servoMotorReadRotationAngle(); // doesn't work yet
  float tq = loadcellReadCurrentValue();

  Serial.print("{,");
  Serial.print(":T="); Serial.print(tq);
  //Serial.print(":A="); Serial.print(pos);
  Serial.print(":MTR="); Serial.print(motor_moving);
  Serial.println(",}");
}

void updateDisplay()
{
  // TODO: update the following parameters on the touch screen:
  // 1-Torque *done*
  // 2-Highest Torque *done*
  //   3-Air Pressure !not ready!
  //   4-State machine state !not ready!
  
  char buffer_tq[10]; 
  float tq = loadcellReadCurrentValue();
  
  // decimal to string float (Arduino.h built-in)
  dtostrf(tq, 6, 2, buffer_tq);
  // display current torque
  ntTORQUE.setText(buffer_tq); 

  if ( tq > highest_torque )
  {
    highest_torque = tq;
    dtostrf(highest_torque, 6, 2, buffer_tq);
    // display highest torque achieved in the current test 
    ntMAXTORQUE.setText(buffer_tq);
  }

}



void setup() 
{
  Serial.begin(9600);  // for debug output
  Serial2.begin(9600); // Serial2 is for Nextion communication
  delay(500); // allow serial to settle

  servoMotorSetup();
  // need to do setupEEPROM() before loadcellSetup()
  // because loadcellSetup() retrieves data from EEPROM
  // like scale_zero_bias and scale_calibration_factor
  setupEEPROM(); 
  loadcellSetup();
  setupNextion();

  pinMode(pinOPTICALHOME, INPUT);
  pinMode(DEBUG_PIN, INPUT_PULLUP);
  pinMode(RELAY_2_CTRL, OUTPUT);

  digitalWrite(RELAY_2_CTRL, LOW);

  // initialize heartbeat to 4 seconds
  StatusSlice.Interval(_heartbeat_interval);
  ParamSlice.Interval(_param_update_interval);
}


void loop() 
{
  // read debug pin 42 state
  DEBUG = !(_SFR_MEM8(0x109) & B10000000);

  checkSerial();
  checkOpticalStops();

  nexLoop(nex_listen_list);

  if (motor_moving)
  {
    ifMovingCheckCountFeedback(); 
  }

/* TODO: UNCOMMENT WHEN READY TO SHOW HEARTBEAT AGAIN
  hb_timer = millis();
  if (StatusSlice.Triggered(hb_timer))
  {
    updateEnvironment(); // output heartbeat to serial
  } 
*/

  param_timer = millis();
  if (ParamSlice.Triggered(param_timer))
  {
    updateDisplay();
  }
}