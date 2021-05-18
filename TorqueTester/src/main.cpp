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
#include <StateMachine.h>
// Unofficial Includes:
#include "ServoMotorControl.h"
#include "LoadCell.h"
#include "Heartbeat.h"
#include "EEPROM_Arduino.h"
#include "relaypins.h"
#include "NEX_DISPLAY.h"
#include "AirPressure.h"


/* === START PROGRAM SPECIFIC DEFINES === */
#define SOP '<' // denotes start of serial data packet
#define EOP '>' // denotes end of serial data packet

#define pinOPTICALHOME 20 // PB7
//#define pinOPTICALEMERGENCY 19 // NOT ATTACHED YET

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
//boolean optical_home_stop_chk      = false; // samples the digital state of the HOME optical stop
volatile boolean opticalHOME_stop_hit      = false; // true for 10ms when rising edge detected on HOME optical stop
//boolean optical_emergency_stop_chk = false; // samples the digital state of the EMERGENCY optical stop
volatile boolean opticalEMERGENCY_stop_hit = false; // true for 10ms when rising edge detected on EMERGENCY optical stop
//boolean prev_opt_home_reading      = false; // state control variable for determining rising edge of HOME stop
//boolean prev_opt_emergency_reading = false; // state control variable for determining rising edge of EMERGENCY stop
//long opt_home_debounce_time             = 0;     // 10ms debounce timer that starts upon detection of rising edge of HOME stop
//long opt_emergency_debounce_time        = 0;     // 10ms debounce timer that starts upon detection of rising edge of EMERGENCY stop

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




/* ==== START STATE MACHINE DEFINITIONS ==== */
StateMachine machine = StateMachine();

// ========= State Functions =========

// **********************************
// **** STATE0 IDLE (WAIT) STATE ****
// **********************************
void state0()
{
  Serial.println("State 0 - IDLE STATE");
}

/* DO WE NEED THIS????
// generic transition to State 0
bool transitionS0()
{
  return true;
}
*/

// State0 -> State1 transition criteria
bool transitionS0S1()
{
  return true;
}


// *********************************
// **** STATE1 HOME MOTOR STATE ****
// *********************************
void state1()
{
  Serial.println("State 1 - HOME MOTOR STATE");
}

// State1 -> State0 transition criteria
bool transitionS1S0()
{ 
  // operator pressed cancel in State1
  return false; // false for testing
}

// State1 -> State2 transition criteria
bool transitionS1S2()
{ 
  return true;
}


// ********************************
// **** STATE2 LOAD CLUB STATE ****
// ********************************
void state2()
{
  Serial.println("State 2 - LOAD CLUB STATE");
}

// State2 -> State0 transition criteria
bool transitionS2S0()
{ 
  // operator pressed cancel in State2
  return false; // false for testing
}

// State2 -> State3 transition criteria
bool transitionS2S3()
{ 
  return true;
}


// ************************************
// **** STATE3 READY TO TEST STATE ****
// ************************************
void state3()
{
  Serial.println("State 3 - READY TO TEST STATE");
}

// State3 -> State0 transition criteria
bool transitionS3S0()
{ 
  // operator pressed cancel in State3
  return false; // false for testing
}

// State3 -> State4 transition criteria
bool transitionS3S4()
{ 
  return true;
}


// ***************************************
// **** STATE4 TEST IN PROGRESS STATE ****
// ***************************************
void state4()
{
  Serial.println("State 4 - TEST IN PROGRESS STATE");
}

// State4 -> State0 transition criteria
bool transitionS4S0()
{ 
  // Operator pressed CANCEL during test
  return false; // false for testing
}

// State4 -> State5 transition criteria
bool transitionS4S5()
{
  // 2 conditions can transition you from S4 to S5 (in order of importance)
  //   1. Emergency optical endstop activated (club broke and failed torque test)
  //   2. Expected torque value achieved (club passed torque test) 
  return true;
}


// ************************************
// **** STATE5 TEST COMPLETE STATE ****
// ************************************
void state5()
{
  Serial.println("State 5 - TEST COMPLETE STATE");
}

// State5 -> State0 transition criteria
bool transitionS5S0()
{ 
  // 3 things happen in this state:
  //  1. Test results are displayed (largest torque value achieved)
  //  2. Test status is displayed (PASS/FAIL)
  //  3. A continue button is displayed to take user back to State0
  return true;
}

State* S0 = machine.addState(&state0);
State* S1 = machine.addState(&state1);
State* S2 = machine.addState(&state2);
State* S3 = machine.addState(&state3);
State* S4 = machine.addState(&state4);
State* S5 = machine.addState(&state5);

/* ===== END STATE MACHINE DEFINITIONS ===== */









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
}




// interrupt function for optical HOME switch
void homeOpticalStopInt()
{
  opticalHOME_stop_hit = true;
  servoMotorEnable(MOTOR_DISABLED);
}

// interrupt
void emergencyOpticalStopInt()
{
  opticalEMERGENCY_stop_hit = true;
  servoMotorEnable(MOTOR_DISABLED);
}




/*
* TODO: MAY NEED TO RE-THINK THIS.. NOT ALL MOTOR MOVEMENT
*       FUNCTIONS TRACK start_counts AND total_counts, so 
*       the IF condition of this function never gets entered
*       and servoMotorReadQuadratureCount() spits out infinite
*       debug text
*/
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
}




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
  char buffer_air[10]; 
  float tq = loadcellReadCurrentValue();
  float air = readAirPressure();
  
  // decimal to string float (Arduino.h built-in)
  dtostrf(tq, 6, 2, buffer_tq);
  dtostrf(air, 6, 2, buffer_air);
  // display current torque
  ntTORQUE.setText(buffer_tq);
  ntAIR.setText(buffer_air); 

  // track the highest torque value achieved since reset
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

  pinMode(pinOPTICALHOME, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinOPTICALHOME), 
                  homeOpticalStopInt,
                  RISING);

/* TODO: UNCOMMENT WHEN EMERGENCY STOP HOOKED UP
  pinMode(pinOPTICALEMERGENCY, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinOPTICALEMERGENCY), 
                  emergencyOpticalStopInt,
                  RISING);
*/

  pinMode(DEBUG_PIN, INPUT_PULLUP);

  servoMotorSetup();
  setupAirValve();
  // need to do setupEEPROM() before loadcellSetup()
  // because loadcellSetup() retrieves data from EEPROM
  // like scale_zero_bias and scale_calibration_factor
  setupEEPROM(); 
  loadcellSetup();
  setupNextion();

  // initialize heartbeat and param update to touchscreen intervals
  StatusSlice.Interval(_heartbeat_interval);
  ParamSlice.Interval(_param_update_interval);

  //S0->addTransition(&transitionS0, S0); // idk if this is needed
  S0->addTransition(&transitionS0S1, S1); // IDLE to HOME MOTOR
  S1->addTransition(&transitionS1S0, S0); // HOME MOTOR to IDLE
  S1->addTransition(&transitionS1S2, S2); // HOME MOTOR to LOAD CLUB
  S2->addTransition(&transitionS2S0, S0); // LOAD CLUB to IDLE
  S2->addTransition(&transitionS2S3, S3); // LOAD CLUB to START TEST
  S3->addTransition(&transitionS3S0, S0); // START TEST to IDLE
  S3->addTransition(&transitionS3S4, S4); // START TEST to TEST RUNNING
  S4->addTransition(&transitionS4S0, S0); // TEST RUNNING to IDLE
  S4->addTransition(&transitionS4S5, S5); // TEST RUNNING to TEST FINISHED
  S5->addTransition(&transitionS5S0, S0); // TEST FINISHED to IDLE
}




void loop() 
{
  // read debug pin 42 state
  DEBUG = !(_SFR_MEM8(0x109) & B10000000);

  checkSerial();
  nexLoop(nex_listen_list);

  if ( motor_moving ) { ifMovingCheckCountFeedback(); }

  if ( opticalHOME_stop_hit )
  {
    //servoMotorEnable(MOTOR_DISABLED); // done in interrupt function
    if ( DEBUG ) { Serial.println("*OPTICAL HOME STOP HIT*"); }
    opticalHOME_stop_hit = false;
  }

  if ( opticalHOME_stop_hit )
  {
    //servoMotorEnable(MOTOR_DISABLED); // done in interrupt function
    if ( DEBUG ) { Serial.println("*OPTICAL EMERGENCY STOP HIT*"); }
    opticalEMERGENCY_stop_hit = false;
  }

/* TODO: UNCOMMENT WHEN READY TO SHOW HEARTBEAT AGAIN
  hb_timer = millis();
  if (StatusSlice.Triggered(hb_timer)) { updateEnvironment(); } // output heartbeat to serial 
*/

  /* TODO: UNCOMMENT WHEN READY TO HAVE TOUCHSCREEN HOOKED UP
  param_timer = millis();
  if (ParamSlice.Triggered(param_timer)) { updateDisplay(); }
  */

  machine.run();
  //delay(1000); // may need this delay if loop is too fast for state machine
}