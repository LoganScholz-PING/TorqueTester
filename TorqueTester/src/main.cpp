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
// all updateable text fields are below
extern NexText ntCURRENTTORQUE_pg0;
extern NexText ntMAXTORQUE_pg0;
extern NexText ntSTATUS_pg0;
extern NexText ntMAXTORQUE_pg3;
extern NexText ntMAXTORQUE_pg4;
extern NexText ntTESTRESULT_pg4;
extern NexText ntCURRENTREADING_pg5;
extern NexTouch *nex_listen_list[];
/* ==== END NEX_DISPLAY.cpp EXTERNS ==== */




/* ==== START STATE MACHINE DEFINITIONS ==== */
StateMachine machine = StateMachine();

// enumerations for tracking current machine state
enum MACHINESTATE { IDLE = 0, HOMEMTR = 1, LOADCLUB = 2, READY = 3, RUNNING = 4, COMPLETE = 5, UNDEF = 99 };
MACHINESTATE _machine_state = IDLE;

// ========= State Functions =========

// **********************************
// **** STATE0 IDLE (WAIT) STATE ****
// **********************************
void state0()
{
  /* STATE 0 (IDLE STATE)
   *
   * State 0 Description:
   * This is the "Wait" state, entered upon machine initialization,
   * when a test is completed, or when an unknown or cancel condition
   * forces the test back to start.
   * 
   * State 0 Actions:
   * Motor Disabled
   * Results of previous test displayed (if any)
   * Clamps manually OPENABLE (but cannot close!!!!)
   * 
   * State 0 Transition Criteria:
   * S0->S1: User presses "START" button on page 0
   * 
   */
  _machine_state = IDLE;
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
  /* STATE 1 (HOME STATE)
   *
   * State 1 Description:
   * In this state operator will be asked to manually home
   * the motor assembly to the HOME position, once the metal
   * rod on the motor assembly breaks the HOME optical stop
   * the operator will be allowed to move forward to the next state
   * 
   * State 1 Actions:
   * Motor enabled
   * Motor Jog CW/CCW buttons become interactable
   * 
   * State 1 Transition Criteria:
   * S1->S2: Motor is homed to HOME optical stop
   * S1->S0: Operator presses Cancel button
   */
  _machine_state = HOMEMTR;
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
  /* STATE 2 (LOADCLUB STATE)
   *
   * State 2 Description:
   * In this state operator will be allowed to open AND close the
   * clamps (state 0 operator could only OPEN clamp, not close). 
   * When the clamps are open, PSI will be near 0. When the clamps
   * are closed, PSI will be higher (somewhere between 70psi and 90psi)
   * 
   * State 2 Actions:
   * Access to clamp relay is granted with the "CLOSE CAMP" button
   * 
   * State 2 Transition Criteria:
   * S2->S3: PSI is somewhere within valid range
   * S2->S0: Operator presses "CANCEL"
   */
  
  _machine_state = LOADCLUB;
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
  /* STATE 3 (READY STATE)
   *
   * State 3 Description:
   * In state 2 operator pressed "CLOSE CLAMP" button and achieved
   * a PSI value that is within the valid range (70psi to ~90psi).
   * At this point in time the club is firmly clamped and the test
   * is ready to begin. State 3 is essentially an "idle" state
   * waiting for the operator to press the "START TEST" button on
   * the "Page 2 - LoadClub" nextion display page to kick off the
   * test officially
   * 
   * State 3 Actions:
   * Reset all previous torque data (torque max value) to prepare
   *  for the new upcoming test
   * 
   * State 3 Transition Criteria:
   * S3->S4: Operator presses the "START TEST" button AND PSI is within
   *         valid range
   * S3->S0: Operator presses the CANCEL button
   * 
   */

  _machine_state = READY;
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
  /* STATE 4 (RUNNING STATE)
   *
   * State 4 Description:
   * In this state the test is actually running. The motor
   * will be rotating towards the club and applying a 
   * torque force to the head of the club while the shaft is
   * firmly clamped. The maximum value of the torque will be
   * tracked and displayed on the screen in real-time. 
   * 
   * State 4 Actions:
   * Motor enabled and rotating such that it applies a torque
   *  force to the club head
   * Highest achieved torque actively being tracked on screen
   * 
   * State 4 Transition Criteria:
   * S4->S5 (pass): the club withstands an acceptable amount of torque,
   *                the motor backs off a bit, and the nextion display
   *                page moves to the final "Page 4 - FINISHED" page and
   *                displays the test result (PASS) along with the highest
   *                achieved torque value.
   * S4->S5 (fail): the club breaks before achieving the maximum amount of
   *                allowable torque and the motor continues moving until
   *                it breaks the EMERGENCY optical stop. The nextion display
   *                page is moved to "Page 4 - FINISHED" and displays the
   *                test result (FAIL) and the highest achieved torque value.
   *        S4->S0: Operator presses "CANCEL TEST" button.
   */

  _machine_state = RUNNING;
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
  /* STATE 5 (COMPLETE STATE)
   *
   * State 5 Description:
   * By this point in the test 1 of 2 things have happend: either
   * the motor assembly broke the head off the club (FAIL), or the club head
   * withstood an acceptable amount of torque (PASS). The nextion display
   * "Page 4 - Finished" will indicate the PASS/FAIL status of the test
   * along with the highest achieved torque value. From here the operator can
   * press the "Finish" button to return to "Page 0 - Main"
   * 
   * State 5 Actions:
   * Display final maximum torque value achieved during test
   * Display overall test status (PASS/FAIL)
   * 
   * State 5 Transition Criteria:
   * S5->S0: Operator presses "FINISH" button
   * 
   */

  _machine_state = COMPLETE;
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
  char buffer_tq[10];
  char buffer_air[10]; 
  float tq = loadcellReadCurrentValue();
  
  // decimal to string float (Arduino.h built-in)
  dtostrf(tq, 6, 2, buffer_tq);
  // display current torque
  ntCURRENTTORQUE_pg0.setText(buffer_tq);
  ntCURRENTREADING_pg5.setText(buffer_tq);

  // track the highest torque value achieved since reset
  if ( tq > highest_torque )
  {
    highest_torque = tq;
    dtostrf(highest_torque, 6, 2, buffer_tq);
    // display highest torque achieved in the current test 
    ntMAXTORQUE_pg0.setText(buffer_tq);
    ntMAXTORQUE_pg3.setText(buffer_tq);
    ntMAXTORQUE_pg4.setText(buffer_tq);
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

  //if ( motor_moving ) { ifMovingCheckCountFeedback(); }  // may be obsolete after state machine implementation

  if ( opticalHOME_stop_hit )
  {
    //servoMotorEnable(MOTOR_DISABLED); // done in interrupt function
    if ( DEBUG ) { Serial.println("*OPTICAL HOME STOP HIT*"); }
    opticalHOME_stop_hit = false;
  }

  if ( opticalEMERGENCY_stop_hit )
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