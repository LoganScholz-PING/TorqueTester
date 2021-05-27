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
#define pinOPTICALEMERGENCY 19 // NOT ATTACHED YET
#define pinESTOP 18 // LOW->HIGH transition when E-STOP BUTTON activated

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
volatile boolean opticalHOME_stop_hit      = false; 
volatile boolean opticalEMERGENCY_stop_hit = false; 

// heartbeat vars
TimeSlice StatusSlice(2000);       // heartbeat object
TimeSlice ParamSlice(100);
long _heartbeat_interval    = 4000; // 4 second heartbeat
long _param_update_interval = 200;  // refresh parameters on the touch screen every 250ms
unsigned long hb_timer      = 0;    // holds the heartbeat timer in main loop
unsigned long param_timer   = 0;    // holds the param timer in main loop

// Nextion Display vars
float highest_torque = 0;

// misc vars
int START_MOTOR      = 1;      // for passing to runMotor()
int STOP_MOTOR       = 0;      // for passing to runMotor()
boolean motor_moving = false;  // for tracking if motor is moving

boolean OVERALL_TEST_PASS = false;
/* ==== END PROGRAM FLOW CONTROL VARIABLES ==== */



/* === START ServoMotorControl.cpp EXTERNS === */
extern boolean _motor_enable;
/* ==== END ServoMotorControl.cpp EXTERNS ==== */


/* === START LoadCell.cpp EXTERNS === */
extern float scale_calibration_factor;
extern long  scale_zero_bias;
extern float scale_calibration_load;
/* ==== END LoadCell.cpp EXTERNS ==== */


/* === START NEX_DISPLAY.cpp EXTERNS === */
// all updateable text fields are below
extern NexText ntCURRENTTORQUE_pg0;
extern NexText ntMAXTORQUE_pg0;
extern NexText ntSTATUS_pg0;
extern NexText ntMAXTORQUE_pg3;
extern NexText ntMAXTORQUE_pg4;
extern NexText ntTESTRESULT_pg4;
extern NexText ntCURRENTREADING_pg5;

// all pages are below
extern NexPage npMAIN_PAGE;
extern NexPage npHOMEMTR_PAGE;
extern NexPage npLOADCLUB_PAGE;
extern NexPage npTESTGOGOGO_PAGE;
extern NexPage npFINISHED_PAGE;
extern NexPage npCALIBRATE_PAGE;

// button listen list below
extern NexTouch *nex_listen_list[];

// these track if a button has been pressed on a page
// because i don't know how to do this any better right now
extern bool nbSTARTTEST_pg0_bool;
extern bool nbOPENCLAMP_pg0_bool;
extern bool nbCALIBRATE_pg0_bool;
extern bool nbTARE_pg0_bool;
extern bool nbMOVECW_pg1_bool;
extern bool nbCANCEL_pg1_bool;
extern bool nbMOVECCW_pg1_bool;
extern bool nbSTOPMOTOR_pg1_bool;
extern bool nbSKIPMOTOR_pg1_bool;
extern bool nbOPENCLAMP_pg2_bool;
extern bool nbSTARTTEST_pg2_bool;
extern bool nbCANCEL_pg2_bool;
extern bool nbCLOSECLAMP_pg2_bool;
extern bool nbCANCEL_pg3_bool;
extern bool nbFINISH_pg4_bool;
extern bool nbTARE_pg5_bool;
extern bool nbPLUS1000_pg5_bool;
extern bool nbPLUS100_pg5_bool;
extern bool nbPLUS10_pg5_bool;
extern bool nbMINUS1000_pg5_bool;
extern bool nbMINUS100_pg5_bool;
extern bool nbMINUS10_pg5_bool;
extern bool nbENDCAL_pg5_bool;
extern bool nbSAVESETTINGS_pg5_bool;
/* ==== END NEX_DISPLAY.cpp EXTERNS ==== */



/* ==== START STATE MACHINE DEFINITIONS ==== */
StateMachine machine = StateMachine();

// enumerations for tracking current machine state
enum MACHINESTATE { IDLE = 0, HOMEMTR = 1, LOADCLUB = 2, READY = 3, 
                    RUNNING = 4, COMPLETE = 5, CALIBRATE = 6, UNDEF = 99 };
MACHINESTATE _machine_state = IDLE;

// ========= State Functions =========


// **********************************
// **** STATE0 IDLE (WAIT) STATE ****
// **********************************
void state0()
{
  /* STATE 0 (IDLE STATE)
   *
   * - State 0 Description:
   * This is the "Wait" state, entered upon machine initialization,
   * when a test is completed, or when an unknown or cancel condition
   * forces the test back to start.
   * 
   * - State 0 Actions:
   * Motor Disabled
   * Results of previous test displayed (if any)
   * Clamps manually OPENABLE (but cannot close!!!!)
   * 
   * - State 0 Transition Criteria:
   * S0->S1: User presses "START" button on page 0
   */
  

  if(machine.executeOnce)
  {
    _machine_state = IDLE;
    servoMotorEnable(MOTOR_DISABLED);
    nbSTARTTEST_pg0_bool = false;
    nbOPENCLAMP_pg0_bool = false;
    nbCALIBRATE_pg0_bool = false;
    nbTARE_pg0_bool = false;

    OVERALL_TEST_PASS = false;

    npMAIN_PAGE.show();
  }
  
  if (nbOPENCLAMP_pg0_bool)
  {
    digitalWrite(CLAMP_PIN, false);
    nbOPENCLAMP_pg0_bool = false;
  }
}

// State0 -> State1 transition criteria
bool transitionS0S1()
{
  if (nbSTARTTEST_pg0_bool)
  {
    // operator pressed "START TEST" button on page 0
    nbSTARTTEST_pg0_bool = false;
    return true;
  }
  return false;
}

// State0 -> State6 transition criteria
bool transitionS0S6()
{
  if (nbCALIBRATE_pg0_bool)
  {
    nbCALIBRATE_pg0_bool = false;
    return true;
  }
  return false;
}


// *********************************
// **** STATE1 HOME MOTOR STATE ****
// *********************************
void state1()
{
  /* STATE 1 (HOME STATE)
   *
   * - State 1 Description:
   * In this state operator will be asked to manually home
   * the motor assembly to the HOME position, once the metal
   * rod on the motor assembly breaks the HOME optical stop
   * the operator will be allowed to move forward to the next state
   * 
   * - State 1 Actions:
   * Motor enabled
   * Motor Jog CW/CCW buttons become interactable
   * 
   * - State 1 Transition Criteria:
   * S1->S2: Motor is homed to HOME optical stop
   * S1->S0: Operator presses Cancel button
   */
  

  if (machine.executeOnce)
  {
    _machine_state = HOMEMTR;
    nbMOVECW_pg1_bool = false;
    nbMOVECCW_pg1_bool = false;
    nbCANCEL_pg1_bool = false;
    nbSTOPMOTOR_pg1_bool = false;
    opticalHOME_stop_hit = false; // just in case the optical stop was broken beforehand
    nbSKIPMOTOR_pg1_bool = false;
    npHOMEMTR_PAGE.show();
  }

  if (nbSTOPMOTOR_pg1_bool)
  {
    servoMotorEnable(MOTOR_DISABLED);
    nbSTOPMOTOR_pg1_bool = false;
  }

  if (nbMOVECW_pg1_bool) 
  {
    // user pressed "MOVE CW" button
    nbMOVECW_pg1_bool = false;
    servoMotorEnable(MOTOR_DISABLED);
    servoMotorDirection(MOTOR_CW);
    runMotor(START_MOTOR, 0.01);
  }

  if (nbMOVECCW_pg1_bool) 
  {
    // user pressed "MOVE CW" button
    nbMOVECCW_pg1_bool = false;
    servoMotorEnable(MOTOR_DISABLED);
    servoMotorDirection(MOTOR_CCW);
    runMotor(START_MOTOR, 0.01);
  }
}

// State1 -> State0 transition criteria
bool transitionS1S0()
{ 
  // operator pressed cancel in State1
  if (nbCANCEL_pg1_bool)
  {
    nbCANCEL_pg1_bool = false;
    return true;
  }
  return false;
}

// State1 -> State2 transition criteria
bool transitionS1S2()
{ 
  if (opticalHOME_stop_hit || nbSKIPMOTOR_pg1_bool )
  {
    opticalHOME_stop_hit = false;
    nbSKIPMOTOR_pg1_bool = false;
    return true;
  }
  return false;
}


// ********************************
// **** STATE2 LOAD CLUB STATE ****
// ********************************
void state2()
{
  /* STATE 2 (LOADCLUB STATE)
   *
   * - State 2 Description:
   * In this state operator will be allowed to open AND close the
   * clamps (state 0 operator could only OPEN clamp, not close). 
   * When the clamps are open, PSI will be near 0. When the clamps
   * are closed, PSI will be higher (somewhere between 70psi and 90psi)
   * 
   * - State 2 Actions:
   * Access to clamp relay is granted with the "CLOSE CAMP" button
   * 
   * - State 2 Transition Criteria:
   * S2->S3: PSI is somewhere within valid range
   * S2->S0: Operator presses "CANCEL"
   */
  
  if ( machine.executeOnce )
  {
    _machine_state = LOADCLUB;
    nbCANCEL_pg2_bool = false;
    nbSTARTTEST_pg2_bool = false;
    nbOPENCLAMP_pg2_bool = false;
    nbCLOSECLAMP_pg2_bool = false;

    npLOADCLUB_PAGE.show();
  }

  if ( nbOPENCLAMP_pg2_bool )
  {
    digitalWrite(CLAMP_PIN, false);
    nbOPENCLAMP_pg2_bool = false;
  }

  if ( nbCLOSECLAMP_pg2_bool )
  {
    digitalWrite(CLAMP_PIN, true);
    nbCLOSECLAMP_pg2_bool = false;
  }
}

// State2 -> State0 transition criteria
bool transitionS2S0()
{ 
  // operator pressed cancel in State2
  if (nbCANCEL_pg2_bool)
  {
    nbCANCEL_pg2_bool = false;
    return true;
  }
  return false;
}

// State2 -> State4 transition criteria (NOTE: state 3 is now obsolete
//                                       because we do not have an air
//                                       pressure check)
//bool transitionS2S3() // !!! STATE 3 OBE !!!
bool transitionS2S4()
{ 
  float air = readAirPressure();

  // originally the plan was to test for just air pressure to
  // determine if the club is in the clamp acceptably. Unfortunately,
  // the way the air pressure transducer is hooked up, PSI is always
  // > 70 as long as air is connected, so this won't work. We'll keep
  // the air pressure check in just in case we end up moving the 
  // pressure transducer to the line that pressurizes when the clamps
  // are closed
  if ( nbSTARTTEST_pg2_bool && (air >= 70) )
  {
    nbSTARTTEST_pg2_bool = false;
    highest_torque = 0;
    return true;
  }
  return false;
}


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// **** STATE3 READY TO TEST STATE ****
// !!!!!THIS STATE IS OBE DUE TO AIR PRESSURE CHECK NOT BEING IN PLACE!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//void state3()
//{
  /* STATE 3 (READY STATE)
   *
   * - State 3 Description:
   * In state 2 operator pressed "CLOSE CLAMP" button and achieved
   * a PSI value that is within the valid range (70psi to ~90psi).
   * At this point in time the club is firmly clamped and the test
   * is ready to begin. State 3 is essentially an "idle" state
   * waiting for the operator to press the "START TEST" button on
   * the "Page 2 - LoadClub" nextion display page to kick off the
   * test officially
   * 
   * - State 3 Actions:
   * Reset all previous torque data (torque max value) to prepare
   *  for the new upcoming test
   * 
   * - State 3 Transition Criteria:
   * S3->S4: Operator presses the "START TEST" button AND PSI is within
   *         valid range
   * S3->S0: Operator presses the CANCEL button
   * 
   */
/* state 3 OBE
  if ( machine.executeOnce )
  {
    _machine_state = READY;

    nbCANCEL_pg2_bool = false;
    nbSTARTTEST_pg2_bool = false;
    nbOPENCLAMP_pg2_bool = false;
    nbCLOSECLAMP_pg2_bool = false;
  }


}

// State3 -> State0 transition criteria
bool transitionS3S0()
{ 
  // operator pressed cancel in State3
  if (nbCANCEL_pg2_bool)
  {
    nbCANCEL_pg2_bool = false;
    return true;
  }
  return false; // false for testing
}

// State3 -> State4 transition criteria
bool transitionS3S4()
{ 
  float air = readAirPressure();

  // club still clamped in and operator presses start
  if ( (air >= 70.0) && (nbSTARTTEST_pg2_bool) )
  {
    nbSTARTTEST_pg2_bool = false;
    return true;
  }
  return false;
}
*/ 

// ***************************************
// **** STATE4 TEST IN PROGRESS STATE ****
// ***************************************
void state4()
{
  /* STATE 4 (RUNNING STATE)
   *
   * - State 4 Description:
   * In this state the test is actually running. The motor
   * will be rotating towards the club and applying a 
   * torque force to the head of the club while the shaft is
   * firmly clamped. The maximum value of the torque will be
   * tracked and displayed on the screen in real-time. This
   * state exists on "Page 3 - TESTGOGOGO"
   * 
   * - State 4 Actions:
   * Motor enabled and rotating such that it applies a torque
   *  force to the club head
   * Highest achieved torque actively being tracked on screen
   * 
   * - State 4 Transition Criteria:
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

  if ( machine.executeOnce )
  {
    _machine_state = RUNNING;
    nbCANCEL_pg3_bool = false;
    opticalEMERGENCY_stop_hit = false;

    highest_torque = 0;

    npTESTGOGOGO_PAGE.show();

    servoMotorEnable(MOTOR_DISABLED);
    servoMotorDirection(MOTOR_CW);
    runMotor(START_MOTOR, 0.01); // may need to adjust speed slower
  }
}

// State4 -> State0 transition criteria
bool transitionS4S0()
{ 
  // Operator pressed CANCEL during test
  if ( nbCANCEL_pg3_bool )
  {
    servoMotorEnable(MOTOR_DISABLED);
    nbCANCEL_pg3_bool = false;
    return true;
  }
  return false; // false for testing
}

// State4 -> State5 transition criteria
bool transitionS4S5()
{
  // 2 conditions can transition you from S4 to S5 (in order of importance)
  //   1. Emergency optical endstop activated (club broke and failed torque test)
  //   2. Expected torque value achieved (club passed torque test) 

  if ( opticalEMERGENCY_stop_hit )
  {
    opticalEMERGENCY_stop_hit = false;
    // the emergency optical stop was broken so that means
    // the club broke and the test FAILed overall

    // motor backoff clubhead
    servoMotorEnable(MOTOR_DISABLED);
    servoMotorDirection(MOTOR_CCW); // back the torque tester off the club
    runMotor(START_MOTOR, 0.05);
    delay(3000); // let the motor move away for 3 seconds
    servoMotorEnable(MOTOR_DISABLED);

    OVERALL_TEST_PASS = false;
    return true;
  }

  float tq = loadcellReadCurrentValue();
  if ( (tq >= 100.0) && (!opticalEMERGENCY_stop_hit) )
  {
    // club withstood max torque successfully
    // so the test is considered a PASS

    // motor backoff clubhead
    servoMotorEnable(MOTOR_DISABLED);
    servoMotorDirection(MOTOR_CCW); // back the torque tester off the club
    runMotor(START_MOTOR, 0.05);
    delay(3000); // let the motor move away for 3 seconds
    servoMotorEnable(MOTOR_DISABLED);
    
    OVERALL_TEST_PASS = true;
    return true;
  }

  return false;
}


// ************************************
// **** STATE5 TEST COMPLETE STATE ****
// ************************************
void state5()
{
  /* STATE 5 (COMPLETE STATE)
   *
   * - State 5 Description:
   * By this point in the test 1 of 2 things have happend: either
   * the motor assembly broke the head off the club (FAIL), or the club head
   * withstood an acceptable amount of torque (PASS). The nextion display
   * "Page 4 - Finished" will indicate the PASS/FAIL status of the test
   * along with the highest achieved torque value. From here the operator can
   * press the "Finish" button to return to "Page 0 - Main"
   * 
   * - State 5 Actions:
   * Display final maximum torque value achieved during test
   * Display overall test status (PASS/FAIL)
   * 
   * - State 5 Transition Criteria:
   * S5->S0: Operator presses "FINISH" button
   * 
   */

  if ( machine.executeOnce )
  {
    _machine_state = COMPLETE;
    servoMotorEnable(MOTOR_DISABLED);
    nbFINISH_pg4_bool = false;

    npFINISHED_PAGE.show();

    if (OVERALL_TEST_PASS)
    {
      ntTESTRESULT_pg4.setText("PASS");
    } 
    else
    {
      ntTESTRESULT_pg4.setText("FAIL");
    }

    char buffer_tq[10];
    dtostrf(highest_torque, 6, 2, buffer_tq);
    // display highest torque achieved in the current test 
    // only display on page 4 (current page)
    ntMAXTORQUE_pg4.setText(buffer_tq); 
  }
}

// State5 -> State0 transition criteria
bool transitionS5S0()
{ 
  if (nbFINISH_pg4_bool)
  {
    nbFINISH_pg4_bool = false;
    return true;
  }
  return false;
}


// ************************************
// ***** STATE6 CALIBRATION STATE *****
// ************************************
void state6()
{
  /* STATE 6 (CALIBRATION STATE)
   *
   * - State 6 Description:
   *   This state is designed to allow the operator to calibrate the load cell
   *   (with a calibrated weight attached to the load cell)
   * 
   * - State 6 Actions:
   *   User can use the +/- buttons to modify the calibration factor until the 
   *    force reported by the load cell matches the calibrated weight attached
   *    to the load cell
   *   User can tare the load cell as well here
   * 
   * - State 6 Transition Criteria:
   *   S6->S0: Operator presses "END CAL" button
   */

  if ( machine.executeOnce )
  {
    _machine_state = CALIBRATE;
    servoMotorEnable(MOTOR_DISABLED);

    highest_torque = 0;
    
    nbTARE_pg5_bool = false;
    nbPLUS1000_pg5_bool = false;
    nbPLUS100_pg5_bool = false;
    nbPLUS10_pg5_bool = false;
    nbMINUS1000_pg5_bool = false;
    nbMINUS100_pg5_bool = false;
    nbMINUS10_pg5_bool = false;
    nbENDCAL_pg5_bool = false;
    nbSAVESETTINGS_pg5_bool = false;

    npCALIBRATE_PAGE.show();
  }

  if ( nbSAVESETTINGS_pg5_bool )
  {
    WriteEEPROM(true);
    nbSAVESETTINGS_pg5_bool = false;
  }

  if ( nbTARE_pg5_bool )
  {
    loadcellTare();
    nbTARE_pg5_bool = false;
  }

  if ( nbPLUS1000_pg5_bool )
  {
    nbPLUS1000_pg5_bool = false;
    scale_calibration_factor += 1000;
    loadcellSetCalibrationFactor(scale_calibration_factor);
  }

  if ( nbPLUS100_pg5_bool )
  {
    nbPLUS100_pg5_bool = false;
    scale_calibration_factor += 100;
    loadcellSetCalibrationFactor(scale_calibration_factor);
  }

  if ( nbPLUS10_pg5_bool )
  {
    nbPLUS10_pg5_bool = false;
    scale_calibration_factor += 10;
    loadcellSetCalibrationFactor(scale_calibration_factor);
  }

  if ( nbMINUS1000_pg5_bool )
  {
    nbMINUS1000_pg5_bool = false;
    scale_calibration_factor -= 1000;
    loadcellSetCalibrationFactor(scale_calibration_factor);
  }

  if ( nbMINUS100_pg5_bool )
  {
    nbMINUS100_pg5_bool = false;
    scale_calibration_factor -= 100;
    loadcellSetCalibrationFactor(scale_calibration_factor);
  }

  if ( nbMINUS10_pg5_bool )
  {
    nbMINUS10_pg5_bool = false;
    scale_calibration_factor -= 10;
    loadcellSetCalibrationFactor(scale_calibration_factor);
  }
}

// State6 -> State0 transition criteria
bool transitionS6S0()
{ 
  if (nbENDCAL_pg5_bool)
  {
    nbENDCAL_pg5_bool = false;
    return true;
  }
  return false;
}

State* S0 = machine.addState(&state0);
State* S1 = machine.addState(&state1);
State* S2 = machine.addState(&state2);
//State* S3 = machine.addState(&state3);
State* S4 = machine.addState(&state4);
State* S5 = machine.addState(&state5);
State* S6 = machine.addState(&state6);

/* ===== END STATE MACHINE DEFINITIONS ===== */









/* Function Name: void checkSerial()
 * 
 * Returns: None
 *
 * Description:
 *  - inspects incoming serial data for "packetized" data. A proper 
 *    serial data packet starts with an '<' and ends with an '>'
 */

/*
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

*/


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

void estopPressedInt()
{
  servoMotorEnable(MOTOR_DISABLED);
  digitalWrite(CLAMP_PIN, false);
  // unconditionally return back to State0 (IDLE)
  machine.transitionTo(S0);
}


void updateEnvironment()
{
  float tq = loadcellReadCurrentValue();
  float air = readAirPressure();

  Serial.print("{,");
  Serial.print(":T="); Serial.print(tq);
  Serial.print(":A="); Serial.print(air);
  Serial.print(":MTR="); Serial.print(motor_moving);
  Serial.print(":SM="); Serial.print(_machine_state);
  Serial.println(",}");
}



void updateDisplay()
{
  char buffer_tq[10]; 
  char high_tq[10];
  float tq = loadcellReadCurrentValue();
  
  // decimal to string float (Arduino.h built-in)
  dtostrf(tq, 6, 2, buffer_tq);
  if ( _machine_state == CALIBRATE ) { ntCURRENTREADING_pg5.setText(buffer_tq); }

  // track the highest torque value achieved since reset
  if ( tq > highest_torque )
  {
    highest_torque = tq;
    dtostrf(highest_torque, 6, 2, high_tq);
  }

  if ( _machine_state == RUNNING )  { ntMAXTORQUE_pg3.setText(high_tq); }
  if ( _machine_state == COMPLETE ) { ntMAXTORQUE_pg4.setText(high_tq); }

  if ( _machine_state == IDLE )      
  { 
    ntCURRENTTORQUE_pg0.setText(buffer_tq);
    ntMAXTORQUE_pg0.setText(buffer_tq);
  }
}




void setup() 
{
  Serial.begin(9600);  // for debug output
  delay(250); // allow serial to settle
  Serial2.begin(115200); // Serial2 is for Nextion communication
  delay(500); // allow serial to settle

  pinMode(pinOPTICALHOME, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinOPTICALHOME), 
                  homeOpticalStopInt,
                  RISING);

  pinMode(pinOPTICALEMERGENCY, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinOPTICALEMERGENCY), 
                  emergencyOpticalStopInt,
                  RISING);

  pinMode(pinESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinESTOP),
                  estopPressedInt,
                  RISING);

  //pinMode(DEBUG_PIN, INPUT_PULLUP);

  servoMotorSetup();
  setupAirValve();
  // need to do setupEEPROM() before loadcellSetup()
  // because loadcellSetup() retrieves data from EEPROM
  // like scale_zero_bias and scale_calibration_factor
  setupEEPROM(); 
  loadcellSetup();
  setupNextion();

  // initialize heartbeat and param update to touchscreen intervals
  //StatusSlice.Interval(_heartbeat_interval);
  ParamSlice.Interval(_param_update_interval);

  S0->addTransition(&transitionS0S1, S1); // IDLE to HOME MOTOR
  S0->addTransition(&transitionS0S6, S6); // IDLE to CALIBRATE LOAD CELL
  S1->addTransition(&transitionS1S0, S0); // HOME MOTOR to IDLE
  S1->addTransition(&transitionS1S2, S2); // HOME MOTOR to LOAD CLUB
  S2->addTransition(&transitionS2S0, S0); // LOAD CLUB to IDLE
  //S2->addTransition(&transitionS2S3, S3); // LOAD CLUB to START TEST !!OBSOLETE!!
  S2->addTransition(&transitionS2S4, S4); // LOAD CLUB to TEST RUNNING
  //S3->addTransition(&transitionS3S0, S0); // START TEST to IDLE !!OBSOLETE!!
  //S3->addTransition(&transitionS3S4, S4); // START TEST to TEST RUNNING !!OBSOLETE!!
  S4->addTransition(&transitionS4S0, S0); // TEST RUNNING to IDLE
  S4->addTransition(&transitionS4S5, S5); // TEST RUNNING to TEST FINISHED
  S5->addTransition(&transitionS5S0, S0); // TEST FINISHED to IDLE
  S6->addTransition(&transitionS6S0, S0); // CALIBRATION to IDLE
}




void loop() 
{
  // read debug pin 42 state
  //DEBUG = !(_SFR_MEM8(0x109) & B10000000);

  //checkSerial();
  
  /* TODO: UNCOMMENT WHEN READY TO SHOW HEARTBEAT AGAIN */
  /*
  hb_timer = millis();
  if (StatusSlice.Triggered(hb_timer)) { updateEnvironment(); } // output heartbeat to serial 
  */
  
  nexLoop(nex_listen_list);
  
  param_timer = millis();
  if (ParamSlice.Triggered(param_timer)) { updateDisplay(); }
  
  machine.run();
}