#include <Arduino.h>
#include <Nextion.h>
#include "NEX_DISPLAY.h"
#include "LoadCell.h"
#include "relaypins.h"



/* === START NEXTION TOUCH DISPLAY DEFINITIONS === */
/* Nextion BUTTON Object Declarations */
NexButton nbHOME   = NexButton(0, 4,  "b2");   // rotate the motor either X CCW or until it hits the HOME stop
NexButton nbTARE   = NexButton(0, 8,  "b4");   // tare the load cell
NexButton nbJOGCW  = NexButton(0, 10, "b6");   // JOG the motor X degrees CW
NexButton nbJOGCCW = NexButton(0, 11, "b7");   // JOG the motor X degrees CCW
NexButton nbSTART  = NexButton(0, 5,  "b3");   // start the torque test
NexButton nbSTOP   = NexButton(0, 12, "b8");   // cancel the torque test (emergency)
NexButton nbOPEN   = NexButton(0, 2,  "b0");   // open the shaft-holder clamps
NexButton nbCLOSE  = NexButton(0, 3,  "b1");   // close the shaft holder clamps
NexButton nbCALIB  = NexButton(0, 9,  "b5");   // calibrate the load cell... this will take some thinking

/* Nextion TEXT FIELD Object Declarations */
NexText   ntTORQUE    = NexText(0, 19, "t9");  // current torque value
NexText   ntAIR       = NexText(0, 20, "t10"); // current air pressure value
NexText   ntMAXTORQUE = NexText(0, 13, "t4");  // maximum torque value achieved during a test
NexText   ntRESULT    = NexText(0, 16, "t6");  // PASS/FAIL/ABORT status of test
NexText   ntSTATUS    = NexText(0, 18, "t8");  // essentially the current state of the state machine for debugging
/* ==== END NEXTION TOUCH DISPLAY DEFINITIONS ==== */


/* === START NEXTION NexTouch OBJECT INSTANTIATION === */
NexTouch *nex_listen_list[] = 
{
    &nbHOME,
    &nbTARE,
    &nbJOGCW,
    &nbJOGCCW,
    &nbSTART,
    &nbSTOP,
    &nbOPEN,
    &nbCLOSE,
    &nbCALIB,
    NULL
};
/* ==== END NEXTION NexTouch OBJECT INSTANTIATION ==== */


/* === START main.cpp EXTERNS === */

extern float highest_torque; // reset to 0 when new test starts
/* ==== END main.cpp EXTERNS ==== */


void setupNextion()
{
  nexInit();
  
  nbHOME.attachPop(   nbHOMECallback,   &nbHOME   );
  nbTARE.attachPop(   nbTARECallback,   &nbTARE   );
  nbJOGCW.attachPop(  nbJOGCWCallback,  &nbJOGCW  );
  nbJOGCCW.attachPop( nbJOGCCWCallback, &nbJOGCCW );
  nbSTART.attachPop(  nbSTARTCallback,  &nbSTART  );
  nbSTOP.attachPop(   nbSTOPCallback,   &nbSTOP   );
  nbOPEN.attachPop(   nbOPENCallback,   &nbOPEN   );
  nbCLOSE.attachPop(  nbCLOSECallback,  &nbCLOSE  );
  nbCALIB.attachPop(  nbCALIBCallback,  &nbCALIB  );
}




void nbHOMECallback(void *ptr)
{
    // TODO
    ntSTATUS.setText("inside nbHOMECallback");
    ntTORQUE.setText("maybe"); // example of how to update text on nextion display !REMOVE!
}

void nbTARECallback(void *ptr)
{
    // TODO
    loadcellTare();
    ntSTATUS.setText("Load Cell Tared");
}

void nbJOGCWCallback(void *ptr)
{
    // TODO
    ntSTATUS.setText("inside nbJOGCWCallback");
}

void nbJOGCCWCallback(void *ptr)
{
    // TODO
    ntSTATUS.setText("inside nbJOGCCWCallback");
}

void nbSTARTCallback(void *ptr)
{
    // TODO
    ntSTATUS.setText("inside nbSTARTCallback");
    ntMAXTORQUE.setText("0");
    highest_torque = 0;
}

void nbSTOPCallback(void *ptr)
{
    // TODO
    ntSTATUS.setText("inside nbSTOPCallback");
}

void nbOPENCallback(void *ptr)
{
    // TODO
    digitalWrite(RELAY_2_CTRL, LOW);
    ntSTATUS.setText("CLAMP OPENED");
}

void nbCLOSECallback(void *ptr)
{
    // TODO
    digitalWrite(RELAY_2_CTRL, HIGH);
    ntSTATUS.setText("CLAMP CLOSED");
}

void nbCALIBCallback(void *ptr)
{
    // TODO
    ntSTATUS.setText("inside nbCALIBCallback");
}
