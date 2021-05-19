#include <Arduino.h>
#include <Nextion.h>
#include "NEX_DISPLAY.h"
#include "LoadCell.h"
#include "AirPressure.h"


// the booleans below are meant to track if buttons have been pressed
// in the callback functions... this is the best way I can think to 
// do this for now
//
// idea: throw these in a 21 element boolean array so we can quickly
//       iterate through the array to reset all elements FALSE(????)
bool nbSTARTTEST_pg0_bool  = false;
bool nbOPENCLAMP_pg0_bool  = false;
bool nbCALIBRATE_pg0_bool  = false;
bool nbTARE_pg0_bool       = false;
bool nbMOVECW_pg1_bool     = false;
bool nbCANCEL_pg1_bool     = false;
bool nbMOVECCW_pg1_bool    = false;
bool nbOPENCLAMP_pg2_bool  = false;
bool nbSTARTTEST_pg2_bool  = false;
bool nbCANCEL_pg2_bool     = false;
bool nbCLOSECLAMP_pg2_bool = false;
bool nbCANCEL_pg3_bool     = false;
bool nbFINISH_pg4_bool     = false;
bool nbTARE_pg5_bool       = false;
bool nbPLUS1000_pg5_bool   = false;
bool nbPLUS100_pg5_bool    = false;
bool nbPLUS10_pg5_bool     = false;
bool nbMINUS1000_pg5_bool  = false;
bool nbMINUS100_pg5_bool   = false;
bool nbMINUS10_pg5_bool    = false;
bool nbENDCAL_pg5_bool     = false;





/* === START NEXTION TOUCH DISPLAY DEFINITIONS === */
/* Nextion BUTTON Object Declarations */
// PAGE 0 BUTTONS
NexButton nbSTARTTEST_pg0 = NexButton( 0, 6,  "b2");
NexButton nbOPENCLAMP_pg0 = NexButton( 0, 2,  "b0");
NexButton nbCALIBRATE_pg0 = NexButton( 0, 5,  "b1");
NexButton nbTARE_pg0      = NexButton( 0, 11, "b3");
// PAGE 1 BUTTONS
NexButton nbMOVECW_pg1  = NexButton( 1, 3, "b0");
NexButton nbCANCEL_pg1  = NexButton( 1, 5, "b2");
NexButton nbMOVECCW_pg1 = NexButton( 1, 4, "b1");
// PAGE 2 BUTTONS
NexButton nbOPENCLAMP_pg2  = NexButton( 2, 1, "b0");
NexButton nbSTARTTEST_pg2  = NexButton( 2, 5, "b3");
NexButton nbCANCEL_pg2     = NexButton( 2, 4, "b2");
NexButton nbCLOSECLAMP_pg2 = NexButton( 2, 3, "b1");
// PAGE 3 BUTTONS
NexButton nbCANCEL_pg3 = NexButton( 3, 1, "b0");
// PAGE 4 BUTTONS
NexButton nbFINISH_pg4 = NexButton( 4, 7, "b0");
// PAGE 5 BUTTONS
NexButton nbTARE_pg5      = NexButton( 5, 9, "b7");
NexButton nbPLUS1000_pg5  = NexButton( 5, 2, "b0");
NexButton nbPLUS100_pg5   = NexButton( 5, 3, "b1");
NexButton nbPLUS10_pg5    = NexButton( 5, 4, "b2");
NexButton nbMINUS1000_pg5 = NexButton( 5, 5, "b3");
NexButton nbMINUS100_pg5  = NexButton( 5, 6, "b4");
NexButton nbMINUS10_pg5   = NexButton( 5, 7, "b5");
NexButton nbENDCAL_pg5    = NexButton( 5, 8, "b6");


/* Nextion TEXT FIELD Object Declarations */
// example: NexText   ntSTATUS    = NexText(0, 18, "t8");  // essentially the current state of the state machine for debugging
// PAGE 0 TEXT FIELDS
NexText ntCURRENTTORQUE_pg0 = NexText( 0, 10, "t6");
NexText ntMAXTORQUE_pg0     = NexText( 0, 4,  "t2");
NexText ntSTATUS_pg0        = NexText( 0, 8,  "t4");

// PAGE 3 TEXT FIELDS
NexText ntMAXTORQUE_pg3 = NexText( 3, 3, "t1");

// PAGE 4 TEXT FIELDS
NexText ntMAXTORQUE_pg4  = NexText( 4, 3, "t2");
NexText ntTESTRESULT_pg4 = NexText( 4, 6, "t5");

// PAGE 5 TEXT FIELDS
NexText ntCURRENTREADING_pg5 = NexText( 5, 11, "t2");   

/* ==== END NEXTION TOUCH DISPLAY DEFINITIONS ==== */


/* === START NEXTION NexTouch OBJECT INSTANTIATION === */
NexTouch *nex_listen_list[] = 
{
    // put all the buttons here
    &nbSTARTTEST_pg0,
    &nbOPENCLAMP_pg0,
    &nbCALIBRATE_pg0,
    &nbTARE_pg0,
    &nbMOVECW_pg1,
    &nbCANCEL_pg1,
    &nbMOVECCW_pg1,
    &nbOPENCLAMP_pg2,
    &nbSTARTTEST_pg2,
    &nbCANCEL_pg2,
    &nbCLOSECLAMP_pg2,
    &nbCANCEL_pg3,
    &nbFINISH_pg4,
    &nbTARE_pg5,
    &nbPLUS1000_pg5,
    &nbPLUS100_pg5,
    &nbPLUS10_pg5,
    &nbMINUS1000_pg5,
    &nbMINUS100_pg5,
    &nbMINUS10_pg5,
    &nbENDCAL_pg5,
    NULL
};
/* ==== END NEXTION NexTouch OBJECT INSTANTIATION ==== */


/* === START main.cpp EXTERNS === */
extern float highest_torque; // reset to 0 when new test starts
/* ==== END main.cpp EXTERNS ==== */


void setupNextion()
{
    nexInit();

    nbSTARTTEST_pg0.attachPop(  nbSTARTTEST_pg0Callback,  &nbSTARTTEST_pg0);
    nbOPENCLAMP_pg0.attachPop(  nbOPENCLAMP_pg0Callback,  &nbOPENCLAMP_pg0);
    nbCALIBRATE_pg0.attachPop(  nbCALIBRATE_pg0Callback,  &nbCALIBRATE_pg0);
    nbTARE_pg0.attachPop(       nbTARE_pg0Callback,       &nbTARE_pg0);
    nbMOVECW_pg1.attachPop(     nbMOVECW_pg1Callback,     &nbMOVECW_pg1);
    nbCANCEL_pg1.attachPop(     nbCANCEL_pg1Callback,     &nbCANCEL_pg1);
    nbMOVECCW_pg1.attachPop(    nbMOVECCW_pg1Callback,    &nbMOVECCW_pg1);
    nbOPENCLAMP_pg2.attachPop(  nbOPENCLAMP_pg2Callback,  &nbOPENCLAMP_pg2);
    nbSTARTTEST_pg2.attachPop(  nbSTARTTEST_pg2Callback,  &nbSTARTTEST_pg2);
    nbCANCEL_pg2.attachPop(     nbCANCEL_pg2Callback,     &nbCANCEL_pg2);
    nbCLOSECLAMP_pg2.attachPop( nbCLOSECLAMP_pg2Callback, &nbCLOSECLAMP_pg2);
    nbCANCEL_pg3.attachPop(     nbCANCEL_pg3Callback,     &nbCANCEL_pg3);
    nbFINISH_pg4.attachPop(     nbFINISH_pg4Callback,     &nbFINISH_pg4);
    nbTARE_pg5.attachPop(       nbTARE_pg5Callback,       &nbTARE_pg5);
    nbPLUS1000_pg5.attachPop(   nbPLUS1000_pg5Callback,   &nbPLUS1000_pg5);
    nbPLUS100_pg5.attachPop(    nbPLUS100_pg5Callback,    &nbPLUS100_pg5);
    nbPLUS10_pg5.attachPop(     nbPLUS10_pg5Callback,     &nbPLUS10_pg5);
    nbMINUS1000_pg5.attachPop(  nbMINUS1000_pg5Callback,  &nbMINUS1000_pg5);
    nbMINUS100_pg5.attachPop(   nbMINUS100_pg5Callback,   &nbMINUS100_pg5);
    nbMINUS10_pg5.attachPop(    nbMINUS10_pg5Callback,    &nbMINUS10_pg5);
    nbENDCAL_pg5.attachPop(     nbENDCAL_pg5Callback,     &nbENDCAL_pg5);
}



// TODO: UPDATE THIS FOR OPENING, CLOSING CLAMPS?
void AirValve(boolean open)
{
    // [open == true] will DISCONNECT NO2 from COM2
    // [open == false] will CONNECT NO2 to COM2
    digitalWrite(CLAMP_PIN, (open) ? 0:1);
}



// ==== STARTING NEXTION BUTTON CALLBACK FUNCTIONS ====
void nbSTARTTEST_pg0Callback(void *ptr)  { nbSTARTTEST_pg0_bool  = true; }
void nbOPENCLAMP_pg0Callback(void *ptr)  { nbOPENCLAMP_pg0_bool  = true; }
void nbCALIBRATE_pg0Callback(void *ptr)  { nbCALIBRATE_pg0_bool  = true; }
void nbTARE_pg0Callback(void *ptr)       { nbTARE_pg0_bool       = true; }
void nbMOVECW_pg1Callback(void *ptr)     { nbMOVECW_pg1_bool     = true; }
void nbCANCEL_pg1Callback(void *ptr)     { nbCANCEL_pg1_bool     = true; }
void nbMOVECCW_pg1Callback(void *ptr)    { nbMOVECCW_pg1_bool    = true; }
void nbOPENCLAMP_pg2Callback(void *ptr)  { nbOPENCLAMP_pg2_bool  = true; }
void nbSTARTTEST_pg2Callback(void *ptr)  { nbSTARTTEST_pg2_bool  = true; }
void nbCANCEL_pg2Callback(void *ptr)     { nbCANCEL_pg2_bool     = true; }
void nbCLOSECLAMP_pg2Callback(void *ptr) { nbCLOSECLAMP_pg2_bool = true; }
void nbCANCEL_pg3Callback(void *ptr)     { nbCANCEL_pg3_bool     = true; }
void nbFINISH_pg4Callback(void *ptr)     { nbFINISH_pg4_bool     = true; }
void nbTARE_pg5Callback(void *ptr)       { nbTARE_pg5_bool       = true; }
void nbPLUS1000_pg5Callback(void *ptr)   { nbPLUS1000_pg5_bool   = true; }
void nbPLUS100_pg5Callback(void *ptr)    { nbPLUS100_pg5_bool    = true; }
void nbPLUS10_pg5Callback(void *ptr)     { nbPLUS10_pg5_bool     = true; }
void nbMINUS1000_pg5Callback(void *ptr)  { nbMINUS1000_pg5_bool  = true; }   
void nbMINUS100_pg5Callback(void *ptr)   { nbMINUS100_pg5_bool   = true; }
void nbMINUS10_pg5Callback(void *ptr)    { nbMINUS10_pg5_bool    = true; }
void nbENDCAL_pg5Callback(void *ptr)     { nbENDCAL_pg5_bool     = true; }


