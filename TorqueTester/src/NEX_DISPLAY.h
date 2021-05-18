/*
 *      Author:          Logan Scholz [logans@ping.com]
 *      Creation Date:   MAY-6-2021
 *      REV:             NC
 * 
 *      File Name:       NEX_DISPLAY.h
 * 
 * Nextion touch display NX8048K070_011C
 * 
 * Object declarations and function prototypes
 * 
 */ 

#ifndef NEX_DISPLAY_H
#define NEX_DISPLAY_H

/* === START NEXTION TOUCH DISPLAY CALLBACK FUNCTION PROTOTYPES === */
void setupNextion();
void nbHOMECallback(void *ptr);
void nbTARECallback(void *ptr);
void nbJOGCWCallback(void *ptr);
void nbJOGCCWCallback(void *ptr);
void nbSTARTCallback(void *ptr);
void nbSTOPCallback(void *ptr);
void nbOPENCallback(void *ptr);
void nbCLOSECallback(void *ptr);
void nbCALIBCallback(void *ptr);
/* ==== END NEXTION TOUCH DISPLAY CALLBACK FUNCTION PROTOTYPES ==== */

#endif