/*
 *      Author: Logan Scholz [logans@ping.com]
 *      Date:   APR-8-2021
 *      REV:    NC
 * 
 *      File Name: relaypins.h
 * 
 * Relay Control Pins:
 * Arduino digital pins 4, 5, 6, 7 are used to 
 * control relays 4, 3, 2, and 1, respectively. 
 * 
 * Instructions:
 * set RELAY_X_CTRL pin HIGH to connect the
 * corresponding relay's COM to NO (and
 * disconnect NC from COM) 
 * 
 */

#define RELAY_1_CTRL 7
#define RELAY_2_CTRL 6
#define RELAY_3_CTRL 5
#define RELAY_4_CTRL 4