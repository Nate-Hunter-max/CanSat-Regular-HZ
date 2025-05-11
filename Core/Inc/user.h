/*
 * user.h
 *
 *  Created on: May 11, 2025
 *      Author: Nate Hunter
 */

#ifndef INC_USER_H_
#define INC_USER_H_

/*___________________________________Settings___________________________________*/
#define TEAM_NAME "SHARAGA_FOREVER!"  //Team name
#define SD_FILENAME "gg.wp"						//microSD file name, BIN content
#define SD_FILENAME_WQ "gg.wq"				//microSD memory dump file name, BIN content
#define DATA_PERIOD 85								//Data update period, ms
#define DATA_PERIOD_LND 85						//Data update period after landing, ms
#define PRESS_BUFFER_LEN 15						//Buffer for landing detection
#define	PRESS_LAND_DELTA 10						//Set land flag if buffered pressure difference < [this val], Pa
/*_____________________________Settings_(Danger_Zone)____________________________*/
#define FRAME_SIZE 56			//frame for LoRa & WQ, bytes
#define START_TH 10      //Set start flag if altitude > [this val], cm
#define EJECT_TH 240			//Set EJC flag if adc_val < [this_val], 8bit
#define RS_ANGLE 30			//RS Servo angle,<=120 deg
#define RS_DELAY 3000			//Time while RS stays active (TEST ONLY), ms

/*___________________________________Bit Macro___________________________________*/
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1 << (bit)))

#endif /* INC_USER_H_ */
