/**
 * @file gngga_parser.c
 * @brief Implementation of NMEA GNGGA sentence parser for GNSS/GPS data processing
 *
 * @details This file contains the state machine implementation for parsing GNGGA NMEA sentences,
 *          which provide essential GNSS position and quality information.
 */

#include "gngga_parser.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/**
 * @brief Reset the temporary buffer used for storing field values during parsing
 *
 * @param p Pointer to the parser instance
 */
static void reset_temp(GNGGA_Parser *p) {
	memset(p->temp, 0, GNGGA_TEMP_LENGTH);
	p->index = 0;
}

/**
 * @brief Handle IDLE state - waiting for start of GNGGA sentence
 *
 * @param p Pointer to the parser instance
 * @param c Current character being processed
 *
 * @note Uses a simple pattern matching algorithm to detect "$GNGGA," prefix
 */
static void idle_state(GNGGA_Parser *p, uint8_t c) {
	static uint8_t matchIndex = 0;
	const char target[] = "$GNGGA,";

	if (c == target[matchIndex]) {
		matchIndex++;
		if (matchIndex == sizeof(target) - 1) {
			matchIndex = 0;
			p->state = STATE_PARSE_TIME;
		}
	} else {
		matchIndex = 0;
	}

	if (p->state != p->lastState) {
		p->lastState = p->state;
		p->data.finish = false;
		reset_temp(p);
	}
}

/**
 * @brief Parse time field from GNGGA sentence
 *
 * @param p Pointer to the parser instance
 * @param c Current character being processed
 */
static void parse_time_state(GNGGA_Parser *p, uint8_t c) {
	if (c != ',') {
		if (p->index < GNGGA_TEMP_LENGTH)
			p->temp[p->index++] = c;
	} else {
		p->data.time = atof(p->temp);
		reset_temp(p);
		p->state = STATE_PARSE_LATLON;
	}
}

/**
 * @brief Parse latitude and longitude fields from GNGGA sentence
 *
 * @param p Pointer to the parser instance
 * @param c Current character being processed
 *
 * @note Handles both numeric conversion and hemisphere sign adjustment
 */
static void parse_latlon_state(GNGGA_Parser *p, uint8_t c) {
	p->sepCounter += (c == ',');

	if (p->lastSepCounter == 0 && p->sepCounter == 0) {
		if (p->index < GNGGA_TEMP_LENGTH)
			p->temp[p->index++] = c;
	} else if (p->lastSepCounter == 0 && p->sepCounter == 1) {
		p->lastSepCounter = 1;
		p->data.latitude = atof(p->temp);
		reset_temp(p);
	} else if (p->lastSepCounter == 1 && p->sepCounter == 1) {
		if (c == 'S')
			p->data.latitude = -p->data.latitude;
	} else if (p->lastSepCounter == 1 && p->sepCounter == 2) {
		p->lastSepCounter = 2;
	} else if (p->lastSepCounter == 2 && p->sepCounter == 2) {
		if (p->index < GNGGA_TEMP_LENGTH)
			p->temp[p->index++] = c;
	} else if (p->lastSepCounter == 2 && p->sepCounter == 3) {
		p->lastSepCounter = 3;
		p->data.longitude = atof(p->temp);
		reset_temp(p);
	} else if (p->lastSepCounter == 3 && p->sepCounter == 3) {
		if (c == 'W')
			p->data.longitude = -p->data.longitude;
	} else {
		p->state = STATE_PARSE_INTS;
		p->lastSepCounter = 0;
		p->sepCounter = 0;
		reset_temp(p);
	}
}

/**
 * @brief Parse integer fields (quality and number of satellites) from GNGGA sentence
 *
 * @param p Pointer to the parser instance
 * @param c Current character being processed
 */
static void parse_ints_state(GNGGA_Parser *p, uint8_t c) {
	p->sepCounter += (c == ',');

	if (p->lastSepCounter == 0 && p->sepCounter == 0) {
		if (p->index < GNGGA_TEMP_LENGTH)
			p->temp[p->index++] = c;
	} else if (p->lastSepCounter == 0 && p->sepCounter == 1) {
		p->lastSepCounter = 1;
		p->data.quality = atoi(p->temp);
		reset_temp(p);
	} else if (p->lastSepCounter == 1 && p->sepCounter == 1) {
		if (p->index < GNGGA_TEMP_LENGTH)
			p->temp[p->index++] = c;
	} else {
		p->data.numSats = atoi(p->temp);
		reset_temp(p);
		p->state = STATE_PARSE_FLOATS;
		p->lastSepCounter = 0;
		p->sepCounter = 0;
	}
}

/**
 * @brief Parse floating-point fields (HDOP, altitude, geoidal separation) from GNGGA sentence
 *
 * @param p Pointer to the parser instance
 * @param c Current character being processed
 */
static void parse_floats_state(GNGGA_Parser *p, uint8_t c) {
	p->sepCounter += (c == ',');

	if (p->lastSepCounter == 0 && p->sepCounter == 0) {
		if (p->index < GNGGA_TEMP_LENGTH)
			p->temp[p->index++] = c;
	} else if (p->lastSepCounter == 0 && p->sepCounter == 1) {
		p->lastSepCounter = 1;
		p->data.hdop = atof(p->temp);
		reset_temp(p);
	} else if (p->lastSepCounter == 1 && p->sepCounter == 1) {
		if (p->index < GNGGA_TEMP_LENGTH)
			p->temp[p->index++] = c;
	} else if (p->lastSepCounter == 1 && p->sepCounter == 2) {
		p->lastSepCounter = 2;
		p->data.altitude = atof(p->temp);
		reset_temp(p);
	} else if (p->lastSepCounter == 2 && p->sepCounter == 3) {
		p->lastSepCounter = 3;
	} else if (p->lastSepCounter == 3 && p->sepCounter == 3) {
		if (p->index < GNGGA_TEMP_LENGTH)
			p->temp[p->index++] = c;
	} else if (p->lastSepCounter == 3 && p->sepCounter == 4) {
		p->data.geoidalSep = atof(p->temp);
		reset_temp(p);
	} else if (c == '\n') {
		p->data.finish = true;
		p->state = STATE_IDLE;
		p->sepCounter = 0;
		p->lastSepCounter = 0;
	}
}

/**
 * @brief Initialize the GNGGA parser
 *
 * @param p Pointer to the parser instance
 * @param huart Pointer to UART handle used for receiving NMEA data
 */
void GNGGA_Init(GNGGA_Parser *p, UART_HandleTypeDef *huart) {
	p->huart = huart;
	p->state = STATE_IDLE;
	p->lastState = STATE_IDLE;
	p->index = 0;
	p->sepCounter = 0;
	p->lastSepCounter = 0;
	p->data.finish = false;
	p->rxBuf.size = GNGGA_BUFFER_CAPACITY;
	p->rxBuf.item_size = sizeof(p->rxData);

	CB_Init(&p->rxBuf);
	HAL_UART_Receive_IT(p->huart, &p->rxData, 1);  // Start UART reception in interrupt mode
}

/**
 * @brief Main processing loop for the parser state machine
 *
 * @param p Pointer to the parser instance
 *
 * @note This function should be called periodically to process received data
 */
void GNGGA_Loop(GNGGA_Parser *p) {
	uint8_t c;
	if (!p->newData)
		return;
	while (CB_Pop(&p->rxBuf, &c)) {  // Process all available characters in buffer
		switch (p->state) {
			case STATE_IDLE:
				idle_state(p, c);
				break;
			case STATE_PARSE_TIME:
				parse_time_state(p, c);
				break;
			case STATE_PARSE_LATLON:
				parse_latlon_state(p, c);
				break;
			case STATE_PARSE_INTS:
				parse_ints_state(p, c);
				break;
			case STATE_PARSE_FLOATS:
				parse_floats_state(p, c);
				break;
		}
	}
}

/**
 * @brief UART receive interrupt handler
 *
 * @param p Pointer to the parser instance
 *
 * @note This function should be called from HAL_UART_RxCpltCallback
 */
void GNGGA_UART_IRQHandler(GNGGA_Parser *p) {
	p->newData = 1;
	CB_Push(&p->rxBuf, &p->rxData);  // Store received character in circular buffer
	HAL_UART_Receive_IT(p->huart, &p->rxData, 1);  // Restart UART reception
}
