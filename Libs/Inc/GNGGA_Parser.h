/**
 * @file GNGGA_Parser.h
 * @brief NMEA GNGGA sentence parser for GNSS/GPS data processing
 *
 * @author [Nate Hunter]
 * @date [15.05.2025]
 * @version 1.0
 */

#ifndef GNGGA_PARSER_H
#define GNGGA_PARSER_H

#include "main.h"
#include "CircularBuffer.h"
#include <stdbool.h>

#define GNGGA_TEMP_LENGTH 12
#define GNGGA_BUFFER_CAPACITY 128

/**
 * @brief Parsed GNGGA data structure.
 */
typedef struct {
    bool finish;
    float time;
    float latitude;
    float longitude;
    int quality;
    int numSats;
    float hdop;
    float altitude;
    float geoidalSep;
} GNGGA_Data;

/**
 * @brief Parser FSM states.
 */
typedef enum {
    STATE_IDLE,
    STATE_PARSE_TIME,
    STATE_PARSE_LATLON,
    STATE_PARSE_INTS,
    STATE_PARSE_FLOATS
} GNGGA_State;

/**
 * @brief GNGGA parser with UART and circular buffer support.
 */
typedef struct {
    UART_HandleTypeDef *huart;
    CircularBuffer rxBuf;
    uint8_t rxData;

    GNGGA_State state;
    GNGGA_State lastState;

    char temp[GNGGA_TEMP_LENGTH];
    uint8_t index;
    uint8_t sepCounter;
    uint8_t lastSepCounter;
    uint8_t newData;

    GNGGA_Data data;
} GNGGA_Parser;

/**
 * @brief Initialize parser and register UART.
 *
 * @param parser Pointer to parser.
 * @param huart Pointer to UART.
 */
void GNGGA_Init(GNGGA_Parser *parser, UART_HandleTypeDef *huart);

/**
 * @brief FSM execution function. Call this periodically.
 *
 * @param parser Pointer to parser.
 */
void GNGGA_Loop(GNGGA_Parser *parser);

/**
 * @brief UART RX interrupt handler — call from HAL_UART_RxCpltCallback.
 *
 * @param parser Pointer to parser.
 */
void GNGGA_UART_IRQHandler(GNGGA_Parser *parser);

#endif // GNGGA_PARSER_H
