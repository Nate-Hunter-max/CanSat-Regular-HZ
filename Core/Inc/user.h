/**
 * @file user.h
 * @brief User configuration and utility functions for CanSat-Regular-HZ
 * @author Nate Hunter
 * @date 2025-06-26
 * @version 1.1.0
 */

#ifndef INC_USER_H_
#define INC_USER_H_

#include "main.h"
#include "MS5611.h"
#include "lis3mdl.h"
#include "lsm6ds3.h"
#include "LoRa.h"
#include "MicroSD.h"
#include "W25Qx.h"
#include "CircularBuffer.h"
#include "GNGGA_Parser.h"
#include <math.h>
#include "telemetry_lora.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern SD_HandleTypeDef hsd;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart6;

/**
 * @defgroup user_settings User Configuration Settings
 * @{
 */

/**
 * @defgroup safe_settings Safe Configuration Parameters
 * @brief Parameters that can be safely modified without system risks
 * @{
 */
#define TEAM_NAME "SHARAGA_FOREVER!"     ///< Team identifier for logging and comms
#define SD_FILENAME "gg.wp"              ///< Primary microSD filename for waypoint data (BIN format)
#define SD_FILENAME_WQ "gg.wq"           ///< microSD dump filename for diagnostics (BIN format)
#define DATA_PERIOD 50                   ///< Main data update period in milliseconds
#define DATA_PERIOD_LND 85               ///< Post-landing data update period in milliseconds
#define PRESS_BUFFER_LEN 64              ///< Circular buffer size for landing detection
#define PRESS_LAND_DELTA 10              ///< Max pressure variation for landing detection (Pa)
/** @} */

/**
 * @defgroup critical_settings Critical Configuration Parameters
 * @brief Parameters affecting system safety and operation
 * @warning Modify these values only with thorough testing
 * @{
 */
#define START_TH 150                      ///< Altitude threshold for launch detection (cm)
#define EJECT_TH 240                     ///< ADC threshold for ejection trigger (8-bit value)
/** @} */

/** @} */ // End of user_settings group

/**
 * @defgroup bit_operations Bit Manipulation Macros
 * @brief Helper macros for bit-level operations
 * @{
 */

/**
 * @brief Reads a specific bit from a value
 * @param value Source value to read from
 * @param bit Bit position to read (0-based)
 */
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

/**
 * @brief Sets a specific bit in a value
 * @param value Target value to modify
 * @param bit Bit position to set (0-based)
 */
#define bitSet(value, bit) ((value) |= (1 << (bit)))

/**
 * @brief Resets (clears) a specific bit in a value
 * @param value Target value to modify
 * @param bit Bit position to reset (0-based)
 */
#define bitReset(value, bit) ((value) &= ~(1 << (bit)))

/** @} */ // End of bit_operations group

/* Function prototypes */

/**
 * @brief Handle system errors by entering fail-safe mode
 * @param errCode Error code identifying the failure condition
 * @note This function typically doesn't return as it enters infinite loop
 */
void Error(uint8_t errCode);

/**
 * @brief Control LED flashing pattern
 * @param led LED control pattern (bitmask or duration encoded)
 * @details The exact interpretation depends on hardware implementation
 */
void FlashLED(uint16_t led);

/**
 * @brief Store absolute IMU vector data
 * @param dat Pointer to IMU data structure
 * @warning This function may perform memory-intensive operations
 */
void StoreVectAbs(TelemetryRaw *dat);

/**
 * @brief Save complete IMU dataset to persistent storage
 * @param rawData Pointer to IMU data structure
 * @param tx Pointer to tx frame structure
 * @param lora Pointer to LoRa device
 * @param wq Pointer to W25Qx flash
 */
void ImuSaveAll(TelemetryRaw *rawData, TelemetryPacket *tx, LoRa_Handle_t* lora, W25Qx_Device* wq);

/**
 * @brief Retrieve complete IMU dataset from storage
 * @param rawData Pointer to IMU data structure for population
 * @return void Data is returned via the rawData parameter
 */
void ImuGetAll(TelemetryRaw *rawData);

/**
 * @brief Compare two uint32_t values and return absolute difference
 * @param a Pointer to first value
 * @param b Pointer to second value
 * @return uint32_t Absolute difference between the values
 * @note Used primarily for circular buffer difference calculations
 */
uint32_t compare_uint32(const void *a, const void *b);

/**
 * @brief BN220 Initialization & NMEA setttings
 */
uint8_t BN220_Init(void);

/**
 * @brief Converts GPS coordinates from degrees-minutes format to decimal degrees
 *
 * This function processes GPS data from a GNGGA parser, checks if the data is valid,
 * and converts the latitude and longitude from degrees-minutes format to decimal degrees.
 * The converted values are stored in the provided IMU data structure.
 *
 * @param gps_parser Pointer to the GNGGA_Parser structure containing raw GPS data
 * @param rawData Pointer to the IMU data structure where converted coordinates will be stored
 */
void BN220_TryGet(GNGGA_Parser* gps_parser, TelemetryRaw* rawData);

#endif /* INC_USER_H_ */
