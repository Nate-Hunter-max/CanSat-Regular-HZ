/**
 * @file user.h
 * @brief User configuration and utility functions for CanSat-Regular-HZ
 *
 * @author [Nate Hunter]
 * @date [10.05.2025]
 * @version 1.0
 */

#ifndef INC_USER_H_
#define INC_USER_H_

#include "main.h"
#include "MS5611.h"
#include "LIS3.h"
#include "LSM6.h"
#include "LoRa.h"
#include "MicroSD.h"
#include "W25Qx.h"
#include "CircularBuffer.h"
#include <math.h>

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
#define DATA_PERIOD 85                   ///< Main data update period in milliseconds
#define DATA_PERIOD_LND 85               ///< Post-landing data update period in milliseconds
#define PRESS_BUFFER_LEN 15              ///< Circular buffer size for landing detection
#define PRESS_LAND_DELTA 10              ///< Max pressure variation for landing detection (Pa)
/** @} */

/**
 * @defgroup critical_settings Critical Configuration Parameters
 * @brief Parameters affecting system safety and operation
 * @warning Modify these values only with thorough testing
 * @{
 */
#define FRAME_SIZE 64                    ///< Communication frame size for LoRa/WQ (bytes)
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

/** @} */ // End of bit_operations group

/** @brief IMU data structure */
typedef struct {
	uint32_t time;          ///< Milliseconds from start
	int32_t temp;           ///< MS56 temperature (centigrade*10e2)
	uint32_t press;         ///< MS56 pressure (Pa)
	float magData[3];       ///< LIS3 mag (mG)
	float accelData[3];     ///< LSM6 accel (mG)
	float gyroData[3];      ///< LSM6 gyro (mdps)
	int32_t altitude;       ///< Altitude (zero at start, cm)
	float lat;              ///< Latitude from GPS
	float lon;              ///< Longitude from GPS
	uint32_t flags;         ///< Flags (0|0|0|0|Land|ResSys|Eject|Start)
	//*_____64 bytes - frame threshold_____*/
	uint32_t press0;        ///< MS56 pressure at 0 Alt (Pa)
	float vectAbs;          ///< Absolute value of accel vector
	uint32_t wqAdr;         ///< WQ address
} ImuData;

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
void StoreVectAbs(ImuData *dat);

/**
 * @brief Save complete IMU dataset to persistent storage
 * @param imuData Pointer to IMU data structure
 */
void ImuSaveAll(ImuData *imuData, LoRa_HandleTypeDef* lora, W25Qx_Device* wq);

/**
 * @brief Retrieve complete IMU dataset from storage
 * @param imuData Pointer to IMU data structure for population
 * @return void Data is returned via the imuData parameter
 */
void ImuGetAll(ImuData *imuData);

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

#endif /* INC_USER_H_ */
