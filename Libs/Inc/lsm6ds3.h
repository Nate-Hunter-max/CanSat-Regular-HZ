/**
 * @file lsm6ds3.h
 * @brief LSM6DS3TR-C 3D accelerometer and gyroscope driver for STM32 (I2C/SPI, scaled data output only)
 * @author Nate Hunter
 * @date 2025-06-25
 * @version v2.0.0
 */

#ifndef LSM6DS3_H
#define LSM6DS3_H

#include "main.h"

// Interface selection
#define LSM6_USE_SPI
//#define LSM6_USE_I2C

/// WHO_AM_I expected value
#define LSM6DS3TR_WHO_AM_I       0x69
#define LSM6DS3_WHO_AM_I       0x6A

/// Register map
#define LSM6DS3_REG_WHO_AM_I   0x0F
#define LSM6DS3_REG_CTRL1_XL   0x10
#define LSM6DS3_REG_CTRL2_G    0x11
#define LSM6DS3_REG_CTRL3_C    0x12
#define LSM6DS3_REG_OUTX_L_G   0x22
#define LSM6DS3_REG_OUTX_L_XL  0x28

/// Accelerometer full-scale values (g)
typedef enum {
	LSM6DS3_XL_2G = 0, LSM6DS3_XL_4G = 2, LSM6DS3_XL_8G = 3, LSM6DS3_XL_16G = 1
} LSM6DS3_AccelFS;

/// Gyroscope full-scale values (dps)
typedef enum {
	LSM6DS3_GYRO_125DPS = 1, LSM6DS3_GYRO_250DPS = 0, LSM6DS3_GYRO_500DPS = 2, LSM6DS3_GYRO_1000DPS = 4, LSM6DS3_GYRO_2000DPS = 6
} LSM6DS3_GyroFS;

/// Output data rate (ODR) settings for XL and G
typedef enum {
	LSM6DS3_ODR_OFF = 0x00,
	LSM6DS3_ODR_12HZ5 = 0x01,
	LSM6DS3_ODR_26HZ = 0x02,
	LSM6DS3_ODR_52HZ = 0x03,
	LSM6DS3_ODR_104HZ = 0x04,
	LSM6DS3_ODR_208HZ = 0x05,
	LSM6DS3_ODR_416HZ = 0x06,
	LSM6DS3_ODR_833HZ = 0x07,
	LSM6DS3_ODR_1660HZ = 0x08,
	LSM6DS3_ODR_3330HZ = 0x09,
	LSM6DS3_ODR_6660HZ = 0x0A
} LSM6DS3_ODR;

/// Device context
typedef struct {
#ifdef LSM6_USE_SPI
	SPI_HandleTypeDef *spi;     ///< SPI handle
	GPIO_TypeDef *cs_port;      ///< Chip select port
	uint16_t cs_pin;            ///< Chip select pin
#else
    I2C_HandleTypeDef *i2c;     ///< I2C handle
    uint8_t i2c_addr;           ///< I2C device address (7-bit)
#endif
	uint32_t timeout;           ///< Timeout for HAL communication

	float accelScale;           ///< Scale factor for accelerometer (g/LSB)
	float gyroScale;            ///< Scale factor for gyroscope (dps/LSB)
	LSM6DS3_ODR accelODR;       ///< Output data rate for accelerometer
	LSM6DS3_ODR gyroODR;        ///< Output data rate for gyroscope
} LSM6DS3_Handle;

/**
 * @brief Initialize LSM6DS3 device
 * @param dev Pointer to the device structure
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_Init(LSM6DS3_Handle *dev, LSM6DS3_AccelFS accelFS, LSM6DS3_GyroFS gyroFS);

/**
 * @brief Read acceleration and gyroscope data from LSM6DS3TR-C.
 *
 * @param dev Pointer to the device structure
 * @param accel Pointer to 3-element array for acceleration (g)
 * @param gyro Pointer to 3-element array for gyro (dps)
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_ReadData(LSM6DS3_Handle *dev, float accel[3], float gyro[3]);

#endif // LSM6DS3_H
