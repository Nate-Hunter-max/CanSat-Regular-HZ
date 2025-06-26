/**
 * @file lis3mdl.h
 * @brief Library for LIS3MDL 3-axis magnetometer (I2C/SPI interface)
 * @author Nate Hunter
 * @date 2025-06-26
 * @version v2.0.0
 */

#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "main.h"

/**

 * @brief Select communication interface (Only one should be enabled)
 */
#define LIS3_USE_SPI
//#define LIS3_USE_I2C

/**
 * @brief LIS3MDL register map definitions
 */
#define LIS3MDL_WHO_AM_I         0x0F /**< Device ID register */
#define LIS3MDL_CTRL_REG1        0x20 /**< Control register 1 */
#define LIS3MDL_CTRL_REG2        0x21 /**< Control register 2 */
#define LIS3MDL_CTRL_REG3        0x22 /**< Control register 3 */
#define LIS3MDL_CTRL_REG4        0x23 /**< Control register 4 */
#define LIS3MDL_CTRL_REG5        0x24 /**< Control register 5 */
#define LIS3MDL_OUT_X_L          0x28 /**< X-axis low byte */
#define LIS3MDL_OUT_X_H          0x29 /**< X-axis high byte */
#define LIS3MDL_OUT_Y_L          0x2A /**< Y-axis low byte */
#define LIS3MDL_OUT_Y_H          0x2B /**< Y-axis high byte */
#define LIS3MDL_OUT_Z_L          0x2C /**< Z-axis low byte */
#define LIS3MDL_OUT_Z_H          0x2D /**< Z-axis high byte */
#define LIS3MDL_TEMP_OUT_L       0x2E /**< Temperature low byte */
#define LIS3MDL_TEMP_OUT_H       0x2F /**< Temperature high byte */
#define LIS3MDL_OFFSET_X_L       0x05 /**< X-axis offset low byte */
#define LIS3MDL_OFFSET_X_H       0x06 /**< X-axis offset high byte */
#define LIS3MDL_OFFSET_Y_L       0x07 /**< Y-axis offset low byte */
#define LIS3MDL_OFFSET_Y_H       0x08 /**< Y-axis offset high byte */
#define LIS3MDL_OFFSET_Z_L       0x09 /**< Z-axis offset low byte */
#define LIS3MDL_OFFSET_Z_H       0x0A /**< Z-axis offset high byte */

/**
 * @brief Full-scale selection enum
 */
typedef enum {
	LIS3_SCALE_4_GAUSS = 0, /**< ±4 gauss range */
	LIS3_SCALE_8_GAUSS, /**< ±8 gauss range */
	LIS3_SCALE_12_GAUSS, /**< ±12 gauss range */
	LIS3_SCALE_16_GAUSS /**< ±16 gauss range */
} LIS3_FullScale;

/**
 * @brief Output data rate selection enum
 */
typedef enum {
	LIS3_ODR_0_625_HZ = 0, /**< 0.625 Hz */
	LIS3_ODR_1_25_HZ, /**< 1.25 Hz */
	LIS3_ODR_2_5_HZ, /**< 2.5 Hz */
	LIS3_ODR_5_HZ, /**< 5 Hz */
	LIS3_ODR_10_HZ, /**< 10 Hz */
	LIS3_ODR_20_HZ, /**< 20 Hz */
	LIS3_ODR_40_HZ, /**< 40 Hz */
	LIS3_ODR_80_HZ /**< 80 Hz */
} LIS3_ODR;

/**
 * @brief LIS3MDL device instance configuration structure
 */
typedef struct {
#ifdef LIS3_USE_SPI
	SPI_HandleTypeDef *spi; /**< SPI handle */
	GPIO_TypeDef *cs_port; /**< GPIO port for CS */
	uint16_t cs_pin; /**< GPIO pin for CS */
#elif defined(LIS3_USE_I2C)
	I2C_HandleTypeDef *i2c;     /**< I2C handle */
	uint8_t address;            /**< I2C address */
#endif
	uint32_t timeout; /**< Communication timeout (ms) */
	LIS3_ODR output_data_rate; /**< Output data rate */
	LIS3_FullScale scale; /**< Full scale range */
} LIS3MDL_Device;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the LIS3MDL sensor
 * @param dev Pointer to the device structure
 * @return 1 on success, 0 on failure
 */
uint8_t LIS3MDL_Init(LIS3MDL_Device *dev);

/**
 * @brief Apply configured settings to sensor registers
 * @param dev Pointer to the device structure
 * @return 1 on success, 0 on failure
 */
uint8_t LIS3MDL_ApplySettings(LIS3MDL_Device *dev);

/**
 * @brief Read scaled magnetic and optional temperature data
 * @param dev Pointer to the device structure
 * @param axes Pointer to float[3] array for X, Y, Z in gauss
 * @param temperature Optional pointer to int16_t for temperature
 * @return 1 on success, 0 on failure
 */
uint8_t LIS3MDL_ReadData(LIS3MDL_Device *dev, float axes[3], int16_t *temperature);

/**
 * @brief Apply hard-iron offset calibration values
 * @param dev Pointer to the device structure
 * @param x_offset X-axis offset (LSB)
 * @param y_offset Y-axis offset (LSB)
 * @param z_offset Z-axis offset (LSB)
 * @return 1 on success, 0 on failure
 */
uint8_t LIS3MDL_ApplyHardIronCalibration(LIS3MDL_Device *dev, int16_t x_offset, int16_t y_offset, int16_t z_offset);

#ifdef __cplusplus
}
#endif

#endif // LIS3MDL_H
