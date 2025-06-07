/**
 * @file LSM6.h
 * @brief Unified driver for LSM6DS3TR-C over SPI or I2C
 *
 * @author [Nate Hunter]
 * @date [12.05.2025]
 * @version 1.2
 */

#pragma once

#include "main.h"

#ifdef LSM6_USE_SPI
#elif defined(LSM6_USE_I2C)
#else
#error "LSM6: Please define LSM6_USE_SPI or LSM6_USE_I2C"
#endif


/**
 * @defgroup LSM6_Registers Register addresses
 * @brief LSM6DS3TR-C register map
 * @{
 */
#define LSM6_WHO_AM_I    0x0F  /**< Who Am I register */
#define LSM6_OUTX_L_G    0x22  /**< Gyroscope output register start */
#define LSM6_CTRL1_XL    0x10  /**< Accelerometer control register */
#define LSM6_CTRL2_G     0x11  /**< Gyroscope control register */
#define LSM6_CTRL3_C     0x12  /**< Common control register */
/**@}*/

/**
 * @defgroup LSM6_Interface Control bits and interface settings
 * @brief Bit definitions for SPI/I2C read/write
 * @{
 */
#define LSM6_READ_BIT    (1 << 7) /**< SPI read bit */
/**@}*/

/**
 * @defgroup LSM6_FS_Settings Full-scale settings
 * @brief Full-scale settings for accelerometer and gyroscope
 * @{
 */
#define LSM6_ACCEL_2G     (0b00 << 2)
#define LSM6_ACCEL_4G     (0b10 << 2)
#define LSM6_ACCEL_8G     (0b11 << 2)
#define LSM6_ACCEL_16G    (0b01 << 2)

#define LSM6_GYRO_245DPS   (0b00 << 2)
#define LSM6_GYRO_500DPS   (0b01 << 2)
#define LSM6_GYRO_1000DPS  (0b10 << 2)
#define LSM6_GYRO_2000DPS  (0b11 << 2)
/**@}*/

/**
 * @defgroup LSM6_ODR_Settings Output data rate settings
 * @brief Output data rate configuration
 * @{
 */
#define LSM6_CFG_STBY        (0b0000 << 4)
#define LSM6_CFG_1_6_Hz      (0b1011 << 4)
#define LSM6_CFG_12_5_Hz     (0b0001 << 4)
#define LSM6_CFG_26_Hz       (0b0010 << 4)
#define LSM6_CFG_52_Hz       (0b0011 << 4)
#define LSM6_CFG_104_Hz      (0b0100 << 4)
#define LSM6_CFG_208_Hz      (0b0101 << 4)
#define LSM6_CFG_416_Hz      (0b0110 << 4)
#define LSM6_CFG_833_Hz      (0b0111 << 4)
#define LSM6_CFG_1_66_kHz    (0b1000 << 4)
#define LSM6_CFG_3_33_kHz    (0b1001 << 4)
#define LSM6_CFG_6_66_kHz    (0b1010 << 4)
/**@}*/

/**
 * @brief Sensitivity values for accelerometer [mg/LSB] and gyroscope [mdps/LSB]
 */
static const float lsm6SensA[4] = { 0.061f, 0.488f, 0.122f, 0.244f };
static const float lsm6SensG[4] = { 8.75f, 17.5f, 35.0f, 70.0f };

/**
 * @brief Initialize the LSM6DS3TR-C sensor.
 *
 * @param interface SPI_HandleTypeDef* or I2C_HandleTypeDef*, depending on interface
 * @param port_or_addr SPI: GPIO port of NSS. I2C: NULL.
 * @param pin_or_addr SPI: NSS pin. I2C: I2C address (<<1).
 * @return uint8_t 1 on success, 0 on error.
 */
uint8_t LSM6_Init(void *interface, void *port_or_addr, uint16_t pin_or_addr);

/**
 * @brief Read sensor values.
 *
 * @param accel Pointer to 3-element array to receive accelerometer data [g]
 * @param gyro Pointer to 3-element array to receive gyroscope data [dps]
 */
void LSM6_Read(float *accel, float *gyro);

/**
 * @brief Configure full-scale and ODR for accelerometer and gyroscope.
 *
 * @param A_CFG Accelerometer configuration byte (ODR | FS)
 * @param G_CFG Gyroscope configuration byte (ODR | FS)
 */
void LSM6_ConfigAG(uint8_t A_CFG, uint8_t G_CFG);
