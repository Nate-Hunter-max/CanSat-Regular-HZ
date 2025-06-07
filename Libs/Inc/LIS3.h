/**
 * @file LIS3.h
 * @brief LIS3MDL magnetometer driver for STM32 using SPI or I2C.
 *
 * @author [Nate Hunter]
 * @date [13.05.2025]
 * @version 1.1
 */

#pragma once
#include "main.h"

/// @defgroup LIS3_Registers LIS3MDL Register Map
/// @{
#define LIS_READ_BIT     (1 << 7)
#define LIS_MULTY_BIT    (1 << 6)
#define LIS_WHO_AM_I     0x0F
#define LIS_CTRL_REG1    0x20
#define LIS_CTRL_REG2    0x21
#define LIS_CTRL_REG3    0x22
#define LIS_CTRL_REG4    0x23
#define LIS_CTRL_REG5    0x24
#define LIS_STATUS       0x27
#define LIS_OUT_X_L      0x28
#define LIS_OFF_X_L      0x05
/// @}

/// @defgroup LIS3_BitMasks LIS3MDL Control Bit Definitions
/// @{
#define LIS_TEMP_EN      (1 << 7)
#define LIS_MODE_LP      (0b00 << 5)
#define LIS_MODE_MP      (0b01 << 5)
#define LIS_MODE_HP      (0b10 << 5)
#define LIS_MODE_UHP     (0b11 << 5)

#define LIS_ODR_06       (0b000 << 2)
#define LIS_ODR_1        (0b001 << 2)
#define LIS_ODR_2        (0b010 << 2)
#define LIS_ODR_5        (0b011 << 2)
#define LIS_ODR_10       (0b100 << 2)
#define LIS_ODR_20       (0b101 << 2)
#define LIS_ODR_40       (0b110 << 2)
#define LIS_ODR_80       (0b111 << 2)
#define LIS_FAST_ODR     (1 << 1)

#define LIS_SCALE_4      (0b00 << 5)
#define LIS_SCALE_8      (0b01 << 5)
#define LIS_SCALE_12     (0b10 << 5)
#define LIS_SCALE_16     (0b11 << 5)

#define LIS_CYCLIC       0b00
#define LIS_SINGLE       0b01
#define LIS_STBY         0b10

#define LIS_CTRL1 0
#define LIS_CTRL2 1
#define LIS_CTRL3 2
#define LIS_CTRL4 3
#define LIS_CTRL5 4
/// @}

#ifdef LIS3_USE_SPI
uint8_t LIS3_Init(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *NSS_Port, uint16_t NSS_Pin);
#elif defined(LIS3_USE_I2C)
uint8_t LIS3_Init(I2C_HandleTypeDef *i2cHandle, uint8_t i2c_addr);
#else
#error "Please define LIS3_USE_SPI or LIS3_USE_I2C"
#endif

/**
 * @brief Configure LIS3MDL control register.
 * @param reg Control register number (0 = CTRL_REG1, 1 = CTRL_REG2, ...).
 * @param cfg Configuration byte.
 */
void LIS3_Config(uint8_t reg, uint8_t cfg);

/**
 * @brief Read and convert raw magnetic data to gauss.
 * @param mag Pointer to 3-element float array [X, Y, Z] in gauss.
 */
void LIS3_Read(float* mag);
