/**
 * @file MS5611.h
 * @brief Interface for MS5611 barometric pressure sensor (SPI)
 * @author [Nate Hunter]
 * @date [06.06.2025]
 * @version 1.1
 */

#pragma once

#include "main.h"

/** @brief MS5611 command: Reset the device */
#define MS56_CMD_RESET        0x1E

/** @brief MS5611 command: Read calibration data (PROM) */
#define MS56_CMD_CAL_READ     0xA0

/** @brief MS5611 command: Start pressure conversion (D1) */
#define MS56_CMD_CONV_D1      0x40

/** @brief MS5611 command: Start temperature conversion (D2) */
#define MS56_CMD_CONV_D2      0x50

/** @brief MS5611 command: Read ADC result (24-bit) */
#define MS56_CMD_ADC_READ     0x00

/** @brief Oversampling rate: 256 */
#define MS56_OSR_256          0b000

/** @brief Oversampling rate: 512 */
#define MS56_OSR_512          0b001

/** @brief Oversampling rate: 1024 */
#define MS56_OSR_1024         0b010

/** @brief Oversampling rate: 2048 */
#define MS56_OSR_2048         0b011

/** @brief Oversampling rate: 4096 */
#define MS56_OSR_4096         0b100

/**
 * @brief Internal calibration coefficients from PROM
 */
struct MS5611_CalData {
    float SENS_T1;    ///< Pressure sensitivity
    float OFF_T1;     ///< Pressure offset
    float TCS;        ///< Temperature coefficient of pressure sensitivity
    float TCO;        ///< Temperature coefficient of pressure offset
    float T_REF;      ///< Reference temperature
    float TEMPSENS;   ///< Temperature coefficient of the temperature
};

/**
 * @brief Initialize MS5611 sensor via SPI
 * @param spiHandle Pointer to SPI handle
 * @param NSS_Port GPIO port for chip select
 * @param NSS_Pin GPIO pin for chip select
 * @return 1 if successful, 0 if CRC check failed
 */
uint8_t MS5611_Init(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *NSS_Port, uint16_t NSS_Pin);

/**
 * @brief Set oversampling ratio
 * @param tempOS Oversampling ratio for temperature (MS56_OSR_*)
 * @param pressOS Oversampling ratio for pressure (MS56_OSR_*)
 */
void MS5611_SetOS(uint8_t tempOS, uint8_t pressOS);

/**
 * @brief Read and compensate temperature and pressure
 * @param temp Pointer to output temperature in 0.01°C
 * @param press Pointer to output pressure in Pa
 */
void MS5611_Read(int32_t *temp, uint32_t *press);

/**
 * @brief Calculate altitude in centimeters from pressure
 * @param pressure Measured pressure in Pa
 * @param seaLevelPressure Sea level reference pressure in Pa
 * @return Altitude in centimeters
 */
int32_t MS5611_GetAltitude(uint32_t* pressure, uint32_t* seaLevelPressure);
