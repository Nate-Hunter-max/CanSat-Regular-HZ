/**
 * @file LSM6.c
 * @brief Unified driver for LSM6DS3TR-C over SPI or I2C
 */

#include "LSM6.h"

#ifdef USE_SPI
static SPI_HandleTypeDef *spi;
static GPIO_TypeDef *_NSS_Port;
static uint16_t _NSS_Pin;
#else
static I2C_HandleTypeDef *i2c;
static uint16_t _i2c_addr;
#endif

static uint16_t _fsAccel;
static uint16_t _fsGyro;
static uint8_t txbuf[1];

/**
 * @brief Initialize the LSM6DS3TR-C sensor.
 *
 * @param interface SPI_HandleTypeDef* or I2C_HandleTypeDef*, depending on interface
 * @param port_or_addr SPI: GPIO port of NSS. I2C: NULL.
 * @param pin_or_addr SPI: NSS pin. I2C: I2C address (<<1).
 * @return uint8_t 1 on success, 0 on error.
 */
uint8_t LSM6_Init(void *interface, void *port_or_addr, uint16_t pin_or_addr) {
#ifdef USE_SPI
	spi = (SPI_HandleTypeDef*)interface;
	_NSS_Port = (GPIO_TypeDef*)port_or_addr;
	_NSS_Pin = pin_or_addr;
	txbuf[0] = LSM6_READ_BIT | LSM6_WHO_AM_I;
	_NSS_Port->ODR &= ~_NSS_Pin;
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	HAL_SPI_Receive(spi, txbuf, 1, 1000);
	_NSS_Port->ODR |= _NSS_Pin;
#else
	i2c = (I2C_HandleTypeDef*)interface;
	_i2c_addr = pin_or_addr; // already shifted <<1
	HAL_I2C_Mem_Read(i2c, _i2c_addr, LSM6_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, txbuf, 1, 1000);
#endif
	if (*txbuf != 0x6A) return 0;

#ifdef USE_SPI
	_NSS_Port->ODR &= ~_NSS_Pin;
	txbuf[0] = LSM6_CTRL3_C;
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	txbuf[0] = (1 << 6) | 1; // BDU enable
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	_NSS_Port->ODR |= _NSS_Pin;
#else
	txbuf[0] = (1 << 6) | 1;
	HAL_I2C_Mem_Write(i2c, _i2c_addr, LSM6_CTRL3_C, I2C_MEMADD_SIZE_8BIT, txbuf, 1, 1000);
#endif
	return 1;
}

/**
 * @brief Configure accelerometer and gyroscope full-scale and ODR.
 *
 * @param A_CFG Accelerometer config byte (ODR | FS)
 * @param G_CFG Gyroscope config byte (ODR | FS)
 */
void LSM6_ConfigAG(uint8_t A_CFG, uint8_t G_CFG) {
#ifdef USE_SPI
	_NSS_Port->ODR &= ~_NSS_Pin;
	txbuf[0] = LSM6_CTRL1_XL;
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	HAL_SPI_Transmit(spi, &A_CFG, 1, 1000);
	txbuf[0] = LSM6_CTRL2_G;
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	HAL_SPI_Transmit(spi, &G_CFG, 1, 1000);
	_NSS_Port->ODR |= _NSS_Pin;
#else
	HAL_I2C_Mem_Write(i2c, _i2c_addr, LSM6_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &A_CFG, 1, 1000);
	HAL_I2C_Mem_Write(i2c, _i2c_addr, LSM6_CTRL2_G, I2C_MEMADD_SIZE_8BIT, &G_CFG, 1, 1000);
#endif
	_fsAccel = (A_CFG >> 2) & 0x03;
	_fsGyro = (G_CFG >> 2) & 0x03;
}

/**
 * @brief Read acceleration and gyroscope data from LSM6DS3TR-C.
 *
 * @param accel Pointer to 3-element array for acceleration (g)
 * @param gyro Pointer to 3-element array for gyro (dps)
 */
void LSM6_Read(float *accel, float *gyro) {
	int16_t raw[6];
#ifdef USE_SPI
	_NSS_Port->ODR &= ~_NSS_Pin;
	txbuf[0] = LSM6_READ_BIT | LSM6_OUTX_L_G;
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	HAL_SPI_Receive(spi, (uint8_t*)raw, 12, 1000);
	_NSS_Port->ODR |= _NSS_Pin;
#else
	HAL_I2C_Mem_Read(i2c, _i2c_addr, LSM6_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw, 12, 1000);
#endif
	for (uint8_t i = 0; i < 3; i++) {
		gyro[i] = (float)raw[i] * lsm6SensG[_fsGyro];
		accel[i] = (float)raw[i + 3] * lsm6SensA[_fsAccel];
	}
}
