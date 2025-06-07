/**
 * @file LIS3.c
 * @brief LIS3MDL magnetometer driver for STM32 with support for I2C and SPI.
 */

#include "LIS3.h"

static uint8_t _mode = LIS_STBY;
static float _scale;

#ifdef LIS3_USE_SPI
static SPI_HandleTypeDef *spi;
static GPIO_TypeDef *_NSS_Port;
static uint16_t _NSS_Pin;
static uint8_t txbuf[1];

uint8_t LIS3_Init(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *NSS_Port, uint16_t NSS_Pin) {
	spi = spiHandle;
	_NSS_Port = NSS_Port;
	_NSS_Pin = NSS_Pin;

	txbuf[0] = LIS_READ_BIT | LIS_WHO_AM_I;
	_NSS_Port->ODR &= ~_NSS_Pin;
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	HAL_SPI_Receive(spi, txbuf, 1, 1000);
	_NSS_Port->ODR |= _NSS_Pin;

	return (*txbuf == 0x3D);
}
#endif

#ifdef LIS3_USE_I2C
static I2C_HandleTypeDef *i2c;
static uint8_t _i2c_addr;

uint8_t LIS3_Init(I2C_HandleTypeDef *i2cHandle, uint8_t i2c_addr) {
	i2c = i2cHandle;
	_i2c_addr = i2c_addr; // already shifted <<1

	uint8_t who_am_i = 0;
	HAL_I2C_Mem_Read(i2c, _i2c_addr, LIS_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
	return (who_am_i == 0x3D);
}
#endif

void LIS3_Config(uint8_t reg, uint8_t cfg) {
	uint8_t addr = LIS_CTRL_REG1 + reg;
	if (reg == LIS_CTRL3) _mode = cfg & 0b11;

#ifdef LIS3_USE_SPI
	_NSS_Port->ODR &= ~_NSS_Pin;
	txbuf[0] = addr;
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	txbuf[0] = cfg;
	HAL_SPI_Transmit(spi, txbuf, 1, 1000);
	_NSS_Port->ODR |= _NSS_Pin;
#elif defined(LIS3_USE_I2C)
	HAL_I2C_Mem_Write(i2c, _i2c_addr, addr, I2C_MEMADD_SIZE_8BIT, &cfg, 1, 100);
#endif

	if (reg == LIS_CTRL1) {
		uint8_t val = (cfg >> 3) & (0b11 << 2); // copy OM to Z
#ifdef LIS3_USE_SPI
		_NSS_Port->ODR &= ~_NSS_Pin;
		txbuf[0] = LIS_CTRL_REG1 + 3;
		HAL_SPI_Transmit(spi, txbuf, 1, 1000);
		txbuf[0] = val;
		HAL_SPI_Transmit(spi, txbuf, 1, 1000);
		_NSS_Port->ODR |= _NSS_Pin;
#elif defined(LIS3_USE_I2C)
		HAL_I2C_Mem_Write(i2c, _i2c_addr, LIS_CTRL_REG1 + 3, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
#endif
	}

	if (reg == LIS_CTRL2) {
		switch (cfg & LIS_SCALE_16) {
			case LIS_SCALE_4: _scale = 6842.0e-3f; break;
			case LIS_SCALE_8: _scale = 3421.0e-3f; break;
			case LIS_SCALE_12: _scale = 2281.0e-3f; break;
			case LIS_SCALE_16: _scale = 1711.0e-3f; break;
			default: break;
		}
	}
}

void LIS3_Read(float *mag) {
	uint8_t status = 0;
	uint8_t reg = LIS_STATUS;
	int16_t x, y, z;

	if (_mode == LIS_SINGLE) {
		LIS3_Config(LIS_CTRL3, LIS_SINGLE);
	}

#ifdef LIS3_USE_SPI
	UNUSED(reg);
	uint8_t cmd = LIS_READ_BIT | LIS_STATUS;
	while (!(status & (1 << 3))) {
		_NSS_Port->ODR &= ~_NSS_Pin;
		HAL_SPI_Transmit(spi, &cmd, 1, 1000);
		HAL_SPI_Receive(spi, &status, 1, 1000);
		_NSS_Port->ODR |= _NSS_Pin;
	}
	_NSS_Port->ODR &= ~_NSS_Pin;
	cmd = LIS_READ_BIT | LIS_MULTY_BIT | LIS_OUT_X_L;
	HAL_SPI_Transmit(spi, &cmd, 1, 1000);
	HAL_SPI_Receive(spi, (uint8_t*)&x, 2, 1000);
	HAL_SPI_Receive(spi, (uint8_t*)&y, 2, 1000);
	HAL_SPI_Receive(spi, (uint8_t*)&z, 2, 1000);
	_NSS_Port->ODR |= _NSS_Pin;
#elif defined(LIS3_USE_I2C)
	while (!(status & (1 << 3))) {
		HAL_I2C_Mem_Read(i2c, _i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	}
	uint8_t raw[6];
	HAL_I2C_Mem_Read(i2c, _i2c_addr, LIS_OUT_X_L | 0x80, I2C_MEMADD_SIZE_8BIT, raw, 6, 100);
	x = (int16_t)(raw[1] << 8 | raw[0]);
	y = (int16_t)(raw[3] << 8 | raw[2]);
	z = (int16_t)(raw[5] << 8 | raw[4]);
#endif

	mag[0] = x / _scale;
	mag[1] = y / _scale;
	mag[2] = z / _scale;
}
