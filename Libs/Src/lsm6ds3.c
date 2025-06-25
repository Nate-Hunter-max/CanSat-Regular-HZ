/**
 * @file lsm6ds3.c
 * @brief Implementation of LSM6DS3TR-C driver for STM32 (I2C/SPI)
 * @author Nate Hunter
 * @date 2025-06-25
 * @version v2.0.0
 */

#include "lsm6ds3.h"

/// @brief Write one byte to register
static uint8_t LSM6DS3_WriteReg(LSM6DS3_Handle *dev, uint8_t reg, uint8_t value) {
#ifdef LSM6_USE_SPI
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	uint8_t tx[2] = { reg & 0x7F, value };
	HAL_StatusTypeDef res = HAL_SPI_Transmit(dev->spi, tx, 2, dev->timeout);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return (res == HAL_OK);
#else
    return (HAL_I2C_Mem_Write(dev->i2c, dev->i2c_addr, reg,
            I2C_MEMADD_SIZE_8BIT, &value, 1, dev->timeout) == HAL_OK);
#endif
}

/// @brief Read multiple bytes starting from register
static uint8_t LSM6DS3_ReadRegs(LSM6DS3_Handle *dev, uint8_t reg, uint8_t *data, uint8_t len) {
#ifdef LSM6_USE_SPI
	reg |= 0x80; // SPI read
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef res = HAL_SPI_Transmit(dev->spi, &reg, 1, dev->timeout);
	if (res == HAL_OK) {
		res = HAL_SPI_Receive(dev->spi, data, len, dev->timeout);
	}
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return (res == HAL_OK);
#else
    return (HAL_I2C_Mem_Read(dev->i2c, dev->i2c_addr, reg,
            I2C_MEMADD_SIZE_8BIT, data, len, dev->timeout) == HAL_OK);
#endif
}

uint8_t LSM6DS3_Init(LSM6DS3_Handle *dev, LSM6DS3_AccelFS accelFS, LSM6DS3_GyroFS gyroFS) {
	uint8_t who_am_i;
	if (!LSM6DS3_ReadRegs(dev, LSM6DS3_REG_WHO_AM_I, &who_am_i, 1))
		return 0;
	if (!((who_am_i == LSM6DS3_WHO_AM_I) || (who_am_i == LSM6DS3TR_WHO_AM_I)))
		return 0;

	// Apply control settings
	if (!LSM6DS3_WriteReg(dev, LSM6DS3_REG_CTRL1_XL, (dev->accelODR << 4) | (accelFS << 2)))
		return 0;
	if (!LSM6DS3_WriteReg(dev, LSM6DS3_REG_CTRL2_G, (dev->gyroODR << 4) | (gyroFS << 1)))
		return 0;
	if (!LSM6DS3_WriteReg(dev, LSM6DS3_REG_CTRL3_C, 0x44))
		return 0;

	// Set scale factors
	switch (accelFS) {
		case LSM6DS3_XL_2G:
			dev->accelScale = 0.061f / 1000;
			break;
		case LSM6DS3_XL_4G:
			dev->accelScale = 0.122f / 1000;
			break;
		case LSM6DS3_XL_8G:
			dev->accelScale = 0.244f / 1000;
			break;
		case LSM6DS3_XL_16G:
			dev->accelScale = 0.488f / 1000;
			break;
		default:
			return 0;
	}

	switch (gyroFS) {
		case LSM6DS3_GYRO_125DPS:
			dev->gyroScale = 4.375f / 1000;
			break;
		case LSM6DS3_GYRO_250DPS:
			dev->gyroScale = 8.75f / 1000;
			break;
		case LSM6DS3_GYRO_500DPS:
			dev->gyroScale = 17.5f / 1000;
			break;
		case LSM6DS3_GYRO_1000DPS:
			dev->gyroScale = 35.0f / 1000;
			break;
		case LSM6DS3_GYRO_2000DPS:
			dev->gyroScale = 70.0f / 1000;
			break;
		default:
			return 0;
	}

	return 1;
}

uint8_t LSM6DS3_ReadData(LSM6DS3_Handle *dev, float accel[3], float gyro[3]) {
	uint8_t buffer[12];
	if (!LSM6DS3_ReadRegs(dev, LSM6DS3_REG_OUTX_L_G, buffer, 12))
		return 0;

	// Gyroscope
	for (int i = 0; i < 3; i++) {
		int16_t raw = (int16_t) (buffer[i * 2 + 1] << 8 | buffer[i * 2]);
		gyro[i] = raw * dev->gyroScale;
	}

	// Accelerometer
	for (int i = 0; i < 3; i++) {
		int16_t raw = (int16_t) (buffer[6 + i * 2 + 1] << 8 | buffer[6 + i * 2]);
		accel[i] = raw * dev->accelScale;
	}

	return 1;
}
