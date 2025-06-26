/**
 * @file lis3mdl.c
 * @brief Implementation for LIS3MDL 3-axis magnetometer (I2C/SPI interface)
 * @author Nate Hunter
 * @date 2025-06-26
 * @version v2.0.0
 */

#include "lis3mdl.h"
#include "main.h"

static uint8_t LIS3MDL_ReadReg(LIS3MDL_Device *dev, uint8_t reg, uint8_t *data, uint16_t len);
static uint8_t LIS3MDL_WriteReg(LIS3MDL_Device *dev, uint8_t reg, uint8_t *data, uint16_t len);

uint8_t LIS3MDL_Init(LIS3MDL_Device *dev) {
	return LIS3MDL_ApplySettings(dev);
}

uint8_t LIS3MDL_ApplySettings(LIS3MDL_Device *dev) {
	uint8_t ctrl1 = (dev->output_data_rate << 2) | 0x60;
	uint8_t ctrl2 = (dev->scale << 5);
	uint8_t ctrl3 = 0x00;
	uint8_t ctrl4 = 0x0C;
	uint8_t ctrl5 = 0x40;

	if (!LIS3MDL_WriteReg(dev, LIS3MDL_CTRL_REG1, &ctrl1, 1))
		return 0;
	if (!LIS3MDL_WriteReg(dev, LIS3MDL_CTRL_REG2, &ctrl2, 1))
		return 0;
	if (!LIS3MDL_WriteReg(dev, LIS3MDL_CTRL_REG3, &ctrl3, 1))
		return 0;
	if (!LIS3MDL_WriteReg(dev, LIS3MDL_CTRL_REG4, &ctrl4, 1))
		return 0;
	if (!LIS3MDL_WriteReg(dev, LIS3MDL_CTRL_REG5, &ctrl5, 1))
		return 0;

	return 1;
}

/**
 * @brief Read scaled magnetic and optional temperature data
 * @param dev Pointer to the device structure
 * @param axes Pointer to float[3] array for X, Y, Z in gauss
 * @param temperature Optional pointer to int16_t for temperature
 * @return 1 on success, 0 on failure
 */
uint8_t LIS3MDL_ReadData(LIS3MDL_Device *dev, float axes[3], int16_t *temperature) {
	uint8_t raw[6];
	if (!LIS3MDL_ReadReg(dev, LIS3MDL_OUT_X_L | 0x80, raw, 6))
		return 0;

	// Combine raw values
	int16_t raw_x = (int16_t) (raw[1] << 8 | raw[0]);
	int16_t raw_y = (int16_t) (raw[3] << 8 | raw[2]);
	int16_t raw_z = (int16_t) (raw[5] << 8 | raw[4]);

	// Sensitivity values from datasheet (LSB/Gauss)
	float sensitivity;
	switch (dev->scale) {
		case LIS3_SCALE_4_GAUSS:
			sensitivity = 6842.0f;
			break;
		case LIS3_SCALE_8_GAUSS:
			sensitivity = 3421.0f;
			break;
		case LIS3_SCALE_12_GAUSS:
			sensitivity = 2281.0f;
			break;
		case LIS3_SCALE_16_GAUSS:
			sensitivity = 1711.0f;
			break;
		default:
			sensitivity = 6842.0f;
			break;
	}

	// Convert to gauss
	axes[0] = raw_x / sensitivity;
	axes[1] = raw_y / sensitivity;
	axes[2] = raw_z / sensitivity;

	// Optionally read temperature
	if (temperature != NULL) {
		uint8_t temp[2];
		if (!LIS3MDL_ReadReg(dev, LIS3MDL_TEMP_OUT_L | 0x80, temp, 2))
			return 0;
		*temperature = (int16_t) (temp[1] << 8 | temp[0]);
	}

	return 1;
}

uint8_t LIS3MDL_ApplyHardIronCalibration(LIS3MDL_Device *dev, int16_t x_offset, int16_t y_offset, int16_t z_offset) {
	uint8_t data[6];
	data[0] = x_offset & 0xFF;
	data[1] = (x_offset >> 8) & 0xFF;
	data[2] = y_offset & 0xFF;
	data[3] = (y_offset >> 8) & 0xFF;
	data[4] = z_offset & 0xFF;
	data[5] = (z_offset >> 8) & 0xFF;

	return LIS3MDL_WriteReg(dev, 0x05, data, 6);
}

static uint8_t LIS3MDL_ReadReg(LIS3MDL_Device *dev, uint8_t reg, uint8_t *data, uint16_t len) {
#ifdef LIS3_USE_SPI
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	uint8_t addr = reg | 0xC0;
	if (HAL_SPI_Transmit(dev->spi, &addr, 1, dev->timeout) != HAL_OK) {
		HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
		return 0;
	}
	if (HAL_SPI_Receive(dev->spi, data, len, dev->timeout) != HAL_OK) {
		HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
		return 0;
	}
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return 1;
#elif LIS3_USE_I2C
    return HAL_I2C_Mem_Read(dev->i2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, data, len, dev->i2c->Init.Timeout) == HAL_OK;
#endif
}

static uint8_t LIS3MDL_WriteReg(LIS3MDL_Device *dev, uint8_t reg, uint8_t *data, uint16_t len) {
#ifdef LIS3_USE_SPI
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	uint8_t addr = reg & 0x3F;
	if (HAL_SPI_Transmit(dev->spi, &addr, 1, dev->timeout) != HAL_OK) {
		HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
		return 0;
	}
	if (HAL_SPI_Transmit(dev->spi, data, len, dev->timeout) != HAL_OK) {
		HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
		return 0;
	}
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return 1;
#elif LIS3_USE_I2C
    return HAL_I2C_Mem_Write(dev->i2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, data, len, dev->i2c->Init.Timeout) == HAL_OK;
#endif
}
