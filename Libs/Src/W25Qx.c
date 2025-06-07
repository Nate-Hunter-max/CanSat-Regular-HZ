/**
 * @file W25Qx.c
 * @brief Driver for Winbond W25Qx series SPI Flash memory.
 *
 *  Created on: Dec 29, 2024
 *      Author: Nate Hunter
 *
 * This library supports W25Q80, W25Q16, W25Q32, W25Q64, and W25Q128.
 */

#include "W25Qx.h"
#include "main.h"
#include <string.h>

// Internal buffers for SPI communication
static uint8_t txbuf[4];
static uint8_t rxbuf[4];

/**
 * @brief Initialize the W25Qx device.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @return 1 if initialization succeeds, 0 otherwise.
 */
uint8_t W25Qx_Init(W25Qx_Device *dev) {
    dev->txbuf = txbuf;
    dev->rxbuf = rxbuf;

    // Check JEDEC ID
    txbuf[0] = W25Qx_CMD_JEDEC_ID;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(dev->spi, txbuf, rxbuf, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    if (rxbuf[1] != W25Qx_MANUFACTURER_ID) {
        return 0;
    }

    uint16_t device_id = (rxbuf[2] << 8) | rxbuf[3];

    // Assign device capacity
    switch (device_id) {
        case W25Q80_DEVICE_ID:
            dev->capacity = 8 * 1024 * 1024 / 8;
            break;
        case W25Q16_DEVICE_ID:
            dev->capacity = 16 * 1024 * 1024 / 8;
            break;
        case W25Q32_DEVICE_ID:
            dev->capacity = 32 * 1024 * 1024 / 8;
            break;
        case W25Q64_DEVICE_ID:
            dev->capacity = 64 * 1024 * 1024 / 8;
            break;
        case W25Q128_DEVICE_ID:
            dev->capacity = 128 * 1024 * 1024 / 8;
            break;
        default:
            return 0;
    }

    return 1;
}

/**
 * @brief Wait for the W25Qx device to be ready.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param timeout Timeout in milliseconds.
 * @return 1 if the device is ready, 0 otherwise.
 */
uint8_t W25Qx_WaitForReady(W25Qx_Device *dev, uint32_t timeout) {
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout) {
        txbuf[0] = W25Qx_CMD_READ_SR1;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(dev->spi, txbuf, rxbuf, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

        if ((rxbuf[1] & W25Qx_SR1_BUSY) == 0) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Erase a 4KB sector.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param address Address of the sector to erase.
 * @return 1 if the operation succeeds, 0 otherwise.
 */
uint8_t W25Qx_EraseSector(W25Qx_Device *dev, uint32_t address) {
    if (!W25Qx_WaitForReady(dev, W25Qx_TIMEOUT)) {
        return 0;
    }

    txbuf[0] = W25Qx_CMD_WRITE_ENABLE;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spi, txbuf, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    txbuf[0] = W25Qx_CMD_SECTOR_ERASE;
    txbuf[1] = (address >> 16) & 0xFF;
    txbuf[2] = (address >> 8) & 0xFF;
    txbuf[3] = address & 0xFF;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spi, txbuf, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    return W25Qx_WaitForReady(dev, W25Qx_TIMEOUT);
}

/**
 * @brief Read data from the W25Qx device.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param address Address to read from.
 * @param buffer Pointer to the buffer to store read data.
 * @param length Number of bytes to read.
 * @return 1 if the operation succeeds, 0 otherwise.
 */
uint8_t W25Qx_ReadData(W25Qx_Device *dev, uint32_t address, void *buffer, uint32_t length) {
    if (!W25Qx_WaitForReady(dev, W25Qx_TIMEOUT)) {
        return 0;
    }

    txbuf[0] = W25Qx_CMD_READ;
    txbuf[1] = (address >> 16) & 0xFF;
    txbuf[2] = (address >> 8) & 0xFF;
    txbuf[3] = address & 0xFF;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spi, txbuf, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(dev->spi, buffer, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    return 1;
}

/**
 * @brief Write data to the W25Qx device.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param address Address to write to.
 * @param buffer Pointer to the data to write.
 * @param length Number of bytes to write.
 * @return 1 if the operation succeeds, 0 otherwise.
 */
uint8_t W25Qx_WriteData(W25Qx_Device *dev, uint32_t address, const void *buffer, uint32_t length) {
    uint32_t remaining = length;
    uint32_t page_offset = address % W25Qx_PAGE_SIZE;
    uint32_t to_write;

    while (remaining > 0) {
        to_write = W25Qx_PAGE_SIZE - page_offset;
        if (to_write > remaining) {
            to_write = remaining;
        }

        if (!W25Qx_WaitForReady(dev, W25Qx_TIMEOUT)) {
            return 0;
        }

        txbuf[0] = W25Qx_CMD_WRITE_ENABLE;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(dev->spi, txbuf, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

        txbuf[0] = W25Qx_CMD_PAGE_PROGRAM;
        txbuf[1] = (address >> 16) & 0xFF;
        txbuf[2] = (address >> 8) & 0xFF;
        txbuf[3] = address & 0xFF;

        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(dev->spi, txbuf, 4, HAL_MAX_DELAY);
        HAL_SPI_Transmit(dev->spi, buffer, to_write, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

        address += to_write;
        buffer += to_write;
        remaining -= to_write;
        page_offset = 0;
    }

    return 1;
}

uint8_t W25Qx_EraseChip(W25Qx_Device *dev) {
    if (!W25Qx_WaitForReady(dev, W25Qx_TIMEOUT)) {
        return 0;
    }

    // Enable write operations
    dev->txbuf[0] = W25Qx_CMD_WRITE_ENABLE;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spi, dev->txbuf, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    // Send chip erase command
    dev->txbuf[0] = W25Qx_CMD_CHIP_ERASE;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spi, dev->txbuf, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    // Wait for the operation to complete
    return W25Qx_WaitForReady(dev, W25Qx_TIMEOUT*100); // Chip erase can take a long time
}

uint8_t W25Qx_EraseSectors(W25Qx_Device *dev, uint32_t start_address, uint32_t length) {
    uint32_t end_address = start_address + length;
    uint32_t current_address = start_address;

    // Align start address to sector boundary
    current_address -= current_address % W25Qx_SECTOR_SIZE;

    while (current_address < end_address) {
        if (!W25Qx_EraseSector(dev, current_address)) {
            return 0; // Abort on failure
        }
        current_address += W25Qx_SECTOR_SIZE;
    }

    return 1;
}

