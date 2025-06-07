/**
 * @file W25Qx.h
 * @brief Header file for Winbond W25Qx SPI Flash memory driver.
 *
 * This header declares the functions and structures for interacting with
 * W25Qx series SPI Flash memory devices.
 *
 * @author [Nate Hunter]
 * @date [29.12.2024]
 * @version 1.0
 */

#ifndef W25QX_H
#define W25QX_H

#include "main.h"

// Constants for supported devices
#define W25Qx_MANUFACTURER_ID 0xEF
#define W25Q80_DEVICE_ID      0x4014
#define W25Q16_DEVICE_ID      0x4015
#define W25Q32_DEVICE_ID      0x4016
#define W25Q64_DEVICE_ID      0x4017
#define W25Q128_DEVICE_ID     0x4018

// Command definitions
#define W25Qx_CMD_READ          0x03
#define W25Qx_CMD_WRITE_ENABLE  0x06
#define W25Qx_CMD_PAGE_PROGRAM  0x02
#define W25Qx_CMD_SECTOR_ERASE  0x20
#define W25Qx_CMD_READ_SR1      0x05
#define W25Qx_CMD_JEDEC_ID      0x9F
#define W25Qx_CMD_CHIP_ERASE		0xC7

// Status Register 1 bits
#define W25Qx_SR1_BUSY          0x01

// Device parameters
#define W25Qx_PAGE_SIZE         256
#define W25Qx_SECTOR_SIZE       4096
#define W25Qx_TIMEOUT           1000

/**
 * @brief Structure representing a W25Qx device.
 */
typedef struct {
    SPI_HandleTypeDef *spi;  ///< SPI handle for communication.
    GPIO_TypeDef *cs_port;   ///< GPIO port for chip select (CS).
    uint16_t cs_pin;         ///< GPIO pin for chip select (CS).
    uint32_t capacity;       ///< Device capacity in bytes.
    uint8_t *txbuf;          ///< Pointer to the TX buffer.
    uint8_t *rxbuf;          ///< Pointer to the RX buffer.
} W25Qx_Device;

/**
 * @brief Initialize the W25Qx device.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @return 1 if initialization succeeds, 0 otherwise.
 */
uint8_t W25Qx_Init(W25Qx_Device *dev);

/**
 * @brief Wait for the W25Qx device to be ready.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param timeout Timeout in milliseconds.
 * @return 1 if the device is ready, 0 otherwise.
 */
uint8_t W25Qx_WaitForReady(W25Qx_Device *dev, uint32_t timeout);

/**
 * @brief Erase a 4KB sector.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param address Address of the sector to erase.
 * @return 1 if the operation succeeds, 0 otherwise.
 */
uint8_t W25Qx_EraseSector(W25Qx_Device *dev, uint32_t address);

/**
 * @brief Read data from the W25Qx device.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param address Address to read from.
 * @param buffer Pointer to the buffer to store read data.
 * @param length Number of bytes to read.
 * @return 1 if the operation succeeds, 0 otherwise.
 */
uint8_t W25Qx_ReadData(W25Qx_Device *dev, uint32_t address, void *buffer, uint32_t length);

/**
 * @brief Write data to the W25Qx device.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param address Address to write to.
 * @param buffer Pointer to the data to write.
 * @param length Number of bytes to write.
 * @return 1 if the operation succeeds, 0 otherwise.
 */
uint8_t W25Qx_WriteData(W25Qx_Device *dev, uint32_t address, const void *buffer, uint32_t length);

/**
 * @brief Erase the entire chip.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @return 1 if the operation succeeds, 0 otherwise.
 */
uint8_t W25Qx_EraseChip(W25Qx_Device *dev);

/**
 * @brief Erase multiple sectors sequentially.
 *
 * @param dev Pointer to the W25Qx device structure.
 * @param start_address Starting address of the first sector to erase.
 * @param length Number of bytes to erase (will be aligned to sector boundaries).
 * @return 1 if the operation succeeds, 0 otherwise.
 */
uint8_t W25Qx_EraseSectors(W25Qx_Device *dev, uint32_t start_address, uint32_t length);

#endif // W25QX_H
