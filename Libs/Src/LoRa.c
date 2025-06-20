/**
 * @file LoRa.c
 * @brief LoRa module driver implementation
 * @author Nate Hunter
 * @date 2025-06-13
 * @version 1.3
 */

#include "LoRa.h"
#include "main.h"
#include <string.h>

/* Static buffer for SPI transactions */
static uint8_t txBuffer[1];

/* Private function prototypes */
static inline void LoRa_writeReg(LoRa_Handle_t *handle, LoRa_Register_t reg, uint8_t *data, uint8_t count);
static inline void LoRa_writeRegByte(LoRa_Handle_t *handle, LoRa_Register_t reg, uint8_t data);
static inline uint8_t LoRa_readRegByte(LoRa_Handle_t *handle, LoRa_Register_t reg);
static inline void LoRa_readReg(LoRa_Handle_t *handle, LoRa_Register_t reg, uint8_t *data, uint8_t count);

/**
 * @brief Initialize the LoRa module
 * @param handle Pointer to LoRa handle structure
 * @return uint8_t Status of initialization (1 = success, 0 = error)
 */
uint8_t LoRa_Init(LoRa_Handle_t *handle) {
    uint8_t id;

    /* Read chip version */
    txBuffer[0] = LORA_REG_VERSION;
    handle->nssPort->ODR &= ~handle->nssPin;
    HAL_SPI_Transmit(handle->spi, txBuffer, 1, 1000);
    HAL_SPI_Receive(handle->spi, &id, 1, 1000);
    handle->nssPort->ODR |= handle->nssPin;

    if (id != 0x12) {
        return 0;
    }

    /* Set sleep mode and apply configuration */
    LoRa_writeRegByte(handle, LORA_REG_OP_MODE, 0x00);
    LoRa_SetConfig(handle, &handle->config);

    return 1;
}

/**
 * @brief Configure the LoRa module parameters
 * @param handle Pointer to LoRa handle structure
 * @param config Pointer to configuration structure
 */
void LoRa_SetConfig(LoRa_Handle_t *handle, LoRa_Config_t *config) {
    LoRa_writeRegByte(handle, LORA_REG_OP_MODE, 0b10000000); // Standby mode
    handle->config = *config;

	uint32_t Fr = config->frequency * 16384, PrL = config->preambleLength;
	Fr = ((Fr << 16) & 0xFF0000) | ((Fr << 0) & 0x00FF00) | ((Fr >> 16) & 0x0000FF);
	PrL = ((PrL << 8) & 0xFF00) | ((PrL >> 8) & 0x00FF);

	LoRa_writeReg(handle, LORA_REG_FR_MSB, (uint8_t*) &Fr, 3);
	LoRa_writeRegByte(handle, LORA_REG_MODEM_CONFIG_1, (config->bandwidth << 4) | (config->codingRate << 1) | config->headerMode);
	LoRa_writeRegByte(handle, LORA_REG_MODEM_CONFIG_2, (uint8_t) (config->spreadingFactor << 4) | (config->crcEnabled << 2));
	LoRa_writeRegByte(handle, LORA_REG_MODEM_CONFIG_3, config->lowDataRateOptimize << 3);
	LoRa_writeReg(handle, LORA_REG_PREAMBLE_MSB, (uint8_t*) &PrL, 2);
	LoRa_writeRegByte(handle, LORA_REG_PAYLOAD_LENGTH, config->payloadLength);
	LoRa_writeRegByte(handle, LORA_REG_PAYLOAD_MAX_LENGTH, config->payloadLength);
	LoRa_writeRegByte(handle, LORA_REG_LNA, (0x01 << 5) | 0x03);
	LoRa_writeRegByte(handle, LORA_REG_PA_CONFIG, (1 << 7) | (0x07 << 4) | config->txPower);
	LoRa_writeRegByte(handle, LORA_REG_FIFO_TX_BASE_ADDR, config->txAddr);
	LoRa_writeRegByte(handle, LORA_REG_FIFO_RX_BASE_ADDR, config->rxAddr);
	LoRa_writeRegByte(handle, LORA_REG_OP_MODE, 0b10000001 | (1 << 3));
	LoRa_writeRegByte(handle, LORA_REG_OP_MODE, 0x05);
}

/**
 * @brief Transmit data via LoRa
 * @param handle Pointer to LoRa handle structure
 * @param data Pointer to transmit data buffer
 * @param len Length of data to transmit
 */
void LoRa_Transmit(LoRa_Handle_t *handle, void *data, uint8_t len) {
    /* Set standby mode */
    LoRa_writeRegByte(handle, LORA_REG_OP_MODE, 0x01);
    LoRa_writeRegByte(handle, LORA_REG_PAYLOAD_LENGTH, len);

    /* Write data to FIFO */
    LoRa_writeRegByte(handle, LORA_REG_FIFO_ADDR_SPI, handle->config.txAddr);
    txBuffer[0] = LORA_REG_FIFO | LORA_SPI_WRITE_BIT;
    handle->nssPort->ODR &= ~handle->nssPin;
    HAL_SPI_Transmit(handle->spi, txBuffer, 1, 1000);
    HAL_SPI_Transmit(handle->spi, data, len, 1000);
    handle->nssPort->ODR |= handle->nssPin;

    /* Start transmission */
    LoRa_writeRegByte(handle, LORA_REG_OP_MODE, 0x03);

    /* Wait for transmission to complete */
    while (!(LoRa_readRegByte(handle, LORA_REG_IRQ_FLAGS) & LORA_FLAG_TX_DONE));

    /* Clear flag and return to receive mode */
    LoRa_writeRegByte(handle, LORA_REG_IRQ_FLAGS, LORA_FLAG_TX_DONE);
    LoRa_writeRegByte(handle, LORA_REG_OP_MODE, 0x05);
}

/**
 * @brief Receives a complete and valid LoRa packet if available.
 * @param handle Pointer to LoRa handle structure.
 * @param rxData Pointer to buffer to store received data.
 * @param len Pointer to store length of received data.
 * @return 1 if a valid packet is received, 0 otherwise.
 */
uint8_t LoRa_Receive(LoRa_Handle_t *handle, void *rxData, uint8_t *len) {
    uint8_t irqFlags = LoRa_readRegByte(handle, LORA_REG_IRQ_FLAGS);

    // Check if a packet was received
    if (!(irqFlags & LORA_FLAG_RX_DONE)) {
        return 0;
    }

    // Check CRC if enabled in config
    if (handle->config.crcEnabled) {
        if (irqFlags & LORA_FLAG_PAYLOAD_CRC_ERROR) {
            LoRa_writeRegByte(handle, LORA_REG_IRQ_FLAGS, LORA_FLAG_PAYLOAD_CRC_ERROR); // Clear CRC error flag
            return 0; // Invalid packet
        }
    }

    // Read FIFO address of received packet
    uint8_t currentAddr = LoRa_readRegByte(handle, LORA_REG_FIFO_RX_CURRENT_ADDR);
    LoRa_writeRegByte(handle, LORA_REG_FIFO_ADDR_SPI, currentAddr);

    // Read number of bytes received
    *len = LoRa_readRegByte(handle, LORA_REG_RX_NB_BYTES);

    // Read payload from FIFO
    LoRa_readReg(handle, LORA_REG_FIFO, rxData, *len);

    // Clear RxDone flag
    LoRa_writeRegByte(handle, LORA_REG_IRQ_FLAGS, LORA_FLAG_RX_DONE);

    return 1;
}

/* Private functions implementation */

/**
 * @brief Write data to LoRa register
 * @param handle Pointer to LoRa handle structure
 * @param reg Register address
 * @param data Pointer to data buffer
 * @param count Number of bytes to write
 */
static inline void LoRa_writeReg(LoRa_Handle_t *handle, LoRa_Register_t reg, uint8_t *data, uint8_t count) {
    txBuffer[0] = reg | LORA_SPI_WRITE_BIT;
    handle->nssPort->ODR &= ~handle->nssPin;
    HAL_SPI_Transmit(handle->spi, txBuffer, 1, 1000);
    HAL_SPI_Transmit(handle->spi, data, count, 1000);
    handle->nssPort->ODR |= handle->nssPin;
}

/**
 * @brief Write single byte to LoRa register
 * @param handle Pointer to LoRa handle structure
 * @param reg Register address
 * @param data Data byte to write
 */
static inline void LoRa_writeRegByte(LoRa_Handle_t *handle, LoRa_Register_t reg, uint8_t data) {
    LoRa_writeReg(handle, reg, &data, 1);
}

/**
 * @brief Read single byte from LoRa register
 * @param handle Pointer to LoRa handle structure
 * @param reg Register address
 * @return uint8_t Read data byte
 */
static inline uint8_t LoRa_readRegByte(LoRa_Handle_t *handle, LoRa_Register_t reg) {
    txBuffer[0] = reg & ~LORA_SPI_WRITE_BIT;
    uint8_t result = 0;
    handle->nssPort->ODR &= ~handle->nssPin;
    HAL_SPI_Transmit(handle->spi, txBuffer, 1, 1000);
    HAL_SPI_Receive(handle->spi, &result, 1, 1000);
    handle->nssPort->ODR |= handle->nssPin;
    return result;
}

/**
 * @brief Read data from LoRa register
 * @param handle Pointer to LoRa handle structure
 * @param reg Register address
 * @param data Pointer to data buffer
 * @param count Number of bytes to read
 */
static inline void LoRa_readReg(LoRa_Handle_t *handle, LoRa_Register_t reg, uint8_t *data, uint8_t count) {
    txBuffer[0] = reg & ~LORA_SPI_WRITE_BIT;
    handle->nssPort->ODR &= ~handle->nssPin;
    HAL_SPI_Transmit(handle->spi, txBuffer, 1, 1000);
    HAL_SPI_Receive(handle->spi, data, count, 1000);
    handle->nssPort->ODR |= handle->nssPin;
}

/**
 * @brief Enable DIO0 interrupt with given mapping.
 * @param handle Pointer to LoRa handle structure
 * @param irqMapping Value to set in RegDioMapping1 (bits 7-6)
 */
void LoRa_EnableDIO0Interrupt(LoRa_Handle_t *handle, uint8_t irqMapping) {
    // Set DIO0 mapping (RegDioMapping1 bits 7-6)
    uint8_t reg = LoRa_readRegByte(handle, 0x40); // RegDioMapping1
    reg &= ~(0xC0); // Clear bits 7 and 6
    reg |= (irqMapping << 6);
    LoRa_writeRegByte(handle, 0x40, reg);
}

/**
 * @brief Disable DIO0 interrupt (set to unused state)
 * @param handle Pointer to LoRa handle structure
 */
void LoRa_DisableDIO0Interrupt(LoRa_Handle_t *handle) {
    uint8_t reg = LoRa_readRegByte(handle, 0x40); // RegDioMapping1
    reg &= ~(0xC0); // Clear bits 7 and 6
    LoRa_writeRegByte(handle, 0x40, reg);
}

