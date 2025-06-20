/**
 * @file LoRa.h
 * @brief LoRa module driver header file
 * @author Nate Hunter
 * @date 2025-06-13
 * @version 1.3
 */

#ifndef LORA_H
#define LORA_H

#include "main.h"

/**
 * @defgroup LoRa_Registers LoRa Register Definitions
 * @brief Memory-mapped registers of the LoRa module
 * @{
 */
typedef enum {
    LORA_REG_FIFO                 = 0x00,  ///< FIFO register address
    LORA_REG_OP_MODE              = 0x01,  ///< Operation mode control register
    LORA_REG_FR_MSB               = 0x06,  ///< Frequency setting MSB register
    LORA_REG_FR_MID               = 0x07,  ///< Frequency setting MID register
    LORA_REG_FR_LSB               = 0x08,  ///< Frequency setting LSB register
    LORA_REG_PA_CONFIG            = 0x09,  ///< Power amplifier configuration
    LORA_REG_FIFO_ADDR_SPI        = 0x0D,  ///< FIFO address pointer in SPI mode
    LORA_REG_FIFO_TX_BASE_ADDR    = 0x0E,  ///< FIFO transmit base address
    LORA_REG_FIFO_RX_BASE_ADDR    = 0x0F,  ///< FIFO receive base address
    LORA_REG_LNA                  = 0x0C,  ///< Low noise amplifier control
    LORA_REG_FIFO_RX_CURRENT_ADDR = 0x10,  ///< Current FIFO receive address
    LORA_REG_IRQ_FLAGS_MASK       = 0x11,  ///< IRQ flags mask register
    LORA_REG_IRQ_FLAGS            = 0x12,  ///< IRQ flags register
    LORA_REG_RX_NB_BYTES          = 0x13,  ///< Number of received bytes
    LORA_REG_MODEM_CONFIG_1       = 0x1D,  ///< Modem configuration 1
    LORA_REG_MODEM_CONFIG_2       = 0x1E,  ///< Modem configuration 2
    LORA_REG_PREAMBLE_MSB         = 0x20,  ///< Preamble length MSB
    LORA_REG_PREAMBLE_LSB         = 0x21,  ///< Preamble length LSB
    LORA_REG_PAYLOAD_LENGTH       = 0x22,  ///< Payload length
    LORA_REG_PAYLOAD_MAX_LENGTH   = 0x23,  ///< Maximum payload length
    LORA_REG_FIFO_RX_BYTE_ADDR    = 0x25,  ///< FIFO receive byte address
    LORA_REG_MODEM_CONFIG_3       = 0x26,  ///< Modem configuration 3
    LORA_REG_VERSION              = 0x42   ///< Chip version
} LoRa_Register_t;
/** @} */

/**
 * @defgroup LoRa_Flags LoRa Status Flags
 * @brief Bitmask definitions for IRQ flags
 * @{
 */
typedef enum {
    LORA_FLAG_CAD_DETECTED        = 0x01,  ///< Channel activity detected flag
    LORA_FLAG_FHSS_CHANGE_CHANNEL = 0x02,  ///< Frequency hop change channel flag
    LORA_FLAG_CAD_DONE            = 0x04,  ///< Channel activity detection done
    LORA_FLAG_TX_DONE             = 0x08,  ///< Transmission complete flag
    LORA_FLAG_VALID_HEADER        = 0x10,  ///< Valid header received flag
    LORA_FLAG_PAYLOAD_CRC_ERROR   = 0x20,  ///< Payload CRC error flag
    LORA_FLAG_RX_DONE             = 0x40,  ///< Reception complete flag
    LORA_FLAG_RX_TIMEOUT          = 0x80   ///< Reception timeout flag
} LoRa_Flag_t;
/** @} */

/**
 * @defgroup LoRa_SPI_Bits LoRa SPI Control Bits
 * @brief SPI transaction control bits
 * @{
 */
typedef enum {
    LORA_SPI_WRITE_BIT = (1 << 7),  ///< SPI write operation bit
    LORA_SPI_READ_BIT  = 0x76       ///< SPI read operation value
} LoRa_SpiBit_t;
/** @} */

/**
 * @brief LoRa module configuration structure
 */
typedef struct {
    uint32_t frequency;          ///< Frequency in Hz (e.g., 433000000 for 433 MHz)
    uint8_t bandwidth;           ///< Bandwidth: 7.8 | 10.4 | 15.6 | ... | 500 kHz (values: 0-9)
    uint8_t spreadingFactor;     ///< Spreading factor: 6-12
    uint8_t codingRate;          ///< Coding rate: 0=4/5, 1=4/6, 2=4/7, 3=4/8
    uint8_t headerMode;          ///< Header mode: 1=fixed, 0=explicit
    uint8_t crcEnabled;          ///< CRC: 0=disabled, 1=enabled
    uint8_t lowDataRateOptimize; ///< Low data rate optimization: 0=disabled, 1=enabled
    uint8_t preambleLength;      ///< Preamble length (minimum: 4)
    uint8_t payloadLength;       ///< Payload length (maximum: 255)
    uint8_t txPower;             ///< Transmission power: 0-15
    uint8_t txAddr;              ///< FIFO base address for transmission: 0-255
    uint8_t rxAddr;              ///< FIFO base address for reception: 0-255
} LoRa_Config_t;

/**
 * @brief LoRa module handle structure
 */
typedef struct {
    SPI_HandleTypeDef *spi;      ///< SPI bus handle
    GPIO_TypeDef *nssPort;       ///< Chip select GPIO port
    uint16_t nssPin;             ///< Chip select GPIO pin
    LoRa_Config_t config;        ///< LoRa configuration parameters
} LoRa_Handle_t;

/* Function prototypes */
uint8_t LoRa_Init(LoRa_Handle_t *handle);
void LoRa_SetConfig(LoRa_Handle_t *handle, LoRa_Config_t *config);
void LoRa_Transmit(LoRa_Handle_t *handle, void *data, uint8_t len);
uint8_t LoRa_Receive(LoRa_Handle_t *handle, void *rxData, uint8_t *len);
void LoRa_EnableDIO0Interrupt(LoRa_Handle_t *handle, uint8_t irqMapping);
void LoRa_DisableDIO0Interrupt(LoRa_Handle_t *handle);

#endif /* LORA_H */
