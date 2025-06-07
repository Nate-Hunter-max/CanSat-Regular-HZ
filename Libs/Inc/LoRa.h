/**
 * @file LoRa.h
 * @brief LoRa module driver header file.
 *
 * This file contains definitions and functions for controlling a LoRa radio module.
 *
 * @author [Nate Hunter]
 * @date [11.05.2025]
 * @version 1.1
 */

#pragma once
#include "main.h"

/**
 * @defgroup LoRa_Registers LoRa Register Definitions
 * @brief Memory-mapped registers of the LoRa module
 * @{
 */
#define LORA_Fifo                0x00  ///< FIFO register address
#define LORA_RegOpMode           0x01  ///< Operation mode control register
#define LORA_RegFrMsd            0x06  ///< Frequency setting MSB register
#define LORA_RegFrMid            0x07  ///< Frequency setting MID register
#define LORA_RegFrLsd            0x08  ///< Frequency setting LSB register
#define LORA_RegPaConfig         0x09  ///< Power amplifier configuration
#define LORA_RegFifoAddrSpi      0x0D  ///< FIFO address pointer in SPI mode
#define LORA_RegFifoTxBaseAddr   0x0E  ///< FIFO transmit base address
#define LORA_RegFifoRxBaseAddr   0x0F  ///< FIFO receive base address
#define LORA_RegLna              0x0C  ///< Low noise amplifier control
#define LORA_RegFifoRxCurrentAdr 0x10  ///< Current FIFO receive address
#define LORA_RegIrqFlagsMask     0x11  ///< IRQ flags mask register
#define LORA_RegIrqFlags         0x12  ///< IRQ flags register
#define LORA_RegRxNbBytes        0x13  ///< Number of received bytes
#define LORA_RegModemConfig1     0x1D  ///< Modem configuration 1
#define LORA_RegModemConfig2     0x1E  ///< Modem configuration 2
#define LORA_RegPreambleMsd      0x20  ///< Preamble length MSB
#define LORA_RegPreambleLsd      0x21  ///< Preamble length LSB
#define LORA_RegPayloadLenght    0x22  ///< Payload length
#define LORA_RegPayloadMaxLenght 0x23  ///< Maximum payload length
#define LORA_RegFifoRxByteAddr   0x25  ///< FIFO receive byte address
#define LORA_RegModemConfig3     0x26  ///< Modem configuration 3
#define LORA_RegVersion          0x42  ///< Chip version
/** @} */

/**
 * @defgroup LoRa_Flags LoRa Status Flags
 * @brief Bitmask definitions for IRQ flags
 * @{
 */
#define LORA_Flag_CadDetected       0x01  ///< Channel activity detected flag
#define LORA_Flag_FhssChangeChannel 0x02  ///< Frequency hop change channel flag
#define LORA_Flag_CadDone           0x04  ///< Channel activity detection done
#define LORA_Flag_TxDone            0x08  ///< Transmission complete flag
#define LORA_Flag_ValidHeader       0x10  ///< Valid header received flag
#define LORA_Flag_PayloadCrcError   0x20  ///< Payload CRC error flag
#define LORA_Flag_RxDone            0x40  ///< Reception complete flag
#define LORA_Flag_RxTimeout         0x80  ///< Reception timeout flag
/** @} */

/**
 * @defgroup LoRa_SPI_Bits LoRa SPI Control Bits
 * @brief SPI transaction control bits
 * @{
 */
#define LORA_WRITE_BIT (1<<7)  ///< SPI write operation bit
#define LORA_READ_BIT  0x76    ///< SPI read operation value
/** @} */

/**
 * @struct LoRaConfig
 * @brief LoRa module configuration structure
 *
 * Contains all configurable parameters for the LoRa radio module.
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
} LoRaConfig;

/**
 * @struct LoRa_HandleTypeDef
 * @brief LoRa module handle structure
 *
 * Contains hardware interface information and configuration for the LoRa module.
 */
typedef struct {
    SPI_HandleTypeDef *spi;      ///< SPI bus handle
    GPIO_TypeDef *NSS_Port;      ///< Chip select GPIO port
    uint16_t NSS_Pin;            ///< Chip select GPIO pin
    LoRaConfig config;           ///< LoRa configuration parameters
} LoRa_HandleTypeDef;

/**
 * @brief Initializes the LoRa module
 * @param handle Pointer to LoRa handle structure
 * @return uint8_t Status of initialization (1 = success, 0 = error)
 */
uint8_t LoRa_Init(LoRa_HandleTypeDef *handle);

/**
 * @brief Configures the LoRa module parameters
 * @param handle Pointer to LoRa handle structure
 * @param config Pointer to configuration structure
 */
void LoRa_SetConfig(LoRa_HandleTypeDef *handle, LoRaConfig *config);

/**
 * @brief Transmits data via LoRa
 * @param handle Pointer to LoRa handle structure
 * @param data Pointer to transmit data buffer
 * @param len Length of data to transmit
 */
void LoRa_Transmit(LoRa_HandleTypeDef *handle, void *data, uint8_t len);

/**
 * @brief Receives data via LoRa
 * @param handle Pointer to LoRa handle structure
 * @param rxData Pointer to receive data buffer
 * @param len Pointer to store received data length
 * @return uint8_t Reception status (0 = success, non-zero = error)
 */
uint8_t LoRa_Receive(LoRa_HandleTypeDef *handle, void *rxData, uint8_t *len);
