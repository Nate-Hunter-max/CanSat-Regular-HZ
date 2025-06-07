/*
 * LoRa_new.h
 *
 *  Created on: Nov 29, 2024
 *      Author: Nate Hunter
 */
#include "LoRa.h"

static uint8_t txbuf[1];

static inline void LoRa_WriteReg(LoRa_HandleTypeDef *handle, uint8_t reg, uint8_t *data, uint8_t count);
static inline void LoRa_WriteRegByte(LoRa_HandleTypeDef *handle, uint8_t reg, uint8_t data);
static inline uint8_t LoRa_ReadRegByte(LoRa_HandleTypeDef *handle, uint8_t reg);
static inline void LoRa_ReadReg(LoRa_HandleTypeDef *handle, uint8_t reg, uint8_t *data, uint8_t count);

uint8_t LoRa_Init(LoRa_HandleTypeDef *handle) {
    uint8_t id;
    txbuf[0] = LORA_RegVersion;
    handle->NSS_Port->ODR &= ~handle->NSS_Pin;
    HAL_SPI_Transmit(handle->spi, txbuf, 1, 1000);
    HAL_SPI_Receive(handle->spi, &id, 1, 1000);
    handle->NSS_Port->ODR |= handle->NSS_Pin;

    if (id != 0x12) return 0;

    LoRa_WriteRegByte(handle, LORA_RegOpMode, 0x00); // Sleep mode
    LoRa_SetConfig(handle, &handle->config);        // Применить настройки
    return 1;
}

void LoRa_SetConfig(LoRa_HandleTypeDef *handle, LoRaConfig *config) {
    LoRa_WriteRegByte(handle, LORA_RegOpMode, 0b10000000); // Standby mode
    handle->config = *config;

	uint32_t Fr = config->frequency * 16384, PrL = config->preambleLength;
	Fr = ((Fr << 16) & 0xFF0000) | ((Fr << 0) & 0x00FF00) | ((Fr >> 16) & 0x0000FF);
	PrL = ((PrL << 8) & 0xFF00) | ((PrL >> 8) & 0x00FF);

	LoRa_WriteReg(handle, LORA_RegFrMsd, (uint8_t*) &Fr, 3);
	LoRa_WriteRegByte(handle, LORA_RegModemConfig1, (config->bandwidth << 4) | (config->codingRate << 1) | config->headerMode);
	LoRa_WriteRegByte(handle, LORA_RegModemConfig2, (uint8_t) (config->spreadingFactor << 4) | (config->crcEnabled << 2));
	LoRa_WriteRegByte(handle, LORA_RegModemConfig3, config->lowDataRateOptimize << 3);
	LoRa_WriteReg(handle, LORA_RegPreambleMsd, (uint8_t*) &PrL, 2);
	LoRa_WriteRegByte(handle, LORA_RegPayloadLenght, config->payloadLength);
	LoRa_WriteRegByte(handle, LORA_RegPayloadMaxLenght, config->payloadLength);
	LoRa_WriteRegByte(handle, LORA_RegLna, (0x01 << 5) | 0x03);
	LoRa_WriteRegByte(handle, LORA_RegPaConfig, (1 << 7) | (0x07 << 4) | config->txPower);
	LoRa_WriteRegByte(handle, LORA_RegFifoTxBaseAddr, config->txAddr);
	LoRa_WriteRegByte(handle, LORA_RegFifoRxBaseAddr, config->rxAddr);
	LoRa_WriteRegByte(handle, LORA_RegOpMode, 0b10000001 | (1 << 3));
	LoRa_WriteRegByte(handle, LORA_RegOpMode, 0x05);

}

void LoRa_Transmit(LoRa_HandleTypeDef *handle, void *data, uint8_t len) {
    LoRa_WriteRegByte(handle, LORA_RegOpMode, 0x01); // Standby
    LoRa_WriteRegByte(handle, LORA_RegPayloadLenght, len);

    LoRa_WriteRegByte(handle, LORA_RegFifoAddrSpi, handle->config.txAddr);
    txbuf[0] = LORA_Fifo | LORA_WRITE_BIT;
    handle->NSS_Port->ODR &= ~handle->NSS_Pin;
    HAL_SPI_Transmit(handle->spi, txbuf, 1, 1000);
    HAL_SPI_Transmit(handle->spi, data, len, 1000);
    handle->NSS_Port->ODR |= handle->NSS_Pin;

    LoRa_WriteRegByte(handle, LORA_RegOpMode, 0x03); // Transmit
    while (!(LoRa_ReadRegByte(handle, LORA_RegIrqFlags) & LORA_Flag_TxDone));
    LoRa_WriteRegByte(handle, LORA_RegIrqFlags, LORA_Flag_TxDone); // Сбросить флаг
    LoRa_WriteRegByte(handle, LORA_RegOpMode, 0x05); // RxCont
}

uint8_t LoRa_Receive(LoRa_HandleTypeDef *handle, void *rxData, uint8_t *len) {
    if (!(LoRa_ReadRegByte(handle, LORA_RegIrqFlags) & LORA_Flag_RxDone)) return 0;

    uint8_t currentAdr = LoRa_ReadRegByte(handle, LORA_RegFifoRxCurrentAdr);
    LoRa_WriteRegByte(handle, LORA_RegFifoAddrSpi, currentAdr);

    *len = LoRa_ReadRegByte(handle, LORA_RegRxNbBytes);
    LoRa_ReadReg(handle, LORA_Fifo, rxData, *len);
    LoRa_WriteRegByte(handle, LORA_RegIrqFlags, LORA_Flag_RxDone);
    return 1;
}

static inline void LoRa_WriteReg(LoRa_HandleTypeDef *handle, uint8_t reg, uint8_t *data, uint8_t count) {
    txbuf[0] = reg | LORA_WRITE_BIT;
    handle->NSS_Port->ODR &= ~handle->NSS_Pin;
    HAL_SPI_Transmit(handle->spi, txbuf, 1, 1000);
    HAL_SPI_Transmit(handle->spi, data, count, 1000);
    handle->NSS_Port->ODR |= handle->NSS_Pin;
}

static inline void LoRa_WriteRegByte(LoRa_HandleTypeDef *handle, uint8_t reg, uint8_t data) {
    LoRa_WriteReg(handle, reg, &data, 1);
}

static inline uint8_t LoRa_ReadRegByte(LoRa_HandleTypeDef *handle, uint8_t reg) {
    txbuf[0] = reg & ~LORA_WRITE_BIT;
    uint8_t res = 0;
    handle->NSS_Port->ODR &= ~handle->NSS_Pin;
    HAL_SPI_Transmit(handle->spi, txbuf, 1, 1000);
    HAL_SPI_Receive(handle->spi, &res, 1, 1000);
    handle->NSS_Port->ODR |= handle->NSS_Pin;
    return res;
}

static inline void LoRa_ReadReg(LoRa_HandleTypeDef *handle, uint8_t reg, uint8_t *data, uint8_t count) {
    txbuf[0] = reg & ~LORA_WRITE_BIT;
    handle->NSS_Port->ODR &= ~handle->NSS_Pin;
    HAL_SPI_Transmit(handle->spi, txbuf, 1, 1000);
    HAL_SPI_Receive(handle->spi, data, count, 1000);
    handle->NSS_Port->ODR |= handle->NSS_Pin;
}
