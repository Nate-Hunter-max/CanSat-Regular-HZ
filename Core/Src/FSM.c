/**
 * @file FSM.c
 * @brief Implementation of the Finite State Machine for the flight controller
 * @author Nate Hunter
 * @date 2025-06-25
 * @version v1.2.0
 */

#include "FSM.h"
#include "user.h"
#include "GNGGA_Parser.h"
#include <math.h>

/** @brief System states */
enum states {
	INIT,       ///< Initialization state
	LORA_WAIT,  ///< Waiting for LoRa command
	MAIN,       ///< Main flight state
	LANDING,    ///< Landing state
	DUMP        ///< Memory dump state
};

//@formatter:off
/** @brief LoRa config*/
static LoRa_Config_t loraCfg = {
		.frequency = 433,
		.bandwidth = 0x08,
		.spreadingFactor = 7,
		.codingRate = 0b001,
		.headerMode = 0,
		.crcEnabled = 1,
		.lowDataRateOptimize = 0,
		.preambleLength = 6,
		.payloadLength = sizeof (TelemetryPacket),
		.txAddr = 255,
		.rxAddr = 0,
		.txPower = 0x01
};
//@formatter:on

static enum states lastState = LANDING;
static enum states currentState = INIT;
TelemetryRaw rawData;
TelemetryPacket txPack;
uint8_t rxbuf[32];
uint8_t rxlen;

static CircularBuffer cbPress = { .item_size = sizeof(rawData.press), .size = PRESS_BUFFER_LEN };

/** @brief Parser for BN220 */
GNGGA_Parser gps_parser;

/** @brief LoRa struct */
static LoRa_Handle_t lora = { .spi = &hspi1, .nssPort = LORA_NSS_GPIO_Port, .nssPin = LORA_NSS_Pin, };

/** @brief W25Q128 struct */
static W25Qx_Device wq = { .spi = &hspi1, .cs_port = WQ_NSS_GPIO_Port, .cs_pin = WQ_NSS_Pin, .capacity = 16777216 };

/** @brief LSM6 struct */
LSM6DS3_Handle lsm6 = { .spi = &hspi1, .cs_port = LSM_NSS_GPIO_Port, .cs_pin = LSM_NSS_Pin, .accelODR = LSM6DS3_ODR_26HZ,
		.gyroODR = LSM6DS3_ODR_12HZ5, .timeout = 100 };

/** @brief LIS3 struct */
LIS3MDL_Device lis3 = { .spi = &hspi1, .cs_port = LIS_NSS_GPIO_Port, .cs_pin = LIS_NSS_Pin, .output_data_rate = LIS3_ODR_20_HZ,
		.timeout = 100 };
/**
 * @brief Initialize all system components
 */
static void init_state(void) {
	static uint8_t errorCode = 0;
	if (currentState != lastState) {
		lastState = currentState;
		if (!MS5611_Init(&hspi1, MS_NSS_GPIO_Port, MS_NSS_Pin))
			errorCode = 1;
		if(!LIS3MDL_Init(&lis3))
			errorCode = 2;
		if (!LSM6DS3_Init(&lsm6, LSM6DS3_XL_16G, LSM6DS3_GYRO_2000DPS))
			errorCode = 3;
		if (!LoRa_Init(&lora))
			errorCode = 4;
		if (!microSD_Init())
			errorCode = 5;
		if (!W25Qx_Init(&wq))
			errorCode = 6;
		if (BN220_Init() != HAL_OK)
			errorCode = 7;

		if (!errorCode) {
			MS5611_SetOS(MS56_OSR_4096, MS56_OSR_4096);

			LoRa_EnableDIO0Interrupt(&lora, 0); //Enable RX_Done Interrupt

			CB_Init(&cbPress);
			GNGGA_Init(&gps_parser, &huart6);
			rawData.wqAdr = 0;
			currentState = LORA_WAIT;
		} else {
			Error(errorCode);
		}
	}
}

/**
 * @brief Handle LoRa waiting state
 */
static void lora_wait_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
		MS5611_Read(&rawData.temp, &rawData.press0);
	}

	if (!rawData.flags.ping) {
		if (HAL_GetTick() % 300 > 150)
			FlashLED(LED1_Pin);
		else
			FlashLED(LED2_Pin);
	} else if (HAL_GetTick() % 500 > 250)
		FlashLED(LED1_Pin);
	else
		FlashLED(0);

	if (rxlen > 0) {
		rxlen = 0;
		rawData.flags.command = 1;
		rawData.flags.ok = 1;

		if (rxbuf[0] == '0') {
			rawData.flags.ping = 1;
			ImuGetAll(&rawData);
			ImuSaveAll(&rawData, &txPack, &lora, &wq);
			return;
		}

		if (rawData.flags.ping) {
			switch (rxbuf[0]) {
				case '1':
					currentState = MAIN;
					break;
				case '2':
					currentState = DUMP;
					break;
				case '3':
					rawData.flags.wait = 1;
					rawData.flags.ok = 0;
					ImuGetAll(&rawData);
					ImuSaveAll(&rawData, &txPack, &lora, &wq);
					W25Qx_EraseChip(&wq);
					microSD_RemoveFile(SD_FILENAME);
					microSD_RemoveFile(SD_FILENAME_WQ);
					rawData.flags.wait = 0;
					rawData.flags.ok = 1;
					break;
				default:
					rawData.flags.ok = 0;
					break;
			}
		}
		ImuGetAll(&rawData);
		ImuSaveAll(&rawData, &txPack, &lora, &wq);
		rawData.flags.command = 0;
	}
}

/**
 * @brief Handle main flight state
 */
static void main_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
		for (uint8_t i = 0; i < 10; i++) {
			MS5611_Read(&rawData.temp, &rawData.press0);
			HAL_Delay(50);
		}
		MS5611_Read(&rawData.temp, &rawData.press0);
		StoreVectAbs(&rawData);
		rawData.time = HAL_GetTick();
	}

	BN220_TryGet(&gps_parser, &rawData);

	if (HAL_GetTick() - rawData.time >= DATA_PERIOD) {
		HAL_ADC_Start(&hadc1);
		ImuGetAll(&rawData);

		if (rawData.altitude > START_TH)
			rawData.flags.start = 1;

		if (rawData.flags.start && ADC1->DR < EJECT_TH)
			rawData.flags.eject = 1;

		if (rawData.flags.eject) {
			CB_Push(&cbPress, &rawData.press);
			if (CB_Diff(&cbPress, compare_uint32) < PRESS_LAND_DELTA) {
				rawData.flags.land = 1;
				currentState = LANDING;
			}
		}
		ImuSaveAll(&rawData, &txPack, &lora, &wq);
	}
}

/**
 * @brief Handle landing state
 */
static void landing_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
		rawData.time = HAL_GetTick();
	}

	BN220_TryGet(&gps_parser, &rawData);

	if (HAL_GetTick() - rawData.time >= DATA_PERIOD_LND) {
		ImuGetAll(&rawData);
		ImuSaveAll(&rawData, &txPack, &lora, &wq);
	}
}

/**
 * @brief Handle memory dump state
 */
static void dump_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
	}

	uint8_t buf[sizeof(TelemetryPacket)];
	for (uint32_t addr = 0; addr < 0xFFFFFF; addr += sizeof(TelemetryPacket)) {
		FlashLED(LED2_Pin);
		W25Qx_ReadData(&wq, addr, buf, sizeof(TelemetryPacket));
		if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF)
			break;
		LoRa_Transmit(&lora, buf, sizeof(TelemetryPacket));
		FlashLED(LED1_Pin);
		if (microSD_Write(buf, sizeof(TelemetryPacket), SD_FILENAME_WQ) != FR_OK) {
			FlashLED(LED_ERR_Pin);
			HAL_Delay(1000);
		}
	}
	LoRa_Transmit(&lora, "Done\n", 5);
	currentState = LORA_WAIT;
}

void FSM_Init(void) {
	lora.config = loraCfg;
	lastState = LANDING;
	currentState = INIT;
}

void FSM_Update(void) {
	switch (currentState) {
		case INIT:
			init_state();
			break;
		case LORA_WAIT:
			lora_wait_state();
			break;
		case MAIN:
			main_state();
			break;
		case LANDING:
			landing_state();
			break;
		case DUMP:
			dump_state();
		default:
			break;
	}
}

/**
 * @brief override weak UART callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart6) {
		GNGGA_UART_IRQHandler(&gps_parser);
	}
}

/**
 * @brief override weak EXTI callback
 *
 * @note handle PB15 (LoRa RXDone)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
		case LORA_DIO0_Pin:
			LoRa_Receive(&lora, rxbuf, &rxlen);
			break;
	}
}
