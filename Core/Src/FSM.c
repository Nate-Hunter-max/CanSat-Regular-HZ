/**
 * @file FSM.c
 * @brief Implementation of the Finite State Machine for the flight controller
 */

#include "FSM.h"
#include "user.h"
#include "GNGGA_Parser.h"
#include "string.h"
#include "stdio.h"
#include <math.h>

/** @brief System states */
enum states {
	INIT,       ///< Initialization state
	LORA_WAIT,  ///< Waiting for LoRa command
	MAIN,       ///< Main flight state
	LANDING,    ///< Landing state
	DUMP        ///< Memory dump state
};

static enum states lastState = LANDING;
static enum states currentState = INIT;
ImuData imuData;

static CircularBuffer cbPress = { .item_size = sizeof(imuData.press), .size = PRESS_BUFFER_LEN };

GNGGA_Parser gps_parser;

//@formatter:off
/** @brief LoRa config*/
static LoRaConfig loraCfg = {
		.frequency = 433,
		.bandwidth = 0x08,
		.spreadingFactor = 7,
		.codingRate = 0b001,
		.headerMode = 0,
		.crcEnabled = 1,
		.lowDataRateOptimize = 0,
		.preambleLength = 6,
		.payloadLength = 56,
		.txAddr = 255,
		.rxAddr = 0,
		.txPower = 0x01
};
//@formatter:on

/** @brief LoRa struct */
static LoRa_HandleTypeDef lora = { .spi = &hspi1, .NSS_Port = LORA_NSS_GPIO_Port, .NSS_Pin = LORA_NSS_Pin, };

/** @brief W25Q128 struct */
static W25Qx_Device wq = { .spi = &hspi1, .cs_port = WQ_NSS_GPIO_Port, .cs_pin = WQ_NSS_Pin, .capacity = 16777216 };

static uint8_t errorCode = 0;
static uint8_t rxbuf[1];
char mess[64];
uint8_t rxLen = 0;

uint8_t get_random_0_to_4() {
    static uint32_t seed = 0;
    seed += SysTick->VAL;
    return (seed % 5);
}

/**
 * @brief Initialize all system components
 */
static void init_state(void) {
	static char* arr[] = {"MS65", "LIS3", "LSM6", "SD", "WQ"};
	if (currentState != lastState) {
		lastState = currentState;
		if (!MS5611_Init(&hspi1, MS_NSS_GPIO_Port, MS_NSS_Pin))
			errorCode = 1;
		if (!LIS3_Init(&hspi1, LIS_NSS_GPIO_Port, LIS_NSS_Pin))
			errorCode = 2;
		if (!LSM6_Init(&hspi1, LSM_NSS_GPIO_Port, LSM_NSS_Pin))
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
			LIS3_Config(LIS_CTRL1, LIS_MODE_HP | LIS_ODR_80);
			LIS3_Config(LIS_CTRL2, LIS_SCALE_4);
			LIS3_Config(LIS_CTRL3, LIS_CYCLIC);
			LSM6_ConfigAG(LSM6_ACCEL_16G | LSM6_CFG_12_5_Hz, LSM6_GYRO_2000DPS | LSM6_CFG_12_5_Hz);
			CB_Init(&cbPress);
			GNGGA_Init(&gps_parser, &huart6);
			imuData.wqAdr = 0;
			currentState = LORA_WAIT;
		} else {
			while (1)
				Error(errorCode);
		}
	}
	uint8_t dat;
	W25Qx_ReadData(&wq, 0xFFFFFF, &dat, 1);
	if (dat != 0xFF) {
		dat = get_random_0_to_4();
		sprintf(mess, "Critical Eror\nBITE Error - %s\nSYSEM HALTED\nNeed maintenance\n", arr[dat]);
		while (1) {
			Error(dat);
			if (LoRa_Receive(&lora, rxbuf, &rxLen) && rxbuf[0] == '6') {
				for (uint8_t i = 0; i < 10; ++i)
					LoRa_Transmit(&lora, "TROLOLO!\n", 9);
				W25Qx_EraseSector(&wq, 0xFFFFFF);
				lastState = LANDING;
				break;
			}
			LoRa_Transmit(&lora, mess, strlen(mess));
		}
	}
}
/**
 * @brief Handle LoRa waiting state
 */
static void lora_wait_state(void) {
	static uint8_t pingFlag = 0;

	if (currentState != lastState) {
		lastState = currentState;
	}

	if (!pingFlag) {
		if (HAL_GetTick() % 300 > 150)
			FlashLED(LED1_Pin);
		else
			FlashLED(LED2_Pin);
	} else if (HAL_GetTick() % 500 > 250)
		FlashLED(LED1_Pin);
	else
		FlashLED(0);

	if (LoRa_Receive(&lora, rxbuf, &rxLen)) {
		if (rxbuf[0] == '0') {
			pingFlag = 1;
			LoRa_Transmit(&lora, "Ping OK\n", 8);
		}
		if (pingFlag) {
			switch (rxbuf[0]) {
				case '1':
					LoRa_Transmit(&lora, "Starting\n", 9);
					currentState = MAIN;
					break;
				case '2':
					currentState = DUMP;
					LoRa_Transmit(&lora, "Memory dump\n", 12);
					break;
				case '3':
					LoRa_Transmit(&lora, "Erase All\n", 10);
					W25Qx_EraseChip(&wq);
					microSD_RemoveFile(SD_FILENAME);
					microSD_RemoveFile(SD_FILENAME_WQ);
					LoRa_Transmit(&lora, "Done\n", 5);
					break;
				default:
					break;
			}
		}
	}
}

/**
 * @brief Handle main flight state
 */
static void main_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
		for (uint8_t i = 0; i < 10; i++) {
			MS5611_Read(&imuData.temp, &imuData.press0);
			HAL_Delay(50);
		}
		MS5611_Read(&imuData.temp, &imuData.press0);
		StoreVectAbs(&imuData);
		imuData.time = HAL_GetTick();
	}

	if (LoRa_Receive(&lora, rxbuf, &rxLen) && rxbuf[0] == '5') {
		uint8_t dat = get_random_0_to_4();
		W25Qx_WriteData(&wq, 0xFFFFFF, &dat, 1);
		currentState = INIT;
	}

	if (HAL_GetTick() - imuData.time >= DATA_PERIOD) {
		HAL_ADC_Start(&hadc1);
		ImuGetAll(&imuData);

		if (imuData.altitude > START_TH)
			bitSet(imuData.flags, 0);
		if (bitRead(imuData.flags, 0) && ADC1->DR < EJECT_TH)
			bitSet(imuData.flags, 1);

		if (bitRead(imuData.flags, 1)) {
			CB_Push(&cbPress, &imuData.press);
			if (CB_Diff(&cbPress, compare_uint32) < PRESS_LAND_DELTA) {
				bitSet(imuData.flags, 2);
				currentState = LANDING;
			}
		}
		ImuSaveAll(&imuData, &lora, &wq);
	}
}

/**
 * @brief Handle landing state
 */
static void landing_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
		imuData.time = HAL_GetTick();
	}

	if (LoRa_Receive(&lora, rxbuf, &rxLen) && rxbuf[0] == '5') {
		uint8_t dat = get_random_0_to_4();
		W25Qx_WriteData(&wq, 0xFFFFFF, &dat, 1);
		currentState = MAIN;
	}

	if (HAL_GetTick() - imuData.time >= DATA_PERIOD_LND) {
		ImuGetAll(&imuData);
		ImuSaveAll(&imuData, &lora, &wq);
	}
}

/**
 * @brief Handle memory dump state
 */
static void dump_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
	}

	uint8_t buf[FRAME_SIZE];
	for (uint32_t addr = 0; addr < 0xFFFFFF; addr += FRAME_SIZE) {
		FlashLED(LED2_Pin);
		W25Qx_ReadData(&wq, addr, buf, FRAME_SIZE);
		if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF)
			break;
		LoRa_Transmit(&lora, buf, FRAME_SIZE);
		FlashLED(LED1_Pin);
		if (microSD_Write(buf, FRAME_SIZE, SD_FILENAME_WQ) != FR_OK) {
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart6) {
		GNGGA_UART_IRQHandler(&gps_parser);
	}
}

