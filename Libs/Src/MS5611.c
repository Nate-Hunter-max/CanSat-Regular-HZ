/**
 * @file MS5611.c
 * @brief Driver for MS5611 pressure sensor via SPI interface
 */

#include "MS5611.h"
#include <math.h>
#include <string.h>

static SPI_HandleTypeDef *spi;
static GPIO_TypeDef *_NSS_Port;
static uint16_t _NSS_Pin;
static uint8_t overSamplimg = 0; ///< Format: |0|OS_Press[4:6]|OS_temp[1:3]|0|
static struct MS5611_CalData calData;
static const uint8_t delayArr[5] = { 1, 2, 3, 5, 9 };

/**
 * @brief Send command to MS5611
 */
static void MS5611_SendCmd(uint8_t cmd) {
    _NSS_Port->ODR &= ~_NSS_Pin;
    HAL_SPI_Transmit(spi, &cmd, 1, 1000);
    _NSS_Port->ODR |= _NSS_Pin;
}

/**
 * @brief Read 16-bit value from PROM address
 */
static uint16_t MS5611_ReadPROM(uint8_t addr) {
    uint8_t cmd = MS56_CMD_CAL_READ | (addr << 1);
    uint8_t rxBuf[2];
    _NSS_Port->ODR &= ~_NSS_Pin;
    HAL_SPI_Transmit(spi, &cmd, 1, 1000);
    HAL_SPI_Receive(spi, rxBuf, 2, 1000);
    _NSS_Port->ODR |= _NSS_Pin;
    return (rxBuf[0] << 8) | rxBuf[1];
}

/**
 * @brief Read ADC result (24-bit)
 */
static void MS5611_GetADC(uint8_t cmd, uint32_t *res) {
    uint8_t rxBuf[3];
    uint8_t convCmd = cmd | ((overSamplimg >> 3) & 0b1110);
    MS5611_SendCmd(convCmd);

    if(cmd & (1 << 4)) HAL_Delay(delayArr[(overSamplimg >> 1) & 0b111]);
    else HAL_Delay(delayArr[(overSamplimg >> 4) & 0b111]);

    _NSS_Port->ODR &= ~_NSS_Pin;
    uint8_t adcCmd = MS56_CMD_ADC_READ;
    HAL_SPI_Transmit(spi, &adcCmd, 1, 1000);
    HAL_SPI_Receive(spi, rxBuf, 3, 1000);
    _NSS_Port->ODR |= _NSS_Pin;

    *res = ((uint32_t)rxBuf[0] << 16) | ((uint32_t)rxBuf[1] << 8) | rxBuf[2];
}

/**
 * @brief CRC4 check from AN520 app note
 */
static uint8_t MS5611_CRC4(uint16_t *prom) {
    uint16_t n_rem = 0;
    prom[7] &= 0xFF00;
    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 0) n_rem ^= (prom[cnt >> 1] >> 8);
        else n_rem ^= (prom[cnt >> 1] & 0x00FF);
        for (int n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
            else n_rem <<= 1;
        }
    }
    return (n_rem >> 12) & 0xF;
}

/**
 * @brief Initialize MS5611 sensor
 * @param spiHandle SPI handle
 * @param NSS_Port Chip select GPIO port
 * @param NSS_Pin Chip select GPIO pin
 * @return 1 if success, 0 if CRC fails
 */
uint8_t MS5611_Init(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *NSS_Port, uint16_t NSS_Pin) {
    uint16_t rawCal[8];
    spi = spiHandle;
    _NSS_Port = NSS_Port;
    _NSS_Pin = NSS_Pin;

    MS5611_SendCmd(MS56_CMD_RESET);
    HAL_Delay(3);

    for(uint8_t i = 0; i < 8; i++) {
        rawCal[i] = MS5611_ReadPROM(i);
    }

    if (MS5611_CRC4(rawCal) != (rawCal[7] & 0xF))
        return 0;

    calData.SENS_T1 = rawCal[1] * 32768.0;
    calData.OFF_T1 = rawCal[2] * 65536.0;
    calData.TCS = rawCal[3] / 256.0;
    calData.TCO = rawCal[4] / 128.0;
    calData.T_REF = rawCal[5] * 256.0;
    calData.TEMPSENS = rawCal[6] / 8388608.0;
    return 1;
}

/**
 * @brief Set oversampling ratio
 * @param tempOS Oversampling for temperature (0-4)
 * @param pressOS Oversampling for pressure (0-4)
 */
void MS5611_SetOS(uint8_t tempOS, uint8_t pressOS) {
    overSamplimg = ((tempOS & 0x07) | ((pressOS & 0x07) << 3)) << 1;
}

/**
 * @brief Read compensated temperature and pressure
 * @param temp Pointer to output temperature in 0.01 °C
 * @param press Pointer to output pressure in Pa
 */
void MS5611_Read(int32_t *temp, uint32_t *press) {
    uint32_t D1, D2;
    float dT, off, sens, tempComp, dt2 = 0, off2 = 0, sens2 = 0;

    MS5611_GetADC(MS56_CMD_CONV_D1, &D1);
    MS5611_GetADC(MS56_CMD_CONV_D2, &D2);

    dT = D2 - calData.T_REF;
    *temp = 2000 + dT * calData.TEMPSENS;

    if(*temp < 2000) {
        tempComp = *temp - 2000.0;
        dt2 = dT * dT / 2147483648.0;
        off2 = 5.0 * tempComp * tempComp / 2.0;
        sens2 = 5.0 * tempComp * tempComp / 4.0;

        if(*temp < -1500) {
            tempComp = *temp + 1500.0;
            off2 += 7.0 * tempComp * tempComp;
            sens2 += 11.0 * tempComp * tempComp / 2.0;
        }
    }

    off = calData.OFF_T1 + calData.TCO * dT - off2;
    sens = calData.SENS_T1 + calData.TCS * dT - sens2;
    *temp -= dt2;
    *press = (D1 * sens / 2097152.0 - off) / 32768.0;
}

/**
 * @brief Compute altitude from pressure
 * @param pressure Measured pressure in Pa
 * @param seaLevelPressure Reference pressure at sea level in Pa
 * @return Altitude in centimeters
 */
int32_t MS5611_GetAltitude(uint32_t* pressure, uint32_t* seaLevelPressure) {
    return 4433000 * (1.0f - powf((float)(*pressure) / *seaLevelPressure, 0.1903f));
}
