/**
 * @file telemetry_lora.h
 * @brief Telemetry and control data structures and conversion utilities for LoRa communication
 * @author Nate Hunter
 * @date 2025-06-10
 * @version 1.1.0
 */

#ifndef TELEMETRY_LORA_H
#define TELEMETRY_LORA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((packed)) {
	unsigned err :1;     ///< Error flag (bit 0)
	unsigned wait :1;    ///< Waiting state flag (bit 1)
	unsigned ok :1;      ///< OK/Ready flag (bit 2)
	unsigned ping :1;    ///< Ping request/response (bit 3)
	unsigned command :1; ///< Command received (bit 4)
	unsigned land :1;    ///< Landing trigger (bit 5)
	unsigned eject :1;   ///< Ejection trigger (bit 6)
	unsigned start :1;   ///< System start flag (bit 7)
} SystemFlags;

/**
 * @brief Raw telemetry structure (full-precision).
 */
typedef struct {
	uint32_t time;          ///< Milliseconds from start
	int32_t temp;           ///< MS56 temperature (centigrade*10e2)
	uint32_t press;         ///< MS56 pressure (Pa)
	float magData[3];       ///< LIS3 mag (mG)
	float accelData[3];     ///< LSM6 accel (mG)
	float gyroData[3];      ///< LSM6 gyro (dps*10)
	int32_t altitude;       ///< Altitude (zero at start, cm)
	float lat;              ///< Latitude from GPS
	float lon;              ///< Longitude from GPS
	SystemFlags flags;    ///< Status flags (8 bit)
	uint32_t press0;        ///< MS56 pressure at 0 altitude (Pa)
	float vectAbs;          ///< Absolute value of acceleration vector (mG)
	uint32_t wqAdr;         ///< WQ address (if applicable)
} TelemetryRaw;

/**
 * @brief Compressed telemetry structure (LoRa-optimized).
 * @note sizeof(TelemetryPacket) = 36
 */
typedef struct __attribute__((packed)) {
	uint32_t time_ms :24;       ///< Milliseconds from start (24-bit to save space)
	int16_t temp_cC :14;        ///< Temperature in centiCelsius (14-bit, range ±819.2°C)
	uint16_t pressPa :16;       ///< Pressure offset from 60000 in Pascals (16-bit, 0-125535 Pa)
	int16_t mag[3];             ///< Magnetometer data (mG, packed as 16-bit integers)
	int16_t accel[3];           ///< Accelerometer data (mG, packed as 16-bit integers)
	int16_t gyro[3];            ///< Gyroscope data (dps*10, packed as 16-bit integers)
	int32_t altitude_cm :20;    ///< Altitude in cm (20-bit, ~±524km range)
	int32_t lat_1e7 :30;        ///< Latitude * 1e7 (30-bit, ~±0.1° precision)
	int32_t lon_1e7 :30;        ///< Longitude * 1e7 (30-bit, ~±0.1° precision)
	SystemFlags flags;        ///< Status flags (8 bit)
} TelemetryPacket;

/**
 * @brief Converts raw telemetry to a compressed telemetry packet.
 * @param in Pointer to the raw telemetry
 * @param out Pointer to the packed telemetry structure
 */
void Telemetry_convertRawToPacket(const TelemetryRaw *in, TelemetryPacket *out);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_LORA_H
