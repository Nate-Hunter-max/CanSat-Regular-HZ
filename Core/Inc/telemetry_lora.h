/**
 * @file telemetry_lora.h
 * @brief Telemetry and control data structures and conversion utilities for LoRa communication
 * @author Nate Hunter
 * @date 2025-06-10
 */

#ifndef TELEMETRY_LORA_H
#define TELEMETRY_LORA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Raw telemetry structure (full-precision).
 */
typedef struct {
	uint32_t time;          ///< Milliseconds from start
	int32_t temp;           ///< MS56 temperature (centigrade*10e2)
	uint32_t press;         ///< MS56 pressure (Pa)
	float magData[3];       ///< LIS3 mag (mG)
	float accelData[3];     ///< LSM6 accel (mG)
	float gyroData[3];      ///< LSM6 gyro (mdps)
	int32_t altitude;       ///< Altitude (zero at start, cm)
	float lat;              ///< Latitude from GPS
	float lon;              ///< Longitude from GPS
	uint8_t flags;          ///< Flags (0|0|0|0|Land|ResSys|Eject|Start)
	uint32_t press0;        ///< MS56 pressure at 0 Alt (Pa)
	float vectAbs;          ///< Absolute value of accel vector
	uint32_t wqAdr;         ///< WQ address
} TelemetryRaw;

/**
 * @brief Compressed telemetry structure (LoRa-optimized).
 * @note sizeof(TelemetryPacket) = 36
 */
typedef struct __attribute__((packed)) {
	uint32_t time_ms :24;
	int16_t temp_cC :14;
	uint16_t pressPa :16;
	int16_t mag[3];
	int16_t accel[3];
	int16_t gyro[3];
	int32_t altitude_cm :20;
	int32_t lat_1e7 :30;
	int32_t lon_1e7 :30;
	uint8_t flags;
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
