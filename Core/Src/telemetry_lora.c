/**
 * @file telemetry_lora.c
 * @brief Implementation of telemetry/control data conversion utilities for LoRa communication
 * @author Nate Hunter
 * @date 2025-06-10
 */

#include "telemetry_lora.h"
#include <math.h>
#include <string.h>

/**
 * @brief Clamp signed value to specified bit width.
 * @param val Value to clamp
 * @param bits Number of bits (e.g., 14)
 * @return Clamped value
 */
static int32_t clampSigned(int32_t val, uint8_t bits) {
	int32_t max = (1 << (bits - 1)) - 1;
	int32_t min = -(1 << (bits - 1));
	if (val > max)
		return max;
	if (val < min)
		return min;
	return val;
}

/**
 * @brief Clamp unsigned value to specified bit width.
 * @param val Value to clamp
 * @param bits Number of bits (e.g., 16)
 * @return Clamped value
 */
static uint32_t clampUnsigned(uint32_t val, uint8_t bits) {
	uint32_t max = (1U << bits) - 1;
	return (val > max) ? max : val;
}

void Telemetry_convertRawToPacket(const TelemetryRaw *in, TelemetryPacket *out) {
	out->time_ms = clampUnsigned(in->time, 24);
	out->temp_cC = clampSigned(in->temp / 10, 14); // from x100 to x10
	out->pressPa = clampUnsigned((in->press > 60000) ? in->press - 60000 : 0, 16);

	for (int i = 0; i < 3; ++i) {
		out->mag[i] = clampSigned((int32_t) roundf(in->magData[i]*1000), 14);
		out->accel[i] = clampSigned((int32_t) roundf(in->accelData[i]*1000), 15);
		out->gyro[i] = clampSigned((int32_t) roundf(in->gyroData[i]*10), 16);
	}

	out->altitude_cm = clampSigned(in->altitude, 20);
	out->lat_1e7 = clampSigned((int32_t) (in->lat * 1e7f), 30);
	out->lon_1e7 = clampSigned((int32_t) (in->lon * 1e7f), 30);
	out->flags = in->flags;
}
