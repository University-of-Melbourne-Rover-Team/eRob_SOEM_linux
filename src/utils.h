/**
 * Helper functions
 */

#ifndef URT_UTILS_H
#define URT_UTILS_H

#include <stdint.h>
#include <cmath>

constexpr int32_t EROB_COUNTS_PER_REV = 1 << 19; // 19-bit encoder
constexpr double EROB_DEG_PER_COUNT = 360.0 / EROB_COUNTS_PER_REV;

/**
 * Converts encoder count to degrees
 * @param {int32_t} count: positions counts or velocity counts/s
 * @return {double} angle in degrees
 */
double count2angle(int32_t count);

/**
 * Converts degrees to encoder count, rounded to nearest count
 * @param {float} angle: angular position in degrees, or angular velocity in degrees/s
 * @return {int32_t} counts
 */
int32_t angle2count(double angle);

#endif