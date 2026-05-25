#pragma once
#include <stdbool.h>

/**
 * @brief Computes the LiPo 3S battery percentage via a lookup table.
 *
 * Only updates the value if current is below the disturbance threshold.
 * Returns the last valid value otherwise.
 *
 * @param voltage   Voltage measured by INA219 (V)
 * @param current   Current measured by INA219 (A)
 * @param percent   Output: percentage [0.0, 100.0]
 * @return true if the value was just recomputed, false if from cache
 */
bool battery_get_percent(float voltage, float current, float *percent);

/** Resets the cached value (e.g. after recharge) */
void battery_reset(void);
