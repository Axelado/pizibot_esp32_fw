#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initializes 4 LEDC channels (20 kHz, 10-bit, 2 per motor) for IN1/IN2 PWM control.
 *
 * Must be called once from app_main before any other motor function.
 * Leaves motors stopped (brake).
 *
 * @return ESP_OK on success
 */
esp_err_t motors_init(void);

/**
 * @brief Applies a speed setpoint in rad/s to each wheel (open-loop).
 *
 * Direct rad/s → PWM duty conversion via WHEEL_MAX_RADS, no encoder feedback.
 * Reserved for manual tests. In normal operation, use motors_set_raw()
 * from the PID task that closes the loop.
 * Value is clamped to ±WHEEL_MAX_RADS before conversion.
 * Positive = forward, negative = reverse.
 *
 * @param left_rads  Left wheel speed in rad/s
 * @param right_rads Right wheel speed in rad/s
 */
void motors_set(float left_rads, float right_rads);

/**
 * @brief Applies a raw PWM duty to each wheel.
 *
 * Range: [-1000, 1000]. Positive = forward, negative = reverse, 0 = brake.
 *
 * @param left  Left wheel duty
 * @param right Right wheel duty
 */
void motors_set_raw(int left, int right);

/**
 * @brief Stops both motors (active brake: IN1=IN2=L).
 */
void motors_stop(void);
