#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initializes the 4 quadrature encoder channels (4x resolution).
 *
 * Configures PIN_ENC_L_A/B (left) and PIN_ENC_R_A/B (right) as ANYEDGE interrupts.
 * Call once from app_main AFTER gpio_install_isr_service().
 *
 * @note Pin assignments are defined in config.h.
 *       If encoders are open-collector, add external 10 kΩ pull-ups to 3.3V.
 *
 * @return ESP_OK on success
 */
esp_err_t encoders_init(void);

/**
 * @brief Reads the tick counters of both wheels.
 *
 * Atomic read. Values are signed: positive = forward, negative = reverse.
 * Safe to call from any FreeRTOS task.
 *
 * @param left  Pointer to the left wheel counter
 * @param right Pointer to the right wheel counter
 */
void encoders_get(int32_t *left, int32_t *right);

/**
 * @brief Resets both counters to zero atomically.
 */
void encoders_reset(void);
