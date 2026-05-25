#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initializes the INA219 on the provided I2C bus.
 *
 * Configures: 32V bus range, shunt gain /8 (±320mV), 12-bit resolution,
 * continuous mode. Shunt resistance defined by INA219_SHUNT_OHMS in config.h.
 *
 * @param bus  Master I2C bus handle (created in app_main)
 * @return ESP_OK on success
 */
esp_err_t ina219_init(i2c_master_bus_handle_t bus);

/**
 * @brief Reads bus voltage and current.
 *
 * @param voltage  Bus voltage in volts
 * @param current  Current in amperes (positive = from source)
 * @return ESP_OK on success
 */
esp_err_t ina219_read(float *voltage, float *current);
