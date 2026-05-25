#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initializes the MPU6050 on the provided I2C bus.
 *
 * Configures: gyro ±500°/s, accel ±2g, DLPF 42 Hz, sample rate 100 Hz.
 * Verifies WHO_AM_I on startup.
 *
 * @param bus  Master I2C bus handle (created in app_main)
 * @return ESP_OK on success
 */
esp_err_t mpu6050_init(i2c_master_bus_handle_t bus);

/**
 * @brief Reads accelerometer and gyroscope in SI units.
 *
 * @param ax, ay, az  Acceleration in m/s²
 * @param gx, gy, gz  Angular velocity in rad/s
 * @return ESP_OK on success
 */
esp_err_t mpu6050_read(float *ax, float *ay, float *az,
                       float *gx, float *gy, float *gz);
