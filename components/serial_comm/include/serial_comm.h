#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initializes UART0 at 115200 baud with the ESP-IDF driver.
 * @return ESP_OK on success
 */
esp_err_t serial_comm_init(void);

/**
 * @brief Reads available bytes from UART0 and parses complete commands.
 *
 * Call periodically from task_serial_rx (non-blocking).
 * Updates internal setpoints accessible via serial_comm_get_setpoints().
 */
void serial_comm_process_rx(void);

/**
 * @brief Returns the last setpoints received from the Pi.
 *
 * Atomic read. CMD_STOP resets both setpoints to 0.
 *
 * @param left   Left wheel setpoint in rad/s
 * @param right  Right wheel setpoint in rad/s
 */
void serial_comm_get_setpoints(float *left, float *right);

/** @brief Sends "ENC <ticks_L> <ticks_R>\n" */
void serial_comm_send_enc(int32_t left, int32_t right);

/** @brief Sends "IMU <ax> <ay> <az> <gx> <gy> <gz>\n" */
void serial_comm_send_imu(float ax, float ay, float az,
                          float gx, float gy, float gz);

/** @brief Sends "BATT <voltage> <current> <percent>\n" */
void serial_comm_send_batt(float voltage, float current, float percent);
