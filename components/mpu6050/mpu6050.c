#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MPU6050";

/* Registers */
#define MPU_ADDR        0x68
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_PWR_MGMT1   0x6B
#define REG_WHO_AM_I    0x75
#define REG_ACCEL_XOUT  0x3B  /* start of 14-byte block */

/* Sensitivities for the selected configuration */
#define GYRO_SENS   65.5f   /* LSB/(°/s) for ±500°/s */
#define ACCEL_SENS  16384.0f /* LSB/g     for ±2g    */
#define G_MPS2      9.80665f
#define DEG2RAD     (3.14159265f / 180.0f)

/* Gyro bias calibration: number of samples averaged at init, 5 ms apart */
#define GYRO_CAL_SAMPLES  200

static i2c_master_dev_handle_t s_dev;
static float s_gyro_bias[3] = { 0 };  /* rad/s, subtracted from every reading */

/* ---------------------------------------------------------------------------
 * I2C helpers
 * --------------------------------------------------------------------------*/
static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, 2, 10);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 10);
}

/* Reads the 6 raw int16 accel/gyro registers (no unit conversion, no bias). */
static esp_err_t read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                          int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t raw[14];
    esp_err_t ret = reg_read(REG_ACCEL_XOUT, raw, 14);
    if (ret != ESP_OK) return ret;

    *ax = (int16_t)((raw[0]  << 8) | raw[1]);
    *ay = (int16_t)((raw[2]  << 8) | raw[3]);
    *az = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] = temperature, ignored */
    *gx = (int16_t)((raw[8]  << 8) | raw[9]);
    *gy = (int16_t)((raw[10] << 8) | raw[11]);
    *gz = (int16_t)((raw[12] << 8) | raw[13]);

    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/
esp_err_t mpu6050_init(i2c_master_bus_handle_t bus)
{
    esp_err_t ret;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU_ADDR,
        .scl_speed_hz    = 400000,
    };
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) return ret;

    /* WHO_AM_I */
    uint8_t who;
    ret = reg_read(REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK || who != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X (expected 0x68)", who);
        return ESP_FAIL;
    }

    /* Wake up */
    ret = reg_write(REG_PWR_MGMT1, 0x00);
    if (ret != ESP_OK) return ret;

    /* DLPF 42 Hz → internal sample rate 1 kHz */
    ret = reg_write(REG_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;

    /* Sample rate 100 Hz : divider = 1000/100 - 1 = 9 */
    ret = reg_write(REG_SMPLRT_DIV, 9);
    if (ret != ESP_OK) return ret;

    /* Gyro ±500°/s */
    ret = reg_write(REG_GYRO_CFG, 0x08);
    if (ret != ESP_OK) return ret;

    /* Accel ±2g */
    ret = reg_write(REG_ACCEL_CFG, 0x00);
    if (ret != ESP_OK) return ret;

    /* Gyro bias calibration: average raw readings while stationary.
     * The robot MUST be still (and flat) during this ~1 s window. */
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int n = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        int16_t rax, ray, raz, rgx, rgy, rgz;
        if (read_raw(&rax, &ray, &raz, &rgx, &rgy, &rgz) == ESP_OK) {
            sum_gx += rgx; sum_gy += rgy; sum_gz += rgz;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (n > 0) {
        s_gyro_bias[0] = ((float)sum_gx / n / GYRO_SENS) * DEG2RAD;
        s_gyro_bias[1] = ((float)sum_gy / n / GYRO_SENS) * DEG2RAD;
        s_gyro_bias[2] = ((float)sum_gz / n / GYRO_SENS) * DEG2RAD;
    }
    ESP_LOGI(TAG, "Gyro bias (rad/s): x=%.5f y=%.5f z=%.5f (%d samples)",
             s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2], n);

    ESP_LOGI(TAG, "Init OK");
    return ESP_OK;
}

esp_err_t mpu6050_read(float *ax, float *ay, float *az,
                       float *gx, float *gy, float *gz)
{
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    esp_err_t ret = read_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    if (ret != ESP_OK) return ret;

    *ax = (raw_ax / ACCEL_SENS) * G_MPS2;
    *ay = (raw_ay / ACCEL_SENS) * G_MPS2;
    *az = (raw_az / ACCEL_SENS) * G_MPS2;
    *gx = (raw_gx / GYRO_SENS)  * DEG2RAD - s_gyro_bias[0];
    *gy = (raw_gy / GYRO_SENS)  * DEG2RAD - s_gyro_bias[1];
    *gz = (raw_gz / GYRO_SENS)  * DEG2RAD - s_gyro_bias[2];

    return ESP_OK;
}
