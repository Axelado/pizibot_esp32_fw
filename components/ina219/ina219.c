#include "ina219.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "INA219";

#define INA219_ADDR         0x40
#define REG_CONFIG          0x00
#define REG_SHUNT_VOLTAGE   0x01
#define REG_BUS_VOLTAGE     0x02

/* Config: 32V bus, gain /8 (±320mV), 12-bit, continuous shunt+bus
 * Bits: BRNG=1, PG=11, BADC=0011, SADC=0011, MODE=111 */
#define INA219_CONFIG       0x399F

/* Resolutions */
#define SHUNT_LSB_UV    10.0f    /* 10 µV per LSB of shunt register */
#define BUS_LSB_MV       4.0f   /* 4 mV per LSB (bits [15:3]) */

static i2c_master_dev_handle_t s_dev;

/* ---------------------------------------------------------------------------
 * I2C helpers (16-bit registers, big-endian)
 * --------------------------------------------------------------------------*/
static esp_err_t reg_write16(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(s_dev, buf, 3, 10);
}

static esp_err_t reg_read16(uint8_t reg, int16_t *out)
{
    uint8_t raw[2];
    esp_err_t ret = i2c_master_transmit_receive(s_dev, &reg, 1, raw, 2, 10);
    if (ret != ESP_OK) return ret;
    *out = (int16_t)((raw[0] << 8) | raw[1]);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/
esp_err_t ina219_init(i2c_master_bus_handle_t bus)
{
    esp_err_t ret;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = INA219_ADDR,
        .scl_speed_hz    = 400000,
    };
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) return ret;

    ret = reg_write16(REG_CONFIG, INA219_CONFIG);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config failed");
        return ret;
    }

    ESP_LOGI(TAG, "Init OK");
    return ESP_OK;
}

esp_err_t ina219_read(float *voltage, float *current)
{
    esp_err_t ret;
    int16_t raw_bus, raw_shunt;

    ret = reg_read16(REG_BUS_VOLTAGE, &raw_bus);
    if (ret != ESP_OK) return ret;

    ret = reg_read16(REG_SHUNT_VOLTAGE, &raw_shunt);
    if (ret != ESP_OK) return ret;

    /* Bits [15:3] for bus voltage */
    *voltage = ((raw_bus >> 3) * BUS_LSB_MV) / 1000.0f;

    /* Shunt voltage in µV → current via Rshunt */
    float vshunt_uv = raw_shunt * SHUNT_LSB_UV;
    *current = (vshunt_uv / 1e6f) / INA219_SHUNT_OHMS;

    return ESP_OK;
}
