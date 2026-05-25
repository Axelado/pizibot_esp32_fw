#include "serial_comm.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "config.h"

#define UART_PORT   UART_NUM_0
#define UART_BAUD   115200
#define RX_BUF_SZ   256
#define RX_LINE_MAX 64

/* ---------------------------------------------------------------------------
 * Shared setpoints — written by process_rx, read by task_pid
 * --------------------------------------------------------------------------*/
static volatile float s_setpoint_left  = 0.0f;
static volatile float s_setpoint_right = 0.0f;
static portMUX_TYPE   s_sp_mux = portMUX_INITIALIZER_UNLOCKED;

/* Line accumulation buffer for incoming data */
static char  s_line_buf[RX_LINE_MAX];
static int   s_line_len = 0;


/* ---------------------------------------------------------------------------
 * Parsing a complete line
 * --------------------------------------------------------------------------*/
static void parse_line(const char *line)
{
    float l, r;
    if (sscanf(line, "WHEEL_VEL %f %f", &l, &r) == 2) {
        if (!isfinite(l) || !isfinite(r)) return;
        if (l >  WHEEL_CMD_MAX_RADS) l =  WHEEL_CMD_MAX_RADS;
        if (l < -WHEEL_CMD_MAX_RADS) l = -WHEEL_CMD_MAX_RADS;
        if (r >  WHEEL_CMD_MAX_RADS) r =  WHEEL_CMD_MAX_RADS;
        if (r < -WHEEL_CMD_MAX_RADS) r = -WHEEL_CMD_MAX_RADS;
        portENTER_CRITICAL(&s_sp_mux);
        s_setpoint_left  = l;
        s_setpoint_right = r;
        portEXIT_CRITICAL(&s_sp_mux);
    } else if (strncmp(line, "CMD_STOP", 8) == 0) {
        portENTER_CRITICAL(&s_sp_mux);
        s_setpoint_left  = 0.0f;
        s_setpoint_right = 0.0f;
        portEXIT_CRITICAL(&s_sp_mux);
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/
esp_err_t serial_comm_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config(UART_PORT, &cfg);
    if (ret != ESP_OK) return ret;

    return uart_driver_install(UART_PORT, RX_BUF_SZ, 0, 0, NULL, 0);
}

void serial_comm_process_rx(void)
{
    uint8_t byte;
    /* Non-blocking read, byte by byte */
    while (uart_read_bytes(UART_PORT, &byte, 1, 0) == 1) {
        if (byte == '\n' || byte == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                parse_line(s_line_buf);
                s_line_len = 0;
            }
        } else if (s_line_len < RX_LINE_MAX - 1) {
            s_line_buf[s_line_len++] = (char)byte;
        } else {
            /* Line too long: drop and reset */
            s_line_len = 0;
        }
    }
}

void serial_comm_get_setpoints(float *left, float *right)
{
    portENTER_CRITICAL(&s_sp_mux);
    *left  = s_setpoint_left;
    *right = s_setpoint_right;
    portEXIT_CRITICAL(&s_sp_mux);
}

void serial_comm_send_enc(int32_t left, int32_t right)
{
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "ENC %ld %ld\n", left, right);
    uart_write_bytes(UART_PORT, buf, n);
}

void serial_comm_send_imu(float ax, float ay, float az,
                          float gx, float gy, float gz)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf),
                     "IMU %.4f %.4f %.4f %.4f %.4f %.4f\n",
                     ax, ay, az, gx, gy, gz);
    uart_write_bytes(UART_PORT, buf, n);
}

void serial_comm_send_batt(float voltage, float current, float percent)
{
    char buf[40];
    int n = snprintf(buf, sizeof(buf), "BATT %.3f %.3f %.1f\n",
                     voltage, current, percent);
    uart_write_bytes(UART_PORT, buf, n);
}
