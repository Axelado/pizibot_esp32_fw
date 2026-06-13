#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "tests.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "serial_comm.h"
#include "mpu6050.h"
#include "ina219.h"
#include "battery.h"

static const char *TAG = "MAIN";

/* ---------------------------------------------------------------------------
 * Shared state between tasks
 * --------------------------------------------------------------------------*/
typedef struct { float ax, ay, az, gx, gy, gz; } imu_data_t;

static imu_data_t     s_imu  = {0};
static portMUX_TYPE   s_imu_mux  = portMUX_INITIALIZER_UNLOCKED;

static float          s_batt_v   = 0.0f;
static float          s_batt_i   = 0.0f;
static float          s_batt_pct = 0.0f;
static portMUX_TYPE   s_batt_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---------------------------------------------------------------------------
 * Tick delta to rad/s conversion
 * ENCODER_PPR = pulses per revolution (pre-quadrature). 4 * ENCODER_PPR =
 * ticks per wheel revolution, as returned by encoders_get() (4x quadrature
 * decoding included).
 * --------------------------------------------------------------------------*/
static inline float ticks_to_rads(int32_t delta, float dt)
{
    return ((float)delta / (4.0f * ENCODER_PPR)) * (2.0f * 3.14159265f) / dt;
}

/* ---------------------------------------------------------------------------
 * task_pid — 100 Hz, priority 6
 * Closed loop: encoders → PID → motors
 * --------------------------------------------------------------------------*/
static void task_pid(void *arg)
{
    pid_t pid_left, pid_right;
    pid_init(&pid_left,  PID_KP, PID_KI, PID_KD, -1000.0f, 1000.0f);
    pid_init(&pid_right, PID_KP, PID_KI, PID_KD, -1000.0f, 1000.0f);

    int32_t ticks_l_prev = 0, ticks_r_prev = 0;
    encoders_get(&ticks_l_prev, &ticks_r_prev);

    float prev_sp_l = 0.0f, prev_sp_r = 0.0f;

    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        int32_t ticks_l, ticks_r;
        encoders_get(&ticks_l, &ticks_r);
        float vel_l = ticks_to_rads(ticks_l - ticks_l_prev, 0.01f);
        float vel_r = ticks_to_rads(ticks_r - ticks_r_prev, 0.01f);
        ticks_l_prev = ticks_l;
        ticks_r_prev = ticks_r;

        float sp_l, sp_r;
        serial_comm_get_setpoints(&sp_l, &sp_r);

        if (sp_l == 0.0f && sp_r == 0.0f) {
            motors_stop();
            pid_reset(&pid_left);
            pid_reset(&pid_right);
            prev_sp_l = 0.0f;
            prev_sp_r = 0.0f;
            continue;
        }

        /* Reset integral on direction reversal to avoid the integral fighting
         * the new direction (e.g. going from forward to backward). */
        if ((sp_l > 0.0f && prev_sp_l < 0.0f) || (sp_l < 0.0f && prev_sp_l > 0.0f) ||
            (sp_r > 0.0f && prev_sp_r < 0.0f) || (sp_r < 0.0f && prev_sp_r > 0.0f)) {
            pid_reset(&pid_left);
            pid_reset(&pid_right);
        }
        prev_sp_l = sp_l;
        prev_sp_r = sp_r;

        int out_l = (int)pid_compute(&pid_left,  sp_l, vel_l, 0.01f);
        int out_r = (int)pid_compute(&pid_right, sp_r, vel_r, 0.01f);
        ESP_LOGD(TAG, "sp=(%.2f,%.2f) vel=(%.2f,%.2f) out=(%d,%d)",
                 sp_l, sp_r, vel_l, vel_r, out_l, out_r);
        motors_set_raw(out_l, out_r);
    }
}

/* ---------------------------------------------------------------------------
 * task_sensors — 100 Hz, priority 3
 * Reads MPU6050 continuously, INA219 at 1 Hz
 * --------------------------------------------------------------------------*/
static void task_sensors(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();
    int tick = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        tick++;

        float ax, ay, az, gx, gy, gz;
        if (mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            portENTER_CRITICAL(&s_imu_mux);
            s_imu.ax = ax; s_imu.ay = ay; s_imu.az = az;
            s_imu.gx = gx; s_imu.gy = gy; s_imu.gz = gz;
            portEXIT_CRITICAL(&s_imu_mux);
        }

        /* INA219 at 1 Hz */
        if (tick % 100 == 0) {
            float v, i;
            if (ina219_read(&v, &i) == ESP_OK) {
                float pct;
                battery_get_percent(v, i, &pct);
                portENTER_CRITICAL(&s_batt_mux);
                s_batt_v   = v;
                s_batt_i   = i;
                s_batt_pct = pct;
                portEXIT_CRITICAL(&s_batt_mux);
            }
        }

    }
}

/* ---------------------------------------------------------------------------
 * task_serial_rx — 50 Hz, priority 5
 * Reads available UART bytes and parses Pi commands
 * --------------------------------------------------------------------------*/
static void task_serial_rx(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(20);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        serial_comm_process_rx();
    }
}

/* ---------------------------------------------------------------------------
 * task_serial_tx — 20 Hz, priority 4
 * Sends ENC + IMU at 20 Hz, BATT at 1 Hz
 * --------------------------------------------------------------------------*/
static void task_serial_tx(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(50);
    TickType_t last_wake = xTaskGetTickCount();
    int tick = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        tick++;

        int32_t enc_l, enc_r;
        encoders_get(&enc_l, &enc_r);
        serial_comm_send_enc(enc_l, enc_r);

        imu_data_t imu;
        portENTER_CRITICAL(&s_imu_mux); 
        imu = s_imu;
        portEXIT_CRITICAL(&s_imu_mux);
        serial_comm_send_imu(imu.ax, imu.ay, imu.az,
                             imu.gx, imu.gy, imu.gz);

        /* BATT at 1 Hz */
        if (tick % 20 == 0) {
            float v, i, pct;
            portENTER_CRITICAL(&s_batt_mux);
            v = s_batt_v; i = s_batt_i; pct = s_batt_pct;
            portEXIT_CRITICAL(&s_batt_mux);
            serial_comm_send_batt(v, i, pct);
        }
    }
}




/* ---------------------------------------------------------------------------
 * app_main
 * --------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------
 * I2C bus initialization — used by tests and normal firmware
 * --------------------------------------------------------------------------*/
static i2c_master_bus_handle_t init_i2c(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = I2C_NUM_0,
        .scl_io_num        = PIN_I2C_SCL,
        .sda_io_num        = PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    return bus;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting pizibot");

#if TEST_MODE == TEST_MOTORS
    ESP_LOGI(TAG, "--- MODE TEST : MOTORS ---");
    test_motors();

#elif TEST_MODE == TEST_ENCODERS
    ESP_LOGI(TAG, "--- MODE TEST : ENCODERS ---");
    test_encoders();

#elif TEST_MODE == TEST_IMU
    ESP_LOGI(TAG, "--- MODE TEST : IMU ---");
    test_imu(init_i2c());

#elif TEST_MODE == TEST_INA219
    ESP_LOGI(TAG, "--- MODE TEST : INA219 ---");
    test_ina219(init_i2c());

#elif TEST_MODE == TEST_SERIAL
    ESP_LOGI(TAG, "--- MODE TEST : SERIAL ---");
    test_serial();

#elif TEST_MODE == TEST_I2C_SCAN
    ESP_LOGI(TAG, "--- MODE TEST : I2C SCAN ---");
    test_i2c_scan(init_i2c());

#elif TEST_MODE == TEST_ALL
    ESP_LOGI(TAG, "--- MODE TEST : ALL ---");
    test_all(init_i2c());

#elif TEST_MODE == TEST_IDENT
    ESP_LOGI(TAG, "--- MODE TEST : IDENTIFICATION ---");
    test_identification();

#elif TEST_MODE == TEST_PID_STEP
    ESP_LOGI(TAG, "--- MODE TEST : PID STEP ---");
    test_pid_step();

#elif TEST_MODE == TEST_ODOM_CALIB
    ESP_LOGI(TAG, "--- MODE TEST : ODOM CALIB ---");
    test_odom_calib();

#elif TEST_MODE == TEST_MOTOR_CURVE
    ESP_LOGI(TAG, "--- MODE TEST : MOTOR CURVE ---");
    test_motor_curve();

#else
    /* Normal firmware */

    /* GPIO and encoders */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(encoders_init());

    /* Motors */
    ESP_ERROR_CHECK(motors_init());

    /* Serial communication */
    ESP_ERROR_CHECK(serial_comm_init());

    i2c_master_bus_handle_t i2c_bus = init_i2c();

    ESP_ERROR_CHECK(mpu6050_init(i2c_bus));
    ESP_ERROR_CHECK(ina219_init(i2c_bus));

    /* FreeRTOS tasks */
    xTaskCreate(task_serial_rx, "serial_rx", 4096, NULL, 5, NULL);
    xTaskCreate(task_serial_tx, "serial_tx", 4096, NULL, 4, NULL);
    xTaskCreate(task_pid,       "pid",        4096, NULL, 6, NULL);
    xTaskCreate(task_sensors,   "sensors",    4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "Init OK — tasks started");
#endif  /* TEST_MODE */
}
