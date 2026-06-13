#include "tests.h"
#include "config.h"
#include "driver/i2c_master.h"

#include "motors.h"
#include "encoders.h"
#include "mpu6050.h"
#include "ina219.h"
#include "serial_comm.h"
#include "pid.h"
#include "battery.h"
#include <math.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------------------------------------------------------------------------
 * TEST_MOTORS
 *
 * Sequence repeated every 8 seconds:
 *   2s forward  at 40%
 *   2s reverse at 40%
 *   2s left forward / right reverse (rotation)
 *   2s stop
 *
 * Observe: both wheels spin in the correct direction at moderate speed.
 * --------------------------------------------------------------------------*/
void test_motors(void)
{
    static const char *TAG = "TEST_MOTORS";
    ESP_ERROR_CHECK(motors_init());

    for (;;) {
        ESP_LOGI(TAG, "Forward 40%%");
        motors_set_raw(400, 400);
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Reverse 40%%");
        motors_set_raw(-400, -400);
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Rotation (L forward / R reverse)");
        motors_set_raw(400, -400);
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Stop");
        motors_stop();
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}

/* ---------------------------------------------------------------------------
 * TEST_ENCODERS
 *
 * Prints ticks and computed speed every 100 ms.
 * Spin the wheels by hand to verify sign and value.
 * --------------------------------------------------------------------------*/
void test_encoders(void)
{
    static const char *TAG = "TEST_ENCODERS";
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(encoders_init());

    int32_t prev_l = 0, prev_r = 0;
    const float dt = 0.1f;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        int32_t l, r;
        encoders_get(&l, &r);

        float vel_l = ((l - prev_l) / (4.0f * ENCODER_PPR)) * (2.0f * 3.14159f) / dt;
        float vel_r = ((r - prev_r) / (4.0f * ENCODER_PPR)) * (2.0f * 3.14159f) / dt;
        prev_l = l;
        prev_r = r;

        ESP_LOGI(TAG, "ticks L=%ld R=%ld | vel L=%.2f R=%.2f rad/s", l, r, vel_l, vel_r);
    }
}

/* ---------------------------------------------------------------------------
 * TEST_IMU
 *
 * Prints accel (m/s²) and gyro (rad/s) at 10 Hz.
 * Place the robot flat: az ≈ +9.8, ax≈ay≈0, gx≈gy≈gz≈0.
 * --------------------------------------------------------------------------*/
void test_imu(i2c_master_bus_handle_t bus)
{
    static const char *TAG = "TEST_IMU";
    ESP_ERROR_CHECK(mpu6050_init(bus));

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        float ax, ay, az, gx, gy, gz;
        esp_err_t ret = mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Read error");
            continue;
        }
        ESP_LOGI(TAG, "A: x=%.2f y=%.2f z=%.2f m/s²  |  G: x=%.3f y=%.3f z=%.3f rad/s",
                 ax, ay, az, gx, gy, gz);
    }
}

/* ---------------------------------------------------------------------------
 * TEST_INA219
 *
 * Prints voltage and current at 1 Hz.
 * Fully charged LiPo 3S: ~12.6V. Idle current: a few tens of mA.
 * --------------------------------------------------------------------------*/
void test_ina219(i2c_master_bus_handle_t bus)
{
    static const char *TAG = "TEST_INA219";
    ESP_ERROR_CHECK(ina219_init(bus));

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        float v, i;
        esp_err_t ret = ina219_read(&v, &i);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Read error");
            continue;
        }
        float pct;
        bool fresh = battery_get_percent(v, i, &pct);
        ESP_LOGI(TAG, "Vbat=%.3f V  |  I=%.3f A  |  P=%.2f W  |  Batt=%.1f%%%s",
                 v, i, v * i, pct, fresh ? "" : " (cache)");
    }
}


/* ---------------------------------------------------------------------------
 * TEST_I2C_SCAN
 *
 * Scans all 7-bit addresses (0x08–0x77) and prints the ones that respond.
 * Useful for finding the actual address of an unknown module.
 * --------------------------------------------------------------------------*/
void test_i2c_scan(i2c_master_bus_handle_t bus)
{
    static const char *TAG = "TEST_I2C_SCAN";
    ESP_LOGI(TAG, "Scanning (0x08-0x77)...");

    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        esp_err_t ret = i2c_master_probe(bus, addr, 20);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Found: 0x%02X", addr);
            found++;
        }
    }

    if (found == 0)
        ESP_LOGW(TAG, "No device found — check wiring and pull-ups");
    else
        ESP_LOGI(TAG, "Scan complete: %d device(s)", found);

    /* No loop — result printed once, then idle */
    for (;;) vTaskDelay(pdMS_TO_TICKS(5000));
}

/* ---------------------------------------------------------------------------
 * TEST_SERIAL
 *
 * Sends dummy ENC/IMU/BATT frames at 2 Hz.
 * Listens and prints commands received from the Pi (or a terminal).
 *
 * From a terminal: send "WHEEL_VEL 1.0 -1.0" or "CMD_STOP"
 * and check the logs.
 * --------------------------------------------------------------------------*/
void test_serial(void)
{
    static const char *TAG = "TEST_SERIAL";
    ESP_ERROR_CHECK(serial_comm_init());

    int tick = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        tick++;

        /* Read incoming commands */
        serial_comm_process_rx();
        float sp_l, sp_r;
        serial_comm_get_setpoints(&sp_l, &sp_r);
        ESP_LOGI(TAG, "Received setpoints: L=%.2f R=%.2f rad/s", sp_l, sp_r);

        /* Send dummy frames */
        serial_comm_send_enc(tick * 10, tick * 12);
        serial_comm_send_imu(0.1f * tick, 0.0f, 9.81f, 0.0f, 0.0f, 0.01f * tick);

        if (tick % 4 == 0)
            serial_comm_send_batt(11.1f, 0.5f, 75.0f);
    }
}

/* ===========================================================================
 * TEST_ALL
 *
 * Simultaneous exercise of all components via 3 independent tasks:
 *
 *   task_all_sensors  (100 Hz) — reads IMU + encoders, INA219 at 1 Hz
 *                                logs everything at 2 Hz
 *   task_all_motors   (—)      — forward/reverse/rotation/stop sequence
 *                                every 10 s, in a loop
 *   task_all_serial   (50 Hz)  — receives WHEEL_VEL/CMD_STOP from Pi,
 *                                sends ENC+IMU at 20 Hz, BATT at 1 Hz
 *
 * Values to observe:
 *   - Encoders: ticks and speed should change during motor phases
 *   - IMU: az ≈ 9.81 m/s² at rest, gyro moves during rotations
 *   - INA219: current rises during motor phases
 *   - Serial: ENC/IMU/BATT frames arrive at Pi (or terminal)
 * ===========================================================================*/

/* Shared state between test tasks */
typedef struct { float ax, ay, az, gx, gy, gz; } _imu_t;
static volatile _imu_t  _s_imu  = {0};
static volatile float   _s_bv   = 0.0f;
static volatile float   _s_bi   = 0.0f;
static volatile float   _s_bpct = 0.0f;
static portMUX_TYPE     _s_mux  = portMUX_INITIALIZER_UNLOCKED;

static void task_all_sensors(void *arg)
{
    static const char *TAG = "ALL.SENSORS";
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();
    int tick = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        tick++;

        float ax, ay, az, gx, gy, gz;
        if (mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            portENTER_CRITICAL(&_s_mux);
            _s_imu.ax = ax; _s_imu.ay = ay; _s_imu.az = az;
            _s_imu.gx = gx; _s_imu.gy = gy; _s_imu.gz = gz;
            portEXIT_CRITICAL(&_s_mux);
        }

        /* INA219 at 1 Hz */
        if (tick % 100 == 0) {
            float v, i;
            if (ina219_read(&v, &i) == ESP_OK) {
                float pct;
                battery_get_percent(v, i, &pct);
                portENTER_CRITICAL(&_s_mux);
                _s_bv = v; _s_bi = i; _s_bpct = pct;
                portEXIT_CRITICAL(&_s_mux);
            }
        }

        /* Full log at 2 Hz */
        if (tick % 50 == 0) {
            int32_t enc_l, enc_r;
            encoders_get(&enc_l, &enc_r);

            portENTER_CRITICAL(&_s_mux);
            float ax_ = _s_imu.ax, ay_ = _s_imu.ay, az_ = _s_imu.az;
            float gx_ = _s_imu.gx, gy_ = _s_imu.gy, gz_ = _s_imu.gz;
            float bv  = _s_bv, bi = _s_bi;
            portEXIT_CRITICAL(&_s_mux);

            ESP_LOGI(TAG, "ENC L=%ld R=%ld", enc_l, enc_r);
            ESP_LOGI(TAG, "IMU a=[%.2f %.2f %.2f] g=[%.3f %.3f %.3f]",
                     ax_, ay_, az_, gx_, gy_, gz_);
            ESP_LOGI(TAG, "BATT %.3fV %.3fA %.1f%%", bv, bi, _s_bpct);
        }
    }
}

static void task_all_motors(void *arg)
{
    static const char *TAG = "ALL.MOTORS";

    /* Sequence: 2s forward → 2s reverse → 2s rotation → 2s stop × infinite */
    static const struct { int l; int r; const char *label; } seq[] = {
        { 400,  400, "Forward" },
        {-400, -400, "Reverse" },
        { 400, -400, "Rotation" },
        {   0,    0, "Stop" },
    };
    const int nb = sizeof(seq) / sizeof(seq[0]);

    for (;;) {
        for (int i = 0; i < nb; i++) {
            ESP_LOGI(TAG, "%s", seq[i].label);
            motors_set_raw(seq[i].l, seq[i].r);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

static void task_all_serial(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(20);   /* 50 Hz rx */
    TickType_t last_wake = xTaskGetTickCount();
    int tick = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        tick++;

        serial_comm_process_rx();

        /* TX at 20 Hz (1 tick per ~2.5) */
        if (tick % 3 == 0) {
            int32_t enc_l, enc_r;
            encoders_get(&enc_l, &enc_r);
            serial_comm_send_enc(enc_l, enc_r);

            portENTER_CRITICAL(&_s_mux);
            float ax = _s_imu.ax, ay = _s_imu.ay, az = _s_imu.az;
            float gx = _s_imu.gx, gy = _s_imu.gy, gz = _s_imu.gz;
            portEXIT_CRITICAL(&_s_mux);
            serial_comm_send_imu(ax, ay, az, gx, gy, gz);
        }

        /* BATT at 1 Hz */
        if (tick % 50 == 0) {
            portENTER_CRITICAL(&_s_mux);
            float bv = _s_bv, bi = _s_bi, bpct = _s_bpct;
            portEXIT_CRITICAL(&_s_mux);
            serial_comm_send_batt(bv, bi, bpct);
        }
    }
}

void test_all(i2c_master_bus_handle_t bus)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(encoders_init());
    ESP_ERROR_CHECK(motors_init());
    ESP_ERROR_CHECK(serial_comm_init());
    ESP_ERROR_CHECK(mpu6050_init(bus));
    ESP_ERROR_CHECK(ina219_init(bus));

    xTaskCreate(task_all_sensors, "all_sensors", 4096, NULL, 3, NULL);
    xTaskCreate(task_all_motors,  "all_motors",  4096, NULL, 2, NULL);
    xTaskCreate(task_all_serial,  "all_serial",  4096, NULL, 4, NULL);
}

/* ===========================================================================
 * TEST_IDENT — Motor parameter identification
 *
 * Applies a fixed PWM step to ONE motor (left), records speed at 100 Hz
 * for TEST_DURATION seconds, then stops.
 *
 * Serial output (CSV format, readable by Python/MATLAB):
 *   t,vel_rads
 *   0.000,0.0000
 *   0.010,0.1234
 *   ...
 *
 * Usage:
 *   minicom -D /dev/ttyUSB0 -b 115200 -C capture.csv
 *   → then plot vel(t) to extract Kmotor and tau
 * ===========================================================================*/
#define IDENT_PWM       512     /* 50% duty out of 1023 */
#define IDENT_DURATION  5.0f    /* secondes */
#define IDENT_DT        0.01f   /* 100 Hz */

static void task_ident(void *arg)
{
    int32_t ticks_prev = 0, ticks_curr = 0;
    int32_t dummy;   /* encoders_get requiert deux pointeurs valides */
    float t = 0.0f;

    /* CSV header */
    printf("t,vel_rads\n");

    motors_set_raw(IDENT_PWM, 0);

    const TickType_t period = pdMS_TO_TICKS((int)(IDENT_DT * 1000));
    TickType_t last_wake = xTaskGetTickCount();

    while (t < IDENT_DURATION) {
        vTaskDelayUntil(&last_wake, period);

        encoders_get(&ticks_curr, &dummy);
        /* 4 * ENCODER_PPR = ticks per wheel revolution (4x quadrature included) */
        float vel = (float)(ticks_curr - ticks_prev)
                    / (4.0f * ENCODER_PPR) * 2.0f * (float)M_PI / IDENT_DT;
        ticks_prev = ticks_curr;

        printf("%.3f,%.4f\n", t, vel);
        t += IDENT_DT;
    }

    motors_stop();
    printf("# identification complete\n");
    vTaskDelete(NULL);
}

void test_identification(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(encoders_init());
    ESP_ERROR_CHECK(motors_init());

    /* High priority for accurate 100 Hz timing */
    xTaskCreate(task_ident, "ident", 4096, NULL, 6, NULL);
}

/* ===========================================================================
 * TEST_PID_STEP — PID gain validation via step response
 *
 * Sequence: 3s at 0 → 3s at STEP_SP → 3s at -STEP_SP → stop
 * Single PID driven by the left wheel; same PWM applied to both motors
 * (both wheels run together, as in normal operation).
 * CSV output: "t,setpoint,mesure,mesure_r,pwm"  → plot with analyse_step.py
 *
 * Visual criteria:
 *   OK             : measured follows setpoint with no overshoot or oscillation
 *   Kp too large   : overshoot or damped oscillations
 *   Ki too large   : slow oscillations (period > 0.5 s)
 *   Gains too small: slow response, steady-state error
 * ===========================================================================*/
#define STEP_SP   4.0f   /* rad/s — choisir < WHEEL_MAX_RADS */
#define STEP_DT   0.01f  /* 100 Hz */

static void task_pid_step(void *arg)
{
    pid_t pid;
    pid_init(&pid, PID_KP, PID_KI, PID_KD, -1000.0f, 1000.0f);

    int32_t ticks_l_prev = 0, ticks_l_curr = 0;
    int32_t ticks_r_prev = 0, ticks_r_curr = 0;
    encoders_get(&ticks_l_prev, &ticks_r_prev);

    float setpoint = 0.0f;
    float t = 0.0f;
    const float total = 9.0f;   /* 3 phases × 3s */

    printf("t,setpoint,mesure,mesure_r,pwm\n");

    const TickType_t period = pdMS_TO_TICKS((int)(STEP_DT * 1000));
    TickType_t last_wake = xTaskGetTickCount();

    while (t < total) {
        vTaskDelayUntil(&last_wake, period);

        /* Setpoint sequence */
        if      (t < 3.0f) setpoint =  0.0f;
        else if (t < 6.0f) setpoint =  STEP_SP;
        else               setpoint = -STEP_SP;

        encoders_get(&ticks_l_curr, &ticks_r_curr);
        float vel_l = (float)(ticks_l_curr - ticks_l_prev)
                    / (4.0f * ENCODER_PPR) * 2.0f * (float)M_PI / STEP_DT;
        float vel_r = (float)(ticks_r_curr - ticks_r_prev)
                    / (4.0f * ENCODER_PPR) * 2.0f * (float)M_PI / STEP_DT;
        ticks_l_prev = ticks_l_curr;
        ticks_r_prev = ticks_r_curr;

        /* Single PID driven by the left wheel, same output applied to both motors */
        int out = (int)pid_compute(&pid, setpoint, vel_l, STEP_DT);
        motors_set_raw(out, out);

        printf("%.3f,%.4f,%.4f,%.4f,%d\n", t, setpoint, vel_l, vel_r, out);
        t += STEP_DT;
    }

    motors_stop();
    printf("# pid_step complete\n");
    vTaskDelete(NULL);
}

void test_pid_step(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(encoders_init());
    ESP_ERROR_CHECK(motors_init());

    xTaskCreate(task_pid_step, "pid_step", 4096, NULL, 6, NULL);
}

/* ===========================================================================
 * TEST_ODOM_CALIB — odometry covariance calibration
 *
 * Drives a single closed-loop motion -- straight line or in-place rotation,
 * selected by ODOM_CALIB_MODE -- computing odometry (x, y, theta) from the
 * encoders with the standard differential-drive model. Stops automatically
 * once the target distance (ODOM_CALIB_DISTANCE) or angle (ODOM_CALIB_ANGLE)
 * is reached, then prints the resulting pose.
 *
 * Compare the printed (x, y, theta) against a real-world measurement (tape
 * measure / protractor marks on the floor) over several runs to estimate
 * the odometry covariance -- see tools/odom_calib/calc_odom_covariance.py.
 *
 * Output:
 *   ODOM_CALIB <mode> <x> <y> <theta> <duration_s>
 *   # odom_calib complete
 * ===========================================================================*/
#define ODOM_CALIB_DT  0.01f  /* 100 Hz */

static void task_odom_calib(void *arg)
{
    pid_t pid_left, pid_right;
    pid_init(&pid_left,  PID_KP, PID_KI, PID_KD, -1000.0f, 1000.0f);
    pid_init(&pid_right, PID_KP, PID_KI, PID_KD, -1000.0f, 1000.0f);

    int32_t ticks_l_prev = 0, ticks_r_prev = 0;
    encoders_get(&ticks_l_prev, &ticks_r_prev);

#if ODOM_CALIB_MODE == ODOM_CALIB_ANGULAR
    const float sp_l = -ODOM_CALIB_SPEED;
    const float sp_r =  ODOM_CALIB_SPEED;
    const char *mode_str = "ANGULAR";
#else
    const float sp_l = ODOM_CALIB_SPEED;
    const float sp_r = ODOM_CALIB_SPEED;
    const char *mode_str = "LINEAR";
#endif

    /* Distance traveled by one wheel per encoder tick (meters) */
    const float ticks_to_m = (2.0f * (float)M_PI * WHEEL_RADIUS) / (4.0f * ENCODER_PPR);

    float x = 0.0f, y = 0.0f, theta = 0.0f, dist = 0.0f, t = 0.0f;

    const TickType_t period = pdMS_TO_TICKS((int)(ODOM_CALIB_DT * 1000));
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        int32_t ticks_l, ticks_r;
        encoders_get(&ticks_l, &ticks_r);
        int32_t delta_l = ticks_l - ticks_l_prev;
        int32_t delta_r = ticks_r - ticks_r_prev;
        ticks_l_prev = ticks_l;
        ticks_r_prev = ticks_r;

        float vel_l = (float)delta_l / (4.0f * ENCODER_PPR) * 2.0f * (float)M_PI / ODOM_CALIB_DT;
        float vel_r = (float)delta_r / (4.0f * ENCODER_PPR) * 2.0f * (float)M_PI / ODOM_CALIB_DT;

        /* Differential-drive odometry update */
        float dl = (float)delta_l * ticks_to_m;
        float dr = (float)delta_r * ticks_to_m;
        float dc = (dl + dr) / 2.0f;
        float dtheta = (dr - dl) / WHEEL_BASE;
        x += dc * cosf(theta + dtheta / 2.0f);
        y += dc * sinf(theta + dtheta / 2.0f);
        theta += dtheta;
        dist += fabsf(dc);
        t += ODOM_CALIB_DT;

        int out_l = (int)pid_compute(&pid_left,  sp_l, vel_l, ODOM_CALIB_DT);
        int out_r = (int)pid_compute(&pid_right, sp_r, vel_r, ODOM_CALIB_DT);
        motors_set_raw(out_l, out_r);

#if ODOM_CALIB_MODE == ODOM_CALIB_ANGULAR
        if (fabsf(theta) >= ODOM_CALIB_ANGLE) break;
#else
        if (dist >= ODOM_CALIB_DISTANCE) break;
#endif
    }

    motors_stop();
    printf("ODOM_CALIB %s %.5f %.5f %.5f %.3f\n", mode_str, x, y, theta, t);
    printf("# odom_calib complete\n");
    vTaskDelete(NULL);
}

void test_odom_calib(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(encoders_init());
    ESP_ERROR_CHECK(motors_init());

    xTaskCreate(task_odom_calib, "odom_calib", 4096, NULL, 6, NULL);
}

/* ===========================================================================
 * TEST_MOTOR_CURVE — open-loop velocity vs PWM duty sweep
 *
 * Wheels off the ground. Sweeps duty from 0 to MOTOR_CURVE_PWM_MAX in steps
 * of MOTOR_CURVE_STEP, open-loop (no PID), both wheels driven equally and
 * forward. At each step: settle for MOTOR_CURVE_SETTLE_S, then measure the
 * average wheel velocity over MOTOR_CURVE_MEASURE_S.
 *
 * Output:
 *   MOTOR_CURVE <duty> <vel_l> <vel_r>
 *   # motor_curve complete
 * ===========================================================================*/
#define MOTOR_CURVE_PWM_MAX  1000  /* matches PWM_RAW_MAX in motors.c */

static void task_motor_curve(void *arg)
{
    int32_t ticks_l_prev = 0, ticks_r_prev = 0;
    encoders_get(&ticks_l_prev, &ticks_r_prev);

    for (int duty = 0; duty <= MOTOR_CURVE_PWM_MAX; duty += MOTOR_CURVE_STEP) {
        motors_set_raw(duty, duty);

        vTaskDelay(pdMS_TO_TICKS((int)(MOTOR_CURVE_SETTLE_S * 1000)));

        encoders_get(&ticks_l_prev, &ticks_r_prev);
        vTaskDelay(pdMS_TO_TICKS((int)(MOTOR_CURVE_MEASURE_S * 1000)));

        int32_t ticks_l, ticks_r;
        encoders_get(&ticks_l, &ticks_r);

        float vel_l = (float)(ticks_l - ticks_l_prev)
                       / (4.0f * ENCODER_PPR) * 2.0f * (float)M_PI / MOTOR_CURVE_MEASURE_S;
        float vel_r = (float)(ticks_r - ticks_r_prev)
                       / (4.0f * ENCODER_PPR) * 2.0f * (float)M_PI / MOTOR_CURVE_MEASURE_S;

        printf("MOTOR_CURVE %d %.4f %.4f\n", duty, vel_l, vel_r);
    }

    motors_stop();
    printf("# motor_curve complete\n");
    vTaskDelete(NULL);
}

void test_motor_curve(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(encoders_init());
    ESP_ERROR_CHECK(motors_init());

    xTaskCreate(task_motor_curve, "motor_curve", 4096, NULL, 6, NULL);
}
