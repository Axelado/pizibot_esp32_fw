#include "motors.h"
#include "config.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

/* ---------------------------------------------------------------------------
 * LEDC configuration
 * --------------------------------------------------------------------------*/
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ 20000U
#define LEDC_RES LEDC_TIMER_10_BIT
#define LEDC_DUTY_MAX 1023 /* 2^10 - 1 */
#define PWM_RAW_MAX 1000

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CH_IN1_LEFT LEDC_CHANNEL_0
#define LEDC_CH_IN2_LEFT LEDC_CHANNEL_1

#define LEDC_CH_IN1_RIGHT LEDC_CHANNEL_2
#define LEDC_CH_IN2_RIGHT LEDC_CHANNEL_3

/* ---------------------------------------------------------------------------
 * Internal utilities
 * --------------------------------------------------------------------------*/

/* Raw duty [0, PWM_RAW_MAX] to LEDC duty [0, LEDC_DUTY_MAX] conversion */
static inline uint32_t raw_to_duty(int magnitude)
{
    if (magnitude <= 0)
        return 0;
    if (magnitude >= PWM_RAW_MAX)
        return LEDC_DUTY_MAX;
    return (uint32_t)((magnitude * LEDC_DUTY_MAX + PWM_RAW_MAX / 2) / PWM_RAW_MAX);
}

/* Applies direction + duty to a wheel via PWM on IN1/IN2.
 * raw > 0 : forward (PWM on IN1, IN2=0)
 * raw < 0 : reverse (IN1=0, PWM on IN2)
 * raw = 0 : brake (IN1=IN2=0) */
static void apply_wheel(ledc_channel_t channel_in1, ledc_channel_t channel_in2,
                        int raw)
{
    uint32_t duty;

    if (raw > 0)
    {
        /* Forward: IN1=H, IN2=L */
        duty = raw_to_duty(raw);
        ledc_set_duty(LEDC_MODE, channel_in1, duty);
        ledc_set_duty(LEDC_MODE, channel_in2, 0);
        ledc_update_duty(LEDC_MODE, channel_in1);
        ledc_update_duty(LEDC_MODE, channel_in2);
    }
    else if (raw < 0)
    {
        /* Reverse: IN1=L, IN2=H */
        duty = raw_to_duty(-raw);
        ledc_set_duty(LEDC_MODE, channel_in1, 0);
        ledc_set_duty(LEDC_MODE, channel_in2, duty);
        ledc_update_duty(LEDC_MODE, channel_in1);
        ledc_update_duty(LEDC_MODE, channel_in2);
    }
    else
    {
        /* Brake: IN1=L, IN2=L */
        ledc_set_duty(LEDC_MODE, channel_in1, 0);
        ledc_set_duty(LEDC_MODE, channel_in2, 0);
        ledc_update_duty(LEDC_MODE, channel_in1);
        ledc_update_duty(LEDC_MODE, channel_in2);
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/

esp_err_t motors_init(void)
{
    esp_err_t ret;

    /* --- LEDC timers --- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    timer_cfg.timer_num = LEDC_TIMER;
    ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK)
        return ret;

    /* --- LEDC channels --- */
    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
    };

    ch_cfg.gpio_num = PIN_MOT_L_IN1;
    ch_cfg.channel = LEDC_CH_IN1_LEFT;
    ch_cfg.timer_sel = LEDC_TIMER;
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK)
        return ret;

    ch_cfg.gpio_num = PIN_MOT_L_IN2;
    ch_cfg.channel = LEDC_CH_IN2_LEFT;
    ch_cfg.timer_sel = LEDC_TIMER;
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK)
        return ret;

    ch_cfg.gpio_num = PIN_MOT_R_IN1;
    ch_cfg.channel = LEDC_CH_IN1_RIGHT;
    ch_cfg.timer_sel = LEDC_TIMER;
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK)
        return ret;

    ch_cfg.gpio_num = PIN_MOT_R_IN2;
    ch_cfg.channel = LEDC_CH_IN2_RIGHT;
    ch_cfg.timer_sel = LEDC_TIMER;
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK)
        return ret;

    /* Brake on startup */
    motors_stop();
    return ESP_OK;
}

void motors_set(float left_rads, float right_rads)
{
    /* Clamp */
    if (left_rads > WHEEL_MAX_RADS)
        left_rads = WHEEL_MAX_RADS;
    if (left_rads < -WHEEL_MAX_RADS)
        left_rads = -WHEEL_MAX_RADS;
    if (right_rads > WHEEL_MAX_RADS)
        right_rads = WHEEL_MAX_RADS;
    if (right_rads < -WHEEL_MAX_RADS)
        right_rads = -WHEEL_MAX_RADS;

    int raw_left = (int)(left_rads / WHEEL_MAX_RADS * PWM_RAW_MAX);
    int raw_right = (int)(right_rads / WHEEL_MAX_RADS * PWM_RAW_MAX);

    motors_set_raw(raw_left, raw_right);
}

void motors_set_raw(int left, int right)
{
    /* Clamp */
    if (left > PWM_RAW_MAX)
        left = PWM_RAW_MAX;
    if (left < -PWM_RAW_MAX)
        left = -PWM_RAW_MAX;
    if (right > PWM_RAW_MAX)
        right = PWM_RAW_MAX;
    if (right < -PWM_RAW_MAX)
        right = -PWM_RAW_MAX;

    apply_wheel(LEDC_CH_IN1_LEFT, LEDC_CH_IN2_LEFT, left);
    apply_wheel(LEDC_CH_IN1_RIGHT, LEDC_CH_IN2_RIGHT, right);
}

void motors_stop(void)
{
    apply_wheel(LEDC_CH_IN1_LEFT, LEDC_CH_IN2_LEFT, 0);
    apply_wheel(LEDC_CH_IN1_RIGHT, LEDC_CH_IN2_RIGHT, 0);
}
