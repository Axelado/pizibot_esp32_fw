#include "encoders.h"
#include "config.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

/* ---------------------------------------------------------------------------
 * 4x quadrature state table
 *
 * State encoding: (A << 1) | B  →  0b00, 0b01, 0b10, 0b11
 * QEM[old_state][new_state] = delta (+1, -1, or 0)
 * Diagonal transitions (00→11, 01→10) are errors → delta 0.
 * --------------------------------------------------------------------------*/
static const int8_t QEM[4][4] = {
    /*           00   01   10   11  ← new state */
    /* 00 */  {  0,  -1,  +1,   0 },
    /* 01 */  { +1,   0,   0,  -1 },
    /* 10 */  { -1,   0,   0,  +1 },
    /* 11 */  {  0,  +1,  -1,   0 },
};

/* ---------------------------------------------------------------------------
 * Variables shared between ISR and tasks
 * --------------------------------------------------------------------------*/
static volatile int32_t s_ticks_left  = 0;
static volatile int32_t s_ticks_right = 0;
static volatile uint8_t s_state_left  = 0;
static volatile uint8_t s_state_right = 0;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---------------------------------------------------------------------------
 * Left encoder ISR — triggered on any edge of channels A and B
 * --------------------------------------------------------------------------*/
static void IRAM_ATTR isr_encoder_left(void *arg)
{
    uint8_t a = (uint8_t)gpio_get_level(PIN_ENC_L_A);
    uint8_t b = (uint8_t)gpio_get_level(PIN_ENC_L_B);
    uint8_t new_state = (a << 1) | b;

    portENTER_CRITICAL_ISR(&s_mux);
    s_ticks_left += QEM[s_state_left][new_state];
    s_state_left  = new_state;
    portEXIT_CRITICAL_ISR(&s_mux);
}

/* ---------------------------------------------------------------------------
 * Right encoder ISR — same logic
 * --------------------------------------------------------------------------*/
static void IRAM_ATTR isr_encoder_right(void *arg)
{
    uint8_t a = (uint8_t)gpio_get_level(PIN_ENC_R_A);
    uint8_t b = (uint8_t)gpio_get_level(PIN_ENC_R_B);
    uint8_t new_state = (a << 1) | b;

    portENTER_CRITICAL_ISR(&s_mux);
    s_ticks_right += QEM[s_state_right][new_state];
    s_state_right  = new_state;
    portEXIT_CRITICAL_ISR(&s_mux);
}

/* ---------------------------------------------------------------------------
 * Initialization
 * --------------------------------------------------------------------------*/
esp_err_t encoders_init(void)
{
    esp_err_t ret;

    /* Common configuration for all 4 pins.
     * Pull-up enabled here; on GPIO34/35/39 (input-only) this option is
     * silently ignored by the driver — add external 10 kΩ pull-ups to 3.3V
     * if encoders are open-collector. */
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_ENC_L_A) |
                        (1ULL << PIN_ENC_L_B) |
                        (1ULL << PIN_ENC_R_A) |
                        (1ULL << PIN_ENC_R_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    

    ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    /* Read initial state to avoid a spurious delta on the first edge */
    s_state_left  = (uint8_t)((gpio_get_level(PIN_ENC_L_A) << 1) |
                               gpio_get_level(PIN_ENC_L_B));
    s_state_right = (uint8_t)((gpio_get_level(PIN_ENC_R_A) << 1) |
                               gpio_get_level(PIN_ENC_R_B));

    /* ISR registration.
     * gpio_install_isr_service() must be called beforehand in app_main.
     * Both channels of each wheel are attached to the same ISR —
     * it re-reads both pin states to recompute direction. */
    ret = gpio_isr_handler_add(PIN_ENC_L_A, isr_encoder_left,  NULL);
    if (ret != ESP_OK) return ret;
    ret = gpio_isr_handler_add(PIN_ENC_L_B, isr_encoder_left,  NULL);
    if (ret != ESP_OK) return ret;
    ret = gpio_isr_handler_add(PIN_ENC_R_A, isr_encoder_right, NULL);
    if (ret != ESP_OK) return ret;
    ret = gpio_isr_handler_add(PIN_ENC_R_B, isr_encoder_right, NULL);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Atomic read of both counters
 * --------------------------------------------------------------------------*/
void encoders_get(int32_t *left, int32_t *right)
{
    portENTER_CRITICAL(&s_mux);
    *left  = s_ticks_left;
    *right = s_ticks_right;
    portEXIT_CRITICAL(&s_mux);
}

/* ---------------------------------------------------------------------------
 * Atomic reset of both counters
 * --------------------------------------------------------------------------*/
void encoders_reset(void)
{
    portENTER_CRITICAL(&s_mux);
    s_ticks_left  = 0;
    s_ticks_right = 0;
    portEXIT_CRITICAL(&s_mux);
}
