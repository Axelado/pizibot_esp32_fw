#include "battery.h"
#include <float.h>
#include <math.h>

/* -------------------------------------------------------------------------
 * Disturbance threshold
 * Below: voltage is reliable, recompute
 * Above: motors active, keep last known value
 * ------------------------------------------------------------------------- */
#define CURRENT_THRESHOLD   1.5f    // A — suited for LiPo 3S 80C 5200mAh

/* Hysteresis: prevents rapid switching around the threshold */
#define CURRENT_HYST        0.2f    // A

/* -------------------------------------------------------------------------
 * LiPo 3S discharge table (total voltage → charge %)
 * Measured at C/5 (~1A), ambient temperature
 * ------------------------------------------------------------------------- */
static const float TABLE[][2] = {
    {12.60f, 100.0f}, {12.45f,  95.0f}, {12.33f,  90.0f},
    {12.24f,  85.0f}, {12.06f,  80.0f}, {11.94f,  75.0f},
    {11.85f,  70.0f}, {11.73f,  65.0f}, {11.61f,  60.0f},
    {11.49f,  55.0f}, {11.37f,  50.0f}, {11.25f,  45.0f},
    {11.13f,  40.0f}, {11.01f,  35.0f}, {10.89f,  30.0f},
    {10.77f,  25.0f}, {10.56f,  20.0f}, {10.35f,  15.0f},
    {10.05f,  10.0f}, { 9.60f,   5.0f}, { 9.00f,   0.0f},
};
#define TABLE_LEN  (sizeof(TABLE) / sizeof(TABLE[0]))

/* -------------------------------------------------------------------------
 * Internal state
 * ------------------------------------------------------------------------- */
static float  s_cached_percent  = -1.0f;   // -1 = never computed
static bool   s_above_threshold = false;    // hysteresis state

/* -------------------------------------------------------------------------
 * Linear interpolation in the table
 * ------------------------------------------------------------------------- */
static float lookup(float voltage) {
    if (voltage >= TABLE[0][0])          return 100.0f;
    if (voltage <= TABLE[TABLE_LEN-1][0]) return   0.0f;

    for (size_t i = 0; i < TABLE_LEN - 1; i++) {
        float v1 = TABLE[i][0],     p1 = TABLE[i][1];
        float v2 = TABLE[i + 1][0], p2 = TABLE[i + 1][1];
        if (voltage <= v1 && voltage >= v2) {
            return p2 + (p1 - p2) * (voltage - v2) / (v1 - v2);
        }
    }
    return 0.0f;
}

/* -------------------------------------------------------------------------
 * API publique
 * ------------------------------------------------------------------------- */
bool battery_get_percent(float voltage, float current, float *percent) {
    /* Hysteresis on threshold:
     * - cross "above" at THRESHOLD + HYST
     * - cross back "below" at THRESHOLD - HYST                            */
    if (!s_above_threshold && current > CURRENT_THRESHOLD + CURRENT_HYST) {
        s_above_threshold = true;
    } else if (s_above_threshold && current < CURRENT_THRESHOLD - CURRENT_HYST) {
        s_above_threshold = false;
    }

    /* First read: always compute to initialize */
    const bool first_read = (s_cached_percent < 0.0f);
    const bool should_update = first_read || !s_above_threshold;

    if (should_update) {
        s_cached_percent = lookup(voltage);
    }

    *percent = s_cached_percent;
    return should_update;
}

void battery_reset(void) {
    s_cached_percent  = -1.0f;
    s_above_threshold = false;
}