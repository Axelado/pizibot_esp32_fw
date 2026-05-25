#pragma once

/**
 * Generic discrete PID — one instance per wheel.
 *
 * Output clamped to [out_min, out_max].
 * Anti-windup by integral clamping (same range as output).
 * dt passed explicitly on each call — no internal clock.
 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;

    float integral;
    float prev_measured;
} pid_t;

/**
 * @brief Initializes (or resets) a PID instance.
 *
 * @param pid      Instance to initialize
 * @param kp       Proportional gain
 * @param ki       Integral gain
 * @param kd       Derivative gain
 * @param out_min  Output lower bound (e.g. -1000)
 * @param out_max  Output upper bound (e.g.  1000)
 */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);

/**
 * @brief Computes one PID iteration.
 *
 * @param pid       PID instance
 * @param setpoint  Target (rad/s)
 * @param measured  Measurement (rad/s)
 * @param dt        Sampling period in seconds (e.g. 0.01 at 100 Hz)
 * @return          Raw PWM output in [out_min, out_max]
 */
float pid_compute(pid_t *pid, float setpoint, float measured, float dt);

/**
 * @brief Resets the integral and previous error to zero.
 *
 * Call after a motor stop to avoid an integral kick on restart.
 */
void pid_reset(pid_t *pid);
