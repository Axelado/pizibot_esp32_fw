#pragma once

/* ---------------------------------------------------------------------------
 * Test mode — set TEST_NONE (0) for normal firmware
 * --------------------------------------------------------------------------*/
#define TEST_NONE       0
#define TEST_MOTORS     1
#define TEST_ENCODERS   2
#define TEST_IMU        3
#define TEST_INA219     4
#define TEST_SERIAL     5
#define TEST_I2C_SCAN   6
#define TEST_ALL        7
#define TEST_IDENT      8
#define TEST_PID_STEP   9
#define TEST_ODOM_CALIB 10
#define TEST_MOTOR_CURVE 11
#define TEST_MOTOR_MINMAX 12

#define TEST_MODE       TEST_NONE



/* ---------------------------------------------------------------------------
 * Encoders
 * --------------------------------------------------------------------------*/
#define PIN_ENC_L_A     18  /* physical wiring: A=22, B=18 — swapped here to correct sign */
#define PIN_ENC_L_B     22
#define PIN_ENC_R_A     19
#define PIN_ENC_R_B     21
#define ENCODER_PPR     330.0f  /* pulses per revolution (pre-quadrature) — ticks/rev = 4 * ENCODER_PPR = 1320, measured via TEST_ENCODERS */

/* ---------------------------------------------------------------------------
 * Motors (DBH-12)
 * --------------------------------------------------------------------------*/
#define PIN_MOT_L_IN1   25
#define PIN_MOT_L_IN2   26
#define PIN_MOT_R_IN1   14
#define PIN_MOT_R_IN2   27

/* ---------------------------------------------------------------------------
 * I2C
 * --------------------------------------------------------------------------*/
#define PIN_I2C_SDA     33
#define PIN_I2C_SCL     32


/* ---------------------------------------------------------------------------
 * INA219
 * --------------------------------------------------------------------------*/
#define INA219_SHUNT_OHMS   0.1f   /* shunt resistance in ohms — measure for accuracy */

/* ---------------------------------------------------------------------------
 * PID gains — from identification (TEST_IDENT + calc_params.py)
 * --------------------------------------------------------------------------*/
#define PID_KP  27.6f
#define PID_KI  512.6f
#define PID_KD  0.0f

/* ---------------------------------------------------------------------------
 * Robot parameters — calibrate from real measurements
 * --------------------------------------------------------------------------*/
#define WHEEL_RADIUS    0.04f   /* meters */
#define WHEEL_TRACK     0.26f    /* wheel track in meters (wheel-to-wheel, measured) */
#define WHEEL_MAX_RADS      14.0f    /* max speed rad/s (physical, no load) */
#define WHEEL_CMD_MAX_RADS  10.0f    /* max commandable speed — must be < WHEEL_MAX_RADS
                                     * so the PID has headroom above the setpoint */
/* ---------------------------------------------------------------------------
 * TEST_ODOM_CALIB — odometry covariance calibration (see tools/odom_calib/)
 * --------------------------------------------------------------------------*/
#define ODOM_CALIB_LINEAR   1
#define ODOM_CALIB_ANGULAR  2

#define ODOM_CALIB_MODE     ODOM_CALIB_ANGULAR

#define ODOM_CALIB_DISTANCE 1.0f       /* meters — used in LINEAR mode */
#define ODOM_CALIB_ANGLE    3.1416f    /* radians (pi = 180°) — used in ANGULAR mode */
#define ODOM_CALIB_SPEED    6.0f       /* rad/s wheel setpoint */

/* ---------------------------------------------------------------------------
 * TEST_MOTOR_CURVE — open-loop velocity vs PWM duty sweep (see tools/motor_curve/)
 * Wheels off the ground. Sweeps duty from 0 to PWM_RAW_MAX (1000) in steps
 * of MOTOR_CURVE_STEP, settles, then measures the average wheel velocity.
 * --------------------------------------------------------------------------*/
#define MOTOR_CURVE_STEP        8       /* PWM duty increment */
#define MOTOR_CURVE_SETTLE_S    1.0f    /* settling time before measuring (s) */
#define MOTOR_CURVE_MEASURE_S   2.0f    /* measurement window (s) */

/* ---------------------------------------------------------------------------
 * TEST_MOTOR_MINMAX — on-ground dead-zone + saturation check (see tools/motor_curve/)
 * Wheels on the ground. Only sweeps duty in [0, MOTOR_MINMAX_LOW_MAX] (dead
 * zone) and [MOTOR_MINMAX_HIGH_MIN, PWM_RAW_MAX] (saturation). Each step runs
 * forward for settle+measure, then the same duty in reverse for the same
 * duration, so the net displacement stays close to zero.
 * --------------------------------------------------------------------------*/
#define MOTOR_MINMAX_LOW_MAX    200     /* sweep 0..this for the dead zone */
#define MOTOR_MINMAX_HIGH_MIN   900     /* sweep this..1000 for saturation */
#define MOTOR_MINMAX_STEP       8       /* PWM duty increment */
#define MOTOR_MINMAX_SETTLE_S   0.3f    /* settling time before measuring (s) */
#define MOTOR_MINMAX_MEASURE_S  0.5f    /* measurement window (s) */
