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

#define TEST_MODE       TEST_NONE



/* ---------------------------------------------------------------------------
 * Encoders
 * --------------------------------------------------------------------------*/
#define PIN_ENC_L_A     18  /* physical wiring: A=22, B=18 — swapped here to correct sign */
#define PIN_ENC_L_B     22
#define PIN_ENC_R_A     19
#define PIN_ENC_R_B     21
#define ENCODER_PPR     1320

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
#define PID_KP  88.0f
#define PID_KI  1660.0f
#define PID_KD  0.0f

/* ---------------------------------------------------------------------------
 * Robot parameters — calibrate from real measurements
 * --------------------------------------------------------------------------*/
#define WHEEL_RADIUS    0.04f   /* meters */
#define WHEEL_BASE      0.17f    /* wheel track in meters */
#define WHEEL_MAX_RADS      4.6f    /* vitesse max rad/s (physique, à vide) */
#define WHEEL_CMD_MAX_RADS  3.5f    /* vitesse max commandable — doit être < WHEEL_MAX_RADS
                                     * pour que le PID ait de la marge au-dessus de la consigne */
