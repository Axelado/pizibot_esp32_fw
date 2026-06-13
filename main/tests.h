#pragma once

#include "driver/i2c_master.h"

void test_motors(void);
void test_encoders(void);
void test_imu(i2c_master_bus_handle_t bus);
void test_ina219(i2c_master_bus_handle_t bus);
void test_lcd(i2c_master_bus_handle_t bus);
void test_serial(void);
void test_i2c_scan(i2c_master_bus_handle_t bus);
void test_all(i2c_master_bus_handle_t bus);
void test_identification(void);
void test_pid_step(void);
void test_odom_calib(void);

