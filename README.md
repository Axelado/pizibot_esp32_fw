# pizibot_esp32_fw

ESP32 firmware for the pizibot differential drive robot, built with ESP-IDF v6.0.1.

## Architecture

The firmware runs 4 FreeRTOS tasks in normal mode:

| Task | Rate | Priority | Role |
|------|------|----------|------|
| `task_pid` | 100 Hz | 6 | Closed-loop wheel velocity control |
| `task_serial_rx` | 50 Hz | 5 | Parse incoming UART commands |
| `task_serial_tx` | 20 Hz | 4 | Send telemetry frames |
| `task_sensors` | 100 Hz | 3 | Read IMU (100 Hz) and battery (1 Hz) |

## Hardware

| Peripheral | Interface | Pins |
|------------|-----------|------|
| Left encoder (A/B) | GPIO | 18, 22 |
| Right encoder (A/B) | GPIO | 19, 21 |
| Left motor (IN1/IN2) | LEDC PWM | 25, 26 |
| Right motor (IN1/IN2) | LEDC PWM | 14, 27 |
| MPU6050 (IMU) | I2C | SDA=33, SCL=32 |
| INA219 (battery) | I2C | SDA=33, SCL=32 |

Motor driver: DBH-12. Encoders: `ENCODER_PPR = 330` pulses per revolution
(pre-quadrature) — the firmware multiplies by 4 (`4 * ENCODER_PPR = 1320`
ticks/rev) to convert encoder ticks to wheel angle, as measured via
`TEST_ENCODERS`.

The MPU6050 gyro bias is calibrated automatically at boot (`mpu6050_init`):
~1 s of raw samples are averaged and subtracted from every subsequent
reading. The robot **must be stationary and flat** for this first second
after reset/power-on.

## Serial protocol

UART0, 115200 baud. All frames are newline-terminated ASCII.

**Incoming (Pi → ESP32):**
```
WHEEL_VEL <left_rad/s> <right_rad/s>
CMD_STOP
```

**Outgoing (ESP32 → Pi):**
```
ENC <ticks_L> <ticks_R>
IMU <ax> <ay> <az> <gx> <gy> <gz>
BATT <voltage_V> <current_A> <percent>
```

## Build & flash

```bash
source idf6.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Test modes

Set `TEST_MODE` in [components/config/config.h](components/config/config.h) before building:

| Value | Mode | Description |
|-------|------|-------------|
| `TEST_NONE` | Normal | Full firmware |
| `TEST_MOTORS` | Motors | Forward / reverse / rotation sequence |
| `TEST_ENCODERS` | Encoders | Print ticks and speed at 10 Hz |
| `TEST_IMU` | IMU | Print accel and gyro at 10 Hz |
| `TEST_INA219` | Battery | Print voltage, current, % at 1 Hz |
| `TEST_SERIAL` | Serial | Echo setpoints, send dummy frames |
| `TEST_I2C_SCAN` | I2C scan | Probe all 7-bit addresses |
| `TEST_ALL` | All | All components simultaneously |
| `TEST_IDENT` | Identification | Step response for motor param identification |
| `TEST_PID_STEP` | PID step | Step response for PID gain validation |
| `TEST_ODOM_CALIB` | Odometry calibration | One straight-line/rotation move, prints computed odometry — see [Odometry calibration](#odometry-calibration-ros-covariance-matrices) |

## PID tuning workflow

### 1. Identify motor parameters

```bash
# In config.h: TEST_MODE = TEST_IDENT
idf.py build flash
python3 tools/capture.py --output data/ident.csv
python3 tools/ident/calc_params.py data/ident.csv --apply
```

`calc_params.py` fits a first-order+delay model (K, tau, L) and prints IMC
PID gains for 3 closed-loop time constants:

| Scenario | τ_cl | Notes |
|---|---|---|
| conservative | 5×tau | slow, no overshoot |
| moderate ★ | 2×tau | good default — used here |
| aggressive | 1×tau | fast, more sensitive to model error |

`Kp`/`Ki` always satisfy `Kp/Ki = tau` (pole-zero cancellation) — tune the
response speed via `--tau-cl <seconds>`, not by changing `Kp`/`Ki`
independently, or you'll break the cancellation and get an oscillatory
response.

See [tools/ident/README.md](tools/ident/README.md) for details.

### 2. Validate PID gains

```bash
# In config.h: TEST_MODE = TEST_PID_STEP
idf.py build flash
python3 tools/capture.py --output data/pid.csv
python3 tools/pid/analyse_step.py data/pid.csv
```

## IMU calibration (ROS covariance matrices)

Estimates the `linear_acceleration_covariance` / `angular_velocity_covariance`
/ `orientation_covariance` matrices for `sensor_msgs/Imu` from a static
capture of the IMU.

```bash
# In config.h: TEST_MODE = TEST_ALL
idf.py build flash
python3 tools/capture.py --output data/imu_calib.csv
python3 tools/imu_calib/calc_imu_covariance.py data/imu_calib.csv
```

Keep the robot stationary and flat during the capture, but with all
subsystems running (e.g. LIDAR powered on) so the recorded noise includes
real operating vibrations, not just the sensor's idle noise floor. The
script prints the mean/variance per axis, sanity-checks the gravity norm
and gyro bias, and outputs the covariance matrices ready to paste into the
ROS yaml/URDF.

`orientation_covariance` is always `[-1, 0, ...]`: the MPU6050 has no
magnetometer/sensor fusion and provides no absolute orientation estimate.

## Odometry calibration (ROS covariance matrices)

Estimates `pose_covariance_diagonal` / `twist_covariance_diagonal` for
`diff_drive_controller`, by comparing the firmware's own odometry (computed
from encoders) against a real-world ground-truth measurement.

```bash
# In config.h: TEST_MODE = TEST_ODOM_CALIB
```

### A. Linear runs (x, y, yaw error for a straight line)

1. `ODOM_CALIB_MODE = ODOM_CALIB_LINEAR`, `ODOM_CALIB_DISTANCE = 1.0`. Build & flash.
2. Mark the robot's start position/heading on the floor.
3. `python3 tools/capture.py --output data/odom_linear_runN.csv`, reset the
   robot — it drives ~1 m forward and stops, printing
   `ODOM_CALIB LINEAR x y theta t`.
4. Measure the real distance traveled (tape measure) and append it to the
   line: `ODOM_CALIB LINEAR 0.998 0.012 0.003 4.21 1.000`.
5. Repeat for 3+ runs, then `cat data/odom_linear_run*.csv > data/odom_linear.csv`.

### B. Angular runs (yaw error for an in-place rotation)

1. `ODOM_CALIB_MODE = ODOM_CALIB_ANGULAR`, `ODOM_CALIB_ANGLE = 1.5708` (π/2 =
   90°). Build & flash.

   Prefer **90°** over 180°/360° for the ground-truth measurement: with the
   chord method below, sensitivity `dd/dθ = r·cos(θ/2)` is best around 90°
   and drops to ~0 near 180° (chord length plateaus), and near 360° the
   robot returns to its start point (chord ≈ 0, angle unmeasurable).
2. Mark the rotation center on the floor, and mark a point **A** at a known
   radius `r` (e.g. 0.5 m) along the robot's heading.
3. Capture, reset — robot rotates ~90° and stops, printing
   `ODOM_CALIB ANGULAR x y theta t`.
4. Realign a ruler with the robot's new heading, mark point **B** at the
   same radius `r`, and measure the chord `d = AB`. Ground-truth angle:
   `θ = 2·asin(d / (2r))` (in degrees). Append it to the line:
   `ODOM_CALIB ANGULAR 0.004 -0.002 1.571 2.37 90.0`.
5. Repeat for 3+ runs, then `cat data/odom_angular_run*.csv > data/odom_angular.csv`.

### C. Compute covariances

```bash
python3 tools/odom_calib/calc_odom_covariance.py data/odom_linear.csv data/odom_angular.csv
```

Prints mean/std/variance of the x, y, yaw errors and the corresponding
`pose_covariance_diagonal` / `twist_covariance_diagonal` for
`diff_drive_controller` (z/roll/pitch/vy/wx/wy left at 0 — unobservable in 2D).

**`WHEEL_BASE` fixed:** an early angular run gave `theta ≈ 2×` the measured
ground-truth angle — the configured `WHEEL_BASE = 0.17` was the robot's
half-track, not the wheel-to-wheel distance. Measured directly:
`WHEEL_BASE = 0.29 m`. Any angular run captured **before** this fix is
invalid (the firmware's `theta` and the loop's stop condition both depend
on `WHEEL_BASE`) — redo the angular runs from scratch with the corrected
value. The linear `x`/`y`/`vx` values are independent of `WHEEL_BASE` and
remain valid.

## Configuration

All tunable parameters are in [components/config/config.h](components/config/config.h):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ENCODER_PPR` | 330 | Pulses per revolution (pre-quadrature); ticks/rev = 4 × `ENCODER_PPR` = 1320, measured via `TEST_ENCODERS` |
| `INA219_SHUNT_OHMS` | 0.1 | Shunt resistance (Ω) |
| `PID_KP` | 27.6 | Proportional gain (IMC, moderate: τ_cl=2τ) |
| `PID_KI` | 512.6 | Integral gain (IMC, moderate: τ_cl=2τ) |
| `PID_KD` | 0.0 | Derivative gain |
| `WHEEL_RADIUS` | 0.04 m | Wheel radius |
| `WHEEL_BASE` | 0.29 m | Wheel track, wheel-to-wheel (measured directly) |
| `WHEEL_MAX_RADS` | 18.1 rad/s | Physical max speed (no load) |
| `WHEEL_CMD_MAX_RADS` | 10.0 rad/s | Max commandable speed (headroom above setpoint) |
| `ODOM_CALIB_MODE` | `ODOM_CALIB_LINEAR` | `TEST_ODOM_CALIB`: linear or angular calibration move |
| `ODOM_CALIB_DISTANCE` | 1.0 m | Linear calibration target distance |
| `ODOM_CALIB_ANGLE` | π rad | Angular calibration target angle |
| `ODOM_CALIB_SPEED` | 6.0 rad/s | Wheel speed setpoint during calibration |
