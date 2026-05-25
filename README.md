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

Motor driver: DBH-12. Encoders: 1320 PPR, 4x quadrature decoding → 5280 counts/rev.

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

## PID tuning workflow

### 1. Identify motor parameters

```bash
# In config.h: TEST_MODE = TEST_IDENT
idf.py build flash
python3 tools/capture.py --output data/ident.csv
python3 tools/ident/calc_params.py data/ident.csv --apply
```

See [tools/ident/README.md](tools/ident/README.md) for details.

### 2. Validate PID gains

```bash
# In config.h: TEST_MODE = TEST_PID_STEP
idf.py build flash
python3 tools/capture.py --output data/pid.csv
python3 tools/pid/analyse_step.py data/pid.csv
```

## Configuration

All tunable parameters are in [components/config/config.h](components/config/config.h):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ENCODER_PPR` | 1320 | Encoder pulses per revolution |
| `INA219_SHUNT_OHMS` | 0.1 | Shunt resistance (Ω) |
| `PID_KP` | 88.0 | Proportional gain |
| `PID_KI` | 1660.0 | Integral gain |
| `PID_KD` | 0.0 | Derivative gain |
| `WHEEL_RADIUS` | 0.04 m | Wheel radius |
| `WHEEL_BASE` | 0.17 m | Wheel track (center to center) |
| `WHEEL_MAX_RADS` | 4.6 rad/s | Physical max speed (no load) |
