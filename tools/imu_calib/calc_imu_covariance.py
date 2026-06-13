#!/usr/bin/env python3
"""
IMU noise estimation -> ROS sensor_msgs/Imu covariance matrices.

The ROS sensor_msgs/Imu message carries three 3x3 covariance matrices
(row-major, 9 values each):

  - linear_acceleration_covariance   (m/s^2)^2
  - angular_velocity_covariance      (rad/s)^2
  - orientation_covariance           (rad)^2

These represent the *noise* of the sensor, estimated as the variance of
each axis while the IMU is perfectly still. Off-diagonal terms (cross-axis
correlation) are assumed negligible and left at 0.

The MPU6050 used here has no magnetometer / onboard sensor fusion, so it
cannot provide an absolute orientation estimate. Per REP-145, the
orientation_covariance[0] should then be set to -1 to tell consumers
("robot_localization", filters, ...) to ignore that field.

Procedure
---------
1. Place the robot/IMU perfectly still on a flat, vibration-free surface
   (motors off, nothing touching the robot).

2. Flash the firmware with a TEST_MODE that streams IMU frames over serial,
   e.g. TEST_ALL (components/config/config.h). Frames look like:

       IMU <ax> <ay> <az> <gx> <gy> <gz>

   (ax,ay,az in m/s^2, gx,gy,gz in rad/s -- see serial_comm_send_imu()).

3. Capture ~30-60 s of data at rest:

       python3 tools/capture.py --output data/imu_calib.csv

   Stop the capture with Ctrl+C once enough samples were recorded.

4. Run this script:

       python3 tools/imu_calib/calc_imu_covariance.py data/imu_calib.csv

Output
------
- mean and variance of each axis
- a sanity check on the gravity vector norm and the gyro bias
- the linear_acceleration_covariance / angular_velocity_covariance
  matrices, ready to paste into a ROS yaml/URDF file
- an optional plot of the raw samples (use --no-plot to skip)

Usage:
    python3 tools/imu_calib/calc_imu_covariance.py [capture.csv] [--no-plot]
"""
import sys
import argparse
import numpy as np

GRAVITY = 9.81  # m/s^2, expected |accel| at rest


def load_imu_frames(path):
    """Parse 'IMU ax ay az gx gy gz' lines into an (N, 6) array."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith('IMU'):
                continue
            parts = line.split()
            if len(parts) != 7:
                continue  # malformed frame, skip
            rows.append([float(x) for x in parts[1:]])

    if not rows:
        print(f"No 'IMU ...' frames found in {path}.")
        sys.exit(1)

    return np.array(rows)


def print_covariance(name, var):
    """Print a 3x3 diagonal covariance matrix in ROS row-major form."""
    cov = [0.0] * 9
    cov[0], cov[4], cov[8] = var
    print(f"\n{name}:")
    print('  [' + ', '.join(f'{v:.6e}' for v in cov) + ']')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv', nargs='?', default='capture.csv',
                        help='Capture file with "IMU ax ay az gx gy gz" lines')
    parser.add_argument('--no-plot', action='store_true',
                        help='Skip the raw-data plot')
    args = parser.parse_args()

    data = load_imu_frames(args.csv)
    n = len(data)
    accel = data[:, 0:3]
    gyro = data[:, 3:6]

    accel_mean = accel.mean(axis=0)
    accel_var = accel.var(axis=0, ddof=1)
    gyro_mean = gyro.mean(axis=0)
    gyro_var = gyro.var(axis=0, ddof=1)

    print(f'Loaded {n} samples from {args.csv}\n')

    print('── Accelerometer (m/s^2) ──────────────────────────')
    print(f'{"axis":>6}  {"mean":>10}  {"variance":>12}  {"std":>10}')
    for axis, m, v in zip('xyz', accel_mean, accel_var):
        print(f'{axis:>6}  {m:>10.4f}  {v:>12.3e}  {np.sqrt(v):>10.4e}')

    print('\n── Gyroscope (rad/s) ───────────────────────────────')
    print(f'{"axis":>6}  {"mean":>10}  {"variance":>12}  {"std":>10}')
    for axis, m, v in zip('xyz', gyro_mean, gyro_var):
        print(f'{axis:>6}  {m:>10.4f}  {v:>12.3e}  {np.sqrt(v):>10.4e}')

    # --- Sanity checks ---
    accel_norm = np.linalg.norm(accel_mean)
    print(f'\n|accel_mean| = {accel_norm:.3f} m/s^2 (expected ~{GRAVITY})')
    if abs(accel_norm - GRAVITY) > 0.5:
        print('  -> Warning: gravity norm is off, check that the IMU is '
              'flat and still during the capture.')

    gyro_bias_norm = np.linalg.norm(gyro_mean)
    print(f'|gyro_mean|  = {gyro_bias_norm:.5f} rad/s (expected ~0)')
    if gyro_bias_norm > 0.02:
        print('  -> Non-negligible gyro bias: consider subtracting '
              f'{tuple(round(g, 5) for g in gyro_mean)} from raw readings '
              'before publishing.')

    # --- ROS covariance matrices ---
    print('\n=== ROS sensor_msgs/Imu covariance matrices ===')
    print_covariance('linear_acceleration_covariance', accel_var)
    print_covariance('angular_velocity_covariance', gyro_var)
    print('\norientation_covariance:')
    print('  [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]')
    print('  (no orientation estimate -- MPU6050 has no fusion/magnetometer)')

    # --- Plot ---
    if not args.no_plot:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        t = np.arange(n)
        for i, label in enumerate('xyz'):
            axes[0].plot(t, accel[:, i], label=f'a{label}', linewidth=1)
            axes[1].plot(t, gyro[:, i], label=f'g{label}', linewidth=1)
        axes[0].set_ylabel('Acceleration (m/s^2)')
        axes[1].set_ylabel('Angular velocity (rad/s)')
        axes[1].set_xlabel('Sample')
        for ax in axes:
            ax.legend(); ax.grid(True)
        fig.suptitle(f'IMU static capture ({n} samples)')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
