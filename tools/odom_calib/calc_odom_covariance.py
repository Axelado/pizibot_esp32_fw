#!/usr/bin/env python3
"""
Odometry covariance estimation for ros2_control's diff_drive_controller.

diff_drive_controller publishes nav_msgs/Odometry with two 6x6 diagonal
covariance matrices:

  - pose_covariance_diagonal  : [x, y, z, roll, pitch, yaw]   (m^2, rad^2)
  - twist_covariance_diagonal : [vx, vy, vz, wx, wy, wz]      (m/s)^2, (rad/s)^2

For a 2D differential-drive robot, only x, y, yaw (pose) and vx, wz (twist)
are observable -- z/roll/pitch/vy/wx/wy are left at 0 (or a large constant
if your stack expects "unobserved").

Unlike the IMU, the encoders themselves are essentially noiseless
(quantization only): the real source of odometry error is wheel slip and
calibration error in WHEEL_RADIUS / WHEEL_BASE. That can only be measured
by comparing the firmware's odometry estimate to a real-world, ground-truth
measurement over repeated runs.

Procedure
---------
This uses TEST_ODOM_CALIB (components/config/config.h): the robot performs
ONE calibration motion -- a straight line or an in-place rotation -- using
closed-loop PID control, computes its own odometry (x, y, theta) from the
encoders, stops, and prints:

    ODOM_CALIB <LINEAR|ANGULAR> <x> <y> <theta> <duration_s>
    # odom_calib complete

### A. Linear runs (estimates x, y, yaw error for a straight trajectory)

1. config.h: TEST_MODE = TEST_ODOM_CALIB, ODOM_CALIB_MODE = ODOM_CALIB_LINEAR,
   ODOM_CALIB_DISTANCE = 1.0  (meters). Build & flash.
2. Mark the robot's start position and heading on the floor (tape/chalk).
3. Capture one run:
       python3 tools/capture.py --output data/odom_linear_run1.csv
   then reset the robot. It drives ~1 m forward and stops.
4. Measure the REAL distance traveled (tape measure, start mark to the
   center of the wheel axle) in meters, and APPEND it to the ODOM_CALIB
   line in the capture file, e.g.:
       ODOM_CALIB LINEAR 0.998 0.012 0.003 4.21 1.000
                                              ^^^^^ measured distance (m)
5. Repeat steps 2-4 for run2, run3, ... (5-10 runs recommended), then:
       cat data/odom_linear_run*.csv > data/odom_linear.csv

A straight line is *commanded*, so the ground-truth y and theta are 0 --
the firmware's computed y/theta are themselves the error.

### B. Angular runs (estimates yaw error for an in-place rotation)

1. config.h: ODOM_CALIB_MODE = ODOM_CALIB_ANGULAR,
   ODOM_CALIB_ANGLE = 6.2832 (2*pi = 360°). Build & flash.
2. Mark the robot's heading on the floor.
3. Capture one run, reset the robot, it rotates in place ~360° and stops.
4. Measure the REAL rotation angle in DEGREES (protractor / floor marks)
   and append it to the line, e.g.:
       ODOM_CALIB ANGULAR 0.004 -0.002 6.301 5.02 358.500
                                                   ^^^^^^^ measured angle (deg)
5. Repeat for run2, run3, ..., then:
       cat data/odom_angular_run*.csv > data/odom_angular.csv

An in-place rotation is commanded, so ground-truth x and y are 0.

### C. Compute covariances

    python3 tools/odom_calib/calc_odom_covariance.py data/odom_linear.csv data/odom_angular.csv

Output
------
- mean/std/variance of the x, y and yaw errors (combining both run sets)
- pose_covariance_diagonal / twist_covariance_diagonal, ready to paste into
  the diff_drive_controller yaml config

Usage:
    python3 tools/odom_calib/calc_odom_covariance.py [linear.csv] [angular.csv]
"""
import sys
import argparse
import numpy as np


def parse_runs(path):
    """Parse 'ODOM_CALIB <mode> x y theta t measured' lines."""
    runs = []
    try:
        f = open(path)
    except OSError as e:
        print(f"Could not open {path}: {e}")
        return runs

    with f:
        for line in f:
            line = line.strip()
            if not line.startswith('ODOM_CALIB'):
                continue
            parts = line.split()
            if len(parts) != 7:
                print(f"Skipping malformed line (missing 'measured' column?): {line}")
                continue
            mode = parts[1]
            x, y, theta, t, measured = (float(v) for v in parts[2:])
            runs.append({'mode': mode, 'x': x, 'y': y, 'theta': theta,
                          't': t, 'measured': measured})
    return runs


def report(label, errors):
    errors = np.asarray(errors)
    if len(errors) == 0:
        print(f'{label:>22}: no data')
        return 0.0
    var = errors.var(ddof=1) if len(errors) > 1 else 0.0
    std = errors.std(ddof=1) if len(errors) > 1 else 0.0
    print(f'{label:>22}: mean={errors.mean():+.5f}  std={std:.5f}  '
          f'var={var:.3e}  (n={len(errors)})')
    return var


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('linear_csv', nargs='?', default='data/odom_linear.csv')
    parser.add_argument('angular_csv', nargs='?', default='data/odom_angular.csv')
    args = parser.parse_args()

    lin = parse_runs(args.linear_csv)
    ang = parse_runs(args.angular_csv)

    if not lin and not ang:
        print('No runs found in either file.')
        sys.exit(1)

    # --- Linear runs: ground truth is a straight line (y=0, theta=0) ---
    lin_err_x     = [r['x'] - r['measured'] for r in lin]
    lin_err_y     = [r['y']                 for r in lin]
    lin_err_theta = [r['theta']             for r in lin]
    lin_duration  = [r['t']                 for r in lin]

    # --- Angular runs: ground truth is an in-place rotation (x=0, y=0) ---
    ang_err_x     = [r['x']                                 for r in ang]
    ang_err_y     = [r['y']                                 for r in ang]
    ang_err_theta = [r['theta'] - np.radians(r['measured']) for r in ang]
    ang_duration  = [r['t']                                 for r in ang]

    print(f'Loaded {len(lin)} linear run(s), {len(ang)} angular run(s)\n')

    print('── Linear runs (target: y=0, theta=0) ──────────────')
    report('x error (m)',     lin_err_x)
    report('y error (m)',     lin_err_y)
    report('theta error (rad)', lin_err_theta)

    print('\n── Angular runs (target: x=0, y=0) ──────────────────')
    report('x error (m)',     ang_err_x)
    report('y error (m)',     ang_err_y)
    report('theta error (rad)', ang_err_theta)

    # --- Combined pose covariance (use all available samples per axis) ---
    var_x   = np.var(lin_err_x + ang_err_x, ddof=1) \
              if len(lin_err_x + ang_err_x) > 1 else 0.0
    var_y   = np.var(lin_err_y + ang_err_y, ddof=1) \
              if len(lin_err_y + ang_err_y) > 1 else 0.0
    var_yaw = np.var(lin_err_theta + ang_err_theta, ddof=1) \
              if len(lin_err_theta + ang_err_theta) > 1 else 0.0

    print('\n── Combined pose covariance (x, y, yaw) ─────────────')
    print(f'var_x   = {var_x:.3e} m^2')
    print(f'var_y   = {var_y:.3e} m^2')
    print(f'var_yaw = {var_yaw:.3e} rad^2')

    # --- Twist covariance: var(distance/time) ~= var(distance) / time^2 ---
    # Rough propagation -- valid as long as the run duration itself is
    # accurately known (it is, it's measured by the firmware).
    mean_t_lin = np.mean(lin_duration) if lin_duration else None
    mean_t_ang = np.mean(ang_duration) if ang_duration else None

    var_vx = (var_x / mean_t_lin**2) if mean_t_lin else 0.0
    var_wz = (var_yaw / mean_t_ang**2) if mean_t_ang else 0.0

    print('\n── Twist covariance (vx, wz) ────────────────────────')
    print('(approximation: var(distance)/duration^2, var(angle)/duration^2)')
    print(f'var_vx = {var_vx:.3e} (m/s)^2')
    print(f'var_wz = {var_wz:.3e} (rad/s)^2')

    print('\n=== diff_drive_controller covariance parameters ===')
    print(f'pose_covariance_diagonal : '
          f'[{var_x:.3e}, {var_y:.3e}, 0.0, 0.0, 0.0, {var_yaw:.3e}]')
    print(f'twist_covariance_diagonal: '
          f'[{var_vx:.3e}, 0.0, 0.0, 0.0, 0.0, {var_wz:.3e}]')
    print('\n(z, roll, pitch, vy, wx, wy left at 0.0 -- not observable for '
          'a 2D differential drive)')


if __name__ == '__main__':
    main()
