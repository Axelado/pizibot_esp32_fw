#!/usr/bin/env python3
"""
Plot wheel velocity vs PWM duty from a TEST_MOTOR_CURVE capture.

Parses "MOTOR_CURVE <duty> <vel_l> <vel_r>" lines and plots vel_l/vel_r
against duty, to visualize the motor's dead zone, linear region and
saturation.

Usage:
    python3 tools/motor_curve/plot_motor_curve.py [capture.csv]
"""
import sys
import numpy as np
import matplotlib.pyplot as plt


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'capture.csv'

    duty, vel_l, vel_r = [], [], []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 4 and parts[0] == 'MOTOR_CURVE':
                duty.append(float(parts[1]))
                vel_l.append(float(parts[2]))
                vel_r.append(float(parts[3]))

    if not duty:
        print(f'No MOTOR_CURVE lines found in {path}')
        sys.exit(1)

    duty = np.array(duty)
    vel_l = np.array(vel_l)
    vel_r = np.array(vel_r)

    plt.figure(figsize=(8, 5))
    plt.plot(duty, vel_l, label='Left wheel')
    plt.plot(duty, vel_r, label='Right wheel')
    plt.xlabel('PWM duty')
    plt.ylabel('Velocity (rad/s)')
    plt.title('Wheel velocity vs PWM duty (open-loop)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
