#!/usr/bin/env python3
"""
Motor parameter identification + IMC PID gain calculation.

Usage:
    python3 tools/ident/calc_params.py                       # reads capture.csv
    python3 tools/ident/calc_params.py data/ident.csv
    python3 tools/ident/calc_params.py data/ident.csv --apply  # writes config.h
"""
import sys
import os
import re
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

IDENT_PWM    = 512      # must match IDENT_PWM in tests.c
PWM_RAW_MAX  = 1000
CONFIG_PATH  = os.path.join(os.path.dirname(__file__),
                            '../../components/config/config.h')


def model(t, K, tau, L):
    """Premier ordre avec retard : v(t) = K*(1-exp(-(t-L)/tau)) pour t>L."""
    return np.where(t < L, 0.0, K * (1.0 - np.exp(-(t - L) / tau)))


def imc_gains(kmotor, tau, tau_cl):
    """IMC formulas for a PI with pole cancellation."""
    ki = 1.0 / (kmotor * tau_cl)
    kp = ki * tau
    return kp, ki


def update_config(wheel_max_rads, kp, ki):
    with open(CONFIG_PATH, 'r') as f:
        content = f.read()

    content = re.sub(r'(#define WHEEL_MAX_RADS\s+)[\d.]+f',
                     rf'\g<1>{wheel_max_rads:.1f}f', content)
    content = re.sub(r'(#define PID_KP\s+)[\d.]+f',
                     rf'\g<1>{kp:.1f}f', content)
    content = re.sub(r'(#define PID_KI\s+)[\d.]+f',
                     rf'\g<1>{ki:.1f}f', content)
    content = re.sub(r'(#define PID_KD\s+)[\d.]+f',
                     r'\g<1>0.0f', content)

    with open(CONFIG_PATH, 'w') as f:
        f.write(content)
    print(f'\nconfig.h updated:')
    print(f'  WHEEL_MAX_RADS = {wheel_max_rads:.1f}f')
    print(f'  PID_KP         = {kp:.1f}f')
    print(f'  PID_KI         = {ki:.1f}f')
    print(f'  PID_KD         = 0.0f')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', nargs='?', default='capture.csv')
    parser.add_argument('--apply', action='store_true',
                        help='Write gains to config.h')
    parser.add_argument('--tau-cl', type=float, default=None,
                        help='Target time constant (s). Default: 5×tau (conservative)')
    args = parser.parse_args()

    # --- Load data ---
    data = np.loadtxt(args.csv, delimiter=',', skiprows=1, comments='#')
    t, v = data[:, 0], data[:, 1]

    # --- Curve fitting ---
    p0 = [v.max(), 0.1, 0.02]
    try:
        popt, _ = curve_fit(model, t, v, p0=p0,
                            bounds=([0, 0.001, 0], [50, 2, 0.5]))
    except RuntimeError:
        print('Curve fit failed — check the data.')
        sys.exit(1)

    K, tau, L = popt
    kmotor = K / IDENT_PWM                   # rad/s per raw unit
    wheel_max_rads = kmotor * PWM_RAW_MAX     # max speed at PWM=1000

    print(f'\n── Identified parameters ──────────────────')
    print(f'K (speed at PWM={IDENT_PWM})   : {K:.3f} rad/s')
    print(f'tau (time constant)        : {tau*1000:.1f} ms')
    print(f'L   (pure delay)           : {L*1000:.1f} ms')
    print(f'Kmotor                     : {kmotor:.5f} rad/s/raw')
    print(f'\n→ Suggested WHEEL_MAX_RADS : {wheel_max_rads:.1f} rad/s')

    # --- PID gains for several τ_cl values ---
    print(f'\n── PID gains (IMC method) ──────────────────')
    print(f'{"τ_cl (ms)":>12}  {"Description":^20}  {"Kp":>8}  {"Ki":>8}')
    print('-' * 55)
    scenarios = [
        (5 * tau, 'conservative ★'),
        (2 * tau, 'moderate'),
        (tau,     'aggressive'),
    ]
    chosen_kp, chosen_ki = None, None
    for tau_cl, label in scenarios:
        kp, ki = imc_gains(kmotor, tau, tau_cl)
        star = ' ←' if 'conservative' in label and args.tau_cl is None else ''
        print(f'{tau_cl*1000:>12.1f}  {label:^20}  {kp:>8.1f}  {ki:>8.1f}{star}')
        if 'conservative' in label and args.tau_cl is None:
            chosen_kp, chosen_ki = kp, ki

    if args.tau_cl is not None:
        chosen_kp, chosen_ki = imc_gains(kmotor, tau, args.tau_cl)
        print(f'\nCustom τ_cl {args.tau_cl*1000:.1f} ms: Kp={chosen_kp:.1f}  Ki={chosen_ki:.1f}')

    # --- Write config.h ---
    if args.apply:
        update_config(wheel_max_rads, chosen_kp, chosen_ki)
    else:
        print(f'\n(Add --apply to write these values to config.h)')

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, v, label='measured', linewidth=1.5)
    ax.plot(t, model(t, *popt), '--',
            label=f'model  K={K:.2f}  τ={tau*1000:.0f}ms  L={L*1000:.1f}ms',
            linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (rad/s)')
    ax.set_title(f'Motor identification — PWM={IDENT_PWM}/1000')
    ax.legend(); ax.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
