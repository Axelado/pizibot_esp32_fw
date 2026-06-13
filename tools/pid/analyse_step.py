#!/usr/bin/env python3
"""
PID step response analysis.

Usage:
    python3 tools/pid/analyse_step.py                  # reads capture.csv
    python3 tools/pid/analyse_step.py data/pid.csv
"""
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', nargs='?', default='capture.csv')
    args = parser.parse_args()

    data = np.loadtxt(args.csv, delimiter=',', skiprows=1, comments='#')
    t, sp, mes, mes_r, pwm = data[:, 0], data[:, 1], data[:, 2], data[:, 3], data[:, 4]

    # --- Metrics for the positive phase (t = 3s to 6s) ---
    mask   = (t >= 3.0) & (t < 6.0)
    t_step  = t[mask] - 3.0
    mes_step = mes[mask]
    sp_val  = np.median(sp[mask])

    print(f'\n── Results ─────────────────────────────────')

    if sp_val > 0.01:
        peak      = mes_step.max()
        overshoot = max(0.0, (peak - sp_val) / sp_val * 100)

        idx10 = np.where(mes_step >= 0.1 * sp_val)[0]
        idx90 = np.where(mes_step >= 0.9 * sp_val)[0]
        t10 = t_step[idx10[0]] if len(idx10) else float('nan')
        t90 = t_step[idx90[0]] if len(idx90) else float('nan')
        rise_time = t90 - t10

        mask_ss = (t >= 5.0) & (t < 6.0)
        steady_error = abs(sp_val - mes[mask_ss].mean())

        ok   = lambda ok: '✓' if ok else '✗'
        print(f'Target setpoint  : {sp_val:.2f} rad/s')
        print(f'Overshoot        : {overshoot:.1f} %      {ok(overshoot < 10)}  (< 10%)')
        print(f'Rise time        : {rise_time*1000:.0f} ms      {ok(rise_time < 0.3)}  (< 300 ms)')
        print(f'Steady-state err : {steady_error:.3f} rad/s  {ok(steady_error < 0.1)}  (< 0.1)')
    else:
        print('Null setpoint in the 3-6s window — check the CSV.')

    # --- Diagnostic PWM ---
    mask_step = (t >= 3.1) & (t < 6.0)
    pwm_max  = pwm[mask_step].max()
    pwm_mean = pwm[mask_step].mean()
    print(f'\nPWM max   : {pwm_max:.0f}/1000')
    print(f'PWM moyen : {pwm_mean:.0f}/1000')
    if pwm_max >= 990:
        print('⚠  PWM saturated — motor stalled or gains too high')
    elif pwm_mean < 30:
        print('⚠  PWM very low — encoder not reading or gains too low')

    # --- Plot ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(t, sp,   '--', label='setpoint',        linewidth=1.5)
    ax1.plot(t, mes,        label='measured (left)', linewidth=1.5)
    ax1.plot(t, mes_r,      label='measured (right)', linewidth=1.5)
    for xv in [3.0, 6.0]:
        ax1.axvline(xv, color='gray', linestyle=':', linewidth=1)
    ax1.set_ylabel('Speed (rad/s)')
    ax1.set_title('PID step response')
    ax1.legend(); ax1.grid(True)

    ax2.plot(t, pwm, color='tab:orange', linewidth=1.5, label='PWM out')
    ax2.axhline( 1000, color='red', linestyle=':', linewidth=1, label='saturation')
    ax2.axhline(-1000, color='red', linestyle=':', linewidth=1)
    for xv in [3.0, 6.0]:
        ax2.axvline(xv, color='gray', linestyle=':', linewidth=1)
    ax2.set_ylabel('PWM raw [-1000, 1000]')
    ax2.set_xlabel('Time (s)')
    ax2.legend(); ax2.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
