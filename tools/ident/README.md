# Motor & PID parameter identification

Two stages:

1. **Identification** — measure the motor's open-loop response (`TEST_IDENT`)
   and derive `WHEEL_MAX_RADS`, `PID_KP`, `PID_KI` with `calc_params.py`.
2. **Validation** — check the closed-loop step response (`TEST_PID_STEP`)
   and refine the gains with `tools/pid/analyse_step.py`.

Run both stages once on the bare motors (no load), and again once the robot
is fully assembled (wheels on the ground, real friction/inertia/battery sag) —
gains tuned at no load are usually a bit too "soft" once loaded.

---

## 1. Motor identification (`TEST_IDENT`)

### 1.1 Prepare the firmware

```
config.h : TEST_MODE = TEST_IDENT
```

Compile and flash.

This applies a fixed PWM step (`IDENT_PWM = 512`, 50%) to the **left motor
only** for `IDENT_DURATION = 5 s` and prints `t,vel_rads` at 100 Hz.

### 1.2 Capture data

```bash
python3 tools/capture.py --output data/ident.csv
```

Then reset the ESP32. The script stops automatically after 5 seconds
(line starting with `#`).

### 1.3 Compute parameters

```bash
# Display only
python3 tools/ident/calc_params.py data/ident.csv

# Display AND write to config.h (conservative by default)
python3 tools/ident/calc_params.py data/ident.csv --apply

# Choose a faster dynamic (τ_cl in seconds)
python3 tools/ident/calc_params.py data/ident.csv --tau-cl 0.08 --apply
```

The script fits a first-order + delay model to the step response and
prints:

| Parameter | Meaning |
|-----------|---------|
| K | Final speed at PWM=512 (rad/s) |
| tau | Mechanical time constant (s) |
| L | Pure delay (s) — typically ~0 |
| Kmotor | Static gain (rad/s per raw PWM unit) |

It then computes `WHEEL_MAX_RADS = Kmotor × 1000` and PID gains for three
target closed-loop time constants (IMC method, pole cancellation):

- `τ_cl = 5×tau` → conservative, recommended starting point (default with `--apply`)
- `τ_cl = 2×tau` → moderate
- `τ_cl = tau`   → aggressive, risk of oscillations

`--apply` writes `WHEEL_MAX_RADS`, `PID_KP`, `PID_KI` to
`components/config/config.h` (and resets `PID_KD` to `0.0f`).

---

## 2. PID validation (`TEST_PID_STEP`)

### 2.1 Prepare the firmware

```
config.h : TEST_MODE = TEST_PID_STEP
```

Compile and flash.

This runs **a single PID computed from the left wheel**, and applies the
same PWM output to **both motors** (so the robot drives forward, as in
normal operation, instead of pivoting on one wheel). Setpoint sequence:
3 s at 0 → 3 s at `+STEP_SP` (2 rad/s) → 3 s at `-STEP_SP`. Output:
`t,setpoint,mesure,mesure_r,pwm` at 100 Hz.

### 2.2 Capture data

```bash
python3 tools/capture.py --output data/pid.csv
```

Then reset the ESP32. The script stops automatically at the end (`# pid_step
complete`).

### 2.3 Analyse

```bash
python3 tools/pid/analyse_step.py data/pid.csv
```

This prints, for the `+STEP_SP` phase (left wheel), and plots both wheels'
speed plus the PWM output:

| Metric | Target |
| ------ | ------ |
| Overshoot | < 10 % |
| Rise time | < 300 ms |
| Steady-state error | < 0.1 rad/s |

It also flags PWM saturation (motor stalled or gains too high) or a very
low mean PWM (encoder not reading or gains too low).

### 2.4 Adjust gains

Edit `PID_KP` / `PID_KI` / `PID_KD` in `components/config/config.h`
manually based on the symptoms:

| Symptom | Likely cause | Fix |
| ------- | ------------ | --- |
| Overshoot / damped oscillations | Kp too high | reduce Kp, or add a bit of Kd |
| Slow oscillations (period > 0.5 s) | Ki too high | reduce Ki |
| Slow rise time, no overshoot | Gains too low | increase Kp (and/or Ki) |
| Persistent steady-state error | Ki too low | increase Ki |
| Asymmetry between left/right (`mesure` vs `mesure_r`) | mechanical/friction difference between wheels | acceptable within tolerance; if large, re-check wheel/encoder mounting |

Re-flash and re-run steps 2.1–2.3 after each change until all three
criteria pass on a few consecutive runs.

---

## 3. Re-tuning on the assembled robot

Once the robot chassis is fully assembled (wheels on the ground, battery
in place), repeat **both stages**:

1. Re-run `TEST_IDENT` → `calc_params.py` to get a fresh `Kmotor`/`tau`
   under real load (these typically increase `tau` and decrease
   `Kmotor`/`WHEEL_MAX_RADS` compared to the no-load bench test).
2. Re-run `TEST_PID_STEP` → `analyse_step.py` and fine-tune `PID_KP`/
   `PID_KI`/`PID_KD` as in 2.4.

Run the validation step a few times — friction (stiction) at startup can
cause some run-to-run variability in overshoot/rise time.
