# Motor parameter identification

## Workflow

### 1. Prepare the firmware

```
config.h : TEST_MODE = TEST_IDENT
```

Compile and flash.

### 2. Capture data

```bash
python3 tools/capture.py --output data/ident.csv
```

Then reset the ESP32. The script stops automatically after 5 seconds.

### 3. Compute parameters

```bash
# Display only
python3 tools/ident/calc_params.py data/ident.csv

# Display AND write to config.h (conservative by default)
python3 tools/ident/calc_params.py data/ident.csv --apply

# Choose a faster dynamic (τ_cl in seconds)
python3 tools/ident/calc_params.py data/ident.csv --tau-cl 0.08 --apply
```

## Identified parameters

| Parameter | Meaning |
|-----------|---------|
| K | Final speed at PWM=512 (rad/s) |
| tau | Mechanical time constant (s) |
| L | Pure delay (s) — typically ~0 |
| Kmotor | Static gain (rad/s per raw unit) |

## Choosing τ_cl

- `τ_cl = 5×tau` → conservative, recommended starting point
- `τ_cl = 2×tau` → moderate
- `τ_cl = tau`   → aggressive, risk of oscillations

Then validate with `TEST_PID_STEP` + `tools/pid/analyse_step.py`.
