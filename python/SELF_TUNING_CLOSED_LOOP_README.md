# Branch 5 Closed-Loop Self-Tuning Regression

This regression connects the real branch-5 heater controller to the existing
`ControllPlant` thermal model:

```text
Heater controller PWM
        -> ThermalPlant_Step()
        -> simulated PV
        -> Heater_myAdptiveControl()
        -> PWM ...
```

## Why the test targets are 130C and 150C

The current `ControllPlant/myPlant_1.c` model is identified at MV = 20%, 50%
and 80%. The 80..100% region is extrapolated. At 25C ambient, the model predicts
only about 168.6C equilibrium even at 100% MV. Therefore 175C is not used as a
closed-loop settling requirement in this regression. A 175C test would mainly
measure the plant-model limitation, not controller quality.

## Run

Windows PowerShell:

```powershell
$env:PC_SIMULATION="1"
python python/test_self_tuning_closed_loop.py
```

Linux/macOS:

```bash
PC_SIMULATION=1 python3 python/test_self_tuning_closed_loop.py
```

The script requires a C11 compiler (`gcc` by default; override with `CC`).

## Test sequence

1. Build all `PID_ControllerSrc/*.c` files plus `ControllPlant/myPlant_1.c` into
   one shared library.
2. Run baseline episodes with Self-Tuning OFF at 50->130C and 50->150C.
3. Reinitialize once, enable Self-Tuning, then repeat 130C / 150C heating
   responses three times while preserving learned parameters across episodes.
4. Verify:
   - Self-Tuning OFF never changes Ki or Predictive Time.
   - Learning responses are qualified heating steps.
   - Ki remains inside 300..1200.
   - Predictive Time remains inside 500..2000 ms.
   - Tune count never decreases inside a learning session.
   - At least one learning response reaches settled state.
5. A zero tune count is reported as a warning rather than an automatic failure,
   because a well-matched baseline controller can legitimately need no change.

## Generated files

`python/self_tuning_closed_loop_trace.csv`

Detailed time-series log containing PV, PWM, PID/FF terms, current Ki,
Predictive Time, qualification state, overshoot, steady error, cooldown and
Tune Count.

`python/self_tuning_closed_loop_summary.csv`

One row per heating episode with settling time, peak, overshoot, steady error,
final Ki, final Predictive Time and Tune Count.

## Reading convergence

Do not judge convergence from Tune Count alone. Compare successive ON episodes
for the same target:

- Overshoot should not trend upward continuously.
- Settling time should remain stable or improve.
- Ki should stop walking once steady error is within the configured threshold.
- Predictive Time should stop walking once overshoot / early braking error no
  longer asks for a change.
- Parameters must remain within their configured safety bounds.

If parameters repeatedly hit a bound, oscillate between two values, or keep
changing without performance improvement, the tuning rule needs refinement
before hardware use.

## Diagnostics behavior after a load disturbance

The Process Observer now clears `settled` when PV leaves the settle band after a
disturbance, so diagnostics accurately report that the system is no longer
settled. This does **not** create a new tuning event by itself. Parameter tuning
still requires a new qualified heating SV step.
