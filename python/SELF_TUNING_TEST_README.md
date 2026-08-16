# Branch 5 Self-Tuning Regression

This test suite validates the `branch5_selfTuningDesign` behavior while keeping self tuning opt-in and diagnostics always active.

## Run

Linux/macOS:

```bash
PC_SIMULATION=1 python3 python/test_self_tuning.py
```

Windows PowerShell:

```powershell
$env:PC_SIMULATION="1"
python python/test_self_tuning.py
```

The script compiles every `PID_ControllerSrc/*.c` file into a shared library and loads the public `Heater_*` API through `ctypes`.

## Cases

1. `test_default_off_diagnostics_on`
   - Self tuning is OFF immediately after initialization.
   - Observer diagnostics still update.
   - No parameter commit occurs while tuning is disabled.

2. `test_small_step_rejected`
   - A +3.0 C heating request is below the +5.0 C qualification threshold.
   - Diagnostics settle normally, but no tuning occurs.

3. `test_qualified_steady_error_commit`
   - A qualified heating response settles with +0.2 C residual error.
   - Ki is trimmed upward by one step.
   - No-overshoot residual error also shortens the predictive horizon by one step.

4. `test_overshoot_increases_prediction`
   - A qualified heating response reaches at least +0.6 C overshoot.
   - Predictive brake horizon increases by 100 ms.

5. `test_cooling_step_diagnostics_only`
   - Cooling response remains visible in diagnostics.
   - It is not eligible for heater predictive-brake learning.

6. `test_cooldown_blocks_second_commit`
   - A second qualified event inside the 30 s cooldown may settle, but cannot commit parameters.

7. `test_session_commit_limit`
   - At most 20 parameter commits are allowed in one controller initialization/session.
   - Event 21 remains diagnostic-only.

8. `test_pid_ki_reconfigure_bumpless`
   - A generated private test helper checks that changing Ki from 300 to 325 inversely rescales the PID integral state.
   - The `Ki * integral` contribution is preserved within integer-rounding error.

## Default protection values

- Self tuning: OFF
- Diagnostics: ON in AUTO
- Minimum qualified heating step: +5.0 C
- Settle band: +/-0.2 C
- Settle time: 3 s
- Steady-error tuning threshold: 0.1 C
- Overshoot threshold: 0.5 C
- Tune cooldown: 30 s
- Maximum commits per session: 20
- Ki range: 300..1200, step 25
- Predictive horizon range: 500..2000 ms, step 100 ms

## Expected output

A successful run prints eight `PASS:` lines followed by:

```text
All 8 self-tuning regression tests passed.
```

A compile failure or assertion failure returns a non-zero process exit code.
