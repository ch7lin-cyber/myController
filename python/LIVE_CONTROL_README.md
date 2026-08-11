# Live Modbus TCP Heater Control

## Target

- IP: `192.168.1.10`
- Port: `2502`
- PV holding register: `92`
- MV holding register: `108`
- Control period: `20 ms`
- Temperature unit: `0.1 degC`
- PWM unit: `0..1000 = 0.0..100.0%`

The tool starts in **STOP** and writes `MV=0` before accepting RUN.

## Install

```powershell
python -m pip install pymodbus
```

A native C compiler is also required. If MinGW GCC is not in PATH:

```powershell
$env:CC="C:\mingw64\bin\gcc.exe"
```

## Start

From the repository root:

```powershell
python .\python\live_modbus_controller.py
```

Optional explicit settings:

```powershell
python .\python\live_modbus_controller.py `
    --host 192.168.1.10 `
    --port 2502 `
    --pv-reg 92 `
    --mv-reg 108 `
    --unit-id 1 `
    --sample-ms 20 `
    --sv 1300
```

If the target Modbus device uses a unit/device ID other than 1, change
`--unit-id`.

## Terminal commands

```text
run
stop
sv 1300
sv 130.0c
status
help
quit
```

Examples:

```text
sv 1300
run
status
stop
quit
```

`run` re-initializes the controller before enabling closed-loop control.
`stop`, `quit`, Ctrl+C, PV range failure, or a Modbus exception causes the
program to attempt `MV=0`.

## CSV

Logs are written under:

```text
python/logs/live_control_YYYYMMDD_HHMMSS.csv
```

Important columns for controller analysis:

- `state`: RUN / STOP
- `sv`, `pv`, `error`: controller native temperature unit (0.1 degC)
- `pid_out`: PID correction
- `ff_pwm`: FF table base PWM
- `ff_offset`: adaptive-learning FF offset
- `raw_sum`: PID + FF + offset diagnostic sum
- `controller_pwm`: final PWM produced by the C controller
- `mv_written`: value actually requested through Modbus register 108
- `host_dpv_filtered_per_sample`: host-side filtered PV delta estimate
- `host_pv_rate_0p1c_per_s`: estimated heating/cooling rate
- `host_predicted_pv`: host-side V3-style future PV estimate
- `host_predicted_error`: SV - predicted PV
- `host_fast_heat_active`: host estimate of Fast Heat state
- `host_predictive_brake_active`: host estimate of Predictive Brake state
- `read_ms`, `write_ms`: Modbus transaction duration
- `control_us`: native C controller execution time
- `cycle_ms`: total work time in one control cycle
- `jitter_ms`: scheduler timing error
- `deadline_miss`: 1 when the 20 ms deadline was missed
- `comm_ok`, `comm_errors`: communication health
- `note`: command/fault event

The `host_*` predictive fields are diagnostic estimates calculated by Python.
The actual actuator command is always `controller_pwm` from the C controller.

## Recommended first real test

1. Start the program. It remains STOP.
2. Confirm PV displayed in terminal is correct.
3. Enter `sv 1300`.
4. Enter `run`.
5. Observe PV and PWM. Be ready to enter `stop`.
6. Stop after the temperature response has clearly passed through the approach
   and hold regions, then enter `quit`.
7. Provide the generated CSV for analysis.

For the first V3 tuning pass, the most useful interval is from ambient through
130 degC and several minutes of post-arrival thermal coast/hold behavior.
