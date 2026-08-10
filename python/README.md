# Python PC Simulation

This folder contains the PC-side regression driver for the adaptive temperature controller.

## Enable the test

The test is opt-in. Define environment variable `PC_SIMULATION` before running it.

Windows CMD:

```bat
set PC_SIMULATION=1
python python\test_adaptive_controller.py
```

Windows PowerShell:

```powershell
$env:PC_SIMULATION="1"
python .\python\test_adaptive_controller.py
```

Linux/macOS:

```bash
PC_SIMULATION=1 python3 python/test_adaptive_controller.py
```

If `PC_SIMULATION` is not defined, the script exits without building or running the simulation.

## What the script does

1. Builds all `PID_ControllerSrc/*.c` files into a shared library.
2. Uses `ctypes` to load the exported controller API.
3. Calls `Heater_Control_InitTimed()` with the configured sample time.
4. Runs the previous sine-wave PV regression through `Heater_myAdptiveControl()`.
5. Writes `python/simulation.csv`.

The compiler defaults to `gcc`. Set environment variable `CC` to use another compatible compiler executable.
