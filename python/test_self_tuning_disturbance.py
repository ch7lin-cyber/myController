#!/usr/bin/env python3
"""Closed-loop load-disturbance regression for branch5 self tuning.

The plant is first driven to 130C. A simulated -10C thermal disturbance is then
injected directly into the plant state while SV remains unchanged. Expected:

1. diagnostics.settled becomes false after PV leaves the settle band;
2. diagnostics.response_active becomes true during recovery;
3. tune_count does not increase because no new qualified SV step occurred;
4. the controller drives the plant back toward SV.
"""

from __future__ import annotations

import ctypes
import os
import subprocess
import sys

import test_self_tuning_closed_loop as cl

TARGET_X10 = 1300
DISTURBANCE_C = 10.0
RECOVERY_TIMEOUT_S = 120.0


def main() -> int:
    if not cl.enabled():
        print("PC_SIMULATION is not defined; disturbance regression is disabled.")
        return 0

    try:
        path = cl.build_library()
        lib = cl.load_api(path)

        plant = cl.ThermalPlant()
        lib.ThermalPlant_Init(
            ctypes.byref(plant),
            ctypes.c_float(cl.RESET_TEMP_C),
            ctypes.c_float(cl.AMBIENT_C),
            ctypes.c_float(cl.SAMPLE_TIME_S),
        )

        status = lib.Heater_Control_InitTimed(cl.SAMPLE_TIME_MS)
        if status != 0:
            raise RuntimeError(f"Heater_Control_InitTimed failed: {status}")
        lib.Heater_SetSelfTuningEnable(1)

        # Create a low-SV state then a qualified heating response.
        cl.prepare_episode(lib, plant)

        max_steps = int(cl.EPISODE_TIMEOUT_S * 1000 / cl.SAMPLE_TIME_MS)
        for _ in range(max_steps):
            pv = int(lib.ThermalPlant_GetTemperature_x10(ctypes.byref(plant)))
            _, _, _, pwm = cl.controller_step(lib, pv, TARGET_X10)
            lib.ThermalPlant_Step(ctypes.byref(plant), ctypes.c_float(pwm / 10.0))
            d = cl.get_diag(lib)
            if d.settled:
                break
        else:
            raise AssertionError("Plant did not settle before disturbance")

        before = cl.get_diag(lib)
        tune_count_before = int(before.tune_count)
        if not before.settled:
            raise AssertionError("Expected settled diagnostics before disturbance")

        # Inject a load-like disturbance without changing SV.
        plant.temperature_c -= ctypes.c_float(DISTURBANCE_C).value

        pv = int(lib.ThermalPlant_GetTemperature_x10(ctypes.byref(plant)))
        _, _, _, pwm = cl.controller_step(lib, pv, TARGET_X10)
        lib.ThermalPlant_Step(ctypes.byref(plant), ctypes.c_float(pwm / 10.0))
        disturbed = cl.get_diag(lib)

        if disturbed.settled:
            raise AssertionError("Diagnostics remained settled after -10C disturbance")
        if not disturbed.response_active:
            raise AssertionError("Observer did not report active recovery response")
        if int(disturbed.tune_count) != tune_count_before:
            raise AssertionError("Load disturbance incorrectly triggered tuning commit")

        recovery_steps = int(RECOVERY_TIMEOUT_S * 1000 / cl.SAMPLE_TIME_MS)
        recovered = False
        for _ in range(recovery_steps):
            pv = int(lib.ThermalPlant_GetTemperature_x10(ctypes.byref(plant)))
            _, _, _, pwm = cl.controller_step(lib, pv, TARGET_X10)
            lib.ThermalPlant_Step(ctypes.byref(plant), ctypes.c_float(pwm / 10.0))
            d = cl.get_diag(lib)
            if int(d.tune_count) != tune_count_before:
                raise AssertionError("Recovery at unchanged SV triggered tuning commit")
            if d.settled:
                recovered = True
                break

        if not recovered:
            raise AssertionError("Controller did not recover to settled state")

        final = cl.get_diag(lib)
        print("PASS: closed-loop load disturbance")
        print(f"Tune count       : {final.tune_count} (unchanged)")
        print(f"Current Ki       : {final.current_ki}")
        print(f"Predictive time  : {final.current_predictive_time_ms} ms")
        print(f"Recovery settle  : {final.settling_time_ms} ms observer time")
        return 0

    except (AssertionError, OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
