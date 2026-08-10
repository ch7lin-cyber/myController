#!/usr/bin/env python3
"""PC regression driver for the adaptive temperature controller.

The test is opt-in. It only runs when environment variable PC_SIMULATION is
set. The script builds the C controller sources as a shared library, loads the
library with ctypes, runs the legacy sine-wave PV regression, and writes a CSV.
"""

from __future__ import annotations

import csv
import ctypes
import math
import os
import pathlib
import platform
import shutil
import subprocess
import sys


SAMPLE_TIME_MS = 20
TEST_TIME_SEC = 20
SINE_PERIOD_SEC = 10.0
PV_MIN = 500
PV_MAX = 1750
SV_VALUE = 1300


def _enabled() -> bool:
    value = os.getenv("PC_SIMULATION")
    if value is None:
        return False
    return value.strip().lower() not in {"", "0", "false", "off", "no"}


def _repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def _shared_library_path(build_dir: pathlib.Path) -> pathlib.Path:
    system = platform.system().lower()
    if system == "windows":
        return build_dir / "myController.dll"
    if system == "darwin":
        return build_dir / "libmyController.dylib"
    return build_dir / "libmyController.so"


def _build_shared_library() -> pathlib.Path:
    root = _repo_root()
    src_dir = root / "PID_ControllerSrc"
    build_dir = pathlib.Path(__file__).resolve().parent / "build"
    build_dir.mkdir(parents=True, exist_ok=True)
    output = _shared_library_path(build_dir)

    gcc = os.getenv("CC", "gcc")
    if shutil.which(gcc) is None:
        raise RuntimeError(f"C compiler not found: {gcc}")

    sources = sorted(str(path) for path in src_dir.glob("*.c"))
    if not sources:
        raise RuntimeError(f"No C sources found in {src_dir}")

    cmd = [
        gcc,
        "-std=c11",
        "-O2",
        "-DPC_SIMULATION",
        "-I",
        str(root),
        "-I",
        str(src_dir),
    ]

    if platform.system().lower() == "windows":
        cmd += ["-shared", "-DSSM_FB_BUILD_DLL"]
    elif platform.system().lower() == "darwin":
        cmd += ["-dynamiclib", "-fPIC", "-DSSM_FB_BUILD_SHARED"]
    else:
        cmd += ["-shared", "-fPIC", "-DSSM_FB_BUILD_SHARED"]

    cmd += sources
    cmd += ["-o", str(output)]

    print("Building shared library:")
    print(" ".join(cmd))
    subprocess.run(cmd, check=True, cwd=root)
    return output


def _load_api(library_path: pathlib.Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(library_path))

    lib.Heater_Control_InitTimed.argtypes = [ctypes.c_uint32]
    lib.Heater_Control_InitTimed.restype = ctypes.c_int

    lib.Heater_GetSampleTimeMs.argtypes = []
    lib.Heater_GetSampleTimeMs.restype = ctypes.c_uint32

    lib.Heater_myAdptiveControl.argtypes = [
        ctypes.c_int16,
        ctypes.c_int16,
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
    ]
    lib.Heater_myAdptiveControl.restype = None

    return lib


def _run_sine_regression(lib: ctypes.CDLL) -> pathlib.Path:
    status = lib.Heater_Control_InitTimed(SAMPLE_TIME_MS)
    if status != 0:
        raise RuntimeError(
            f"Heater_Control_InitTimed({SAMPLE_TIME_MS}) failed: {status}"
        )

    sample_count = (TEST_TIME_SEC * 1000) // SAMPLE_TIME_MS
    sample_per_cycle = (SINE_PERIOD_SEC * 1000.0) / SAMPLE_TIME_MS
    pv_center = (PV_MAX + PV_MIN) / 2.0
    pv_amp = (PV_MAX - PV_MIN) / 2.0

    output_path = pathlib.Path(__file__).resolve().parent / "simulation.csv"

    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_ms",
                "pv",
                "sv",
                "error",
                "pid_out",
                "ff_pwm",
                "ff_offset",
                "total_output",
            ]
        )

        for i in range(sample_count):
            angle = 2.0 * math.pi * i / sample_per_cycle
            pv = int(pv_center + pv_amp * math.sin(angle))

            pid_out = ctypes.c_int32()
            ff_pwm = ctypes.c_int32()
            ff_offset = ctypes.c_int32()

            lib.Heater_myAdptiveControl(
                ctypes.c_int16(pv),
                ctypes.c_int16(SV_VALUE),
                ctypes.byref(pid_out),
                ctypes.byref(ff_pwm),
                ctypes.byref(ff_offset),
            )

            writer.writerow(
                [
                    i * SAMPLE_TIME_MS,
                    pv,
                    SV_VALUE,
                    SV_VALUE - pv,
                    pid_out.value,
                    ff_pwm.value,
                    ff_offset.value,
                    pid_out.value + ff_pwm.value + ff_offset.value,
                ]
            )

    return output_path


def main() -> int:
    if not _enabled():
        print("PC_SIMULATION is not defined; Python controller test is disabled.")
        return 0

    try:
        library_path = _build_shared_library()
        lib = _load_api(library_path)
        csv_path = _run_sine_regression(lib)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"PC simulation failed: {exc}", file=sys.stderr)
        return 1

    print("PC simulation completed.")
    print(f"Library     : {library_path}")
    print(f"Output CSV  : {csv_path}")
    print(f"Sample time : {lib.Heater_GetSampleTimeMs()} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
