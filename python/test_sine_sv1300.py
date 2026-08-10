#!/usr/bin/env python3
"""Controller regression for SV=130.0 C with a 6-second sine-wave PV.

PV range:
    minimum = 100.0 C (1000)
    maximum = 160.0 C (1600)
SV:
    130.0 C (1300)

The test logs pid_out, ff_pwm, ff_offset, the diagnostic raw sum, and the
controller's actual final heater PWM for every sample. It is opt-in and runs
only when PC_SIMULATION is defined in the environment.
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
TEST_TIME_SEC = 30          # five complete 6-second sine periods
SINE_PERIOD_SEC = 6.0
PV_MIN = 1000
PV_MAX = 1600
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


def _find_compiler() -> tuple[str, str]:
    """Return (compiler_path, compiler_kind)."""
    requested = os.getenv("CC")
    if requested:
        path = shutil.which(requested)
        if path is None:
            raise RuntimeError(f"CC is set but compiler was not found: {requested}")
        name = pathlib.Path(path).name.lower()
        kind = "msvc" if name in {"cl", "cl.exe"} else "gnu"
        return path, kind

    for candidate in ("gcc", "clang", "cl"):
        path = shutil.which(candidate)
        if path is not None:
            kind = "msvc" if candidate == "cl" else "gnu"
            return path, kind

    raise RuntimeError(
        "No C compiler found. Install MinGW/LLVM or Visual Studio Build Tools, "
        "or set CC to the compiler executable. Tried: gcc, clang, cl"
    )


def _build_shared_library() -> pathlib.Path:
    root = _repo_root()
    src_dir = root / "PID_ControllerSrc"
    build_dir = pathlib.Path(__file__).resolve().parent / "build"
    build_dir.mkdir(parents=True, exist_ok=True)
    output = _shared_library_path(build_dir)

    cc, compiler_kind = _find_compiler()

    sources = sorted(str(path.resolve()) for path in src_dir.glob("*.c"))
    if not sources:
        raise RuntimeError(f"No C sources found in {src_dir}")

    system = platform.system().lower()

    if compiler_kind == "msvc":
        cmd = [
            cc,
            "/nologo",
            "/LD",
            "/O2",
            "/DPC_SIMULATION",
            "/DSSM_FB_BUILD_DLL",
            f"/I{root}",
            f"/I{src_dir}",
        ]
        cmd += sources
        cmd += [f"/Fe:{output}"]
        run_cwd = build_dir
    else:
        cmd = [
            cc,
            "-std=c11",
            "-O2",
            "-DPC_SIMULATION",
            "-I", str(root),
            "-I", str(src_dir),
        ]

        if system == "windows":
            cmd += ["-shared", "-DSSM_FB_BUILD_DLL"]
        elif system == "darwin":
            cmd += ["-dynamiclib", "-fPIC", "-DSSM_FB_BUILD_SHARED"]
        else:
            cmd += ["-shared", "-fPIC", "-DSSM_FB_BUILD_SHARED"]

        cmd += sources
        cmd += ["-o", str(output)]
        run_cwd = root

    print(f"Compiler: {cc} ({compiler_kind})")
    print("Building shared library:")
    print(" ".join(cmd))
    subprocess.run(cmd, check=True, cwd=run_cwd)

    if not output.exists():
        raise RuntimeError(f"Compiler completed but library was not created: {output}")

    return output


def _load_api(path: pathlib.Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(path))

    lib.Heater_Control_InitTimed.argtypes = [ctypes.c_uint32]
    lib.Heater_Control_InitTimed.restype = ctypes.c_int

    lib.Heater_myAdptiveControl.argtypes = [
        ctypes.c_int16,
        ctypes.c_int16,
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
    ]
    lib.Heater_myAdptiveControl.restype = None

    return lib


def _run(lib: ctypes.CDLL) -> pathlib.Path:
    status = lib.Heater_Control_InitTimed(SAMPLE_TIME_MS)
    if status != 0:
        raise RuntimeError(
            f"Heater_Control_InitTimed({SAMPLE_TIME_MS}) failed: {status}"
        )

    sample_count = (TEST_TIME_SEC * 1000) // SAMPLE_TIME_MS
    samples_per_period = (SINE_PERIOD_SEC * 1000.0) / SAMPLE_TIME_MS
    pv_center = (PV_MAX + PV_MIN) / 2.0
    pv_amplitude = (PV_MAX - PV_MIN) / 2.0

    output_path = pathlib.Path(__file__).resolve().parent / "sv1300_sine6s.csv"

    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "time_ms",
            "pv",
            "sv",
            "error",
            "pid_out",
            "ff_pwm",
            "ff_offset",
            "raw_sum",
            "controller_pwm",
        ])

        for i in range(sample_count):
            angle = 2.0 * math.pi * i / samples_per_period
            pv = int(round(pv_center + pv_amplitude * math.sin(angle)))

            pid_out = ctypes.c_int32()
            ff_pwm = ctypes.c_int32()
            ff_offset = ctypes.c_int32()
            heater_pwm = ctypes.c_int32()

            lib.Heater_myAdptiveControl(
                ctypes.c_int16(pv),
                ctypes.c_int16(SV_VALUE),
                ctypes.byref(pid_out),
                ctypes.byref(ff_pwm),
                ctypes.byref(ff_offset),
                ctypes.byref(heater_pwm),
            )

            raw_sum = pid_out.value + ff_pwm.value + ff_offset.value

            writer.writerow([
                i * SAMPLE_TIME_MS,
                pv,
                SV_VALUE,
                SV_VALUE - pv,
                pid_out.value,
                ff_pwm.value,
                ff_offset.value,
                raw_sum,
                heater_pwm.value,
            ])

    return output_path


def main() -> int:
    if not _enabled():
        print("PC_SIMULATION is not defined; test is disabled.")
        return 0

    try:
        lib_path = _build_shared_library()
        lib = _load_api(lib_path)
        csv_path = _run(lib)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"SV1300 sine regression failed: {exc}", file=sys.stderr)
        return 1

    print("SV1300 sine-wave regression completed.")
    print(f"SV           : {SV_VALUE}")
    print(f"PV range     : {PV_MIN} .. {PV_MAX}")
    print(f"Sine period  : {SINE_PERIOD_SEC:.1f} s")
    print(f"Sample time  : {SAMPLE_TIME_MS} ms")
    print(f"Test duration: {TEST_TIME_SEC} s")
    print(f"CSV          : {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
