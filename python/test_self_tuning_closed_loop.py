#!/usr/bin/env python3
"""Closed-loop thermal plant regression for branch5 self tuning.

Loop:
    Heater controller PWM -> ThermalPlant -> PV -> Heater controller

The test is opt-in and runs only when PC_SIMULATION is enabled. It builds the
controller and ControllPlant sources into one shared library, executes baseline
(Self-Tuning OFF) and learning (Self-Tuning ON) episodes, logs detailed CSV
traces, and writes an episode summary CSV.

Important model limitation:
    The current plant is identified to 80% MV and extrapolated above 80%.
    Its predicted equilibrium at 100% MV is about 168.6 degC, therefore this
    regression uses 130C and 150C setpoints rather than treating 175C as a
    reachable steady-state target.
"""

from __future__ import annotations

import csv
import ctypes
import os
import pathlib
import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass

SAMPLE_TIME_MS = 20
SAMPLE_TIME_S = SAMPLE_TIME_MS / 1000.0
AMBIENT_C = 25.0
RESET_TEMP_C = 50.0
LOW_SV_X10 = 500
EPISODE_TIMEOUT_S = 120.0
PREPARE_LOW_S = 1.0
TARGETS_X10 = (1300, 1500)
LEARNING_REPEATS = 3


class Diagnostics(ctypes.Structure):
    _fields_ = [
        ("self_tuning_enabled", ctypes.c_int32),
        ("response_active", ctypes.c_int32),
        ("settled", ctypes.c_int32),
        ("response_qualified", ctypes.c_int32),
        ("current_ki", ctypes.c_int32),
        ("current_predictive_time_ms", ctypes.c_uint32),
        ("peak_pv", ctypes.c_int32),
        ("overshoot", ctypes.c_int32),
        ("steady_error", ctypes.c_int32),
        ("pv_slope_per_s", ctypes.c_int32),
        ("elapsed_time_ms", ctypes.c_uint32),
        ("settling_time_ms", ctypes.c_uint32),
        ("cooldown_remaining_ms", ctypes.c_uint32),
        ("tune_count", ctypes.c_uint32),
        ("max_tunes_per_session", ctypes.c_uint32),
        ("last_tune_reason", ctypes.c_int32),
    ]


class ThermalPlant(ctypes.Structure):
    _fields_ = [
        ("temperature_c", ctypes.c_float),
        ("ambient_c", ctypes.c_float),
        ("sample_time_s", ctypes.c_float),
        ("tau_s", ctypes.c_float),
        ("mv_percent", ctypes.c_float),
        ("initialized", ctypes.c_bool),
    ]


@dataclass
class EpisodeResult:
    mode: str
    episode: int
    target_x10: int
    settled: bool
    settling_time_ms: int
    peak_pv_x10: int
    overshoot_x10: int
    steady_error_x10: int
    final_ki: int
    final_predictive_ms: int
    tune_count: int
    last_tune_reason: int
    response_qualified: bool


def enabled() -> bool:
    value = os.getenv("PC_SIMULATION")
    return value is not None and value.strip().lower() not in {"", "0", "false", "off", "no"}


def repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def build_dir() -> pathlib.Path:
    path = pathlib.Path(__file__).resolve().parent / "build"
    path.mkdir(parents=True, exist_ok=True)
    return path


def library_path() -> pathlib.Path:
    system = platform.system().lower()
    if system == "windows":
        return build_dir() / "myController_closed_loop.dll"
    if system == "darwin":
        return build_dir() / "libmyController_closed_loop.dylib"
    return build_dir() / "libmyController_closed_loop.so"


def build_library() -> pathlib.Path:
    root = repo_root()
    controller_dir = root / "PID_ControllerSrc"
    plant_dir = root / "ControllPlant"
    output = library_path()

    cc = os.getenv("CC", "gcc")
    if shutil.which(cc) is None:
        raise RuntimeError(f"C compiler not found: {cc}")

    controller_sources = sorted(str(p) for p in controller_dir.glob("*.c"))
    plant_sources = [str(plant_dir / "myPlant_1.c")]
    if not controller_sources:
        raise RuntimeError("No PID controller C sources found")

    cmd = [
        cc,
        "-std=c11",
        "-O2",
        "-DPC_SIMULATION",
        "-I", str(root),
        "-I", str(controller_dir),
        "-I", str(plant_dir),
    ]

    system = platform.system().lower()
    if system == "windows":
        cmd += ["-shared", "-DSSM_FB_BUILD_DLL"]
    elif system == "darwin":
        cmd += ["-dynamiclib", "-fPIC", "-DSSM_FB_BUILD_SHARED"]
    else:
        cmd += ["-shared", "-fPIC", "-DSSM_FB_BUILD_SHARED"]

    cmd += controller_sources + plant_sources
    if system != "windows":
        cmd += ["-lm"]
    cmd += ["-o", str(output)]

    print("Building closed-loop library:")
    print(" ".join(cmd))
    subprocess.run(cmd, check=True, cwd=root)
    return output


def load_api(path: pathlib.Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(path))

    lib.Heater_Control_InitTimed.argtypes = [ctypes.c_uint32]
    lib.Heater_Control_InitTimed.restype = ctypes.c_int
    lib.Heater_SetSelfTuningEnable.argtypes = [ctypes.c_int32]
    lib.Heater_SetSelfTuningEnable.restype = None
    lib.Heater_GetSelfTuningDiagnostics.argtypes = [ctypes.POINTER(Diagnostics)]
    lib.Heater_GetSelfTuningDiagnostics.restype = ctypes.c_int
    lib.Heater_myAdptiveControl.argtypes = [
        ctypes.c_int16,
        ctypes.c_int16,
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
    ]
    lib.Heater_myAdptiveControl.restype = None

    lib.ThermalPlant_Init.argtypes = [
        ctypes.POINTER(ThermalPlant), ctypes.c_float, ctypes.c_float, ctypes.c_float
    ]
    lib.ThermalPlant_Init.restype = None
    lib.ThermalPlant_Reset.argtypes = [ctypes.POINTER(ThermalPlant), ctypes.c_float]
    lib.ThermalPlant_Reset.restype = None
    lib.ThermalPlant_Step.argtypes = [ctypes.POINTER(ThermalPlant), ctypes.c_float]
    lib.ThermalPlant_Step.restype = ctypes.c_float
    lib.ThermalPlant_GetTemperature_x10.argtypes = [ctypes.POINTER(ThermalPlant)]
    lib.ThermalPlant_GetTemperature_x10.restype = ctypes.c_int16

    return lib


def controller_step(lib: ctypes.CDLL, pv_x10: int, sv_x10: int) -> tuple[int, int, int, int]:
    pid = ctypes.c_int32()
    ff = ctypes.c_int32()
    off = ctypes.c_int32()
    pwm = ctypes.c_int32()
    lib.Heater_myAdptiveControl(
        ctypes.c_int16(pv_x10),
        ctypes.c_int16(sv_x10),
        ctypes.byref(pid), ctypes.byref(ff), ctypes.byref(off), ctypes.byref(pwm),
    )
    return pid.value, ff.value, off.value, pwm.value


def get_diag(lib: ctypes.CDLL) -> Diagnostics:
    d = Diagnostics()
    status = lib.Heater_GetSelfTuningDiagnostics(ctypes.byref(d))
    if status != 0:
        raise RuntimeError(f"Heater_GetSelfTuningDiagnostics failed: {status}")
    return d


def prepare_episode(lib: ctypes.CDLL, plant: ThermalPlant) -> None:
    """Create a clean low-SV response without reinitializing tuner history."""
    lib.ThermalPlant_Reset(ctypes.byref(plant), ctypes.c_float(RESET_TEMP_C))
    steps = int(PREPARE_LOW_S * 1000 / SAMPLE_TIME_MS)
    for _ in range(steps):
        pv = int(lib.ThermalPlant_GetTemperature_x10(ctypes.byref(plant)))
        _, _, _, pwm = controller_step(lib, pv, LOW_SV_X10)
        lib.ThermalPlant_Step(ctypes.byref(plant), ctypes.c_float(pwm / 10.0))


def run_episode(
    lib: ctypes.CDLL,
    plant: ThermalPlant,
    mode: str,
    episode: int,
    target_x10: int,
    trace_writer: csv.writer,
) -> EpisodeResult:
    prepare_episode(lib, plant)
    max_steps = int(EPISODE_TIMEOUT_S * 1000 / SAMPLE_TIME_MS)
    settled_seen = False
    d = get_diag(lib)

    for i in range(max_steps):
        pv = int(lib.ThermalPlant_GetTemperature_x10(ctypes.byref(plant)))
        pid, ff, off, pwm = controller_step(lib, pv, target_x10)
        temp_c = float(lib.ThermalPlant_Step(ctypes.byref(plant), ctypes.c_float(pwm / 10.0)))
        d = get_diag(lib)

        if (i % 5) == 0 or d.settled:
            trace_writer.writerow([
                mode, episode, target_x10, i * SAMPLE_TIME_MS,
                pv, temp_c, pwm, pid, ff, off,
                d.current_ki, d.current_predictive_time_ms,
                d.response_active, d.settled, d.response_qualified,
                d.peak_pv, d.overshoot, d.steady_error,
                d.pv_slope_per_s, d.cooldown_remaining_ms,
                d.tune_count, d.last_tune_reason,
            ])

        if d.settled:
            # One extra controller cycle lets any same-cycle diagnostic/commit
            # state become externally visible before the episode is recorded.
            pv2 = int(lib.ThermalPlant_GetTemperature_x10(ctypes.byref(plant)))
            _, _, _, pwm2 = controller_step(lib, pv2, target_x10)
            lib.ThermalPlant_Step(ctypes.byref(plant), ctypes.c_float(pwm2 / 10.0))
            d = get_diag(lib)
            settled_seen = True
            break

    return EpisodeResult(
        mode=mode,
        episode=episode,
        target_x10=target_x10,
        settled=settled_seen,
        settling_time_ms=int(d.settling_time_ms),
        peak_pv_x10=int(d.peak_pv),
        overshoot_x10=int(d.overshoot),
        steady_error_x10=int(d.steady_error),
        final_ki=int(d.current_ki),
        final_predictive_ms=int(d.current_predictive_time_ms),
        tune_count=int(d.tune_count),
        last_tune_reason=int(d.last_tune_reason),
        response_qualified=bool(d.response_qualified),
    )


def run_suite(lib: ctypes.CDLL) -> tuple[pathlib.Path, pathlib.Path, list[EpisodeResult]]:
    out_dir = pathlib.Path(__file__).resolve().parent
    trace_path = out_dir / "self_tuning_closed_loop_trace.csv"
    summary_path = out_dir / "self_tuning_closed_loop_summary.csv"
    results: list[EpisodeResult] = []

    plant = ThermalPlant()
    lib.ThermalPlant_Init(
        ctypes.byref(plant), ctypes.c_float(RESET_TEMP_C),
        ctypes.c_float(AMBIENT_C), ctypes.c_float(SAMPLE_TIME_S)
    )

    with trace_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "mode", "episode", "target_x10", "time_ms", "pv_x10", "plant_temp_c",
            "pwm_x10pct", "pid", "ff", "ff_offset", "ki", "predictive_ms",
            "response_active", "settled", "qualified", "peak_pv", "overshoot",
            "steady_error", "pv_slope_per_s", "cooldown_ms", "tune_count",
            "last_tune_reason",
        ])

        # Baseline: diagnostics active but parameter adaptation remains OFF.
        status = lib.Heater_Control_InitTimed(SAMPLE_TIME_MS)
        if status != 0:
            raise RuntimeError(f"Heater_Control_InitTimed failed: {status}")
        lib.Heater_SetSelfTuningEnable(0)
        for idx, target in enumerate(TARGETS_X10, start=1):
            results.append(run_episode(lib, plant, "OFF", idx, target, w))

        # Learning session: one init, multiple responses so learned parameters persist.
        status = lib.Heater_Control_InitTimed(SAMPLE_TIME_MS)
        if status != 0:
            raise RuntimeError(f"Heater_Control_InitTimed failed: {status}")
        lib.Heater_SetSelfTuningEnable(1)
        episode = 0
        for _ in range(LEARNING_REPEATS):
            for target in TARGETS_X10:
                episode += 1
                results.append(run_episode(lib, plant, "ON", episode, target, w))

    with summary_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "mode", "episode", "target_x10", "settled", "settling_time_ms",
            "peak_pv_x10", "overshoot_x10", "steady_error_x10", "final_ki",
            "final_predictive_ms", "tune_count", "last_tune_reason",
            "response_qualified",
        ])
        for r in results:
            w.writerow([
                r.mode, r.episode, r.target_x10, int(r.settled), r.settling_time_ms,
                r.peak_pv_x10, r.overshoot_x10, r.steady_error_x10, r.final_ki,
                r.final_predictive_ms, r.tune_count, r.last_tune_reason,
                int(r.response_qualified),
            ])

    return trace_path, summary_path, results


def validate(results: list[EpisodeResult]) -> None:
    off = [r for r in results if r.mode == "OFF"]
    on = [r for r in results if r.mode == "ON"]

    if not off or not on:
        raise AssertionError("Missing baseline or learning episodes")
    if any(r.final_ki != 300 for r in off):
        raise AssertionError("Self-Tuning OFF changed Ki")
    if any(r.final_predictive_ms != 1000 for r in off):
        raise AssertionError("Self-Tuning OFF changed predictive time")
    if any(not r.response_qualified for r in on):
        raise AssertionError("Expected heating episodes to be qualified")
    if any(not (300 <= r.final_ki <= 1200) for r in on):
        raise AssertionError("Ki left configured safety range")
    if any(not (500 <= r.final_predictive_ms <= 2000) for r in on):
        raise AssertionError("Predictive time left configured safety range")
    if any(b.tune_count < a.tune_count for a, b in zip(on, on[1:])):
        raise AssertionError("Tune count decreased inside one learning session")

    settled_on = sum(1 for r in on if r.settled)
    if settled_on == 0:
        raise AssertionError("No learning episode reached settled state")

    # A zero tune count is reported, not hard-failed: a well-matched baseline can
    # legitimately require no adjustment. The CSV tells us whether the rule set
    # has useful excitation for this plant/controller combination.
    if on[-1].tune_count == 0:
        print("WARNING: all learning episodes settled without a parameter commit.")
        print("         Inspect summary CSV; excitation/tuning thresholds may be too conservative.")


def print_summary(results: list[EpisodeResult]) -> None:
    print("\nClosed-loop episode summary")
    print("mode ep target  settle(ms) over err  Ki  pred(ms) tunes reason")
    for r in results:
        settle = str(r.settling_time_ms) if r.settled else "TIMEOUT"
        print(
            f"{r.mode:>3} {r.episode:2d} {r.target_x10/10:6.1f}C "
            f"{settle:>10} {r.overshoot_x10/10:4.1f} {r.steady_error_x10/10:4.1f} "
            f"{r.final_ki:4d} {r.final_predictive_ms:8d} {r.tune_count:5d} "
            f"{r.last_tune_reason:6d}"
        )


def main() -> int:
    if not enabled():
        print("PC_SIMULATION is not defined; closed-loop self-tuning test is disabled.")
        return 0

    try:
        path = build_library()
        lib = load_api(path)
        trace, summary, results = run_suite(lib)
        validate(results)
        print_summary(results)
    except (OSError, RuntimeError, AssertionError, subprocess.CalledProcessError) as exc:
        print(f"Closed-loop self-tuning regression failed: {exc}", file=sys.stderr)
        return 1

    print("\nClosed-loop regression completed.")
    print(f"Library : {path}")
    print(f"Trace   : {trace}")
    print(f"Summary : {summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
