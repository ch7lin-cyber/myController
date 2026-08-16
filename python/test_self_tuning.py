#!/usr/bin/env python3
"""Branch-5 self-tuning regression tests.

Run with:
    PC_SIMULATION=1 python python/test_self_tuning.py

The suite builds all PID_ControllerSrc C files into a shared library and tests
only the public Heater_* API. A tiny local test helper is also compiled to
verify the PID Ki reconfigure keeps the integral contribution approximately
bumpless; the helper is generated into python/build and is not a product API.
"""

from __future__ import annotations

import ctypes
import os
import pathlib
import platform
import shutil
import subprocess
import sys

SAMPLE_TIME_MS = 20
SETTLE_SAMPLES = 3000 // SAMPLE_TIME_MS
COOLDOWN_SAMPLES = 30000 // SAMPLE_TIME_MS

REASON_NONE = 0
REASON_KI_INCREASE = 1
REASON_KI_DECREASE = 2
REASON_PREDICTIVE_TIME_INCREASE = 3
REASON_PREDICTIVE_TIME_DECREASE = 4
REASON_MULTIPLE = 5


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


def enabled() -> bool:
    value = os.getenv("PC_SIMULATION")
    return value is not None and value.strip().lower() not in {"", "0", "false", "off", "no"}


def root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def library_path(build_dir: pathlib.Path) -> pathlib.Path:
    system = platform.system().lower()
    if system == "windows":
        return build_dir / "myController_self_tuning.dll"
    if system == "darwin":
        return build_dir / "libmyController_self_tuning.dylib"
    return build_dir / "libmyController_self_tuning.so"


def write_pid_helper(build_dir: pathlib.Path) -> pathlib.Path:
    helper = build_dir / "self_tuning_pid_helper.c"
    helper.write_text(
        r'''
#include <stdint.h>
#include "FB_C_pid.h"

#if defined(_WIN32)
#define TEST_EXPORT __declspec(dllexport)
#else
#define TEST_EXPORT __attribute__((visibility("default")))
#endif

TEST_EXPORT int32_t Test_PID_Ki_Bumpless(void)
{
    APP_FB_PID_T pid;
    APP_FB_PID_PARAMETER_T p = {9000, 300, 5000, 32767, 900, APP_FB_PID_KAW_DEFAULT};
    int64_t old_contribution;
    int64_t new_contribution;

    if(app_fb_pid_init_timed(&pid, &p, 20U) != APP_FB_OK) return -1;
    pid.state.integral = 12000;
    old_contribution = (int64_t)pid.runtime_param.ki * (int64_t)pid.state.integral;

    p.ki = 325;
    if(app_fb_pid_reconfigure_timed(&pid, &p) != APP_FB_OK) return -2;
    new_contribution = (int64_t)pid.runtime_param.ki * (int64_t)pid.state.integral;

    if(old_contribution > new_contribution)
        return (int32_t)(old_contribution - new_contribution);
    return (int32_t)(new_contribution - old_contribution);
}
''',
        encoding="utf-8",
    )
    return helper


def build_library() -> pathlib.Path:
    repo = root()
    src = repo / "PID_ControllerSrc"
    build_dir = pathlib.Path(__file__).resolve().parent / "build"
    build_dir.mkdir(parents=True, exist_ok=True)
    output = library_path(build_dir)

    cc = os.getenv("CC", "gcc")
    if shutil.which(cc) is None:
        raise RuntimeError(f"C compiler not found: {cc}")

    sources = sorted(str(path) for path in src.glob("*.c"))
    sources.append(str(write_pid_helper(build_dir)))

    cmd = [cc, "-std=c11", "-O2", "-DPC_SIMULATION", "-I", str(repo), "-I", str(src)]
    system = platform.system().lower()
    if system == "windows":
        cmd += ["-shared", "-DSSM_FB_BUILD_DLL"]
    elif system == "darwin":
        cmd += ["-dynamiclib", "-fPIC", "-DSSM_FB_BUILD_SHARED"]
    else:
        cmd += ["-shared", "-fPIC", "-DSSM_FB_BUILD_SHARED"]
    cmd += sources + ["-o", str(output)]

    print("Building:")
    print(" ".join(cmd))
    subprocess.run(cmd, check=True, cwd=repo)
    return output


def load_api(path: pathlib.Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(path))
    lib.Heater_Control_InitTimed.argtypes = [ctypes.c_uint32]
    lib.Heater_Control_InitTimed.restype = ctypes.c_int
    lib.Heater_SetSelfTuningEnable.argtypes = [ctypes.c_int32]
    lib.Heater_SetSelfTuningEnable.restype = None
    lib.Heater_GetSelfTuningEnable.argtypes = []
    lib.Heater_GetSelfTuningEnable.restype = ctypes.c_int32
    lib.Heater_GetSelfTuningDiagnostics.argtypes = [ctypes.POINTER(Diagnostics)]
    lib.Heater_GetSelfTuningDiagnostics.restype = ctypes.c_int
    lib.Heater_myAdptiveControl.argtypes = [
        ctypes.c_int16, ctypes.c_int16,
        ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32),
    ]
    lib.Heater_myAdptiveControl.restype = None
    lib.Test_PID_Ki_Bumpless.argtypes = []
    lib.Test_PID_Ki_Bumpless.restype = ctypes.c_int32
    return lib


def init(lib: ctypes.CDLL) -> None:
    status = lib.Heater_Control_InitTimed(SAMPLE_TIME_MS)
    assert status == 0, f"init failed: {status}"


def cycle(lib: ctypes.CDLL, pv: int, sv: int) -> int:
    pid = ctypes.c_int32()
    ff = ctypes.c_int32()
    off = ctypes.c_int32()
    pwm = ctypes.c_int32()
    lib.Heater_myAdptiveControl(
        ctypes.c_int16(pv), ctypes.c_int16(sv),
        ctypes.byref(pid), ctypes.byref(ff), ctypes.byref(off), ctypes.byref(pwm),
    )
    return pwm.value


def diag(lib: ctypes.CDLL) -> Diagnostics:
    d = Diagnostics()
    status = lib.Heater_GetSelfTuningDiagnostics(ctypes.byref(d))
    assert status == 0, f"diagnostics failed: {status}"
    return d


def hold(lib: ctypes.CDLL, pv: int, sv: int, samples: int) -> None:
    for _ in range(samples):
        cycle(lib, pv, sv)


def test_default_off_diagnostics_on(lib: ctypes.CDLL) -> None:
    init(lib)
    assert lib.Heater_GetSelfTuningEnable() == 0
    cycle(lib, 1200, 1300)
    d = diag(lib)
    assert d.self_tuning_enabled == 0
    assert d.response_active == 1
    assert d.response_qualified == 1
    assert d.tune_count == 0
    hold(lib, 1298, 1300, SETTLE_SAMPLES)
    d = diag(lib)
    assert d.settled == 1
    assert d.current_ki == 300
    assert d.current_predictive_time_ms == 1000
    assert d.tune_count == 0


def test_small_step_rejected(lib: ctypes.CDLL) -> None:
    init(lib)
    lib.Heater_SetSelfTuningEnable(1)
    cycle(lib, 1300, 1330)  # +3.0C < +5.0C qualification threshold
    hold(lib, 1328, 1330, SETTLE_SAMPLES)
    d = diag(lib)
    assert d.response_qualified == 0
    assert d.settled == 1
    assert d.tune_count == 0
    assert d.current_ki == 300


def test_qualified_steady_error_commit(lib: ctypes.CDLL) -> None:
    init(lib)
    lib.Heater_SetSelfTuningEnable(1)
    cycle(lib, 1200, 1300)
    hold(lib, 1298, 1300, SETTLE_SAMPLES)
    d = diag(lib)
    assert d.response_qualified == 1
    assert d.tune_count == 1
    assert d.current_ki == 325
    assert d.current_predictive_time_ms == 900
    assert d.last_tune_reason == REASON_MULTIPLE
    assert d.cooldown_remaining_ms > 0


def test_overshoot_increases_prediction(lib: ctypes.CDLL) -> None:
    init(lib)
    lib.Heater_SetSelfTuningEnable(1)
    cycle(lib, 1200, 1300)
    cycle(lib, 1306, 1300)  # +0.6C peak overshoot
    hold(lib, 1300, 1300, SETTLE_SAMPLES)
    d = diag(lib)
    assert d.overshoot >= 6
    assert d.tune_count == 1
    assert d.current_predictive_time_ms == 1100
    assert d.last_tune_reason == REASON_PREDICTIVE_TIME_INCREASE


def test_cooling_step_diagnostics_only(lib: ctypes.CDLL) -> None:
    init(lib)
    lib.Heater_SetSelfTuningEnable(1)
    cycle(lib, 1500, 1300)
    hold(lib, 1300, 1300, SETTLE_SAMPLES)
    d = diag(lib)
    assert d.response_qualified == 0
    assert d.settled == 1
    assert d.tune_count == 0


def test_cooldown_blocks_second_commit(lib: ctypes.CDLL) -> None:
    init(lib)
    lib.Heater_SetSelfTuningEnable(1)
    cycle(lib, 1200, 1300)
    hold(lib, 1298, 1300, SETTLE_SAMPLES)
    first = diag(lib)
    assert first.tune_count == 1

    cycle(lib, 1300, 1400)
    hold(lib, 1398, 1400, SETTLE_SAMPLES)
    second = diag(lib)
    assert second.response_qualified == 1
    assert second.settled == 1
    assert second.tune_count == 1
    assert second.cooldown_remaining_ms > 0


def test_session_commit_limit(lib: ctypes.CDLL) -> None:
    init(lib)
    lib.Heater_SetSelfTuningEnable(1)

    sv = 1000
    for expected in range(1, 21):
        sv += 60
        cycle(lib, sv - 100, sv)
        hold(lib, sv - 2, sv, SETTLE_SAMPLES)
        d = diag(lib)
        assert d.tune_count == expected, (expected, d.tune_count)
        # Let the 30-second cooldown fully expire before the next response.
        hold(lib, sv - 2, sv, COOLDOWN_SAMPLES)

    d = diag(lib)
    assert d.max_tunes_per_session == 20
    assert d.tune_count == 20

    sv += 60
    cycle(lib, sv - 100, sv)
    hold(lib, sv - 2, sv, SETTLE_SAMPLES)
    d = diag(lib)
    assert d.response_qualified == 1
    assert d.settled == 1
    assert d.tune_count == 20


def test_pid_ki_reconfigure_bumpless(lib: ctypes.CDLL) -> None:
    # Difference is in raw Ki*integral units. Integer rounding should keep it tiny
    # relative to the original 300*12000 = 3,600,000 contribution.
    error = lib.Test_PID_Ki_Bumpless()
    assert error >= 0
    assert error <= 325, f"Ki reconfigure discontinuity too large: {error}"


def main() -> int:
    if not enabled():
        print("PC_SIMULATION is not defined; self-tuning regression is disabled.")
        return 0

    tests = [
        test_default_off_diagnostics_on,
        test_small_step_rejected,
        test_qualified_steady_error_commit,
        test_overshoot_increases_prediction,
        test_cooling_step_diagnostics_only,
        test_cooldown_blocks_second_commit,
        test_session_commit_limit,
        test_pid_ki_reconfigure_bumpless,
    ]

    try:
        path = build_library()
        lib = load_api(path)
        for test in tests:
            test(lib)
            print(f"PASS: {test.__name__}")
    except (AssertionError, OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    print(f"All {len(tests)} self-tuning regression tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
