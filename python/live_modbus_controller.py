#!/usr/bin/env python3
"""Live Modbus TCP closed-loop runner for the adaptive heater controller.

Control/Modbus loop: 20 ms by default (50 Hz).
CSV logging:       100 ms by default (10 Hz), decimated from the control loop.

Safety:
- Starts in STOP.
- STOP / quit / Ctrl+C / communication fault attempts MV=0.
- RUN re-initializes the controller.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import datetime as dt
import math
import os
import pathlib
import platform
import queue
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

try:
    from pymodbus.client import ModbusTcpClient
except ImportError as exc:
    raise SystemExit(
        "pymodbus is required. Install with: python -m pip install pymodbus"
    ) from exc

DEFAULT_HOST = "192.168.1.10"
DEFAULT_PORT = 2502
DEFAULT_PV_REG = 92
DEFAULT_MV_REG = 108
DEFAULT_UNIT_ID = 1
DEFAULT_SAMPLE_MS = 20
DEFAULT_LOG_MS = 100
DEFAULT_SV = 1300

PWM_MIN = 0
PWM_MAX = 1000
TEMP_MIN = -2000
TEMP_MAX = 18000

# Mirror Branch-4 Fast Heating Boost V3 for host-side diagnostics only.
FAST_HEAT_ENTER_ERROR = 100
FAST_HEAT_EXIT_ERROR = 50
PREDICT_TIME_MS = 2000
PREDICT_BRAKE_ENTER_ERROR = 100
D_FILTER_TAU_MS = 140


@dataclass
class RuntimeState:
    running: bool = False
    quit_requested: bool = False
    sv: int = DEFAULT_SV
    last_pv: int | None = None
    last_mv: int = 0
    last_pid: int = 0
    last_ff: int = 0
    last_ff_offset: int = 0
    last_raw_sum: int = 0
    last_controller_pwm: int = 0
    host_dpv_filtered: float = 0.0
    host_fast_heat_active: bool = False
    comm_errors: int = 0
    missed_deadlines: int = 0


@dataclass
class LogWindow:
    cycles: int = 0
    cycle_ms_max: float = 0.0
    jitter_ms_abs_max: float = 0.0
    read_ms_max: float = 0.0
    write_ms_max: float = 0.0
    control_us_max: float = 0.0
    deadline_miss_count: int = 0
    comm_error_count: int = 0

    def update(
        self,
        cycle_ms: float,
        jitter_ms: float,
        read_ms: float,
        write_ms: float,
        control_us: float,
        deadline_miss: bool,
        comm_ok: bool,
    ) -> None:
        self.cycles += 1
        self.cycle_ms_max = max(self.cycle_ms_max, cycle_ms)
        self.jitter_ms_abs_max = max(self.jitter_ms_abs_max, abs(jitter_ms))
        self.read_ms_max = max(self.read_ms_max, read_ms)
        self.write_ms_max = max(self.write_ms_max, write_ms)
        self.control_us_max = max(self.control_us_max, control_us)
        self.deadline_miss_count += int(deadline_miss)
        self.comm_error_count += int(not comm_ok)

    def reset(self) -> None:
        self.cycles = 0
        self.cycle_ms_max = 0.0
        self.jitter_ms_abs_max = 0.0
        self.read_ms_max = 0.0
        self.write_ms_max = 0.0
        self.control_us_max = 0.0
        self.deadline_miss_count = 0
        self.comm_error_count = 0


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
    requested = os.getenv("CC")
    if requested:
        path = shutil.which(requested)
        if path is None and pathlib.Path(requested).exists():
            path = str(pathlib.Path(requested).resolve())
        if path is None:
            raise RuntimeError(f"CC compiler not found: {requested}")
        name = pathlib.Path(path).name.lower()
        return path, "msvc" if name in {"cl", "cl.exe"} else "gnu"

    for candidate in ("gcc", "clang", "cl"):
        path = shutil.which(candidate)
        if path:
            return path, "msvc" if candidate == "cl" else "gnu"
    raise RuntimeError("No C compiler found. Set CC or install gcc/clang/cl.")


def _build_shared_library() -> pathlib.Path:
    root = _repo_root()
    src_dir = root / "PID_ControllerSrc"
    build_dir = pathlib.Path(__file__).resolve().parent / "build"
    build_dir.mkdir(parents=True, exist_ok=True)
    output = _shared_library_path(build_dir)

    cc, kind = _find_compiler()
    sources = sorted(str(p.resolve()) for p in src_dir.glob("*.c"))
    if not sources:
        raise RuntimeError(f"No C sources found in {src_dir}")

    system = platform.system().lower()
    if kind == "msvc":
        cmd = [
            cc, "/nologo", "/LD", "/O2", "/DPC_SIMULATION", "/DSSM_FB_BUILD_DLL",
            f"/I{root}", f"/I{src_dir}",
        ] + sources + [f"/Fe:{output}"]
        cwd = build_dir
    else:
        cmd = [
            cc, "-std=c11", "-O2", "-DPC_SIMULATION",
            "-I", str(root), "-I", str(src_dir),
        ]
        if system == "windows":
            cmd += ["-shared", "-DSSM_FB_BUILD_DLL"]
        elif system == "darwin":
            cmd += ["-dynamiclib", "-fPIC", "-DSSM_FB_BUILD_SHARED"]
        else:
            cmd += ["-shared", "-fPIC", "-DSSM_FB_BUILD_SHARED"]
        cmd += sources + ["-o", str(output)]
        cwd = root

    print(f"Compiler: {cc} ({kind})")
    subprocess.run(cmd, check=True, cwd=cwd)
    if not output.exists():
        raise RuntimeError(f"Shared library was not created: {output}")
    return output


def _load_controller(path: pathlib.Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(path))
    lib.Heater_Control_InitTimed.argtypes = [ctypes.c_uint32]
    lib.Heater_Control_InitTimed.restype = ctypes.c_int
    lib.Heater_myAdptiveControl.argtypes = [
        ctypes.c_int16, ctypes.c_int16,
        ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32),
    ]
    lib.Heater_myAdptiveControl.restype = None
    return lib


def _int16_from_register(value: int) -> int:
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def _modbus_read_holding(client, address: int, unit_id: int):
    try:
        return client.read_holding_registers(address=address, count=1, device_id=unit_id)
    except TypeError:
        return client.read_holding_registers(address=address, count=1, slave=unit_id)


def _modbus_write_register(client, address: int, value: int, unit_id: int):
    try:
        return client.write_register(address=address, value=value, device_id=unit_id)
    except TypeError:
        return client.write_register(address=address, value=value, slave=unit_id)


def _read_pv(client, address: int, unit_id: int) -> int:
    result = _modbus_read_holding(client, address, unit_id)
    if result is None or result.isError() or not getattr(result, "registers", None):
        raise RuntimeError(f"PV read failed: {result}")
    return _int16_from_register(result.registers[0])


def _write_mv(client, address: int, value: int, unit_id: int) -> None:
    value = max(PWM_MIN, min(PWM_MAX, int(value)))
    result = _modbus_write_register(client, address, value, unit_id)
    if result is None or result.isError():
        raise RuntimeError(f"MV write failed: {result}")


def _safe_write_zero(client, address: int, unit_id: int) -> None:
    try:
        _write_mv(client, address, 0, unit_id)
    except Exception as exc:
        print(f"WARNING: unable to write MV=0: {exc}", file=sys.stderr)


def _parse_sv(text: str) -> int:
    text = text.strip().lower()
    if text.endswith("c"):
        result = int(round(float(text[:-1]) * 10.0))
    elif "." in text:
        result = int(round(float(text) * 10.0))
    else:
        result = int(text, 10)
    if result < TEMP_MIN or result > TEMP_MAX:
        raise ValueError(f"SV outside {TEMP_MIN}..{TEMP_MAX} (0.1C)")
    return result


def _command_reader(command_queue: queue.Queue[str]) -> None:
    while True:
        try:
            line = input().strip()
        except EOFError:
            command_queue.put("quit")
            return
        if line:
            command_queue.put(line)
            if line.lower() in {"quit", "q", "exit"}:
                return


def _print_help() -> None:
    print(
        "Commands:\n"
        "  run           start closed-loop control\n"
        "  stop          stop control and write MV=0\n"
        "  sv 1300       set SV=130.0C in native 0.1C units\n"
        "  sv 130.0c     set SV using degrees C\n"
        "  status        print current state\n"
        "  help          show commands\n"
        "  quit          write MV=0 and exit"
    )


def _host_diagnostics(
    state: RuntimeState, pv: int, sv: int, sample_ms: int
) -> tuple[float, float, float, bool, bool]:
    if state.last_pv is None:
        state.host_dpv_filtered = 0.0
    else:
        delta = float(pv - state.last_pv)
        alpha = D_FILTER_TAU_MS / (D_FILTER_TAU_MS + float(sample_ms))
        state.host_dpv_filtered = (
            alpha * state.host_dpv_filtered + (1.0 - alpha) * delta
        )

    rate_per_sec = state.host_dpv_filtered * 1000.0 / float(sample_ms)
    predicted_pv = pv + state.host_dpv_filtered * (PREDICT_TIME_MS / float(sample_ms))
    predicted_error = sv - predicted_pv
    error = sv - pv

    if not state.host_fast_heat_active:
        if error >= FAST_HEAT_ENTER_ERROR:
            state.host_fast_heat_active = True
    elif error <= FAST_HEAT_EXIT_ERROR:
        state.host_fast_heat_active = False

    brake = (
        state.host_fast_heat_active
        and state.host_dpv_filtered > 0.0
        and predicted_error <= PREDICT_BRAKE_ENTER_ERROR
    )
    return rate_per_sec, predicted_pv, predicted_error, state.host_fast_heat_active, brake


def _process_command(line, state, lib, sample_ms, client, mv_reg, unit_id) -> str:
    parts = line.strip().split()
    if not parts:
        return ""
    cmd = parts[0].lower()

    if cmd == "run":
        status = lib.Heater_Control_InitTimed(sample_ms)
        if status != 0:
            return f"Controller init failed: {status}"
        state.running = True
        state.host_fast_heat_active = False
        state.host_dpv_filtered = 0.0
        state.last_pv = None
        return f"RUN: SV={state.sv} ({state.sv / 10.0:.1f}C), Ts={sample_ms}ms"

    if cmd == "stop":
        state.running = False
        state.last_mv = 0
        _safe_write_zero(client, mv_reg, unit_id)
        return "STOP: MV=0"

    if cmd == "sv":
        if len(parts) != 2:
            return "Usage: sv 1300   or   sv 130.0c"
        try:
            state.sv = _parse_sv(parts[1])
        except ValueError as exc:
            return f"Invalid SV: {exc}"
        return f"SV={state.sv} ({state.sv / 10.0:.1f}C)"

    if cmd == "status":
        pv_c = state.last_pv / 10.0 if state.last_pv is not None else math.nan
        return (
            f"{'RUN' if state.running else 'STOP'} "
            f"SV={state.sv/10.0:.1f}C PV={pv_c:.1f}C "
            f"PID={state.last_pid} FF={state.last_ff} OFF={state.last_ff_offset} "
            f"PWM={state.last_controller_pwm} COMM_ERR={state.comm_errors} "
            f"MISS={state.missed_deadlines}"
        )

    if cmd in {"help", "?"}:
        _print_help()
        return ""

    if cmd in {"quit", "q", "exit"}:
        state.running = False
        state.quit_requested = True
        _safe_write_zero(client, mv_reg, unit_id)
        return "QUIT: MV=0"

    return f"Unknown command: {cmd}. Type 'help'."


def _csv_path(csv_dir: pathlib.Path) -> pathlib.Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return csv_dir / f"live_control_{stamp}.csv"


def run(args: argparse.Namespace) -> int:
    if not (1 <= args.sample_ms <= 6000):
        raise RuntimeError("sample-ms must be 1..6000")
    if args.log_ms < args.sample_ms:
        raise RuntimeError("log-ms must be >= sample-ms")

    log_every_cycles = max(1, int(round(args.log_ms / args.sample_ms)))
    effective_log_ms = log_every_cycles * args.sample_ms

    lib = _load_controller(_build_shared_library())

    client = ModbusTcpClient(host=args.host, port=args.port, timeout=args.timeout)
    if not client.connect():
        raise RuntimeError(f"Cannot connect to Modbus TCP {args.host}:{args.port}")

    state = RuntimeState(sv=args.sv)
    command_queue: queue.Queue[str] = queue.Queue()
    threading.Thread(target=_command_reader, args=(command_queue,), daemon=True).start()

    csv_dir = pathlib.Path(args.csv_dir).resolve()
    csv_dir.mkdir(parents=True, exist_ok=True)
    output_path = _csv_path(csv_dir)

    fieldnames = [
        "wall_time", "elapsed_s", "sample_index", "state",
        "sv", "pv", "error",
        "pid_out", "ff_pwm", "ff_offset", "raw_sum",
        "controller_pwm", "mv_written",
        "host_dpv_filtered_per_sample", "host_pv_rate_0p1c_per_s",
        "host_predicted_pv", "host_predicted_error",
        "host_fast_heat_active", "host_predictive_brake_active",
        "read_ms", "control_us", "write_ms", "cycle_ms", "jitter_ms",
        "deadline_miss", "comm_ok", "comm_errors",
        "window_cycles", "window_cycle_ms_max", "window_jitter_ms_abs_max",
        "window_read_ms_max", "window_write_ms_max", "window_control_us_max",
        "window_deadline_miss_count", "window_comm_error_count",
        "note",
    ]

    print(f"Connected: {args.host}:{args.port}, unit={args.unit_id}")
    print(
        f"PV={args.pv_reg}, MV={args.mv_reg}, "
        f"control={args.sample_ms}ms ({1000/args.sample_ms:.1f}Hz), "
        f"CSV={effective_log_ms}ms ({1000/effective_log_ms:.1f}Hz)"
    )
    print(f"CSV: {output_path}")
    print("Starts in STOP. Type 'sv 1300', then 'run'.")

    _safe_write_zero(client, args.mv_reg, args.unit_id)

    start = time.perf_counter()
    next_deadline = start
    sample_index = 0
    last_console = start
    period_s = args.sample_ms / 1000.0
    window = LogWindow()
    pending_notes: list[str] = []

    try:
        with output_path.open("w", newline="", encoding="utf-8", buffering=1) as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()

            while not state.quit_requested:
                loop_start = time.perf_counter()

                while True:
                    try:
                        command = command_queue.get_nowait()
                    except queue.Empty:
                        break
                    message = _process_command(
                        command, state, lib, args.sample_ms,
                        client, args.mv_reg, args.unit_id
                    )
                    if message:
                        print(message)
                        pending_notes.append(message)

                if state.quit_requested:
                    break

                comm_ok = True
                pv: int | None = None
                read_ms = control_us = write_ms = 0.0
                mv_written = 0

                try:
                    t0 = time.perf_counter()
                    pv = _read_pv(client, args.pv_reg, args.unit_id)
                    read_ms = (time.perf_counter() - t0) * 1000.0

                    if pv < TEMP_MIN or pv > TEMP_MAX:
                        raise RuntimeError(f"PV out of safety range: {pv}")

                    rate, predicted_pv, predicted_error, host_boost, host_brake = (
                        _host_diagnostics(state, pv, state.sv, args.sample_ms)
                    )

                    if state.running:
                        pid_out = ctypes.c_int32()
                        ff_pwm = ctypes.c_int32()
                        ff_offset = ctypes.c_int32()
                        controller_pwm = ctypes.c_int32()

                        t0 = time.perf_counter()
                        lib.Heater_myAdptiveControl(
                            ctypes.c_int16(pv), ctypes.c_int16(state.sv),
                            ctypes.byref(pid_out), ctypes.byref(ff_pwm),
                            ctypes.byref(ff_offset), ctypes.byref(controller_pwm),
                        )
                        control_us = (time.perf_counter() - t0) * 1_000_000.0

                        state.last_pid = pid_out.value
                        state.last_ff = ff_pwm.value
                        state.last_ff_offset = ff_offset.value
                        state.last_raw_sum = pid_out.value + ff_pwm.value + ff_offset.value
                        state.last_controller_pwm = max(
                            PWM_MIN, min(PWM_MAX, controller_pwm.value)
                        )

                        t0 = time.perf_counter()
                        _write_mv(client, args.mv_reg, state.last_controller_pwm, args.unit_id)
                        write_ms = (time.perf_counter() - t0) * 1000.0
                        mv_written = state.last_controller_pwm
                        state.last_mv = mv_written
                    else:
                        state.last_pid = 0
                        state.last_ff = 0
                        state.last_ff_offset = 0
                        state.last_raw_sum = 0
                        state.last_controller_pwm = 0

                except Exception as exc:
                    comm_ok = False
                    state.comm_errors += 1
                    state.running = False
                    state.last_mv = 0
                    _safe_write_zero(client, args.mv_reg, args.unit_id)
                    pending_notes.append(f"FAULT:{exc}")
                    print(f"FAULT -> STOP/MV=0: {exc}", file=sys.stderr)

                    rate = 0.0
                    predicted_pv = float(pv) if pv is not None else math.nan
                    predicted_error = state.sv - pv if pv is not None else math.nan
                    host_boost = host_brake = False

                now = time.perf_counter()
                cycle_ms = (now - loop_start) * 1000.0
                next_deadline += period_s
                jitter_ms = (now - next_deadline) * 1000.0
                deadline_miss = now > next_deadline
                if deadline_miss:
                    state.missed_deadlines += 1

                window.update(
                    cycle_ms, jitter_ms, read_ms, write_ms, control_us,
                    deadline_miss, comm_ok
                )

                # Only CSV logging is decimated. Control and Modbus stay at 50 Hz.
                log_now = ((sample_index + 1) % log_every_cycles) == 0
                if log_now:
                    writer.writerow({
                        "wall_time": dt.datetime.now().isoformat(timespec="milliseconds"),
                        "elapsed_s": f"{now-start:.6f}",
                        "sample_index": sample_index,
                        "state": "RUN" if state.running else "STOP",
                        "sv": state.sv,
                        "pv": "" if pv is None else pv,
                        "error": "" if pv is None else state.sv - pv,
                        "pid_out": state.last_pid,
                        "ff_pwm": state.last_ff,
                        "ff_offset": state.last_ff_offset,
                        "raw_sum": state.last_raw_sum,
                        "controller_pwm": state.last_controller_pwm,
                        "mv_written": mv_written,
                        "host_dpv_filtered_per_sample": f"{state.host_dpv_filtered:.6f}",
                        "host_pv_rate_0p1c_per_s": f"{rate:.6f}",
                        "host_predicted_pv": f"{predicted_pv:.3f}",
                        "host_predicted_error": f"{predicted_error:.3f}",
                        "host_fast_heat_active": int(bool(host_boost)),
                        "host_predictive_brake_active": int(bool(host_brake)),
                        "read_ms": f"{read_ms:.3f}",
                        "control_us": f"{control_us:.3f}",
                        "write_ms": f"{write_ms:.3f}",
                        "cycle_ms": f"{cycle_ms:.3f}",
                        "jitter_ms": f"{jitter_ms:.3f}",
                        "deadline_miss": int(deadline_miss),
                        "comm_ok": int(comm_ok),
                        "comm_errors": state.comm_errors,
                        "window_cycles": window.cycles,
                        "window_cycle_ms_max": f"{window.cycle_ms_max:.3f}",
                        "window_jitter_ms_abs_max": f"{window.jitter_ms_abs_max:.3f}",
                        "window_read_ms_max": f"{window.read_ms_max:.3f}",
                        "window_write_ms_max": f"{window.write_ms_max:.3f}",
                        "window_control_us_max": f"{window.control_us_max:.3f}",
                        "window_deadline_miss_count": window.deadline_miss_count,
                        "window_comm_error_count": window.comm_error_count,
                        "note": " | ".join(pending_notes),
                    })
                    window.reset()
                    pending_notes.clear()

                if pv is not None:
                    state.last_pv = pv

                if now - last_console >= 1.0:
                    pv_text = "----" if pv is None else f"{pv/10.0:7.1f}C"
                    pred_text = "----" if pv is None else f"{predicted_pv/10.0:7.1f}C"
                    print(
                        f"{'RUN ' if state.running else 'STOP'} "
                        f"PV={pv_text} SV={state.sv/10.0:6.1f}C "
                        f"PID={state.last_pid:5d} FF={state.last_ff:4d} "
                        f"PWM={state.last_controller_pwm:4d} "
                        f"PredPV={pred_text} "
                        f"Loop={cycle_ms:5.1f}ms miss={state.missed_deadlines}"
                    )
                    last_console = now

                sample_index += 1
                sleep_s = next_deadline - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_deadline = time.perf_counter()

    except KeyboardInterrupt:
        print("Ctrl+C -> STOP")
    finally:
        state.running = False
        _safe_write_zero(client, args.mv_reg, args.unit_id)
        client.close()
        print(f"MV forced to 0. CSV saved: {output_path}")

    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live Modbus TCP adaptive heater controller")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--pv-reg", type=int, default=DEFAULT_PV_REG)
    parser.add_argument("--mv-reg", type=int, default=DEFAULT_MV_REG)
    parser.add_argument("--unit-id", type=int, default=DEFAULT_UNIT_ID)
    parser.add_argument("--sample-ms", type=int, default=DEFAULT_SAMPLE_MS)
    parser.add_argument(
        "--log-ms", type=int, default=DEFAULT_LOG_MS,
        help="CSV logging interval in ms; control period is unchanged"
    )
    parser.add_argument("--sv", type=_parse_sv, default=DEFAULT_SV)
    parser.add_argument(
        "--timeout", type=float, default=0.05,
        help="Modbus socket timeout in seconds"
    )
    parser.add_argument(
        "--csv-dir",
        default=str(pathlib.Path(__file__).resolve().parent / "logs"),
        help="Directory for CSV logs",
    )
    return parser.parse_args()


def main() -> int:
    try:
        return run(_parse_args())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"live controller failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
