#!/usr/bin/env python3
"""Attach without halt/reset; test A/JOINT1 via SDK 0x16; save raw evidence.

No flashing, initialization, calibration/zeroing or gain changes are performed.
See HANDOFF_JOINT_DIAGNOSTICS.md for the build and local hardware procedure.
"""
import argparse
import csv
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time

from joint_diag_data import (Benchmark, Clock, Info, Sample, analyze, as_dict,
                             choose_amplitude, clock_summary, flatten_sample,
                             preflight, trajectory)

ROOT = Path(__file__).resolve().parents[2]
BENCH_BUDGET_S = 8.0
MOTION_BUDGET_S = 22.0
DIAG_COMMAND_TID = 0x70
DIAG_OP_CAPTURE = 1
DIAG_OP_GUARD = 2
DIAG_OP_CLOCK_HZ = 3


class Budget:
    def __init__(self, seconds):
        self.start = time.monotonic()
        self.end = self.start + seconds

    def check(self):
        if time.monotonic() >= self.end:
            raise TimeoutError("measurement budget exhausted; test incomplete")

    def remaining(self):
        self.check()
        return self.end-time.monotonic()


def save_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+"\n")


class Probe:
    def __init__(self, elf_path, serial=None, device="STM32H723ZG"):
        import pylink
        from elftools.elf.elffile import ELFFile
        self.elf_path = Path(elf_path)
        self.elf_sha256 = hashlib.sha256(self.elf_path.read_bytes()).hexdigest()
        with self.elf_path.open("rb") as stream:
            elf = ELFFile(stream)
            table = elf.get_section_by_name(".symtab")
            if not table:
                raise ValueError("ELF needs a symbol table")
            self.symbols = {s.name: (int(s["st_value"]), int(s["st_size"])) for s in table.iter_symbols()}
            self.flash = [(int(s["p_paddr"]), s.data()) for s in elf.iter_segments()
                          if s["p_type"] == "PT_LOAD" and s["p_filesz"]
                          and 0x08000000 <= s["p_paddr"] < 0x08100000]
        for name, cls in (("h7_diag", Info), ("h7_diag_clock", Clock), ("h7_diag_bench", Benchmark)):
            if self.symbols.get(name, (0, 0))[1] != C.sizeof(cls):
                raise ValueError(f"wrong/missing diagnostic ABI: {name}; rebuild this branch")
        self.jlink = pylink.JLink()
        try:
            self.jlink.open(serial_no=serial)
            self.jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
            self.jlink.connect(device, speed=4000)
            if self.jlink.halted():
                raise RuntimeError("target is halted; this script never resumes/resets a target")
            ready_by = time.monotonic()+5
            while True:
                info = self.info()
                if (info["magic"] == 0x4a314447 and info["abi"] == 1 and info["record_bytes"] == C.sizeof(Sample)
                        and (info["state"] or info["error"])):
                    break
                if time.monotonic() >= ready_by:
                    raise RuntimeError("diagnostic thread not ready or wrong firmware")
                time.sleep(.05)
            if info["error"] in (1, 5) or info["state"] == 0:
                raise RuntimeError(f"diagnostic startup error/state: {info['error']}/{info['state']}")
            # Flash comparison proves that RAM addresses/configuration are from
            # the actual ELF, not a stale Ozone project. Read-only, no halt.
            for address, data in self.flash:
                for offset in range(0, len(data), 4096):
                    expected = data[offset:offset+4096]
                    if self.read(address+offset, len(expected)) != expected:
                        raise RuntimeError(f"ELF/flash mismatch at 0x{address+offset:08x}")
        except BaseException:
            self.jlink.close()
            raise

    def close(self):
        self.jlink.close()

    def read(self, address, size):
        # Aligned words permit efficient background AHB reads. Trim final word.
        words = self.jlink.memory_read32(address, (size+3)//4)
        return struct.pack("<"+"I"*len(words), *words)[:size]

    def object(self, symbol, cls):
        return cls.from_buffer_copy(self.read(self.symbols[symbol][0], C.sizeof(cls)))

    def info(self):
        return as_dict(self.object("h7_diag", Info))

    def benchmark_request(self, request):
        if request not in (2, 3):
            raise ValueError("invalid benchmark request")
        self.jlink.memory_write32(self.symbols["h7_diag"][0]+Info.request.offset, [request])

    def clock(self):
        addr = self.symbols["h7_diag_clock"][0]
        for _ in range(30):
            before = time.perf_counter()
            row = as_dict(self.object("h7_diag_clock", Clock))
            last = self.jlink.memory_read32(addr, 1)[0]
            after = time.perf_counter()
            if row["lock"] == last and not (last & 1):
                return dict(row, before=before, after=after)
            time.sleep(.001)
        raise RuntimeError("cannot obtain consistent clock snapshot; reduce SWD latency")

    def capture(self, directory, service=lambda: None):
        info = self.info()
        if info["state"] != 3 or not 0 < info["count"] <= info["capacity"]:
            raise RuntimeError("capture not frozen")
        address, capacity_bytes = self.symbols["h7_diag_samples"]
        size = info["count"]*info["record_bytes"]
        if size > capacity_bytes:
            raise RuntimeError("capture ABI size mismatch")
        raw = bytearray()
        for offset in range(0, size, 4096):
            raw.extend(self.read(address+offset, min(4096, size-offset)))
            service()
        if self.info()["generation"] != info["generation"] or self.info()["state"] != 3:
            raise RuntimeError("capture changed while reading")
        directory.mkdir(parents=True, exist_ok=True)
        (directory/"samples.bin").write_bytes(raw)
        save_json(directory/"metadata.json", dict(info, elf_sha256=self.elf_sha256))
        rows = [flatten_sample(Sample.from_buffer_copy(raw, i*info["record_bytes"])) for i in range(info["count"])]
        with (directory/"samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        return rows, info


def calibrate(probe, directory, service=lambda: None, seconds=5.3, budget=None):
    rows, start = [], time.perf_counter()
    while time.perf_counter()-start < seconds:
        if budget:
            budget.check()
        service()
        rows.append(probe.clock())
        time.sleep(.1)
    summary = clock_summary(rows)
    save_json(directory/"clock_raw.json", rows)
    save_json(directory/"clock_summary.json", summary)
    return summary


def wait_until(predicate, service=lambda: None, timeout=10):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        service()
        if predicate():
            return
        time.sleep(.02)
    raise TimeoutError("timed out waiting for hardware state")


def check_snapshot(arm, low, high, running=True):
    s = arm.snapshot()
    if s.age is None or s.age > .05 or s.motor_lost_mask or s.motor_inited_mask != 0x7f:
        raise RuntimeError("feedback stale, motor offline or not initialized")
    if s.robot_state != arm.board_defs.STATE_NORMAL or (running and s.mode != arm.board_defs.MODE_RUN):
        raise RuntimeError(f"unexpected state/mode: {s.robot_state}/{s.mode}")
    for i, q in enumerate(s.joint_angles_deg):
        if not math.isfinite(q) or q < low[i]-.2 or q > high[i]+.2:
            raise RuntimeError(f"joint {i+1} feedback outside soft limits: {q}")
    if any(not math.isfinite(v) for v in s.joint_velocities_deg_s):
        raise RuntimeError("nonfinite velocity")
    return s


def run_motion(probe, args, directory, budget):
    __import__("numpy")  # Warm numerical imports before commanding motion.
    sys.path.insert(0, str(ROOT))
    import sdk
    if "H7_Diag_Link_Command" not in probe.symbols:
        raise RuntimeError("firmware lacks SDK diagnostic commands; load a matching diagnostic ELF")
    info = probe.info()
    if info["profile"] != "ARX-X5" or info["rc_online"]:
        raise RuntimeError("A must be ARX-X5 with the RC offline")
    # Configuration snapshot will be refreshed by a capture before motion.
    if info["state"] in (2, 4):
        raise RuntimeError("another test is active")
    link = sdk.Link(args.port, robot_type="arx_x5", auto_reconnect=False)
    arm = sdk.Arm(0, link)
    commands = []
    plots = []
    began = False
    measurements_complete = False
    damping_confirmed = False

    def command(op, value):
        link.send(DIAG_COMMAND_TID, struct.pack("<BI", op, value), arm_id=0)

    try:
        arm.connect()
        budget.check()
        # Do not silently invoke reset, homing or force=True. The local agent
        # prepares STATE_NORMAL using the existing initialization procedure.
        snapshot = check_snapshot(arm, info["minimum_deg"], info["maximum_deg"], running=False)
        if snapshot.mode != arm.board_defs.MODE_DAMPING:
            raise RuntimeError("A arm must be in damping before arming the diagnostic guard")
        if max(abs(v) for v in snapshot.joint_velocities_deg_s) > 2:
            raise RuntimeError("arm must be stationary before test")
        base = list(snapshot.joint_angles_deg)
        grip = snapshot.gripper_raw_deg
        # A read-only capture refreshes runtime offsets/limits/RC status.
        command(DIAG_OP_CAPTURE, 1)
        wait_until(lambda: probe.info()["state"] == 2, arm.keepalive, timeout=min(1, budget.remaining()))
        command(DIAG_OP_CAPTURE, 2)
        wait_until(lambda: probe.info()["state"] == 3, arm.keepalive, timeout=min(1, budget.remaining()))
        info = probe.info()
        if info["rc_online"]:
            raise RuntimeError("RC is online; position commands would be ignored")
        low, high = info["minimum_deg"], info["maximum_deg"]
        snapshot = check_snapshot(arm, low, high, running=False)
        # Feedback can quantize a few hundredths of a degree past a limit.
        # Hold the nearest legal target, as the firmware normally does.
        base = [min(high[i], max(low[i], q)) for i, q in enumerate(snapshot.joint_angles_deg)]
        grip = snapshot.gripper_raw_deg
        amplitude = choose_amplitude(base[0], args.amplitude, low[0], high[0])
        save_json(directory/"preflight.json", dict(center_deg=base[0], amplitude_deg=amplitude,
            held_joint_targets_deg=base, gripper_target_deg=grip,
            ruckig_library_sha256=hashlib.sha256(Path(args.ruckig_lib).read_bytes()).hexdigest(),
            trajectories=preflight(Path(args.ruckig_lib).resolve(), info, base[0], amplitude)))
        arm.keepalive()
        fresh = check_snapshot(arm, low, high, running=False)
        if max(abs(v) for v in fresh.joint_velocities_deg_s) > 2 or any(
                abs(q-base[i]) > .5 for i, q in enumerate(fresh.joint_angles_deg)):
            raise RuntimeError("arm moved during preflight; prepare a stationary pose and retry")
        # From here cleanup must send damping even if the start wait fails.
        began = True
        command(DIAG_OP_GUARD, 1)
        wait_until(lambda: probe.info()["guard_enable"] == 1, arm.keepalive,
                   timeout=min(1, budget.remaining()))
        if probe.info()["guard_fault"]:
            raise RuntimeError("firmware guard latched before motion")
        arm.start()
        wait_until(lambda: arm.mode == arm.board_defs.MODE_RUN, arm.keepalive, timeout=2)

        def send(q=base[0]):
            budget.check()
            current = check_snapshot(arm, low, high)
            if not low[0]+5 < current.joint_angles_deg[0] < high[0]-5:
                raise RuntimeError("JOINT1 entered soft-limit reserve")
            target = list(base)
            target[0] = q
            if any(not low[i] <= v <= high[i] for i, v in enumerate(target)):
                raise RuntimeError("target outside firmware soft limits")
            started = time.perf_counter()
            arm.send_ctrl_pos([math.radians(v) for v in target], gripper_raw_deg=grip)
            commands.append(dict(time=started, send_end=time.perf_counter(), q=q,
                                 feedback_time=current.taken_at, feedback_q=current.joint_angles_deg[0],
                                 feedback_v=current.joint_velocities_deg_s[0]))

        end = time.monotonic()+.25
        while time.monotonic() < end:
            send()
            time.sleep(.01)
        calibration = calibrate(probe, directory, send, budget=budget)
        cycle_hz = calibration["cycles_per_s"]
        # Correct the independent firmware guard for the externally measured
        # clock, including the hypothesized HSE/SystemCoreClock discrepancy.
        command(DIAG_OP_CLOCK_HZ, round(cycle_hz))
        wait_until(lambda: probe.info()["guard_cycle_hz"] == round(cycle_hz), send,
                   timeout=min(1, budget.remaining()))
        save_json(directory/"initial_runtime.json", info)
        for kind in ("hold", "steps", "smooth"):
            for repeat in range(args.repeats):
                if probe.info()["guard_fault"]:
                    raise RuntimeError("firmware guard latched")
                command(DIAG_OP_CAPTURE, 1)
                wait_until(lambda: probe.info()["state"] == 2, send, timeout=min(1, budget.remaining()))
                start = time.perf_counter()
                next_send = start
                # Real PC monotonic time controls stimulus. No catch-up burst.
                duration = .5 if kind == "hold" else 2.45
                while time.perf_counter()-start < duration:
                    now = time.perf_counter()
                    if now < next_send:
                        time.sleep(next_send-now)
                    now = time.perf_counter()
                    send(trajectory(kind, now-start, base[0], amplitude))
                    next_send = max(next_send+.01, time.perf_counter())
                send()
                if probe.info()["state"] == 2:
                    command(DIAG_OP_CAPTURE, 2)
                wait_until(lambda: probe.info()["state"] == 3, send, timeout=min(1, budget.remaining()))
                case_dir = directory/f"{kind}-{repeat+1}"
                rows, metadata = probe.capture(case_dir, send)
                save_json(case_dir/"stimulus.json", dict(kind=kind, pc_start=start,
                    center_deg=base[0], amplitude_deg=amplitude))
                result = analyze(rows, metadata, cycle_hz)
                save_json(case_dir/"analysis.json", result)
                plots.append((rows, metadata, cycle_hz, case_dir/"waveform.png"))
                print(kind, repeat+1, json.dumps(result, ensure_ascii=False), flush=True)
                if not result["valid_for_phase_conclusion"] or probe.info()["guard_fault"]:
                    raise RuntimeError("invalid control source/guard event; raw capture saved, test stopped")
                end = time.monotonic()+.1
                while time.monotonic() < end:
                    send()
                    time.sleep(.01)
        budget.check()
        measurements_complete = True
    finally:
        try:
            if began:
                # Use the existing damping exit; this is not a static position hold.
                arm.damping()
                wait_until(lambda: arm.mode == arm.board_defs.MODE_DAMPING, arm.keepalive, timeout=2)
                damping_confirmed = True
                command(DIAG_OP_GUARD, 0)
                wait_until(lambda: probe.info()["guard_enable"] == 0, arm.keepalive, timeout=1)
        finally:
            try:
                link.close()
            finally:
                save_json(directory/"commands.json", commands)
                save_json(directory/"timing.json", dict(budget_s=MOTION_BUDGET_S,
                    measurement_and_cleanup_s=time.monotonic()-budget.start,
                    measurements_complete=measurements_complete, damping_confirmed=damping_confirmed,
                    excludes="connection/ELF verification, prior initialization, final plotting"))
    # Matplotlib startup can take seconds: render only after confirmed damping.
    for plot_args in plots:
        plot(*plot_args)


def plot(rows, info, cycles_hz, filename):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from joint_diag_data import unwrap
    t = np.array(unwrap([r["cycles"] for r in rows])) / cycles_hz
    # Plot each controller snapshot at its OWN time, never at a new Cmd time.
    mt = t + np.array([((r["motor_cycles"]-r["cycles"]+(1 << 31)) & 0xffffffff)-(1 << 31)
                       for r in rows])/cycles_hz
    to_joint = info["direction"]/info["reduction"]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, [r["target"] for r in rows], label="0x16 target", alpha=.5)
    axes[0].plot(t, [r["q"] for r in rows], label="post q")
    axes[0].plot(mt, [(r["motor_fdb_pos"]-info["zero_deg"])*to_joint for r in rows], label="feedback q")
    axes[0].set_ylabel("joint deg")
    for key, label in (("speed_comp", "velocity FF"), ("ref_speed", "FF + position PID"), ("fdb_speed", "feedback speed")):
        axes[1].plot(mt, [r["motor_"+key]*to_joint for r in rows], label=label)
    axes[1].set_ylabel("joint deg/s")
    axes[2].plot(mt, [r["motor_pid_out"] for r in rows], label="position PID output")
    axes[2].plot(mt, [info["pos_kp"]*r["motor_pid_error"] for r in rows], label="Kp * error")
    axes[2].plot(mt, [info["pos_kd"]*r["motor_pid_d_error"] for r in rows], label="Kd * d_error")
    axes[2].set_ylabel("motor deg/s")
    axes[2].set_xlabel("externally calibrated time (s)")
    for ax in axes:
        ax.grid(True, alpha=.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(filename, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("inspect", "clock", "bench", "run", "analyze"))
    parser.add_argument("--elf", type=Path)
    parser.add_argument("--serial", type=int, help="J-Link serial number")
    parser.add_argument("--device", default="STM32H723ZG")
    parser.add_argument("--port", help="H7 USB CDC/UART SDK port, e.g. COM8 or /dev/ttyACM0")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--ruckig-lib", type=Path, help="native shared library built from this checkout")
    parser.add_argument("--amplitude", type=float, default=45)
    parser.add_argument("--repeats", type=int, choices=(1,), default=1,
                        help="one pass keeps the complete measurement campaign within 30 s")
    args = parser.parse_args()
    if args.action == "analyze":
        clock = json.loads((args.out/"clock_summary.json").read_text())
        for metadata_path in sorted(args.out.glob("*/metadata.json")):
            info = json.loads(metadata_path.read_text())
            raw = (metadata_path.parent/"samples.bin").read_bytes()
            if info["record_bytes"] != C.sizeof(Sample) or len(raw) != info["count"]*C.sizeof(Sample):
                raise ValueError("invalid raw capture ABI/length")
            rows = [flatten_sample(Sample.from_buffer_copy(raw, i*C.sizeof(Sample))) for i in range(info["count"])]
            save_json(metadata_path.parent/"analysis.json", analyze(rows, info, clock["cycles_per_s"]))
            plot(rows, info, clock["cycles_per_s"], metadata_path.parent/"waveform.png")
        return
    if not args.elf or (args.action == "run" and (not args.port or not args.ruckig_lib)):
        parser.error("--elf required; run also requires --port and --ruckig-lib")
    # Never mix two experiments or silently overwrite previous evidence.
    args.out.mkdir(parents=True, exist_ok=False)
    probe = None
    try:
        probe = Probe(args.elf, args.serial, args.device)
        save_json(args.out/"firmware.json", dict(probe.info(), elf=str(args.elf.resolve()),
            elf_sha256=probe.elf_sha256, flash_verified=True))
        if args.action == "inspect":
            print(json.dumps(probe.info(), indent=2))
        elif args.action == "clock":
            print(json.dumps(calibrate(probe, args.out), indent=2))
        elif args.action == "bench":
            budget = Budget(BENCH_BUDGET_S)
            clock = calibrate(probe, args.out, budget=budget)
            previous_generation = probe.info()["generation"]
            probe.benchmark_request(3)
            wait_until(lambda: probe.info()["generation"] != previous_generation and
                       probe.info()["request"] == 0 and
                       (probe.info()["state"] == 5 or probe.info()["error"] != 0), timeout=budget.remaining())
            bench = as_dict(probe.object("h7_diag_bench", Benchmark))
            bench["request_error"] = probe.info()["error"]
            bench["budget_s"] = BENCH_BUDGET_S
            bench["measurement_s"] = time.monotonic()-budget.start
            for cost in bench["cost"]:
                cost["mean_us"] = cost["mean_cycles"]/clock["cycles_per_s"]*1e6
                cost["max_us"] = cost["max_cycles"]/clock["cycles_per_s"]*1e6
                cost["one_axis_500hz_percent"] = cost["mean_us"]*.05
                cost["six_axes_500hz_percent_estimate"] = cost["one_axis_500hz_percent"]*6
            save_json(args.out/"benchmark.json", bench)
            print(json.dumps(bench, indent=2))
            if bench["request_error"] or bench["status"] or not bench["completed"]:
                raise RuntimeError("benchmark rejected/incomplete; see benchmark.json")
        else:
            run_motion(probe, args, args.out, Budget(MOTION_BUDGET_S-4))
    except BaseException as exc:
        save_json(args.out/"failure.json", dict(error=repr(exc)))
        if probe is not None and args.action == "bench":
            probe.benchmark_request(2)  # Cancel at the next benchmark iteration.
        raise
    finally:
        if probe is not None:
            probe.close()


if __name__ == "__main__":
    main()
