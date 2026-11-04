#!/usr/bin/env python3
"""Compare committed quintic/Ruckig firmware code, optionally FP64 vs FP32."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ("settled", "step_1hz", "smooth_100hz", "smooth_500hz", "random_500hz")
BASELINE = "3317d9b9dbe7afa5f0b0b182fe44cf94a8752ec7"


def run(args, **kwargs):
    result = subprocess.run(args, capture_output=True, text=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"{args[0]} failed ({result.returncode})\n{result.stdout}\n{result.stderr}")
    return result.stdout.strip()


def snapshot(ref, destination):
    commit = run(["git", "rev-parse", "--verify", ref + "^{commit}"], cwd=ROOT)
    paths = ["App", "Bsp/inc", "Core/Inc", "libs", "Drivers/CMSIS/Include",
             "Drivers/CMSIS/Device/ST/STM32H7xx/Include", "Drivers/STM32H7xx_HAL_Driver/Inc"]
    archive = subprocess.run(["git", "archive", commit, *paths], cwd=ROOT,
                             capture_output=True, check=True).stdout
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        tar.extractall(destination, filter="data")
    return commit


def build(source, output, ruckig, cmake, cc):
    output.mkdir(parents=True, exist_ok=True)
    library = []
    if ruckig:
        run([cmake, "-S", str(source / "libs/ruckig"), "-B", str(output / "ruckig"),
             "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF",
             "-DCMAKE_C_FLAGS_RELEASE=-Os -DNDEBUG", "-DCMAKE_CXX_FLAGS_RELEASE=-Os -DNDEBUG"])
        run([cmake, "--build", str(output / "ruckig"), "-j", "4"])
        library = [str(output / "ruckig/libruckig_c.a"), "-lstdc++"]
    includes = ["App", "App/inc", "App/profiles", "Bsp/inc", "Core/Inc",
                "Drivers/CMSIS/Include", "Drivers/CMSIS/Device/ST/STM32H7xx/Include",
                "Drivers/STM32H7xx_HAL_Driver/Inc", "libs/gravity_compensation/c",
                "libs/move_plan/curves", "libs/move_plan/move_plan"]
    includes += [str(p.relative_to(source)) for p in (source / "libs").iterdir() if p.is_dir()]
    sources = [ROOT / "tests/benchmarks/joint_interpolation.c"]
    sources += [source / p for p in ("libs/common/common.c", "libs/controller/adrc.c",
                                   "libs/move_plan/curves/quintic_spline.c")]
    executable = output / "joint-benchmark"
    command = [cc, "-std=gnu11", "-Os", "-DNDEBUG", "-fno-lto", "-ffunction-sections",
               "-fdata-sections", "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast",
               "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-DUSE_PWR_LDO_SUPPLY",
               f"-DBENCH_RUCKIG={int(ruckig)}", *["-I" + str(source / p) for p in includes],
               *map(str, sources), *library, "-Wl,--gc-sections", "-lm", "-o", str(executable)]
    run(command)
    (output / "build-command.txt").write_text(json.dumps(command, indent=2) + "\n")
    return executable


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", default=BASELINE)
    parser.add_argument("--current", default="HEAD")
    parser.add_argument("--double-ref", help="also measure an earlier double Ruckig commit")
    parser.add_argument("--cycles", type=int, default=200000)
    parser.add_argument("--rounds", type=int, default=7)
    parser.add_argument("--warmup", type=int, default=5000)
    parser.add_argument("--cpu", type=int, default=min(os.sched_getaffinity(0)))
    parser.add_argument("--cmake", default=shutil.which("cmake"))
    parser.add_argument("--cc", default=shutil.which("cc"))
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build/interpolation-benchmark")
    args = parser.parse_args()
    if not args.cmake or not args.cc:
        parser.error("CMake and a C/C++17 host toolchain are required")
    if args.cpu not in os.sched_getaffinity(0):
        parser.error("requested CPU is outside this process's affinity")
    if not 1 <= args.cycles <= 5000000 or not 1 <= args.rounds <= 100 or not 500 <= args.warmup <= 1000000:
        parser.error("cycles 1..5000000, rounds 1..100, warmup 500..1000000 required")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    commits, executables, layouts = {}, {}, {}
    current_name = "ruckig_float" if args.double_ref else "ruckig"
    variants = [("quintic", args.baseline)]
    if args.double_ref:
        variants.append(("ruckig_double", args.double_ref))
    variants.append((current_name, args.current))
    names = [name for name, _ in variants]
    for name, ref in variants:
        print(f"Building {name} from {ref}", flush=True)
        source = output / name / "source"
        if source.exists():
            parser.error(f"output already contains {source}; choose a new --output-dir")
        commits[name] = snapshot(ref, source)
        executables[name] = build(source, output / name / "build", name != "quintic", args.cmake, args.cc)
        layouts[name] = json.loads(run([str(executables[name]), "--info"]))
    metadata = {
        "utc": datetime.now(timezone.utc).isoformat(), "platform": platform.platform(),
        "cpuinfo": Path("/proc/cpuinfo").read_text().split("\n\n", 1)[0],
        "compiler": run([args.cc, "--version"]).splitlines()[0],
        "optimization": "-Os, no LTO, no fast-math", "commits": commits, "layouts": layouts,
        "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "benchmark_sha256": hashlib.sha256((ROOT / "tests/benchmarks/joint_interpolation.c").read_bytes()).hexdigest(),
        "pinned_cpu": args.cpu, "cycles_per_run": args.cycles, "warmup": args.warmup,
        "rounds": args.rounds, "control_period_us": 2000,
        "scope": "Host Arm_Ref_Set plus target copy; excludes dynamics, CAN, RTOS and hardware. Not H7 CPU usage.",
    }
    (output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    # Pin only after compilation, so builds can use the available cores.
    os.sched_setaffinity(0, {args.cpu})
    rows = []
    with (output / "raw.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["algorithm", "arms", "scenario", "round", "cycles",
                                                    "cpu_ns", "wall_ns", "checksum"], lineterminator="\n")
        writer.writeheader()
        for arms in (1, 2):
            for scenario, label in enumerate(SCENARIOS):
                print(f"Measuring {arms} arm(s), {label}", flush=True)
                for repeat in range(args.rounds):
                    # Rotate order to limit temperature/load bias.
                    offset = repeat % len(names)
                    order = names[offset:] + names[:offset]
                    for name in order:
                        fields = run([str(executables[name]), str(arms), str(scenario),
                                      str(args.cycles), str(args.warmup)]).split(",")
                        row = dict(algorithm=name, arms=int(fields[0]), scenario=label,
                                   round=repeat+1, cycles=int(fields[2]), cpu_ns=int(fields[3]),
                                   wall_ns=int(fields[4]), checksum=float(fields[5]))
                        rows.append(row)
                        writer.writerow(row)
                        stream.flush()
    summary = []
    for arms in (1, 2):
        for label in SCENARIOS:
            entry = dict(arms=arms, scenario=label)
            for name in names:
                samples = [r["cpu_ns"]/r["cycles"]/1000 for r in rows
                           if r["arms"] == arms and r["scenario"] == label and r["algorithm"] == name]
                for suffix, value in (("mean_us", statistics.mean(samples)), ("min_us", min(samples)),
                                      ("max_us", max(samples)), ("stdev_us", statistics.pstdev(samples))):
                    entry[f"{name}_{suffix}"] = value
                entry[f"{name}_cpu_percent"] = statistics.mean(samples)/2000*100
            entry["added_cpu_percentage_points"] = entry[f"{current_name}_cpu_percent"] - entry["quintic_cpu_percent"]
            entry["time_ratio"] = entry[f"{current_name}_mean_us"]/entry["quintic_mean_us"]
            if args.double_ref:
                entry["float_speedup_over_double"] = entry["ruckig_double_mean_us"]/entry["ruckig_float_mean_us"]
                entry["float_cpu_change_percentage_points"] = entry["ruckig_float_cpu_percent"]-entry["ruckig_double_cpu_percent"]
            summary.append(entry)
    with (output / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(summary)
    print("arms scenario " + " ".join(f"{name}_us {name}_cpu%" for name in names))
    for row in summary:
        values = " ".join(f"{row[name+'_mean_us']:.3f} {row[name+'_cpu_percent']:.4f}" for name in names)
        print(f"{row['arms']} {row['scenario']:14s} {values}")
    print(f"Results: {output}")


if __name__ == "__main__":
    main()
