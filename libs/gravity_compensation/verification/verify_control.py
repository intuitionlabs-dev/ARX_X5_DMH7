#!/usr/bin/env python3

"""使用固件的头文件配置在主机验证控制层。"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[3]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--compile-commands", type=Path,
                    default=ROOT / "build/Debug/compile_commands.json")
parser.add_argument("--build-dir", type=Path, default=ROOT / "build/dynamics-host")
args = parser.parse_args()
commands = json.loads(args.compile_commands.read_text())
command = next(c for c in commands if c["file"].endswith("/App/gimbal_arm.c"))
arguments = shlex.split(command["command"])
flags = [flag for flag in arguments if flag.startswith(("-I", "-D"))]
build = args.build_dir.resolve()
build.mkdir(parents=True, exist_ok=True)
exe = build / "control_dynamics_test"
subprocess.run(["cc", "-std=gnu11", "-O2", "-g", "-ffunction-sections", "-fdata-sections",
                "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast", *flags,
                str(Path(__file__).with_name("control_dynamics_test.c")),
                str(ROOT / "libs/gravity_compensation/c/x5_dynamics.c"),
                str(ROOT / "libs/gravity_compensation/c/x5_gravity.c"),
                str(ROOT / "libs/controller/adrc.c"),
                str(ROOT / "libs/controller/pid.c"),
                str(ROOT / "libs/move_plan/curves/quintic_spline.c"),
                str(ROOT / "libs/common/common.c"), "-Wl,--gc-sections",
                "-Wl,--wrap=x5_dynamics_compute", "-lm", "-o", str(exe)],
               check=True, cwd=ROOT)
subprocess.run([str(exe)], check=True)
