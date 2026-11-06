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
ruckig = build / "ruckig"
subprocess.run(["cmake", "-S", str(ROOT / "libs/ruckig"), "-B", str(ruckig),
                "-DCMAKE_BUILD_TYPE=Release",
                "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer"],
               check=True, cwd=ROOT)
subprocess.run(["cmake", "--build", str(ruckig), "-j", "4"], check=True, cwd=ROOT)
subprocess.run(["cc", "-std=gnu11", "-O2", "-g", "-ffunction-sections", "-fdata-sections",
                "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-UNDEBUG",
                "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast", *flags,
                str(Path(__file__).with_name("control_dynamics_test.c")),
                str(ROOT / "libs/gravity_compensation/c/arm_dynamics.c"),
                str(ROOT / "libs/common/common.c"), str(ruckig / "libruckig_c.a"),
                "-Wl,--gc-sections,--wrap=malloc,--wrap=calloc,--wrap=realloc,--wrap=free,--wrap=ruckig_c_update",
                "-lstdc++", "-lm", "-o", str(exe)], check=True, cwd=ROOT)
subprocess.run([str(exe)], check=True)
