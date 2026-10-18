"""Compile original firmware algorithms on the host and compare Python outputs."""

import ctypes as C
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest

from arx_x5_control import ArmConfig, ArmMode, Command, CommandKind, ControlSystem
from arx_x5_control.config import POSITION_PID
from arx_x5_control.numerics import PID, TrackingDifferentiator
from arx_x5_control.trajectory import Spline, SplineKind, Waypoint
from libs.gravity_compensation import DynamicsCompensator, GravityCompensator

ROOT = Path(__file__).resolve().parents[1]
Float2, Float3, Float6 = C.c_float * 2, C.c_float * 3, C.c_float * 6


@unittest.skipUnless(shutil.which("cc"), "C parity checks require a host C compiler")
class FirmwareParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="arx-c-parity-")
        cls.addClassCleanup(cls.tmp.cleanup)
        library = Path(cls.tmp.name) / "reference.so"
        sources = ["tests/c_reference.c", "libs/common/common.c", "libs/controller/pid.c",
                   "libs/controller/adrc.c", "libs/gravity_compensation/c/x5_gravity.c",
                   "libs/gravity_compensation/c/x5_dynamics.c"]
        sources += [f"libs/move_plan/curves/{name}_spline.c" for name in ("linear", "cubic", "quintic")]
        includes = ["libs/common", "libs/controller", "libs/move_plan/curves", "libs/gravity_compensation/c"]
        subprocess.run([shutil.which("cc"), "-std=c99", "-O2", "-fPIC", "-shared",
                        *["-I" + str(ROOT / p) for p in includes],
                        *[str(ROOT / p) for p in sources], "-lm", "-o", str(library)],
                       check=True, capture_output=True, text=True)
        cls.lib = C.CDLL(str(library))
        fptr = C.POINTER(C.c_float)
        cls.lib.reference_pid_init.argtypes = [C.c_float] * 6
        cls.lib.reference_pid_step.argtypes = [C.c_float] * 2
        cls.lib.reference_pid_step.restype = C.c_float
        cls.lib.reference_td_init.argtypes = [C.c_float]
        cls.lib.reference_td_step.argtypes = [C.c_float, fptr]
        cls.lib.reference_spline.argtypes = [C.c_int, fptr, fptr, C.c_float, C.c_float, fptr]
        cls.lib.x5_gravity_compute.argtypes = [C.c_int, fptr, fptr, C.c_int, fptr]
        cls.lib.x5_gravity_compute.restype = C.c_int
        cls.lib.x5_dynamics_compute.argtypes = [C.c_int, fptr, fptr, fptr, fptr, C.c_int, fptr]
        cls.lib.x5_dynamics_compute.restype = C.c_int
        command_library = Path(cls.tmp.name) / "command.so"
        command_includes = ["App/inc", "Bsp/inc", "Core/Inc", "Drivers/CMSIS/Include",
                            "Drivers/CMSIS/Device/ST/STM32H7xx/Include", "Drivers/STM32H7xx_HAL_Driver/Inc",
                            "libs/move_plan/curves", "libs/move_plan/move_plan", "libs/gravity_compensation/c"]
        command_includes += [str(p.relative_to(ROOT)) for p in (ROOT / "libs").iterdir() if p.is_dir()]
        subprocess.run([shutil.which("cc"), "-std=gnu11", "-O2", "-fPIC", "-shared",
                        "-fvisibility=hidden", "-ffunction-sections", "-fdata-sections",
                        "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-DUSE_PWR_LDO_SUPPLY",
                        "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast",
                        *["-I" + str(ROOT / p) for p in command_includes],
                        *[str(ROOT / p) for p in ("tests/c_command_reference.c", "libs/controller/adrc.c",
                          "libs/common/common.c", "libs/move_plan/curves/quintic_spline.c")],
                        "-Wl,--gc-sections", "-Wl,-z,defs", "-lm", "-o", str(command_library)],
                       check=True, capture_output=True, text=True)
        cls.command_lib = C.CDLL(str(command_library))
        cls.command_lib.reference_command_init.argtypes = [fptr]
        cls.command_lib.reference_command_step.argtypes = [C.c_int] * 3 + [fptr] * 4

    def test_pid_against_original_c(self):
        rng = random.Random(123)
        for config in POSITION_PID:
            self.lib.reference_pid_init(config.kp, config.ki, config.kd,
                                         config.integral_limit, config.output_limit, config.integral_range)
            pid = PID(config)
            sequence = [(.5, 0)] * 10 + [(1, .5), (1, .75), (2, 0)] * 10
            sequence += [(rng.uniform(-100, 100), rng.uniform(-100, 100)) for _ in range(300)]
            for ref, fdb in sequence:
                ref, fdb = C.c_float(ref).value, C.c_float(fdb).value
                expected = self.lib.reference_pid_step(ref, fdb)
                self.assertAlmostEqual(pid.step(ref, fdb), expected, delta=.002)

    def test_td_against_original_c(self):
        for r in (1000, 2000, 4000):
            self.lib.reference_td_init(r)
            td = TrackingDifferentiator(r)
            out = Float3()
            for ref in [100.0]*1000 + [-50.0]*1000 + [0.0]*1000:
                self.lib.reference_td_step(ref, out)
                actual = td.step(ref)
                self.assertAlmostEqual(actual[0], out[0], delta=.002)
                self.assertAlmostEqual(actual[1], out[1], delta=.02)
                self.assertAlmostEqual(td.acceleration, out[2], delta=3.0)

    def test_splines_against_original_c(self):
        rng = random.Random(456)
        for kind in SplineKind:
            for _ in range(200):
                start = Float3(*(rng.uniform(-100, 100) for _ in range(3)))
                end = Float3(*(rng.uniform(-100, 100) for _ in range(3)))
                duration = C.c_float(rng.uniform(.5, 5)).value
                t = C.c_float(rng.uniform(0, duration)).value
                out = Float3()
                self.lib.reference_spline(kind, start, end, duration, t, out)
                actual = Spline(Waypoint(*start), Waypoint(*end), duration, kind).sample(t)
                for a, b in zip((actual.position, actual.velocity, actual.acceleration), out):
                    self.assertAlmostEqual(a, b, delta=.02)

    def test_gravity_against_original_c(self):
        rng = random.Random(789)
        for model_index, model in enumerate(("2023", "master", "2025")):
            for g in ((0, 0, -9.81), (2, -3, -8)):
                gravity = Float3(*g)
                reference = GravityCompensator.from_model(model, gravity=tuple(gravity))
                for _ in range(300):
                    q = Float6(*(rng.uniform(-3.14, 3.14) for _ in range(6)))
                    for mode in (0, 1):
                        out = Float6()
                        self.assertEqual(self.lib.x5_gravity_compute(model_index, q, gravity, mode, out), 0)
                        actual = reference.raw_torques(q) if mode == 0 else reference.sdk_torques(q)
                        for a, b in zip(actual, out):
                            self.assertAlmostEqual(a, b, delta=1e-5)

    def test_dynamics_against_original_c(self):
        rng = random.Random(20261017)
        for model_index, model in enumerate(("2023", "master", "2025")):
            for g in ((0, 0, -9.81), (2, -3, -8), (0, 0, 0), (0, 0, 9.81)):
                gravity = Float3(*g)
                reference = DynamicsCompensator.from_model(model, gravity=tuple(gravity))
                for sample in range(80):
                    q = Float6(*(rng.uniform(-3.14, 3.14) for _ in range(6)))
                    qd = Float6(*(rng.uniform(-3, 3) if sample % 4 in (1, 3) else 0 for _ in range(6)))
                    qdd = Float6(*(rng.uniform(-70, 70) if sample % 4 in (2, 3) else 0 for _ in range(6)))
                    for mode in (0, 1):
                        out = Float6()
                        self.assertEqual(self.lib.x5_dynamics_compute(model_index, q, qd, qdd,
                                                                      gravity, mode, out), 0)
                        actual = reference.torques(q, qd, qdd, sdk_scale=bool(mode))
                        for a, b in zip(actual, out):
                            self.assertAlmostEqual(a, b, delta=5e-5 + 4e-6*abs(b))

    def test_command_sources_and_replanning_against_original_c(self):
        arm = ControlSystem([ArmConfig(gravity_enabled=False)]).arms[0]
        arm.initialized = [True] * 7
        arm.joint_angle = [0, 40, 60, 0, 0, 0]
        self.command_lib.reference_command_init(Float6(*arm.joint_angle))
        out = (C.c_float * 18)()
        rng = random.Random(1017)
        for step in range(400):
            arm.mode = ArmMode.DAMPING if 250 <= step < 260 else ArmMode.RUN
            planning = 160 <= step < 200
            external = 100 <= step < 140
            target = Float6(*(rng.uniform(5, 70) for _ in range(6))) if step % 15 == 0 else target
            if arm.mode != ArmMode.RUN:
                target = Float6(*arm.joint_angle)
            velocity = Float6(*(rng.uniform(-10, 10) for _ in range(6)))
            acc = Float6(*(rng.uniform(-20, 20) for _ in range(6)))
            arm.command.auto = "parity" if planning else None
            arm.command.plan = SimpleNamespace(running=planning, current=tuple(
                Waypoint(*values) for values in zip(target, velocity, acc)))
            arm.control = Command(0, CommandKind.POSITION_VELOCITY if external else CommandKind.POSITION,
                                  tuple(target), 0, tuple(velocity) if external else ())
            arm.command.update(step * 2)
            self.command_lib.reference_command_step(arm.mode, planning, external, target, velocity, acc, out)
            actual = arm.command.angle + arm.command.velocity + arm.command.acceleration
            for index, (a, b) in enumerate(zip(actual, out)):
                self.assertAlmostEqual(a, b, delta=(.002, .02, 2.0)[index // 6])


if __name__ == "__main__":
    unittest.main()
