"""Compile original firmware algorithms on the host and compare Python outputs."""

import ctypes as C
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

from arx_x5_control.config import POSITION_PID
from arx_x5_control.numerics import PID, TrackingDifferentiator
from arx_x5_control.trajectory import Spline, SplineKind, Waypoint
from libs.gravity_compensation import GravityCompensator

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
                   "libs/controller/adrc.c", "libs/gravity_compensation/c/x5_gravity.c"]
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
        for r in (700, 1000, 2000):
            self.lib.reference_td_init(r)
            td = TrackingDifferentiator(r)
            out = Float2()
            for ref in [100.0]*1000 + [-50.0]*1000 + [0.0]*1000:
                self.lib.reference_td_step(ref, out)
                actual = td.step(ref)
                self.assertAlmostEqual(actual[0], out[0], delta=.002)
                self.assertAlmostEqual(actual[1], out[1], delta=.02)

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


if __name__ == "__main__":
    unittest.main()
