import math
import unittest

from arx_x5_control.numerics import PID, PIDConfig, TrackingDifferentiator, fsg
from arx_x5_control.trajectory import PlanGroup, Spline, SplineKind, Waypoint
from arx_x5_control.types import Command, CommandKind, MotorFeedback


class NumericsTests(unittest.TestCase):
    def test_pid_integral_separation_and_firmware_derivative_retention(self):
        pid = PID(PIDConfig(6, .08, 50, 200, 1000, 1))
        self.assertAlmostEqual(pid.step(.5, 0), 28.04)
        self.assertAlmostEqual(pid.step(.5, 0), 28.08)
        self.assertEqual(pid.derivative, .5)
        pid.step(2, 0)
        self.assertEqual(pid.integral, 0)
        self.assertEqual(pid.step(100, 0), 1000)

    def test_td_acceleration_and_convergence(self):
        td = TrackingDifferentiator(700)
        previous = 0
        for _ in range(2500):
            _, velocity = td.step(100)
            self.assertLessEqual(abs(velocity - previous), 1.400001)
            previous = velocity
        self.assertAlmostEqual(td.position, 100)
        self.assertAlmostEqual(td.velocity, 0)
        self.assertEqual(fsg(1, 1), 0)

    def test_spline_boundary_conditions(self):
        a, b = Waypoint(10, 2, -1), Waypoint(-20, 3, .5)
        for kind in SplineKind:
            path = Spline(a, b, 2, kind)
            self.assertAlmostEqual(path.sample(-1).position, a.position)
            self.assertAlmostEqual(path.sample(3).position, b.position)
            if kind != SplineKind.LINEAR:
                self.assertAlmostEqual(path.sample(0).velocity, a.velocity)
                self.assertAlmostEqual(path.sample(2).velocity, b.velocity)
            if kind == SplineKind.QUINTIC:
                self.assertAlmostEqual(path.sample(0).acceleration, a.acceleration)
                self.assertAlmostEqual(path.sample(2).acceleration, b.acceleration)

    def test_multisegment_exact_boundaries_and_skipped_time(self):
        group = PlanGroup.through(((0, 10, 20), (5, 3, 5)), (1, 2))
        group.start(0)
        self.assertAlmostEqual(group.update(1000)[0].position, 10)
        self.assertAlmostEqual(group.current[0].velocity, 7.5)
        self.assertEqual(group.current[1].velocity, 0)
        self.assertAlmostEqual(group.update(4000)[0].position, 20)
        self.assertFalse(group.running)

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            Command(0, CommandKind.POSITION, (0,) * 5)
        with self.assertRaises(ValueError):
            MotorFeedback(0, 0, 0, position=math.nan)
        with self.assertRaises(ValueError):
            Spline(Waypoint(0), Waypoint(1), 0)


if __name__ == "__main__":
    unittest.main()
