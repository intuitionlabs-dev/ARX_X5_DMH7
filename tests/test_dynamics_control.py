import math
from pathlib import Path
import re
from unittest.mock import patch
import unittest

from arx_x5_control import (ArmConfig, ArmMode, Command, CommandKind as K, ControlSystem,
                            DynamicsStatus, MotorAction, RobotState)
from arx_x5_control.config import JOINT_QUINTIC_DURATION_S, MOTOR_KP, POSITION_PID, TD_R
from arx_x5_control.trajectory import JointQuinticReference, Spline, Waypoint
from arx_x5_control.gravity_compensation import DynamicsCompensator, DynamicsError
from test_control import samples


class DynamicsControlTests(unittest.TestCase):
    def system(self, **config):
        system = ControlSystem([ArmConfig(**config)] * 2)
        for arm in system.arms:
            arm.connected = arm.first_init = True
            arm.enter(RobotState.NORMAL, 0)
            arm.initialized = [True] * 7
        return system

    def test_fixed_duration_and_replanning_boundary_conditions(self):
        ref = JointQuinticReference(JOINT_QUINTIC_DURATION_S)
        start = Waypoint(5, 12, -20)
        ref.follow(start)
        first = ref.update(35, 0)
        self.assertEqual(ref.curve.sample(0), start)
        self.assertEqual(first, ref.curve.sample(.002))
        for _ in range(30):
            ref.update(35, 0)
        boundary = ref.current
        ref.update(-10, 0)
        self.assertEqual(ref.curve.sample(0), boundary)
        for _ in range(79):
            ref.update(-10, 0)
        self.assertEqual(ref.current, Waypoint(-10))
        self.assertEqual(ref.update(-10, 0), Waypoint(-10))

    def test_position_command_uses_quintic_and_analytic_acceleration(self):
        system = self.system(gravity_enabled=False)
        arm = system.arms[0]
        path = Spline(Waypoint(0), Waypoint(30), .16)
        for now in range(0, 160, 2):
            commands = [Command(0, K.START), Command(0, K.POSITION, (30, .5, .5, 0, 0, 0))] if now == 0 else ()
            system.step(now, commands=commands, motor_feedback=samples(now, 2))
            expected = path.sample((now + 2) / 1000)
            for actual, value in zip((arm.command.angle[0], arm.command.velocity[0],
                                      arm.command.acceleration[0]),
                                     (expected.position, expected.velocity, expected.acceleration)):
                self.assertAlmostEqual(actual, value, delta=1e-7)
        self.assertEqual((arm.command.angle[0], arm.command.velocity[0], arm.command.acceleration[0]),
                         (30, 0, 0))
        self.assertEqual(system.arms[1].command.angle[0], 0)

    def test_external_velocity_and_plan_handoff_preserve_state(self):
        system = self.system(gravity_enabled=False)
        arm = system.arms[0]
        system.step(0, commands=[Command(0, K.START), Command(0, K.POSITION_VELOCITY,
                    (30, .5, .5, 0, 0, 0), 0, (20,) * 6)], motor_feedback=samples(0, 2))
        self.assertEqual(arm.command.velocity, [20] * 6)
        self.assertEqual(arm.command.acceleration, [0] * 6)
        boundary = arm.command.quintic[0].current
        system.step(2, commands=[Command(0, K.POSITION, (30, .5, .5, 0, 0, 0))],
                    motor_feedback=samples(2, 2))
        self.assertEqual(arm.command.quintic[0].curve.sample(0), boundary)
        arm.command.start_auto("reset")
        system.step(4, motor_feedback=samples(4, 2, positions=(20, 30, 45, 0, 0, 0, 0)))
        system.step(104, motor_feedback=samples(104, 2))
        boundary = arm.command.quintic[0].current
        self.assertNotEqual(boundary.acceleration, 0)
        arm.command.cancel()
        arm.control = None
        system.step(106, motor_feedback=samples(106, 2))
        self.assertEqual(arm.command.quintic[0].curve.sample(0), boundary)

    def test_run_consumes_feedback_velocity_and_reference_acceleration(self):
        system = self.system()
        arm = system.arms[0]
        feedback = samples(0, 2, positions=(0, 30, 45, 10, 0, 0, 0))
        system.step(0, commands=[Command(0, K.START), Command(0, K.POSITION, (10, 40, 60, 0, 0, 0))],
                    motor_feedback=feedback)
        q = tuple(map(math.radians, arm.joint_angle))
        qdd = tuple(map(math.radians, arm.command.acceleration))
        expected = arm.dynamics.torques(q, (0,) * 6, qdd)
        self.assertEqual(arm.dynamics_torque, expected)
        self.assertNotEqual(arm.dynamics_torque, arm.gravity_torque)
        self.assertEqual(arm.snapshot(0).dynamics_torque, expected)
        self.assertEqual(arm.dynamics_status, DynamicsStatus.OK)
        arm.joint_velocity = [30, -20, 10, 5, 0, 0]
        arm.drive(0)
        self.assertEqual(arm.dynamics_torque, arm.dynamics.torques(q,
                         tuple(map(math.radians, arm.joint_velocity)), qdd))

    def test_viscous_feedforward_uses_joint_reference_velocity(self):
        system = self.system(gravity_enabled=False, direction=(-1,) * 6,
                             reduction=(2,) * 6, viscous_damping=(.5,) * 6,
                             torque_scale=(3,) * 6)
        arm = system.arms[0]
        system.step(0, commands=[Command(0, K.START), Command(0, K.POSITION_VELOCITY,
                    (0, .5, .5, 0, 0, 0), 0, (12,) * 6)], motor_feedback=samples(0, 2))
        for motor in arm.bank.motors[:6]:
            self.assertAlmostEqual(motor.output_comp, -math.radians(12) * .5 * 3)

    def test_passive_modes_and_failure_clear_compensation(self):
        system = self.system()
        arm = system.arms[0]
        system.step(0, commands=[Command(0, K.GRAVITY)],
                    motor_feedback=samples(0, 2, positions=(0, 30, 45, 0, 0, 0, 0)))
        self.assertNotEqual(arm.dynamics_torque, (0,) * 6)
        self.assertEqual(arm.dynamics_torque, arm.gravity_torque)
        for now, kind in ((2, K.STOP), (4, K.DAMPING)):
            system.step(now, commands=[Command(0, kind)], motor_feedback=samples(now, 2))
            self.assertEqual(arm.dynamics_torque, (0,) * 6)
            self.assertEqual(arm.command.acceleration, [0] * 6)
        system.step(6, commands=[Command(i, K.START) for i in range(2)], motor_feedback=samples(6, 2))
        with patch.object(arm.dynamics, "torques", side_effect=DynamicsError(DynamicsStatus.NONFINITE_RESULT)):
            result = system.step(8, motor_feedback=samples(8, 2))
        self.assertEqual(arm.actual_mode, ArmMode.STOP)
        self.assertEqual(arm.mode, ArmMode.RUN)
        self.assertEqual(arm.dynamics_torque, (0,) * 6)
        self.assertEqual(arm.dynamics_status, DynamicsStatus.NONFINITE_RESULT)
        self.assertEqual(system.arms[1].actual_mode, ArmMode.RUN)
        self.assertTrue(any(e.kind == "dynamics_error" for e in result.events))
        self.assertTrue(all(r.action in (MotorAction.SOFT_STOP, MotorAction.ENABLE)
                            for r in result.motor_requests if r.arm_id == 0))

    def test_reference_failure_produces_damping(self):
        system = self.system()
        arm = system.arms[0]
        with patch.object(arm.command.quintic[0], "update", side_effect=ValueError("nonfinite")):
            result = system.step(0, commands=[Command(0, K.START)], motor_feedback=samples(0, 2))
        self.assertEqual(arm.actual_mode, ArmMode.DAMPING)
        self.assertEqual(arm.command.velocity, [0] * 6)
        self.assertEqual(arm.command.acceleration, [0] * 6)
        self.assertEqual(arm.dynamics_torque, (0,) * 6)
        self.assertTrue(any(e.kind == "reference_error" for e in result.events))

    def test_dynamics_range_validation_and_static_limit(self):
        solver = DynamicsCompensator.from_model("2023")
        zero = (0,) * 6
        for mode in (False, True):
            q = (0, .4, .8, -.3, 0, .1)
            self.assertEqual(len(solver.torques(q, zero, zero)), 6)
            gravity = solver.sdk_torques(q) if mode else solver.raw_torques(q)
            for actual, expected in zip(solver.torques(q, zero, zero, sdk_scale=mode), gravity):
                self.assertAlmostEqual(actual, expected, delta=1e-12)
        for q, qd, qdd, status in (((129,) * 6, zero, zero, DynamicsStatus.ANGLE_RANGE),
                                  (zero, (math.nan,) * 6, zero, DynamicsStatus.INVALID_ARGUMENT),
                                  (zero, zero, (math.inf,) * 6, DynamicsStatus.INVALID_ARGUMENT),
                                  (zero, (1e308,) * 6, zero, DynamicsStatus.NONFINITE_RESULT)):
            with self.assertRaises(DynamicsError) as error:
                solver.torques(q, qd, qdd)
            self.assertEqual(error.exception.status, status)

    def test_configuration_matches_firmware(self):
        root = Path(__file__).resolve().parents[1]
        cmd = (root / "App/gimbal_cmd.c").read_text()
        driver = (root / "App/gimbal_arm.c").read_text()
        numbers = lambda values: tuple(float(v.strip().rstrip("f")) for v in values.split(",") if v.strip())
        td = re.search(r"const float td_r\[ARM_JOINT_NUM\] = \{(.*?)\}", cmd, re.S)[1]
        self.assertEqual(TD_R, numbers(re.sub(r"/\*.*?\*/", "", td, flags=re.S)))
        for i, config in enumerate(POSITION_PID):
            joint = f"JOINT{i+1}" if i < 6 else "GRIPPER"
            match = re.search(rf"PID_SetConfig_Pos\(&obj->pid_cfgs\[{joint}\]\[0\],\s*(.*?)\);", driver)
            self.assertEqual((config.kp, config.ki, config.kd, config.integral_limit, config.output_limit),
                             numbers(match[1]))
            field = "encos_kpkd" if i < 3 else "pid_mit_kpkd"
            kp = re.search(rf"obj->{field}\[{joint}\]\[0\] = ([^;]+);", driver)[1]
            self.assertEqual(MOTOR_KP[i], float(kp.strip().rstrip("f")))

    def test_angle_range_stops_and_recovers_each_active_mode(self):
        for kind in (K.START, K.GRAVITY):
            system = self.system()
            result = system.step(0, commands=[Command(0, kind)], motor_feedback=samples(
                0, 2, positions=(math.degrees(129), 30, 45, 0, 0, 0, 0)))
            arm = system.arms[0]
            self.assertEqual(arm.actual_mode, ArmMode.STOP)
            self.assertEqual(arm.dynamics_status, DynamicsStatus.ANGLE_RANGE)
            self.assertTrue(any(e.kind == "dynamics_error" for e in result.events))
            system.step(2, motor_feedback=samples(2, 2))
            self.assertEqual(arm.dynamics_status, DynamicsStatus.OK)
            self.assertEqual(arm.actual_mode, arm.mode)


if __name__ == "__main__":
    unittest.main()
