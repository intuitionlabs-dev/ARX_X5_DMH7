import unittest
from math import radians

from arx_x5_control import (ArmConfig, ArmMode, Command, CommandKind as K, ControlSystem,
                            MotorAction as A, MotorFeedback, RobotState as S, ZeroResult, ZeroState)
from arx_x5_control.calibration import ZeroCalibration
from arx_x5_control.motor import MotorBank
from arx_x5_control.simulation import SimulatedMotors


def samples(now, arms=1, positions=(0, .5, .5, 0, 0, 0, 0), **kwargs):
    return tuple(MotorFeedback(a, i, now, position=radians(p), velocity=0, **kwargs)
                 for a in range(arms) for i, p in enumerate(positions))


class ControlTests(unittest.TestCase):
    def make(self, count=1):
        return ControlSystem([ArmConfig(gravity_enabled=False)] * count)

    def normal(self, system, arm_id=0):
        # Individual policy tests bypass only the automatic sequence, which is
        # exercised separately end to end using actual semantic motor requests.
        arm = system.arms[arm_id]
        arm.connected = arm.first_init = True
        arm.enter(S.NORMAL, 0)
        arm.initialized = [True] * 7
        return arm

    def test_startup_handshake_gates_and_feedback(self):
        c = self.make()
        r = c.step(0, commands=[Command(0, K.START)], motor_feedback=samples(0))
        self.assertEqual(c.arms[0].state, S.WAIT_HANDSHAKE)
        self.assertTrue(any(e.kind == "rejected" for e in r.events))
        r = c.step(2, commands=[Command(0, K.CONNECT)], motor_feedback=samples(2))
        self.assertEqual(c.arms[0].state, S.WAIT_INIT)
        self.assertTrue(r.feedback)
        c.step(4, commands=[Command(0, K.START)], motor_feedback=samples(4))
        self.assertEqual(c.arms[0].mode, ArmMode.STOP)

    def test_init_pose_rejection(self):
        c = self.make()
        c.step(0, commands=[Command(0, K.CONNECT)], motor_feedback=samples(0, positions=(50, 0, 0, 0, 0, 0, 0)))
        result = c.step(2, commands=[Command(0, K.INITIALIZE)], motor_feedback=samples(2, positions=(50, 0, 0, 0, 0, 0, 0)))
        self.assertEqual(c.arms[0].state, S.WAIT_INIT)
        self.assertIn("20 degree", result.events[0].detail)

    def test_per_arm_watchdog_recovers_without_reinitialization(self):
        c = self.make(2)
        for i in range(2):
            self.normal(c, i)
        c.step(0, commands=[Command(i, K.START) for i in range(2)], motor_feedback=samples(0, 2))
        c.step(2998, commands=[Command(1, K.HEARTBEAT)], motor_feedback=samples(2998, 2))
        c.step(3000, motor_feedback=samples(3000, 2))
        self.assertEqual(c.arms[0].mode, ArmMode.DAMPING)
        self.assertEqual(c.arms[0].state, S.NORMAL)
        self.assertEqual(c.arms[1].mode, ArmMode.RUN)
        c.step(3002, commands=[Command(0, K.START)], motor_feedback=samples(3002, 2))
        self.assertEqual(c.arms[0].mode, ArmMode.RUN)

    def test_motor_loss_stops_only_affected_arm(self):
        c = self.make(2)
        for i in range(2):
            self.normal(c, i)
        c.step(0, commands=[Command(i, K.START) for i in range(2)], motor_feedback=samples(0, 2))
        fresh = [f for f in samples(20, 2) if not (f.arm_id == 0 and f.motor_index == 4)]
        c.step(20, motor_feedback=fresh)
        self.assertEqual(c.arms[0].actual_mode, ArmMode.STOP)
        self.assertEqual(c.arms[1].actual_mode, ArmMode.RUN)

    def test_soft_limits_and_external_velocity(self):
        c = self.make()
        arm = self.normal(c)
        c.step(0, commands=[Command(0, K.START), Command(0, K.POSITION_VELOCITY,
               (1000,) * 6, 1000, (12,) * 6)], motor_feedback=samples(0))
        self.assertEqual(arm.command.source, [90, 206.26, 171.88, 90, 90, 119.17])
        self.assertEqual(arm.command.gripper_angle, 142)
        self.assertEqual(arm.command.velocity, [12] * 6)
        c.step(2, commands=[Command(0, K.POSITION, (0, .5, .5, 0, 0, 0))], motor_feedback=samples(2))
        self.assertFalse(arm.control.joint_velocity)

    def test_mode_switch_discards_previous_target(self):
        c = self.make()
        arm = self.normal(c)
        c.step(0, commands=[Command(0, K.START), Command(0, K.POSITION, (30,) * 6)], motor_feedback=samples(0))
        c.step(2, commands=[Command(0, K.DAMPING)], motor_feedback=samples(2))
        c.step(4, commands=[Command(0, K.START)], motor_feedback=samples(4))
        self.assertIsNone(arm.control)
        self.assertAlmostEqual(arm.command.angle[0], 0)

    def test_phase_rates_and_dm_keepalive(self):
        c = self.make()
        counts = [0] * 7
        enables = 0
        for t in range(80):
            result = c.step(t, motor_feedback=samples(t))
            for request in result.motor_requests:
                if request.action == A.ENABLE:
                    enables += 1
                else:
                    counts[request.motor_index] += 1
        self.assertEqual(counts, [40, 20, 20, 20, 20, 20, 20])
        self.assertEqual(enables, 1)

    def test_gravity_mode_keeps_gripper_gains(self):
        c = ControlSystem([ArmConfig()])
        self.normal(c)
        requests = []
        for t in range(4):
            result = c.step(t, commands=[Command(0, K.GRAVITY)] if t == 0 else (),
                            motor_feedback=samples(t, positions=(0, 30, 45, 0, 0, 0, 0)))
            requests.extend(result.motor_requests)
        controls = {r.motor_index: r for r in requests if r.action == A.CONTROL}
        self.assertEqual(set(controls), set(range(7)))
        self.assertTrue(all(controls[i].kp == controls[i].kd == 0 for i in range(6)))
        self.assertEqual((controls[6].kp, controls[6].kd), (8, .1))
        self.assertNotEqual(controls[1].torque, 0)

    def test_encos_requires_both_fresh_fields_and_healthy_status(self):
        bank = MotorBank(0)
        m = bank.motors[0]
        m.mode = ArmMode.RUN
        m.accept(MotorFeedback(0, 0, 0, position=1))
        self.assertEqual(m.control(0).action, A.SOFT_STOP)
        m.accept(MotorFeedback(0, 0, 1, velocity=0))
        m.mode = ArmMode.RUN
        self.assertEqual(m.control(1).action, A.CONTROL)
        m.accept(MotorFeedback(0, 0, 100, velocity=0))
        m.mode = ArmMode.RUN
        self.assertEqual(m.control(100).action, A.SOFT_STOP)

    def test_no_stale_timestamp_refresh_or_partial_input_application(self):
        c = self.make()
        c.step(10, motor_feedback=samples(10))
        c.step(11, motor_feedback=samples(5))
        self.assertEqual(c.stale_feedback_count, 7)
        self.assertEqual(c.arms[0].bank.motors[0].last_rx_ms, 10)
        with self.assertRaises(ValueError):
            c.step(12, commands=[Command(0, K.CONNECT), Command(9, K.CONNECT)])
        self.assertEqual(c.last_ms, 11)
        self.assertFalse(c.arms[0].connected)

    def test_shutdown_is_terminal_and_stops_all_motors(self):
        c = self.make()
        c.step(0, motor_feedback=samples(0))
        result = c.shutdown()
        self.assertEqual(sum(r.action == A.SOFT_STOP for r in result.motor_requests), 7)
        self.assertEqual(sum(r.action == A.DISABLE for r in result.motor_requests), 4)
        with self.assertRaises(RuntimeError):
            c.step(1)
        self.assertFalse(c.shutdown().motor_requests)

    def test_full_two_arm_initialization_and_reset(self):
        c, plant = self.make(2), SimulatedMotors(2)
        stages = [0, 0]
        done = [False, False]
        for t in range(14000):
            commands = []
            for i, arm in enumerate(c.arms):
                if t == 0:
                    commands.append(Command(i, K.CONNECT))
                elif t % 500 == 0:
                    commands.append(Command(i, K.HEARTBEAT))
                if t % 2 == 0:
                    if stages[i] == 0 and arm.state == S.WAIT_INIT:
                        commands.append(Command(i, K.INITIALIZE))
                        stages[i] = 1
                    elif stages[i] == 1 and arm.state == S.NORMAL:
                        self.assertTrue(arm.initialized[6])
                        commands.append(Command(i, K.RESET))
                        stages[i] = 2
                    elif stages[i] == 2 and arm.state == S.NORMAL:
                        done[i] = True
            result = c.step(t, commands=commands, motor_feedback=plant.feedback(t))
            plant.apply(result.motor_requests)
            if all(done):
                break
        self.assertTrue(all(done), [(a.state, a.command.auto_step, a.command.gz) for a in c.arms])
        self.assertTrue(all(a.actual_mode == ArmMode.DAMPING for a in c.arms))


class CalibrationTests(unittest.TestCase):
    def test_all_joint_calibration_through_motor_interface(self):
        c = ControlSystem([ArmConfig(gravity_enabled=False)])
        plant = SimulatedMotors(1)
        sent = False
        completed = None
        save_indices = []
        for t in range(12000):
            commands = [Command(0, K.CONNECT)] if t == 0 else []
            if t and t % 500 == 0:
                commands.append(Command(0, K.HEARTBEAT))
            if not sent and c.arms[0].state == S.WAIT_INIT:
                commands.append(Command(0, K.CALIBRATE))
                sent = True
            result = c.step(t, commands=commands, motor_feedback=plant.feedback(t))
            plant.apply(result.motor_requests)
            save_indices.extend(r.motor_index for r in result.motor_requests if r.action == A.SAVE_ZERO)
            for event in result.events:
                if event.kind == "calibrated":
                    completed = event
            if completed:
                break
        self.assertIsNotNone(completed)
        self.assertEqual(completed.zero_state, ZeroState.DONE)
        self.assertEqual(set(save_indices), set(range(6)))
        self.assertEqual(completed.zero_results[6], ZeroResult.SKIP)
        self.assertTrue(all(r in (ZeroResult.OK, ZeroResult.UNCONFIRMED) for r in completed.zero_results[:6]))

    def test_encos_guard(self):
        bank = MotorBank(0)
        self.assertIsNotNone(bank.operation(0, A.SAVE_ZERO, 0))
        self.assertIsNone(bank.operation(1, A.SAVE_ZERO, 500))
        self.assertIsNotNone(bank.operation(1, A.SAVE_ZERO, 501))

    def test_dm_missing_feedback_times_out_and_disables_woken_motor(self):
        bank = MotorBank(0)
        z = ZeroCalibration(bank)
        z.request(1 << 6)
        requests = []
        for t in range(1200):
            requests.extend(z.tick(t))
        self.assertEqual(z.state, ZeroState.TIMEOUT)
        self.assertEqual(z.results[6], ZeroResult.TIMEOUT)
        self.assertTrue(any(r.action == A.ENABLE for r in requests))
        self.assertFalse(z.woken)
        self.assertEqual(requests[-1].action, A.DISABLE)

    def test_comm_timeout_cancels_initialization(self):
        c = ControlSystem([ArmConfig(gravity_enabled=False)])
        c.step(0, commands=[Command(0, K.CONNECT)], motor_feedback=samples(0))
        c.step(2, commands=[Command(0, K.INITIALIZE)], motor_feedback=samples(2))
        c.step(3002, motor_feedback=samples(3002))
        self.assertEqual(c.arms[0].state, S.COMM_LOST)
        self.assertIsNone(c.arms[0].command.auto)
        self.assertFalse(c.arms[0].zero.pending)
        c.step(3004, commands=[Command(0, K.HEARTBEAT)], motor_feedback=samples(3004))
        self.assertEqual(c.arms[0].state, S.WAIT_INIT)


if __name__ == "__main__":
    unittest.main()
