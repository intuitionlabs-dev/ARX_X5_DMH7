import unittest

from arx_x5_control import (ArmConfig, ArmMode, Command, CommandKind as K, ControlSystem,
                            MotorAction as A, MotorFeedback, RobotState as S, ZeroResult, ZeroState)
from arx_x5_control.calibration import ZeroCalibration
from arx_x5_control.motor import MotorBank
from arx_x5_control.runtime import run
from test_control import samples


class FailureTests(unittest.TestCase):
    def test_gripper_return_timeout_reports_failed_initialization(self):
        c = ControlSystem([ArmConfig(gravity_enabled=False)])
        initialized = False
        failed = False
        for now in range(10000):
            commands = [Command(0, K.CONNECT)] if now == 0 else []
            if now == 2:
                commands.append(Command(0, K.INITIALIZE))
            elif now and now % 500 == 0:
                commands.append(Command(0, K.HEARTBEAT))
            # Gripper remains fixed: it detects the stop, but cannot retreat.
            result = c.step(now, commands=commands, motor_feedback=samples(now))
            initialized |= any(e.kind == "initialized" for e in result.events)
            failed |= any(e.kind == "initialization_failed" for e in result.events)
            if failed:
                break
        self.assertFalse(initialized)
        self.assertTrue(failed)
        self.assertEqual(c.arms[0].state, S.WAIT_INIT)
        self.assertFalse(c.arms[0].initialized[6])

    def test_gripper_push_timeout_uses_elapsed_time(self):
        c = ControlSystem([ArmConfig(gravity_enabled=False)])
        arm = c.arms[0]
        arm.mode = ArmMode.RUN
        arm.command.start_auto("gripper_zero")
        arm.command.update(0)
        # Direct command-layer test: bypass the shorter host-command watchdog.
        arm.bank.motors[6].keep_alive(10001)
        arm.command.update(10001)
        self.assertTrue(arm.command.auto_failed)

    def test_encos_missing_ack_with_fresh_zero_position(self):
        bank = MotorBank(0)
        z = ZeroCalibration(bank)
        bank.motors[0].accept(MotorFeedback(0, 0, 0, position=.5, velocity=0))
        z.request(1)
        for now in range(1700):
            if now == 600:
                bank.motors[0].accept(MotorFeedback(0, 0, now, position=0, velocity=0))
            z.tick(now)
        self.assertEqual(z.state, ZeroState.DONE)
        self.assertEqual(z.results[0], ZeroResult.OK)
        self.assertEqual(z.ack_missing_mask, 1)

    def test_encos_success_ack_without_position_is_unconfirmed(self):
        bank = MotorBank(0)
        z = ZeroCalibration(bank)
        z.request(1)
        for now in range(2700):
            if now == 5:
                bank.motors[0].accept(MotorFeedback(0, 0, now, zero_ack=True))
            z.tick(now)
        self.assertEqual(z.state, ZeroState.DONE)
        self.assertEqual(z.results[0], ZeroResult.UNCONFIRMED)

    def test_encos_negative_ack_retries_then_fails(self):
        bank = MotorBank(0)
        z = ZeroCalibration(bank)
        z.request(1)
        reply_at = None
        for now in range(2700):
            if now == reply_at:
                bank.motors[0].accept(MotorFeedback(0, 0, now, zero_ack=False))
            for request in z.tick(now):
                if request.action == A.SAVE_ZERO:
                    reply_at = now + 1
        self.assertEqual(z.state, ZeroState.TIMEOUT)
        self.assertEqual(z.results[0], ZeroResult.FAIL)
        self.assertGreater(z.retries[0], 0)

    def test_disable_confirmation_failure_never_writes_zero(self):
        bank = MotorBank(0)
        z = ZeroCalibration(bank)
        z.request(1 << 3)
        output = []
        for now in range(1600):
            bank.motors[3].accept(MotorFeedback(0, 3, now, position=.5, velocity=0, error_code=1))
            output.extend(z.tick(now))
        self.assertEqual(z.state, ZeroState.TIMEOUT)
        self.assertFalse(any(r.action == A.SAVE_ZERO for r in output))

    def test_other_arm_continues_during_calibration(self):
        c = ControlSystem([ArmConfig(gravity_enabled=False)] * 2)
        c.step(0, commands=[Command(i, K.CONNECT) for i in range(2)], motor_feedback=samples(0, 2))
        c.step(2, commands=[Command(0, K.CALIBRATE)], motor_feedback=samples(2, 2))
        c.step(3, motor_feedback=samples(3, 2))  # calibration emits initial disable requests
        result = c.step(4, motor_feedback=samples(4, 2))
        normal = [r for r in result.motor_requests if r.action in (A.CONTROL, A.SOFT_STOP)]
        self.assertTrue(any(r.arm_id == 1 for r in normal))
        self.assertFalse(any(r.arm_id == 0 for r in normal))

    def test_communication_queue_overflow_is_reported(self):
        c = ControlSystem([ArmConfig(gravity_enabled=False)])
        result = c.step(0, commands=[Command(0, K.CONNECT)] * 8)
        self.assertEqual(c.arms[0].rejected_count, 1)
        self.assertTrue(any("queue full" in e.detail for e in result.events))

    def test_missed_pc_ticks_do_not_starve_odd_phase_motors(self):
        c = ControlSystem([ArmConfig(gravity_enabled=False)])
        indices = set()
        for now in range(0, 12, 2):
            result = c.step(now, motor_feedback=samples(now))
            indices.update(r.motor_index for r in result.motor_requests if r.action == A.SOFT_STOP)
        self.assertEqual(indices, set(range(7)))
        self.assertGreater(c.missed_motor_ticks, 0)

    def test_realtime_runner_stops_on_io_failure(self):
        class FailingIO:
            def poll_commands(self, now):
                return ()

            def poll_feedback(self, now):
                return ()

            def write_motor_requests(self, requests):
                self.last_requests = requests

            def publish_feedback(self, feedback):
                raise OSError("test adapter unavailable")

            def publish_events(self, events):
                pass

        c = ControlSystem([ArmConfig(gravity_enabled=False)])
        io = FailingIO()
        with self.assertRaisesRegex(OSError, "adapter unavailable"):
            run(c, io, duration_s=.1)
        self.assertTrue(c.closed)
        self.assertEqual(sum(r.action == A.SOFT_STOP for r in io.last_requests), 7)


if __name__ == "__main__":
    unittest.main()
