"""Exercise unmodified SDK APIs against the actual Python control core."""

from math import degrees, radians
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import sdk
from sdk import protocol as P

from arx_x5_control import ArmConfig, ArmMode as M, MotorAction, RobotState as S, ZeroResult, ZeroState
from arx_x5_control.sdk_link import LocalLink
from arx_x5_control.simulation import SimulatedMotors


class RecordingMotors(SimulatedMotors):
    def __init__(self, arm_count=2):
        super().__init__(arm_count)
        self.writes = []
        self.thread_ids = set()
        self.fail_poll = self.fail_write = False
        self.lost = set()

    def poll_feedback(self, now_ms):
        self.thread_ids.add(threading.get_ident())
        if self.fail_poll:
            raise OSError("feedback adapter failed")
        return tuple(f for f in self.feedback(now_ms) if (f.arm_id, f.motor_index) not in self.lost)

    def write_motor_requests(self, requests):
        self.thread_ids.add(threading.get_ident())
        self.writes.append(requests)
        if self.fail_write:
            raise OSError("motor output failed")
        self.apply(requests)


class SDKLinkTests(unittest.TestCase):
    def make(self, count=2, *, background=False, io=None):
        io = io or RecordingMotors(count)
        link = LocalLink(io, configurations=[ArmConfig(gravity_enabled=False)] * count,
                         background=background)
        self.addCleanup(link.close)
        link.open()
        return link

    def connect_manual(self, link):
        arms = tuple(link.arm(i) for i in range(len(link.configurations)))
        for arm in arms:
            self.assertEqual(arm.send_version(), 12)
        link.step(0)
        return arms

    def advance(self, link, end, *, keepalive=True):
        for now in range((link.controller.last_ms or 0) + 1, end + 1):
            if keepalive and now % 500 == 0:
                for arm in link.arms():
                    arm.keepalive()
            link.step(now)

    def test_background_connect_wait_and_shutdown_with_no_serial(self):
        with patch.object(sdk.Link, "_open_serial", side_effect=AssertionError("serial accessed")):
            link = self.make(background=True)
            a, b = link.arm(0), sdk.Arm(1, link)
            self.assertIs(a.connect(timeout=1), a)
            b.connect(timeout=1)
            self.assertTrue(a.wait_for(lambda arm: arm.robot_state == S.WAIT_INIT, timeout=1))
            self.assertIsNotNone(a.wait_feedback(timeout=1))
            self.assertEqual(b.robot_state, S.WAIT_INIT)
            self.assertEqual(link.board_version, sdk.VERSION_CODE)
            self.assertIs(link.arm(0), a)
            a.close()  # SDK Arm does not own the shared link.
            self.assertTrue(link.is_open)
            link.close()
            self.assertTrue(link.controller.closed)
            self.assertFalse(link.link_up)
            self.assertIsNone(a.feedback)
            stops = link.motor_io.writes[-1]
            self.assertEqual(sum(r.action == MotorAction.SOFT_STOP for r in stops), 14)
            self.assertEqual(sum(r.action == MotorAction.DISABLE for r in stops), 8)
            self.assertEqual(len(link.motor_io.thread_ids), 1)
            with self.assertRaises(sdk.NotConnected):
                a.keepalive()

    def test_full_sdk_sequence_units_calibration_modes_reset_watchdog(self):
        link = self.make()
        a, b = self.connect_manual(link)
        for arm in (a, b):
            self.assertEqual(arm.robot_state, S.WAIT_INIT)
            arm.init_arm()
        self.advance(link, 8000)
        self.assertEqual((a.robot_state, b.robot_state), (S.NORMAL, S.NORMAL))
        self.assertEqual((a.mode, b.mode), (M.DAMPING, M.DAMPING))
        self.assertEqual(a.feedback.motor_inited_mask, 0x7f)
        with self.assertRaises(sdk.GateRejected):
            a.send_ctrl_pos((0,) * 6, gripper_raw_deg=0)
        a.start()
        self.advance(link, 8004)
        angles = (0.2, 0.3, 0.4, -0.1, 0.15, -0.2)
        velocities = (0.01, 0.02, 0.03, -0.01, 0.02, -0.03)
        self.assertEqual(a.send_ctrl_pos_vel(angles, velocities, gripper_raw_deg=-25), 60)
        self.advance(link, 8008)
        accepted = link.controller.arms[0].control
        for actual, want in zip(accepted.joint_angle, map(degrees, angles)):
            self.assertAlmostEqual(actual, want, places=5)
        for actual, want in zip(accepted.joint_velocity, map(degrees, velocities)):
            self.assertAlmostEqual(actual, want, places=5)
        self.assertEqual(accepted.gripper_angle, -25)
        self.assertIsNone(link.controller.arms[1].control)
        self.assertEqual(b.mode, M.DAMPING)
        self.advance(link, 8250)
        self.assertGreater(a.joint_angles[0], 0)
        for angle, degree in zip(a.joint_angles, a.feedback.joint_angle):
            self.assertAlmostEqual(angle, radians(degree))
        with tempfile.TemporaryDirectory() as tmp:
            store = sdk.GripperCalibStore(Path(tmp) / "calibration.json")
            store.put(0, sdk.GripperCalibration.from_points(0.08, -100, 100))
            calibrated = sdk.Arm(0, link, calib_store=store)
            calibrated.send_ctrl_pos(angles, gripper_m=0.02)
            self.advance(link, 8254)
            self.assertAlmostEqual(link.controller.arms[0].control.gripper_angle, -50)
            self.assertIsNotNone(calibrated.snapshot().gripper_opening)
        for method, mode in ((a.gravity, M.GRAVITY), (a.stop, M.STOP), (a.damping, M.DAMPING)):
            method()
            self.advance(link, link.controller.last_ms + 4)
            self.assertEqual(a.mode, mode)
            self.assertIsNone(link.controller.arms[0].control)
        a.reset()
        self.advance(link, 8300)
        self.assertEqual(a.robot_state, S.RESETTING)
        self.assertEqual(b.robot_state, S.NORMAL)
        self.advance(link, 11000)
        self.assertEqual((a.robot_state, a.mode), (S.NORMAL, M.DAMPING))
        a.start()
        b.start()
        self.advance(link, 11004)
        a.send_ctrl_pos(a.joint_angles, gripper_raw_deg=0)
        self.advance(link, 11008)
        self.advance(link, 14000, keepalive=False)
        b.keepalive()
        self.advance(link, 14012, keepalive=False)
        self.assertEqual(a.mode, M.DAMPING)
        self.assertEqual(b.mode, M.RUN)

    def test_set_zero_returns_real_result_and_isolated_ack(self):
        link = self.make()
        link.motor_io.positions[0][:3] = [0.3] * 3
        a, b = self.connect_manual(link)
        a.set_zero()
        self.advance(link, 6000)
        ack = a.wait_setzero_ack(timeout=0)
        self.assertIsNotNone(ack)
        self.assertEqual(ack.state, ZeroState.DONE)
        self.assertEqual(ack.result[:3], (ZeroResult.OK,) * 3)
        self.assertEqual(ack.result[3:6], (ZeroResult.UNCONFIRMED,) * 3)
        self.assertEqual(ack.result[6], ZeroResult.SKIP)
        self.assertGreater(ack.elapsed_ms, 0)
        self.assertIsNone(b.last_setzero_ack)
        a.set_zero()
        with self.assertRaises(TimeoutError):
            a.wait_setzero_ack(timeout=0)
        self.advance(link, 12000)
        self.assertIsNotNone(a.wait_setzero_ack(timeout=0))

    def test_offline_mask_and_feedback_fields(self):
        link = self.make()
        a, b = self.connect_manual(link)
        link.motor_io.lost.add((0, 4))
        self.advance(link, 40)
        self.assertEqual(a.feedback.motor_lost_mask, 1 << 4)
        self.assertEqual(b.feedback.motor_lost_mask, 0)
        self.assertEqual(a.temperatures, (25,) * 7)
        self.assertLess(a.feedback_age, 0.1)
        with self.assertRaises(sdk.GateRejected):
            a.init_arm()

    def test_invalid_payloads_and_versions_do_not_queue_or_refresh(self):
        link = self.make(1)
        a, = self.connect_manual(link)
        invalid = [(P.TID_VERSION, struct.pack("<I", 0)), (P.TID_VERSION, b""),
                   (P.TID_ARM_START, b"x"), (P.TID_ARM_CTRL_POS, b""),
                   (P.TID_ARM_CTRL_POS, struct.pack("<7f", float("nan"), *([0] * 6))),
                   (P.TID_ARM_CTRL_POS_VEL, struct.pack("<13f", *([float("inf")] * 13))),
                   (P.TID_ARM_FEEDBACK, b""), (255, b"")]
        before = link.n_tx
        for tid, payload in invalid:
            with self.subTest(tid=tid, payload=payload), self.assertRaises(sdk.ProtocolError):
                a.send_raw(tid, payload)
        self.assertEqual(link.n_tx, before)
        self.assertEqual(link.stats["pending_commands"], (0,))
        self.advance(link, 3004, keepalive=False)
        self.assertEqual(a.robot_state, S.COMM_LOST)
        a.keepalive()
        self.advance(link, 3008)
        self.assertEqual(a.robot_state, S.WAIT_INIT)
        with self.assertRaises(sdk.ProtocolError):
            link.inject_bytes(P.encode(P.TID_VERSION, P.encode_version()))

    def test_queue_backpressure_ordering_and_core_gates(self):
        link = self.make()
        a, b = self.connect_manual(link)
        for _ in range(7):
            a.keepalive()
        with self.assertRaisesRegex(sdk.LinkError, "queue full"):
            a.keepalive()
        b.keepalive()
        link.step(1)
        self.assertEqual(link.stats["pending_commands"], (7, 1))
        link.step(2)
        self.assertEqual(link.stats["pending_commands"], (0, 0))
        a.start(force=True)
        self.advance(link, 4)
        self.assertEqual(a.mode, M.STOP)
        self.assertTrue(any(e.kind == "rejected" for e in link.drain_events()))
        self.assertEqual(link.stats["rejected_commands"], (1, 0))

    def test_reopen_clears_feedback_ack_and_pending_commands(self):
        link = self.make(1)
        a, = self.connect_manual(link)
        channel = link.channel(0)
        channel.put_ack(P.SetZeroAck(5, (2,) * 7, 2000), time.monotonic())
        before_seq = channel.feedback_seq
        a.init_arm()
        link.reopen()
        self.assertIs(link.arm(0), a)
        self.assertEqual(link.open_gen, 2)
        self.assertIsNone(a.feedback)
        self.assertIsNone(a.last_setzero_ack)
        link.step(1)
        self.assertEqual(link.controller.arms[0].state, S.WAIT_HANDSHAKE)
        self.assertIsNone(a.feedback)
        a.keepalive()
        self.advance(link, 4)
        self.assertEqual(a.robot_state, S.WAIT_INIT)
        self.assertGreater(channel.feedback_seq, before_seq)
        with self.assertRaises(ValueError):
            link.step(3)
        link.check_health()

    def test_worker_failure_surfaces_and_emits_shutdown(self):
        io = RecordingMotors(1)
        io.fail_poll = True
        link = self.make(1, io=io, background=True)
        link._thr.join(timeout=1)
        self.assertFalse(link.is_open)
        with self.assertRaisesRegex(sdk.LinkError, "feedback adapter failed"):
            link.check_health()
        self.assertTrue(link.controller.closed)
        self.assertTrue(any(r.action == MotorAction.DISABLE for r in io.writes[-1]))
        io.fail_poll = False
        link.reopen()
        link.arm(0).connect(timeout=1)

    def test_output_failure_also_reports_shutdown_failure(self):
        io = RecordingMotors(1)
        link = LocalLink(io, configurations=[ArmConfig(gravity_enabled=False)], background=False)
        link.open()
        io.fail_write = True
        with self.assertRaisesRegex(sdk.LinkError, "motor output failed"):
            link.step(0)
        self.assertTrue(link.controller.closed)
        with self.assertRaisesRegex(sdk.LinkError, "shutdown delivery failed"):
            link.close()
        self.assertFalse(link.link_up)
        self.assertTrue(any(r.action == MotorAction.SOFT_STOP for r in io.writes[-1]))

    def test_single_arm_and_manual_lifecycle_validation(self):
        link = self.make(1)
        for arm_id in (-1, 1, True, 0.0):
            with self.subTest(arm_id=arm_id), self.assertRaises(ValueError):
                link.channel(arm_id)
        with self.assertRaises(sdk.LinkError):
            link.open("/dev/ttyUSB0")
        with self.assertRaises(TypeError):
            LocalLink(object())
        with self.assertRaises(ValueError):
            LocalLink.simulated(configurations=[])
        background = self.make(1, background=True)
        with self.assertRaises(RuntimeError):
            background.step(0)


if __name__ == "__main__":
    unittest.main()
