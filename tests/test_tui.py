from contextlib import redirect_stderr
import io
import os
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from arx_x5_control import ArmMode, MotorAction, RobotState
from arx_x5_control.simulation import SimulatedMotors
from arx_x5_control.sdk_adapter.link import LocalLink
from arx_x5_control.sdk_adapter.tui import LocalConsoleSession, open_session, parse_options
from arx_x5_control.sdk_adapter.tui_app import LocalUi


class TUITests(unittest.TestCase):
    def test_module_entry_reports_terminal_error(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, "-m", "arx_x5_control.sdk_adapter.tui", "--simulate", "--lang", "en",
                                     "--log-dir", directory], capture_output=True, text=True, timeout=5,
                                    env=dict(os.environ, TERM="arx-no-such-terminal"))
        self.assertEqual(result.returncode, 1)
        self.assertIn("Console failed:", result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_options_require_explicit_backend_and_valid_rates(self):
        for arguments in ([], ["--channel", "can0", "--arm-id", "1"],
                          ["--channel", "can0", "--channel", "can1", "--channel", "can2"],
                          ["--simulate", "--channel", "can0"],
                          ["--simulate", "--ctrl-hz", "nan"],
                          ["--simulate", "--fb-hz", "0"]):
            with self.subTest(arguments=arguments), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_options(arguments)
        options = parse_options(["--channel", "can0", "--channel", "can1", "--interface", "socketcan"])
        self.assertFalse(options.fake)
        self.assertEqual(options.baud, 1000000)
        single = parse_options(["--channel", "can0", "--interface", "socketcan"])
        self.assertEqual(single.channel, ["can0"])
        self.assertEqual(single.arm_id, 0)

    def test_sdk_ui_keys_control_both_local_arms(self):
        options = parse_options(["--simulate", "--step", "0.01"])
        with LocalLink.simulated(background=False) as link:
            session = LocalConsoleSession(link, options)
            ui = LocalUi(None, session, options)
            origin = time.monotonic()
            now = 0
            with patch("time.monotonic") as clock:
                def advance(target):
                    nonlocal now
                    while now <= target:
                        clock.return_value = origin+now/1000
                        session.tick(clock.return_value)
                        link.step(now)
                        now += 1
                advance(20)
                self.assertTrue(all(a.robot_state == RobotState.WAIT_INIT for a in session.arms.values()))
                ui.on_key(ord("i"))
                ui.on_key(ord("a"))
                ui.on_key(ord("i"))
                advance(8100)
                self.assertTrue(all(a.robot_state == RobotState.NORMAL for a in session.arms.values()))
                self.assertTrue(all(a.feedback.motor_inited_mask == 127 for a in session.arms.values()))
                ui.on_key(ord("s"))
                advance(8120)
                ui.on_key(ord("1"))
                before = session.target[0]
                ui.on_key(ord("+"))
                advance(9120)
                self.assertAlmostEqual(session.arms[1].joint_angles[0], before+.01, delta=.001)
                self.assertEqual(session.arms[0].mode, ArmMode.DAMPING)
                ui.on_key(ord("o"))
                advance(11200)
                self.assertEqual(session.arms[1].mode, ArmMode.DAMPING)
                self.assertEqual(link.event_overruns, 0)
                ui.on_key(ord("q"))
                self.assertTrue(ui.quit)
        self.assertTrue(link.controller.closed)

    def test_can_session_closes_controller_and_io_on_ui_failure(self):
        class TestIO(SimulatedMotors):
            def __init__(self):
                super().__init__()
                self.closed = False
                self.writes = []
            def __enter__(self):
                return self
            def __exit__(self, *args):
                self.closed = True
            def write_motor_requests(self, requests):
                self.writes.append(tuple(requests))
                super().write_motor_requests(requests)
        motor_io = TestIO()
        options = parse_options(["--channel", "can0", "--channel", "can1", "--interface", "socketcan"])
        with patch("arx_x5_control.sdk_adapter.tui.ProcessCANMotorIO", return_value=motor_io) as factory:
            with self.assertRaisesRegex(RuntimeError, "screen failed"):
                with open_session(options) as session:
                    session.tick(time.monotonic())
                    raise RuntimeError("screen failed")
            self.assertEqual([c.channel for c in factory.call_args.args[0]], ["can0", "can1"])
        self.assertTrue(motor_io.closed)
        self.assertTrue(session.link.controller.closed)
        self.assertEqual(sum(r.action == MotorAction.SOFT_STOP for r in motor_io.writes[-1]), 14)
        self.assertEqual(sum(r.action == MotorAction.DISABLE for r in motor_io.writes[-1]), 8)


if __name__ == "__main__":
    unittest.main()
