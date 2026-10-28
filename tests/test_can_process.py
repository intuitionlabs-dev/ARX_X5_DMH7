from collections import deque
from functools import partial
from pathlib import Path
import tempfile
import time
import unittest
import os
import subprocess
import sys

from arx_x5_control import ArmConfig, CANChannel, CANIOError, ProcessCANMotorIO, MotorAction, MotorRequest
from arx_x5_control.can_protocol import CANFrame
from arx_x5_control.sdk_adapter.link import LocalLink


class ProcessBus:
    def __init__(self, path, config):
        self.path = path
        self.rows = deque()

    def send(self, frame):
        with open(self.path, "a") as output:
            output.write(f"{frame.arbitration_id} {frame.data.hex()}\n")
        if frame.arbitration_id == 0x7ff:
            return
        if frame.arbitration_id in (1, 2, 4):
            reply = CANFrame(frame.arbitration_id, bytes.fromhex("207FFF7FF7FF6464"))
        else:
            reply = CANFrame(0, bytes((frame.arbitration_id,))+bytes.fromhex("80008008001E1E"))
        self.rows.append((reply, time.monotonic_ns()))

    def receive(self):
        rows = tuple(self.rows)
        self.rows.clear()
        return rows

    def close(self):
        with open(self.path, "a") as output:
            output.write("closed\n")


def failing_factory(config):
    raise OSError("process device unavailable")


class ProcessTests(unittest.TestCase):
    def test_performance_windows_include_final_shutdown_transmissions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/"frames")
            io = ProcessCANMotorIO([CANChannel(0, "fake")], transport_factory=partial(ProcessBus, path),
                                   collect_performance=True)
            with io, LocalLink(io, configurations=(ArmConfig(),), collect_performance=True) as link:
                deadline = time.monotonic()+1.2
                while time.monotonic() < deadline:
                    link.check_health()
                    time.sleep(.01)
            windows = io.pop_performance()
            self.assertGreaterEqual(len(windows), 2)
            counts = sum(w["counters"].get("arm0.tx_frames", 0) for w in windows)
            stats = io.stats()
            self.assertEqual(counts, stats["tx_frames"]+stats["shutdown_tx_frames"])
            self.assertEqual(sum(w["counters"].get("arm0.shutdown_tx_frames", 0) for w in windows), 7)
            self.assertEqual(stats["performance_dropped"], 0)
            self.assertNotEqual(stats["worker_pid"], os.getpid())
            self.assertTrue(any("arm0.motor1.rx_period" in w["timings"] for w in windows))
            self.assertTrue(any("can_worker.period" in w["timings"] for w in windows))
            self.assertGreater(link.performance_snapshot()["timings"]["control.duration"]["count"], 50)

    def test_sdk_feedback_clock_and_process_shutdown(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/"frames")
            io = ProcessCANMotorIO([CANChannel(0, "fake")], transport_factory=partial(ProcessBus, path))
            with io:
                with LocalLink(io, configurations=(ArmConfig(),)) as link:
                    arm = link.arm(0).connect()
                    deadline = time.monotonic()+.15
                    while time.monotonic() < deadline:
                        arm.keepalive()
                        link.check_health()
                        time.sleep(.005)
                    snapshot = arm.snapshot()
                    self.assertEqual(snapshot.motor_lost_mask, 0)
                    self.assertLess(snapshot.age, .05)
                    self.assertTrue(io.stats()["process_alive"])
            rows = Path(path).read_text().splitlines()
            self.assertEqual(rows[-1], "closed")
            self.assertIn("8 fffffffffffffffd", rows[-8:])
            self.assertEqual(io.stats()["shutdown_tx_frames"], 7)
            self.assertFalse(io.stats()["process_alive"])

    def test_parent_heartbeat_loss_stops_motors(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/"frames")
            io = ProcessCANMotorIO([CANChannel(0, "fake")], transport_factory=partial(ProcessBus, path))
            io.open()
            io.poll_feedback(0)
            io.write_motor_requests((MotorRequest(0, 3, 5, "dm", MotorAction.CONTROL, 0),))
            time.sleep(.25)
            with self.assertRaisesRegex(CANIOError, "timed out"):
                io.check_health()
            with self.assertRaises(CANIOError):
                io.close()
            self.assertIn("5 fffffffffffffffd", Path(path).read_text())
            self.assertFalse(io.stats()["process_alive"])

    def test_device_open_failure_is_reported(self):
        io = ProcessCANMotorIO([CANChannel(0, "fake")], transport_factory=failing_factory)
        with self.assertRaisesRegex(CANIOError, "process device unavailable"):
            io.open()
        self.assertFalse(io.stats()["process_alive"])

    def test_parent_exit_allows_watchdog_shutdown(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/"frames")
            script = """
from functools import partial
from test_can_process import ProcessBus
from arx_x5_control import CANChannel, ProcessCANMotorIO, MotorRequest, MotorAction
io = ProcessCANMotorIO([CANChannel(0, 'fake')], transport_factory=partial(ProcessBus, PATH))
io.open()
io.poll_feedback(0)
io.write_motor_requests((MotorRequest(0, 3, 5, 'dm', MotorAction.CONTROL, 0),))
""".replace("PATH", repr(path))
            environment = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
            subprocess.run([sys.executable, "-c", script], check=True, timeout=5,
                           env=environment, capture_output=True)
            rows = Path(path).read_text().splitlines()
            self.assertEqual(rows[-1], "closed")
            self.assertIn("5 fffffffffffffffd", rows[-8:])

    def test_flush_preserves_process_heartbeat(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/"frames")
            with ProcessCANMotorIO([CANChannel(0, "fake")], read_only=True,
                                  transport_factory=partial(ProcessBus, path)) as io:
                io.query_feedback()
                io.flush(timeout=6)
                io.check_health()
                self.assertEqual(io.stats()["query_tx_frames"], 13)


if __name__ == "__main__":
    unittest.main()
