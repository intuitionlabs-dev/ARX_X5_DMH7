from collections import deque
import ctypes as C
import math
import os
from pathlib import Path
import random
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from arx_x5_control import (ArmConfig, ArmMode, CANChannel, CANControlIO, CANIOError, CANMotorIO,
                            Command, CommandKind, ControlSystem, MotorAction as A, MotorRequest, RobotState)
from arx_x5_control.can_protocol import CANFrame, decode_feedback, encode_request, feedback_queries
from arx_x5_control.usb2can import CANableTransport, SocketCANTransport
from arx_x5_control.config import MOTOR_IDS
from arx_x5_control.runtime import run
from arx_x5_control.sdk_adapter.link import LocalLink
from sdk import Arm as SDKArm

ROOT = Path(__file__).resolve().parents[1]


def request(arm=0, index=0, action=A.SOFT_STOP, **values):
    return MotorRequest(arm, index, MOTOR_IDS[index], "encos" if index < 3 else "dm", action, 0, **values)


def wait_for(predicate, timeout=2):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.001)
    raise AssertionError("condition did not become true")


class FakeBus:
    def __init__(self):
        self.sent = []
        self.received = deque()
        self.closed = False
        self.gate = None
        self.failure = None
        self.answer = False

    def send(self, frame):
        if self.gate is not None and not self.gate.wait(1):
            raise OSError("test send timeout")
        if self.failure:
            raise self.failure
        self.sent.append((frame, time.monotonic_ns()))
        if self.answer:
            if frame.arbitration_id in (1, 2, 4):
                self.inject(CANFrame(frame.arbitration_id, bytes.fromhex("207FFF7FF7FF6464")))
            elif frame.arbitration_id in (5, 6, 7, 8):
                self.inject(CANFrame(0, bytes((frame.arbitration_id,))+bytes.fromhex("80008008001E1E")))

    def inject(self, frame, acquired_ns=None):
        self.received.append((frame, time.monotonic_ns() if acquired_ns is None else acquired_ns))

    def receive(self):
        if self.failure:
            raise self.failure
        rows = []
        while self.received:
            rows.append(self.received.popleft())
        return rows

    def close(self):
        self.closed = True


class CodecTests(unittest.TestCase):
    def test_special_commands_and_soft_stop_vectors(self):
        self.assertEqual(encode_request(request()).data, bytes.fromhex("0000007FFF7FF7FF"))
        self.assertEqual(encode_request(request(index=3)).data, bytes.fromhex("7FFF7FF0000007FF"))
        self.assertEqual(encode_request(request(action=A.SAVE_ZERO)), CANFrame(0x7FF, b"\x00\x01\x00\x03"))
        for action, byte in ((A.ENABLE, 0xFC), (A.DISABLE, 0xFD), (A.SAVE_ZERO, 0xFE)):
            self.assertEqual(encode_request(request(index=6, action=action)), CANFrame(8, b"\xff"*7+bytes((byte,))))
        with self.assertRaises(ValueError):
            encode_request(request(action=A.ENABLE))

    def test_query_partial_feedback_ack_and_invalid_frames(self):
        for code, field, value in ((1, "position", math.pi/2), (2, "velocity", 3*math.pi), (3, "current", 90)):
            frame = CANFrame(1, bytes((0xA0, code))+struct.pack(">f", 90))
            feedback = decode_feedback(1, frame, 20)
            self.assertAlmostEqual(getattr(feedback, field), value)
            self.assertEqual(feedback.arm_id, 1)
            self.assertEqual(feedback.timestamp_ms, 20)
            for name in ("position", "velocity", "current"):
                if name != field:
                    self.assertIsNone(getattr(feedback, name))
        self.assertTrue(decode_feedback(0, CANFrame(0x7FF, b"\x00\x04\x01\x03"), 0).zero_ack)
        self.assertFalse(decode_feedback(0, CANFrame(0x7FF, b"\x00\x02\x01\x00"), 0).zero_ack)
        for frame in (CANFrame(1, b"\x00"*8), CANFrame(1, b"\xa0\x01"+struct.pack(">f", math.nan)),
                      CANFrame(0, b"\x03"*8), CANFrame(0x7FF, b"\x00\x01\x00\x03")):
            self.assertIsNone(decode_feedback(0, frame, 0))

    def test_dm_feedback_scaling_and_enabled_status(self):
        feedback = decode_feedback(1, CANFrame(0, bytes.fromhex("1580008008001E1F")), 100)
        self.assertEqual((feedback.motor_index, feedback.error_code), (3, 1))
        self.assertEqual((feedback.position, feedback.velocity, feedback.torque), (0, 0, 0))
        self.assertEqual((feedback.mos_temperature, feedback.temperature), (30, 31))
        self.assertIsNone(feedback.current)

    def test_validation_clipping_and_query_whitelist(self):
        for kwargs in ({"position": math.nan}, {"velocity": math.inf}, {"kp": -1}):
            with self.assertRaises(ValueError):
                encode_request(request(action=A.CONTROL, **kwargs))
        maximum = encode_request(request(action=A.CONTROL, position=100, velocity=100, torque=100, kp=1000, kd=100))
        self.assertEqual(maximum.data, bytes.fromhex("1FFFFFFFFFFFFFFF"))
        self.assertEqual(feedback_queries(0), tuple(CANFrame(1, bytes((0xE0, i))) for i in (1, 2, 3)))
        self.assertEqual(feedback_queries(3), (CANFrame(0x7FF, b"\x05\x00\xcc\x00"),))


@unittest.skipUnless(shutil.which("cc"), "requires host C compiler")
class CANParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="arx-can-codec-")
        cls.addClassCleanup(cls.tmp.cleanup)
        target = Path(cls.tmp.name)/"can.so"
        subprocess.run(["cc", "-std=c99", "-shared", "-fPIC", "-O2",
                        "-I"+str(ROOT/"libs/encos_motor"), "-I"+str(ROOT/"libs/common"),
                        str(ROOT/"tests/c_can_reference.c"), str(ROOT/"libs/encos_motor/encos_protocol.c"),
                        str(ROOT/"libs/common/common.c"), "-lm", "-o", str(target)], check=True, capture_output=True)
        cls.lib = C.CDLL(str(target))
        for name in ("reference_encos", "reference_dm"):
            getattr(cls.lib, name).argtypes = [C.c_float]*5+[C.POINTER(C.c_uint8)]
        cls.lib.reference_encos_decode.argtypes = [C.POINTER(C.c_uint8), C.c_int, C.POINTER(C.c_float)]

    def test_control_encoding_against_firmware_c(self):
        rng = random.Random(42)
        for index in (0, 3):
            for _ in range(1500):
                limits = (12.5, 18, 30, 500, 50) if index == 0 else (12.5, 45, 10, 500, 5)
                values = [C.c_float(rng.uniform(-limit if i < 3 else 0, limit)).value
                          for i, limit in enumerate(limits)]
                out = (C.c_uint8*8)()
                function = self.lib.reference_encos if index == 0 else self.lib.reference_dm
                function(*values, out)
                actual = encode_request(request(index=index, action=A.CONTROL,
                    **dict(zip(("position", "velocity", "torque", "kp", "kd"), values))))
                self.assertEqual(actual.data, bytes(out))

    def test_encos_decoding_against_firmware_c(self):
        rng = random.Random(43)
        for _ in range(600):
            data = bytes((0x20+rng.randrange(8),))+rng.randbytes(7)
            raw, out = (C.c_uint8*8)(*data), (C.c_float*5)()
            self.assertGreater(self.lib.reference_encos_decode(raw, 8, out), 0)
            feedback = decode_feedback(0, CANFrame(1, data), 0)
            for value, expected in zip((feedback.position, feedback.velocity, feedback.current,
                                        feedback.temperature, feedback.mos_temperature), out):
                self.assertAlmostEqual(value, expected, delta=2e-6)


class MotorIOTests(unittest.TestCase):
    def make(self, count=2, **options):
        self.buses = [FakeBus() for _ in range(count)]
        io = CANMotorIO([CANChannel(i, f"test{i}") for i in range(count)],
                        transport_factory=lambda channel: self.buses[channel.arm_id], **options)
        io.open()
        self.addCleanup(self.close_healthy, io)
        return io

    @staticmethod
    def close_healthy(io):
        try:
            io.close()
        except CANIOError:
            if not io.stats()["failed"]:
                raise

    def test_nonblocking_tx_order_gap_and_dual_arm_routing(self):
        io = self.make()
        jobs = tuple(request(arm, i, A.DISABLE) for arm in range(2) for i in (3, 4, 5, 6))
        io.write_motor_requests(jobs)
        io.flush()
        for bus in self.buses:
            self.assertEqual([frame.arbitration_id for frame, _ in bus.sent], [5, 6, 7, 8])
            self.assertTrue(all(b-a >= 200_000 for (_, a), (_, b) in zip(bus.sent, bus.sent[1:])))
        io.close()
        self.assertTrue(all(bus.closed for bus in self.buses))

    def test_feedback_timestamps_partial_fields_and_arm_isolation(self):
        io = self.make()
        io.poll_feedback(100)
        at = time.monotonic_ns()
        for arm, bus in enumerate(self.buses):
            bus.inject(CANFrame(1, b"\xa0\x01"+struct.pack(">f", 10+arm)), at)
        wait_for(lambda: io.stats().get("feedback_samples", 0) == 2)
        time.sleep(.005)
        output = io.poll_feedback(110)
        self.assertEqual({v.arm_id for v in output}, {0, 1})
        self.assertTrue(all(v.timestamp_ms < 110 and v.velocity is None for v in output))
        self.assertFalse(io.poll_feedback(111))
        old = at-100_000_000
        self.buses[0].inject(CANFrame(1, b"\xa0\x01"+struct.pack(">f", 10)), old)
        wait_for(lambda: io.stats().get("stale_rx", 0) == 1)

    def test_read_only_blocks_motor_frames_and_queries_receive(self):
        io = self.make(count=1, read_only=True)
        with self.assertRaises(PermissionError):
            io.write_motor_requests((request(index=3, action=A.ENABLE),))
        self.assertFalse(self.buses[0].sent)
        io.query_feedback()
        wait_for(lambda: len(self.buses[0].sent) >= 5)
        frames = [frame for frame, _ in self.buses[0].sent]
        self.assertTrue(all(frame in sum((feedback_queries(i) for i in range(7)), ()) for frame in frames))
        io.close()
        self.assertEqual(io.stats().get("motor_tx_frames", 0), 0)
        self.assertEqual(io.stats().get("shutdown_tx_frames", 0), 0)

    def test_zero_guard_allows_other_arm_and_dm(self):
        io = self.make()
        io.write_motor_requests((request(action=A.SAVE_ZERO), request(index=1),
                                 request(index=3, action=A.DISABLE), request(arm=1)))
        wait_for(lambda: len(self.buses[0].sent) >= 2 and len(self.buses[1].sent) >= 1)
        self.assertEqual(len(self.buses[0].sent), 2)
        self.assertEqual(self.buses[0].sent[1][0].arbitration_id, 5)
        io.flush(timeout=1)
        first, third = self.buses[0].sent[0], self.buses[0].sent[2]
        self.assertGreaterEqual(third[1]-first[1], 501_000_000)

    def test_zero_ack_requires_matching_bus_pending_request_and_preserves_error(self):
        io = self.make()
        io.poll_feedback(0)
        ack = CANFrame(0x7FF, b"\x00\x01\x01\x03")
        self.buses[1].inject(ack)
        self.buses[0].inject(CANFrame(1, b"\xa4\x01"+struct.pack(">f", 10)))
        wait_for(lambda: io.stats().get("rx_frames", 0) == 2)
        io.write_motor_requests((request(action=A.SAVE_ZERO),))
        io.flush()
        self.buses[0].inject(ack)
        self.buses[1].inject(ack)
        wait_for(lambda: io.stats().get("rx_frames", 0) == 4)
        feedback = io.poll_feedback(100)
        acknowledgements = [item for item in feedback if item.zero_ack is not None]
        self.assertEqual(len(acknowledgements), 1)
        self.assertEqual((acknowledgements[0].arm_id, acknowledgements[0].error_code), (0, 4))
        self.assertTrue(acknowledgements[0].zero_ack)

    def test_shutdown_waits_zero_guard_and_disables_dm(self):
        io = self.make(count=1)
        io.write_motor_requests((request(action=A.SAVE_ZERO),))
        io.flush()
        zero_at = self.buses[0].sent[0][1]
        io.close()
        frames = self.buses[0].sent[1:]
        self.assertEqual([frame.data[-1] for frame, _ in frames[:4]], [0xFD]*4)
        self.assertTrue(all(at-zero_at >= 501_000_000 for _, at in frames[4:]))

    def test_atomic_validation_and_duplicate_bus_rejection(self):
        io = self.make()
        bad = MotorRequest(0, 0, 2, "encos", A.CONTROL, 0)
        with self.assertRaises(ValueError):
            io.write_motor_requests((request(), bad))
        self.assertEqual(io.stats()["pending_tx"], 0)
        self.assertFalse(self.buses[0].sent)
        with self.assertRaises(ValueError):
            CANMotorIO((CANChannel(0, "same"), CANChannel(1, "same")))

    def test_queue_backpressure_and_transport_fault_latch(self):
        io = self.make(count=1, tx_capacity=1)
        with self.assertRaises(CANIOError):
            io.write_motor_requests((request(index=3, action=A.DISABLE), request(index=4, action=A.DISABLE)))
        with self.assertRaises(CANIOError):
            io.poll_feedback(0)
        self.assertFalse(self.buses[0].sent)

    def test_transmit_deadline_discards_stale_control_and_stops(self):
        io = self.make(count=1, max_tx_age_ms=5)
        gate = threading.Event()
        self.buses[0].gate = gate
        io.write_motor_requests((request(index=3, action=A.DISABLE), request(action=A.CONTROL, kp=10)))
        time.sleep(.015)
        gate.set()
        wait_for(lambda: io.stats()["failed"])
        with self.assertRaises(CANIOError):
            io.close()
        self.assertNotIn(encode_request(request(action=A.CONTROL, kp=10)), [f for f, _ in self.buses[0].sent])
        self.assertGreater(io.stats().get("shutdown_tx_frames", 0), 0)

    def test_command_stream_timeout_stops_actuated_bus(self):
        io = self.make(command_timeout_ms=15)
        io.write_motor_requests((request(action=A.CONTROL),))
        io.flush()
        wait_for(lambda: io.stats()["failed"])
        with self.assertRaises(CANIOError):
            io.close()
        self.assertFalse(self.buses[1].sent)
        self.assertEqual(io.stats()["shutdown_tx_frames"], 7)

    def test_dm_keepalive_preserves_control_stream_watchdog(self):
        io = self.make(command_timeout_ms=15)
        io.write_motor_requests((request(index=3, action=A.CONTROL), request(index=3, action=A.ENABLE)))
        io.flush()
        wait_for(lambda: io.stats()["failed"])
        with self.assertRaises(CANIOError):
            io.close()
        self.assertEqual(io.stats()["shutdown_tx_frames"], 7)

    def test_receive_overflow_and_usb_disconnect_surface(self):
        io = self.make(count=1, rx_capacity=1)
        frame = CANFrame(1, b"\xa0\x01"+struct.pack(">f", 0))
        self.buses[0].inject(frame)
        self.buses[0].inject(frame)
        wait_for(lambda: io.stats()["failed"])
        with self.assertRaises(CANIOError):
            io.poll_feedback(1)
        other = self.make(count=1)
        self.buses[0].failure = OSError("USB disconnected")
        wait_for(lambda: other.stats()["failed"])
        with self.assertRaisesRegex(CANIOError, "USB disconnected"):
            other.write_motor_requests(())

    def test_local_link_and_runtime_controlio_integration(self):
        io = self.make()
        for bus in self.buses:
            bus.answer = True
        link = LocalLink(io, configurations=[ArmConfig(gravity_enabled=False)]*2, background=False)
        self.addCleanup(link.close)
        link.open()
        for now in range(15):
            link.step(now)
            time.sleep(.001)
        SDKArm(0, link).send_version()
        link.step(16)
        self.assertTrue(link.controller.arms[0].connected)
        self.assertTrue(all(m.last_rx_ms >= 0 for a in link.controller.arms for m in a.bank.motors))
        link.close()
        bus = FakeBus()
        events = []
        with CANControlIO([CANChannel(0, "test")], transport_factory=lambda _: bus,
                          event_sink=events.extend) as control_io:
            run(ControlSystem([ArmConfig(gravity_enabled=False)]), control_io, duration_s=.01)
        self.assertTrue(events)
        self.assertTrue(bus.closed)

    def test_continuous_control_over_two_independent_buses(self):
        io = self.make()
        for bus in self.buses:
            bus.answer = True
        system = ControlSystem([ArmConfig(gravity_enabled=False)]*2)
        for arm in system.arms:
            arm.connected = True
            arm.enter(RobotState.NORMAL, 0)
        origin = time.monotonic_ns()
        last = -1
        try:
            while time.monotonic_ns()-origin < 200_000_000:
                now = (time.monotonic_ns()-origin)//1_000_000
                if now > last:
                    commands = ([Command(arm, CommandKind.START) for arm in (0, 1)] if last < 0 else ())
                    result = system.step(now, commands=commands, motor_feedback=io.poll_feedback(now))
                    io.write_motor_requests(result.motor_requests)
                    last = now
                time.sleep(.0005)
            io.flush()
            self.assertTrue(all(arm.actual_mode == ArmMode.RUN for arm in system.arms))
            self.assertTrue(all(len(bus.sent) > 150 for bus in self.buses))
            self.assertFalse(io.stats()["failed"])
        finally:
            io.write_motor_requests(system.shutdown().motor_requests)

    def test_cli_help_and_validation_in_both_languages(self):
        for language, phrase in (("zh", "只读查询"), ("en", "Read X5")):
            result = subprocess.run([sys.executable, "-m", "arx_x5_control.can_check", "--lang", language, "--help"],
                                    capture_output=True, text=True, cwd=ROOT, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(phrase, result.stdout)
        result = subprocess.run([sys.executable, "-m", "arx_x5_control.can_check", "--channel", "test",
                                 "--seconds", "nan"], capture_output=True, text=True, cwd=ROOT, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertIn("有限正数", result.stderr)


class SerialFake:
    def __init__(self, *args, **kwargs):
        self.writes = []
        self.incoming = bytearray()
        self.closed = False
        self.short = False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def write(self, data):
        self.writes.append(data)
        return len(data)-1 if self.short else len(data)

    def read(self, size):
        data, self.incoming = self.incoming[:size], self.incoming[size:]
        return data

    def reset_input_buffer(self):
        self.incoming.clear()

    def close(self):
        self.closed = True


class TransportTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "posix", "requires POSIX pseudo terminals")
    def test_posix_canable_empty_read_data_and_disconnect(self):
        import pty
        import termios
        import tty
        master, slave = pty.openpty()
        tty.setraw(slave)
        attributes = termios.tcgetattr(slave)
        attributes[6][termios.VMIN] = attributes[6][termios.VTIME] = 0
        termios.tcsetattr(slave, termios.TCSANOW, attributes)
        os.set_blocking(slave, False)
        fake = SerialFake()
        fake.fileno = lambda: slave
        with patch("arx_x5_control.usb2can.slcan.time.sleep"):
            transport = CANableTransport("pty", serial_factory=lambda *a, **k: fake)
        try:
            transport.start()
            self.assertFalse(transport.receive())
            os.write(master, b"t0016A0013F800000\r")
            rows = transport.receive()
            self.assertEqual(rows[0][0], CANFrame(1, bytes.fromhex("A0013F800000")))
            os.close(master)
            master = None
            with self.assertRaisesRegex(OSError, "disconnected"):
                transport.receive()
        finally:
            if master is not None:
                master_close = master
                transport.close()
                os.close(master_close)
            os.close(slave)

    def serial_transport(self):
        fake = SerialFake()
        with patch("arx_x5_control.usb2can.slcan.time.sleep"):
            transport = CANableTransport("test", serial_factory=lambda *a, **k: fake)
            transport.start()
        return transport, fake

    def test_canable_config_chunked_rx_timestamp_and_exact_tx(self):
        transport, serial = self.serial_transport()
        self.assertEqual(serial.writes, [b"C\r", b"S8\r", b"M0\r", b"A0\r", b"O\r"])
        serial.incoming.extend(b"t0016A001")
        self.assertFalse(transport.receive())
        acquired = transport._partial_at
        serial.incoming.extend(b"3F800000\rt00081580008008001E1F\r")
        rows = transport.receive()
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0][1], acquired)
        transport.send(CANFrame(1, b"\xe0\x01"))
        self.assertEqual(serial.writes[-1], b"t0012E001\r")
        transport.close()
        self.assertTrue(serial.closed)

    def test_canable_malformed_and_short_write_fail(self):
        transport, serial = self.serial_transport()
        serial.incoming.extend(b"t0018FFFF\r")
        with self.assertRaises(OSError):
            transport.receive()
        serial.short = True
        with self.assertRaises(OSError):
            transport.send(CANFrame(1, b"\xe0\x01"))
        count = len(serial.writes)
        with self.assertRaises(OSError):
            transport.close()
        self.assertEqual(len(serial.writes), count)
        self.assertTrue(serial.closed)

    def test_socketcan_excludes_local_echo_and_uses_kernel_timestamp(self):
        payload = SocketCANTransport._frame.pack(1, 6, b"\xa0\x01"+struct.pack(">f", 10)+b"\0\0")
        stamp = time.time_ns()-5_000_000
        ancillary = [(socket.SOL_SOCKET, SocketCANTransport._timestampns,
                      struct.pack("@ll", stamp//1_000_000_000, stamp % 1_000_000_000))]
        class FakeSocket:
            def __init__(self):
                self.rows = deque(((payload, ancillary, socket.MSG_DONTROUTE, None),
                                   (payload, ancillary, 0, None)))
            def recvmsg(self, *args):
                if self.rows:
                    return self.rows.popleft()
                raise BlockingIOError()
        transport = SocketCANTransport.__new__(SocketCANTransport)
        transport._socket, transport._channel, transport._index = FakeSocket(), "vcan0", 1
        with patch("socket.if_nametoindex", return_value=1):
            rows = transport.receive()
        self.assertEqual(len(rows), 1)
        self.assertGreaterEqual(time.monotonic_ns()-rows[0][1], 5_000_000)
        with patch("socket.if_nametoindex", return_value=2), self.assertRaises(OSError):
            transport.receive()

    def test_socketcan_initialization_enables_bus_error_frames(self):
        with patch("socket.if_nametoindex", return_value=1), patch("socket.socket") as factory:
            transport = SocketCANTransport("vcan0")
            factory.return_value.setsockopt.assert_any_call(
                socket.SOL_CAN_RAW, 2, struct.pack("=I", 0x1FFFFFFF))
            factory.return_value.bind.assert_called_once_with(("vcan0",))
            transport.close()


if __name__ == "__main__":
    unittest.main()
