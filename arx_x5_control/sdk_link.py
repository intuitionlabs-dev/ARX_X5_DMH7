"""Run the real Python control core behind the existing sdk.Arm API.

Only SDK payloads are translated; no serial device, wire frames or IPC is used.
Import LocalLink explicitly so the independent control core needs no SDK import.
"""

from collections import deque
import struct
import threading
import time

from sdk import Link, LinkError, NotConnected, ProtocolError, protocol

from .config import ArmConfig
from .interfaces import MotorIO
from .system import ControlSystem
from .types import Command, CommandKind as K, TickResult, timestamp


_EMPTY_COMMANDS = {
    protocol.TID_ARM_INIT: K.INITIALIZE,
    protocol.TID_MOTOR_SET_ZERO: K.CALIBRATE,
    protocol.TID_ARM_START: K.START,
    protocol.TID_ARM_STOP: K.STOP,
    protocol.TID_ARM_DAMPING: K.DAMPING,
    protocol.TID_ARM_GRAVITY: K.GRAVITY,
    protocol.TID_ARM_RESET: K.RESET,
}


def _command(tid, payload, arm_id):
    if type(tid) is not int or not isinstance(payload, (bytes, bytearray)):
        raise ProtocolError("tid must be int and payload must be bytes")
    try:
        if tid == protocol.TID_VERSION:
            if struct.unpack("<I", payload)[0] != protocol.VERSION_CODE:
                raise ValueError("SDK protocol version mismatch")
            return Command(arm_id, K.CONNECT)
        if tid in _EMPTY_COMMANDS:
            if payload:
                raise ValueError("this command requires an empty payload")
            return Command(arm_id, _EMPTY_COMMANDS[tid])
        if tid == protocol.TID_ARM_CTRL_POS:
            values = struct.unpack("<7f", payload)
            return Command(arm_id, K.POSITION, values[:6], values[6])
        if tid == protocol.TID_ARM_CTRL_POS_VEL:
            values = struct.unpack("<13f", payload)
            return Command(arm_id, K.POSITION_VELOCITY, values[:6], values[12], values[6:12])
        raise ValueError(f"unsupported local command tid: {tid:#x}")
    except (ValueError, struct.error) as exc:
        raise ProtocolError(str(exc)) from exc


class LocalLink(Link):
    """A shared SDK link owning one Python controller and a MotorIO adapter.

    Use ``LocalLink(motor_io)`` for an injected motor boundary, or explicitly use
    ``LocalLink.simulated()`` without hardware. ``sdk.Arm(0, link)`` and
    ``link.arm(0)`` retain SDK units, prechecks, calibration and wait methods.

    open() starts a best-effort 1 ms thread. With background=False, the caller
    instead drives step(now_ms); blocking SDK waits require another caller to
    keep stepping. No automatic heartbeats: each arm still needs valid commands
    within 3 s. close() delivers controller shutdown requests and joins the
    worker. Reopening creates a fresh controller, requiring connect/init again.

    MotorIO timestamps are milliseconds since this link's construction in
    background mode, or the explicit step clock in manual mode. Keep that clock
    increasing across reopen. MotorIO is retained across reopen and is owned by
    the caller (its devices are neither opened nor closed by this link).
    """

    def __init__(self, motor_io: MotorIO, *, configurations=None, background=True):
        super().__init__(port="local", verify_on_open=False, auto_reconnect=False)
        if not all(callable(getattr(motor_io, name, None)) for name in
                   ("poll_feedback", "write_motor_requests")):
            raise TypeError("motor_io must implement MotorIO")
        self.configurations = tuple(configurations) if configurations is not None else (ArmConfig(), ArmConfig())
        self.controller = ControlSystem(self.configurations)
        self.motor_io = motor_io
        self.background = background
        self.kind = "LOCAL"
        self.last_result = TickResult()
        self._origin_ns = time.monotonic_ns()
        self._last_ms = None
        self._lifecycle_lock = threading.RLock()
        self._step_lock = threading.Lock()
        self._commands = [deque() for _ in self.configurations]
        self._events = deque(maxlen=512)
        self._event_lock = threading.Lock()
        self.event_overruns = 0
        self._failure = self._shutdown_failure = None

    @classmethod
    def simulated(cls, *, configurations=None, background=True):
        """Explicit test plant; still executes ControlSystem, not sdk.SimLink."""
        from .simulation import SimulatedMotors
        configs = tuple(configurations) if configurations is not None else (ArmConfig(), ArmConfig())
        return cls(SimulatedMotors(len(configs)), configurations=configs, background=background)

    def _validate_arm(self, arm_id):
        if type(arm_id) is not int or not 0 <= arm_id < len(self.configurations):
            raise ValueError(f"arm_id must be in 0..{len(self.configurations)-1}")

    def channel(self, arm_id):
        self._validate_arm(arm_id)
        return super().channel(arm_id)

    @property
    def is_open(self):
        return self._alive and not self._stop_ev.is_set()

    def _clear_channels(self):
        with self._chan_lock:
            for ch in self._channels.values():
                with ch.lock:
                    ch.feedback = ch.ack = None
                    ch.feedback_t = ch.ack_t = 0.0
                    ch.ack_event.clear()
                    # Keep sequence numbers monotonic for existing SDK Arm objects.

    def open(self, port=None):
        if port not in (None, "local"):
            raise LinkError("LocalLink does not accept a serial port")
        with self._lifecycle_lock:
            if self.is_open:
                return self
            if self._thr is not None and self._thr.is_alive():
                raise LinkError("previous control worker has not stopped")
            self.controller = ControlSystem(self.configurations)
            self.last_result = TickResult()
            self._failure = self._shutdown_failure = self.link_err = None
            self.board_version = None
            self.version_seen = False
            self._clear_channels()
            with self._write_lock:
                for queue in self._commands:
                    queue.clear()
                self._stop_ev.clear()
                self._alive = self.link_up = True
            self.open_gen += 1
            if self.background:
                self._thr = threading.Thread(target=self._worker, name="arx-x5-control", daemon=True)
                self._thr.start()
            return self

    def close(self):
        with self._lifecycle_lock:
            with self._write_lock:
                self._stop_ev.set()
                self._alive = self.link_up = False
                for queue in self._commands:
                    queue.clear()
            if self._thr is not None:
                if self._thr is threading.current_thread():
                    raise LinkError("close must be called outside the MotorIO callback")
                self._thr.join(timeout=2.0)
                if self._thr.is_alive():
                    raise LinkError("MotorIO is blocking; control worker has not stopped")
                self._thr = None
            elif not self.background and self.open_gen:
                with self._step_lock:
                    self._shutdown()
            self._clear_channels()
            if self._shutdown_failure is not None:
                raise LinkError(f"motor shutdown delivery failed: {self._shutdown_failure}") from self._shutdown_failure

    def reopen(self, port=None):
        self.close()
        return self.open(port)

    def check_health(self):
        """Raise SDK LinkError for worker/adapter failure; no implicit restart."""
        if self._failure is not None:
            raise LinkError(f"local controller failed: {self.link_err}") from self._failure
        if not self.is_open:
            raise NotConnected("LocalLink is not open")

    def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
        self._validate_arm(arm_id)
        command = _command(tid, payload, arm_id)
        with self._write_lock:
            self.check_health()
            queue = self._commands[arm_id]
            if len(queue) >= 7:
                raise LinkError(f"arm {arm_id} command queue full; wait for the next control tick")
            queue.append(command)
            self.n_tx += 1
        # Match Link.send's return convention, without constructing a wire frame.
        return protocol.OVERHEAD + len(payload)

    def inject_bytes(self, data):
        raise ProtocolError("LocalLink accepts SDK commands, not wire frames")

    def _step_once(self, now_ms):
        commands = []
        # Drain only on a command tick, so a second batch cannot overflow the
        # core's seven-slot per-arm queue between its 2 ms command updates.
        if self.controller.command_last_ms is None or now_ms - self.controller.command_last_ms >= 2:
            with self._write_lock:
                for queue in self._commands:
                    commands.extend(queue)
                    queue.clear()
        result = self.controller.step(now_ms, commands=commands,
                                      motor_feedback=self.motor_io.poll_feedback(now_ms))
        self._last_ms = now_ms
        self.last_result = result
        self.motor_io.write_motor_requests(result.motor_requests)
        self._publish(result)
        return result

    def step(self, now_ms):
        """One explicit manual tick; never call in background mode."""
        if self.background:
            raise RuntimeError("step requires background=False")
        timestamp(now_ms)
        with self._step_lock:
            self.check_health()
            if self._last_ms is not None and now_ms <= self._last_ms:
                raise ValueError("LocalLink step timestamps must strictly increase across opens")
            try:
                return self._step_once(now_ms)
            except Exception as exc:
                self._fail(exc)
                self._shutdown()
                self.check_health()

    def _worker(self):
        try:
            while not self._stop_ev.is_set():
                now_ms = (time.monotonic_ns() - self._origin_ns) // 1_000_000
                if self._last_ms is None or now_ms > self._last_ms:
                    with self._step_lock:
                        self._step_once(now_ms)
                delay = (self._origin_ns + (now_ms + 1) * 1_000_000 - time.monotonic_ns()) / 1e9
                self._stop_ev.wait(max(0, delay))
        except Exception as exc:
            self._fail(exc)
        finally:
            self._shutdown()
            self._alive = self.link_up = False

    def _fail(self, exc):
        with self._write_lock:
            if self._failure is None:
                self._failure = exc
            self.link_err = str(self._failure)
            self._alive = self.link_up = False
            self._stop_ev.set()

    def _shutdown(self):
        if self.controller.closed:
            return
        now = max(self._last_ms or 0, self.controller.last_ms or 0)
        if self.background:
            now = max(now, (time.monotonic_ns() - self._origin_ns) // 1_000_000)
        try:
            result = self.controller.shutdown(now)
            self.motor_io.write_motor_requests(result.motor_requests)
            self._publish(result)
        except Exception as exc:
            self._shutdown_failure = exc
            self._fail(exc)

    def _publish(self, result):
        received_at = time.monotonic()
        for fb in result.feedback:
            self.channel(fb.arm_id).put_feedback(protocol.Feedback(
                mode=int(fb.mode), robot_state=int(fb.state),
                motor_lost_mask=sum(int(v) << i for i, v in enumerate(fb.motor_lost)),
                motor_inited_mask=sum(int(v) << i for i, v in enumerate(fb.motor_initialized)),
                joint_angle=fb.joint_angle, joint_vel=fb.joint_velocity,
                gripper_angle=fb.gripper_angle,
                motor_temp=tuple(0 if v is None else min(255, max(0, int(v))) for v in fb.motor_temperature),
            ), received_at)
            self.n_rx += 1
        for event in result.events:
            if event.kind in ("connected", "connection_required"):
                self.board_version = protocol.VERSION_CODE
                self.version_seen = True
                self.n_rx += 1
            elif event.kind == "calibrated":
                self.channel(event.arm_id).put_ack(protocol.SetZeroAck(
                    int(event.zero_state), tuple(map(int, event.zero_results)), event.elapsed_ms,
                ), received_at)
                self.n_rx += 1
        with self._event_lock:
            self.event_overruns += max(0, len(self._events) + len(result.events) - self._events.maxlen)
            self._events.extend(result.events)

    def drain_events(self):
        """Drain diagnostic core events; history keeps the latest 512 entries."""
        with self._event_lock:
            events = tuple(self._events)
            self._events.clear()
            return events

    def wait_feedback(self, arm_id, after_seq=0, timeout=3.0, poll=0.005):
        channel = self.channel(arm_id)
        deadline = time.monotonic() + timeout
        while True:
            self.check_health()
            feedback, _, seq = channel.snapshot_feedback()
            if feedback is not None and seq > after_seq:
                return feedback
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._stop_ev.wait(min(poll, remaining))

    @property
    def stats(self):
        stats = super().stats
        with self._write_lock:
            stats["pending_commands"] = tuple(len(q) for q in self._commands)
        stats.update(missed_motor_ticks=self.controller.missed_motor_ticks,
                     missed_command_ticks=self.controller.missed_command_ticks,
                     stale_feedback_count=self.controller.stale_feedback_count,
                     rejected_commands=tuple(a.rejected_count for a in self.controller.arms),
                     event_overruns=self.event_overruns)
        return stats

    def describe(self):
        return "local Python ControlSystem / injected MotorIO"
