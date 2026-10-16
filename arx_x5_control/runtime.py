"""Optional PC monotonic-clock runner. Python scheduling is best effort."""

import time

from .interfaces import ControlIO
from .system import ControlSystem


def run(system: ControlSystem, io: ControlIO, *, stop=None, duration_s=None):
    """Run until stop.is_set(), duration or KeyboardInterrupt; always emit stops.

    A fresh system is required. The adapter receives timestamps relative to this
    invocation; translate device clock timestamps into this same domain.
    """
    if system.last_ms is not None or system.closed:
        raise ValueError("run requires a fresh controller")
    if duration_s is not None and (not 0 < duration_s < float("inf")):
        raise ValueError("duration_s must be finite and positive")
    origin = time.monotonic_ns()
    deadline = origin
    try:
        while stop is None or not stop.is_set():
            now_ns = time.monotonic_ns()
            if duration_s is not None and now_ns - origin >= duration_s * 1e9:
                break
            if now_ns < deadline:
                time.sleep((deadline - now_ns) / 1e9)
                continue
            now_ms = (now_ns - origin) // 1_000_000
            if system.last_ms is None or now_ms > system.last_ms:
                result = system.step(now_ms, commands=io.poll_commands(now_ms),
                                     motor_feedback=io.poll_feedback(now_ms))
                io.write_motor_requests(result.motor_requests)
                io.publish_feedback(result.feedback)
                io.publish_events(result.events)
            deadline = origin + (now_ms + 1) * 1_000_000
    except KeyboardInterrupt:
        pass
    finally:
        result = system.shutdown((time.monotonic_ns() - origin) // 1_000_000)
        io.write_motor_requests(result.motor_requests)
        io.publish_events(result.events)
