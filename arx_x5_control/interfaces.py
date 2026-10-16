"""Interfaces for a future same-device Python communication adapter."""

from typing import Iterable, Protocol

from .types import ArmFeedback, Command, Event, MotorFeedback, MotorRequest


class MotorIO(Protocol):
    """Decoded motor boundary, called serially by the controller owner.

    Both methods must be nonblocking. Feedback timestamps use the same clock as
    now_ms and retain acquisition time; old samples must not be restamped. Motor
    requests must be delivered in order, including shutdown and zeroing guards.
    Raise on output failure instead of silently dropping requests.
    """

    def poll_feedback(self, now_ms: int) -> Iterable[MotorFeedback]: ...
    def write_motor_requests(self, requests: tuple[MotorRequest, ...]) -> None: ...


class ControlIO(MotorIO, Protocol):
    """Nonblocking methods, called by one control thread, in order.

    poll_* must return drained snapshots, not repeat old feedback as new data.
    Motor requests preserve ordering (including SAVE_ZERO/ENABLE/DISABLE).
    A transport that cannot accept output must raise, never silently discard it.
    Motor feedback/zero acknowledgements arrive on subsequent poll_feedback calls.
    No CAN, UART, USB, sockets, shared memory or process queues are provided here.
    """

    def poll_commands(self, now_ms: int) -> Iterable[Command]: ...
    def publish_feedback(self, feedback: tuple[ArmFeedback, ...]) -> None: ...
    def publish_events(self, events: tuple[Event, ...]) -> None: ...
