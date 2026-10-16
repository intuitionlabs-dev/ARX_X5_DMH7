"""Run a complete two-arm sequence against the in-memory test plant."""

import argparse
from dataclasses import asdict
from enum import Enum
import json
from math import isfinite
import time

from . import Command, CommandKind as K, ControlSystem, RobotState as S
from .simulation import SimulatedMotors


def _json(value):
    return json.dumps(value, default=lambda item: item.name if isinstance(item, Enum) else asdict(item))


def demo(seconds=12.0, realtime=False, emit=print):
    system, plant = ControlSystem(), SimulatedMotors()
    stage = [0, 0]
    stage_ms = [0, 0]
    origin = time.monotonic()
    try:
        for now in range(int(seconds * 1000)):
            if realtime:
                delay = origin + now / 1000 - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            commands = []
            for i, arm in enumerate(system.arms):
                if now == 0:
                    commands.append(Command(i, K.CONNECT))
                elif now % 500 == 0:
                    commands.append(Command(i, K.HEARTBEAT))
                if now % 2:
                    continue
                if stage[i] == 0 and arm.state == S.WAIT_INIT:
                    commands.append(Command(i, K.INITIALIZE))
                    stage[i] = 1
                elif stage[i] == 1 and arm.state == S.NORMAL:
                    commands.extend((Command(i, K.START), Command(i, K.POSITION,
                                     (10, 15, 20, 5, -5, 10), -30)))
                    stage[i], stage_ms[i] = 2, now
                elif stage[i] == 2 and now - stage_ms[i] >= 1000:
                    commands.append(Command(i, K.GRAVITY))
                    stage[i], stage_ms[i] = 3, now
                elif stage[i] == 3 and now - stage_ms[i] >= 500:
                    commands.append(Command(i, K.DAMPING))
                    stage[i], stage_ms[i] = 4, now
                elif stage[i] == 4 and now - stage_ms[i] >= 500:
                    commands.append(Command(i, K.RESET))
                    stage[i] = 5
            result = system.step(now, commands=commands, motor_feedback=plant.feedback(now))
            plant.apply(result.motor_requests)
            for event in result.events:
                emit(_json(asdict(event)))
        summary = {"simulated": True, "duration_s": seconds,
                   "sequence_complete": all(s == 5 and a.state == S.NORMAL for s, a in zip(stage, system.arms)),
                   "arms": [{"arm_id": a.id, "state": a.state.name, "mode": a.actual_mode.name,
                             "gripper_initialized": a.initialized[6]} for a in system.arms]}
        emit(_json(summary))
        return summary
    finally:
        plant.apply(system.shutdown().motor_requests)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="run the local simulated sequence (default)")
    parser.add_argument("--seconds", type=float, default=12.0, help="simulated duration, default 12")
    parser.add_argument("--realtime", action="store_true", help="pace the simulation to wall clock time")
    args = parser.parse_args()
    if not isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    try:
        demo(args.seconds, args.realtime)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
