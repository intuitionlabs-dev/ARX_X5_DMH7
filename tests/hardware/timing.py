"""测量停止模式下的控制周期和 CAN 周期，影子控制器仅驱动模拟电机。"""

import argparse
from array import array
from collections import Counter
from functools import partial
import json
import math
from pathlib import Path
import platform
import statistics
import time

from arx_x5_control import (ArmMode, CANChannel, CANMotorIO, Command, CommandKind,
                            ControlSystem, MotorAction, ProcessCANMotorIO, RobotState, log)
from arx_x5_control.sdk_adapter.link import LocalLink
from arx_x5_control.simulation import SimulatedMotors
from arx_x5_control.sdk_adapter.tui import LocalConsoleSession, parse_options


class TracedTransport:
    def __init__(self, directory, config):
        self.base = CANMotorIO._default_transport(config)
        self.path = Path(directory)/f"can-{config.arm_id}.bin"
        self.rows = array("q")

    def start(self):
        getattr(self.base, "start", lambda: None)()

    def receive(self):
        return self.base.receive()

    def send(self, frame):
        start = time.monotonic_ns()
        self.base.send(frame)
        end = time.monotonic_ns()
        periodic = frame.arbitration_id in (1, 2, 4, 5, 6, 7, 8) and frame.data[:7] != b"\xff"*7
        self.rows.extend((start, end, frame.arbitration_id if periodic else -1))

    def close(self):
        try:
            self.base.close()
        finally:
            with self.path.open("wb") as output:
                self.rows.tofile(output)


class GuardedIO:
    def __init__(self, base):
        self.base = base
        self.ages = array("q")
        self.actions = Counter()

    def poll_feedback(self, now):
        rows = self.base.poll_feedback(now)
        self.ages.extend(now-row.timestamp_ms for row in rows)
        return rows

    def write_motor_requests(self, requests):
        requests = tuple(requests)
        if any(r.action not in (MotorAction.SOFT_STOP, MotorAction.ENABLE, MotorAction.DISABLE) for r in requests):
            raise RuntimeError("物理电机接口仅允许停止和使能保活请求")
        self.actions.update(r.action.value for r in requests)
        self.base.write_motor_requests(requests)


class ShadowControl:
    def __init__(self):
        self.system = ControlSystem()
        self.plant = SimulatedMotors()
        for arm in self.system.arms:
            arm.connected = arm.first_init = True
            arm.enter(RobotState.NORMAL, 0)
            arm.mode = ArmMode.RUN
            arm.initialized = [True]*7
        self.last_target = -10

    def step(self, now):
        commands = ()
        if now-self.last_target >= 10:
            target = (2*math.sin(now/1000), .5, .5, 0, 0, 0)
            commands = tuple(Command(i, CommandKind.POSITION, target, 0) for i in (0, 1))
            self.last_target = now
        result = self.system.step(now, commands=commands, motor_feedback=self.plant.feedback(now))
        self.plant.apply(result.motor_requests)


class TracedLink(LocalLink):
    def __init__(self, motor_io, *, loaded=False):
        super().__init__(motor_io)
        self.trace = array("q")
        self.shadow = ShadowControl() if loaded else None

    def _step_once(self, now):
        start = time.monotonic_ns()
        before = self.controller.command_last_ms
        result = super()._step_once(now)
        command_tick = self.controller.command_last_ms != before
        if self.shadow is not None:
            self.shadow.step(now)
        self.trace.extend((start, time.monotonic_ns(), now, int(command_tick)))
        return result


def distribution(values):
    data = sorted(values)
    if not data:
        return {"count": 0}
    def percentile(ratio):
        rank = (len(data)-1)*ratio
        lo = int(rank)
        return data[lo]+(data[min(lo+1, len(data)-1)]-data[lo])*(rank-lo)
    return dict(count=len(data), mean=statistics.fmean(data), minimum=data[0],
                p50=percentile(.5), p95=percentile(.95), p99=percentile(.99),
                p999=percentile(.999), maximum=data[-1], stdev=statistics.pstdev(data))


def interval_stats(stamps, target_ms):
    deltas = [(b-a)/1e6 for a, b in zip(stamps, stamps[1:])]
    return dict(target_ms=target_ms, interval_ms=distribution(deltas),
                absolute_jitter_ms=distribution([abs(x-target_ms) for x in deltas]),
                over_1_5_periods=sum(x > 1.5*target_ms for x in deltas),
                over_2_periods=sum(x > 2*target_ms for x in deltas))


def cpu_state():
    root = Path("/sys/devices/system/cpu/cpufreq")
    policies = list(root.glob("policy*"))
    return {name: dict(Counter((p/name).read_text().strip() for p in policies if (p/name).exists()))
            for name in ("scaling_governor", "energy_performance_preference")}


def run_scenario(options, scenario):
    directory = Path(options.output)/scenario
    directory.mkdir(parents=True, exist_ok=True)
    tui_opts = parse_options(["--simulate"] if options.simulate else
                            [argument for channel in options.channel for argument in ("--channel", channel)])
    base = SimulatedMotors() if options.simulate else ProcessCANMotorIO(
        [CANChannel(i, channel) for i, channel in enumerate(options.channel)],
        transport_factory=partial(TracedTransport, str(directory)))
    guarded = GuardedIO(base)
    link = TracedLink(guarded, loaded=scenario == "dynamics")
    report = dict(scenario=scenario, cpu_start=cpu_state(), kernel=platform.release(),
                  python=platform.python_version(), requested_duration_s=options.seconds,
                  measurement_clock="time.monotonic_ns", simulated=options.simulate)
    samples = []
    measured_start = measured_end = None
    try:
        if not options.simulate:
            base.open()
        link.open()
        session = LocalConsoleSession(link, tui_opts)
        warmup_end = time.monotonic()+options.warmup
        while time.monotonic() < warmup_end:
            session.tick(time.monotonic())
            link.check_health()
            time.sleep(.005)
        if any(a.snapshot() is None or a.snapshot().motor_lost_mask for a in session.arms.values()):
            raise RuntimeError("电机反馈未就绪")
        before_stats = link.stats
        guarded.ages = array("q")
        measured_start = time.monotonic_ns()
        deadline = measured_start+int(options.seconds*1e9)
        next_sample = measured_start
        while time.monotonic_ns() < deadline:
            session.tick(time.monotonic())
            link.check_health()
            now = time.monotonic_ns()
            if now >= next_sample:
                snapshots = [arm.snapshot() for arm in session.arms.values()]
                if any(s.mode != 0 for s in snapshots):
                    raise RuntimeError("物理机械臂必须保持 STOP")
                samples.append(dict(at_ns=now, arms=[dict(arm_id=s.arm_id,
                    angles=s.joint_angles_deg, lost=s.motor_lost_mask, age_ms=s.age*1000,
                    temperatures=s.temperatures) for s in snapshots]))
                next_sample = now+50_000_000
            time.sleep(.005)
        measured_end = time.monotonic_ns()
        after_stats = link.stats
        report["scheduler_delta"] = {key: after_stats[key]-before_stats[key] for key in
            ("missed_motor_ticks", "missed_command_ticks", "stale_feedback_count", "event_overruns")}
        report["rejected_commands"] = after_stats["rejected_commands"]
    except Exception as error:
        report["error"] = str(error)
    finally:
        for resource in (link, None if options.simulate else base):
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    report.setdefault("shutdown_errors", []).append(str(error))
        measured_end = measured_end or time.monotonic_ns()
        report["cpu_end"] = cpu_state()
        report["physical_request_counts"] = dict(guarded.actions)
        if not options.simulate:
            report["io_stats"] = base.stats()
        with (directory/"control.bin").open("wb") as output:
            link.trace.tofile(output)
        (directory/"feedback.json").write_text(json.dumps(samples)+"\n")
        if measured_start is not None:
            report["start_ns"], report["end_ns"] = measured_start, measured_end
            report["measured_duration_s"] = (measured_end-measured_start)/1e9
            rows = [tuple(link.trace[i:i+4]) for i in range(0, len(link.trace), 4)
                    if measured_start <= link.trace[i] <= measured_end]
            report["motor_loop"] = interval_stats([r[0] for r in rows], 1)
            report["command_loop"] = interval_stats([r[0] for r in rows if r[3]], 2)
            report["iteration_duration_ms"] = distribution([(r[1]-r[0])/1e6 for r in rows])
            report["feedback_age_ms"] = distribution(guarded.ages)
            report["step_over_1ms"] = sum(r[1]-r[0] > 1_000_000 for r in rows)
            report["motor_lost_observations"] = sum(bool(arm["lost"]) for s in samples for arm in s["arms"])
            report["sdk_age_ms"] = distribution([a["age_ms"] for s in samples for a in s["arms"]])
            report["joint_span_deg"] = [[max(s["arms"][a]["angles"][j] for s in samples)
                -min(s["arms"][a]["angles"][j] for s in samples) for j in range(6)] for a in (0, 1)] if samples else []
            report["can"] = []
            for arm in (0, 1):
                path = directory/f"can-{arm}.bin"
                if not path.exists():
                    continue
                data = array("q")
                with path.open("rb") as source:
                    data.frombytes(source.read())
                sends = [tuple(data[i:i+3]) for i in range(0, len(data), 3)
                         if measured_start <= data[i] <= measured_end]
                report["can"].append(dict(arm_id=arm, frames=len(sends),
                    rate_hz=len(sends)/report["measured_duration_s"],
                    frame_interval_ms=distribution([(b[0]-a[0])/1e6 for a, b in zip(sends, sends[1:])]),
                    write_duration_ms=distribution([(r[1]-r[0])/1e6 for r in sends]),
                    motors={str(m): interval_stats([r[0] for r in sends if r[2] == m], 2 if m == 1 else 4)
                            for m in (1, 2, 4, 5, 6, 7, 8)}))
        (directory/"report.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({key: value for key, value in report.items() if key != "can"}), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", action="append", default=[])
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--warmup", type=float, default=2)
    parser.add_argument("--scenario", choices=("all", "baseline", "dynamics"), default="all")
    parser.add_argument("--output", default="build/timing")
    options = parser.parse_args()
    if not options.simulate and len(options.channel) != 2:
        parser.error("请指定两个 CANable2 稳定通道路径")
    if not all(math.isfinite(x) and x > 0 for x in (options.seconds, options.warmup)):
        parser.error("采样时长和预热时长必须为有限正数")
    log.configure(level="ERROR")
    scenarios = ("baseline", "dynamics") if options.scenario == "all" else (options.scenario,)
    reports = [run_scenario(options, scenario) for scenario in scenarios]
    return 1 if any(r.get("error") or r.get("shutdown_errors") for r in reports) else 0


if __name__ == "__main__":
    raise SystemExit(main())
