"""通过 SDK 执行单臂或双臂初始化、小幅运动、模式和复位测试。"""
import argparse
import dataclasses
import json
import math
from pathlib import Path
import time

from arx_x5_control import ArmConfig, CANChannel, ProcessCANMotorIO as CANMotorIO, log
from arx_x5_control.sdk_adapter.link import LocalLink

RESET = (0, .5, .5, 0, 0, 0)


class FullFlow:
    def __init__(self, simulation=False, *, channels=(), output=None, transport_config=None, recorder=None):
        self.simulation = simulation
        self.channels = tuple(channels)
        self.arm_count = 2 if simulation else len(self.channels)
        self.transport_config = transport_config
        self.recorder = recorder
        self.origin = time.monotonic()
        self.report = dict(simulated=simulation, sequence_complete=False, stages=[], events=[], arms=[])
        self.stage = "connect"
        self.heartbeats = [0]*self.arm_count
        self.omit_heartbeat = set()
        self.last_sample = 0
        self.initial = None
        self.samples = []
        self.output = Path(output or ("build/full-flow/simulation-report.json" if simulation else "build/full-flow/full-flow-report.json"))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.telemetry = self.output.with_suffix(".jsonl").open("w")

    def record(self, kind, **values):
        row = dict(t_s=round(time.monotonic()-self.origin, 4), kind=kind, **values)
        self.telemetry.write(json.dumps(row, default=str)+"\n")
        if kind == "stage":
            self.report["stages"].append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
        return row

    def begin(self, name, arm_id=None):
        self.stage = name
        self.record("stage", name=name, arm_id=arm_id)

    def monitor(self):
        self.link.check_health()
        if self.io is not None:
            self.io.check_health()
        now = time.monotonic()
        for i, arm in enumerate(self.arms):
            if i not in self.omit_heartbeat and now-self.heartbeats[i] >= .25:
                arm.keepalive()
                self.heartbeats[i] = now
        for event in self.link.drain_events():
            row = dataclasses.asdict(event)
            self.report["events"].append(row)
            self.record("event", event=row)
            if event.kind in ("rejected", "auto_aborted", "initialization_failed", "dynamics_error", "reference_error"):
                raise RuntimeError(f"arm {event.arm_id}: {event.kind}: {event.detail}")
        snapshots = [arm.snapshot() for arm in self.arms]
        if any(s is None for s in snapshots):
            return snapshots
        with self.link._step_lock:
            detail = [dict(errors=[m.error_code for m in arm.bank.motors], zero_busy=arm.zero.busy,
                           auto=arm.command.auto, auto_step=arm.command.auto_step,
                           dynamics_status=int(arm.dynamics_status), torque=list(arm.dynamics_torque),
                           gz=dict(arm.command.gz)) for arm in self.link.controller.arms]
        for i, (snapshot, core) in enumerate(zip(snapshots, detail)):
            if snapshot.age > .1:
                raise RuntimeError(f"arm {i}: stale controller feedback")
            if snapshot.motor_lost_mask and self.initial is not None and not core["zero_busy"]:
                raise RuntimeError(f"arm {i}: motor lost mask {snapshot.motor_lost_mask}")
            if any(code for code in core["errors"][:3]) or any(code not in (0, 1) for code in core["errors"][3:]):
                raise RuntimeError(f"arm {i}: motor errors {core['errors']}")
            if max(snapshot.temperatures) >= 65:
                raise RuntimeError(f"arm {i}: motor temperature limit")
            if self.initial is not None:
                movement = max(abs(a-b) for a, b in zip(snapshot.joint_angles_deg, self.initial[i]))
                if movement > 15:
                    raise RuntimeError(f"arm {i}: joint excursion exceeds 15 degrees: {movement}")
                if max(map(abs, snapshot.joint_velocities_deg_s)) > 180:
                    raise RuntimeError(f"arm {i}: joint speed exceeds 180 deg/s: {snapshot.joint_velocities_deg_s}; q={snapshot.joint_angles_deg}; core={core}")
        if now-self.last_sample >= .02:
            row = self.record("sample", stage=self.stage, arms=[dict(**dataclasses.asdict(snapshot),
                core=core) for i, (snapshot, core) in enumerate(zip(snapshots, detail))])
            self.samples.append(row)
            self.last_sample = now
        return snapshots

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            snapshots = self.monitor()
            if all(s is not None for s in snapshots) and predicate(snapshots):
                return snapshots
            time.sleep(.01)
        raise TimeoutError(label)

    def hold(self, seconds, callback=None):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.monitor()
            if callback:
                callback()
            time.sleep(.01)

    def run_arm(self, index):
        arm = self.arms[index]
        self.begin("initialize", index)
        arm.init_arm()
        self.wait(lambda s: s[index].robot_state == 3, 1, "initialization not accepted")
        self.wait(lambda s: s[index].robot_state == 4 and bool(s[index].motor_inited_mask & 64),
                  28, "initialization/gripper homing timeout")
        self.report["arms"].append(dict(arm_id=index, channel="simulation" if self.simulation else self.channels[index].channel, initialized=True))

        self.begin("position_positive_2deg", index)
        arm.start()
        self.wait(lambda s: s[index].mode == 1, 1, "run mode not reached")
        origin = list(RESET)
        target = origin.copy()
        target[0] += 2.0
        gripper = arm.raw_gripper_angle_deg
        command = lambda values: arm.send_ctrl_pos(tuple(map(math.radians, values)), gripper_raw_deg=gripper)
        self.hold(1.5, lambda: command(target))
        actual = arm.snapshot().joint_angles_deg
        error = max(abs(a-b) for a, b in zip(actual, target))
        if error > 1.5:
            raise RuntimeError(f"arm {index}: position tracking error {error:.3f} degrees")
        self.report["arms"][-1]["position_positive_error_deg"] = error

        self.begin("position_return", index)
        self.hold(1.2, lambda: command(origin))
        self.begin("position_velocity_negative_2deg", index)
        start = time.monotonic()
        while time.monotonic()-start < 1.2:
            self.monitor()
            t = min(1.0, time.monotonic()-start)
            blend = 10*t**3-15*t**4+6*t**5
            slope = 30*t*t-60*t**3+30*t**4 if t < 1 else 0
            target = origin.copy()
            target[0] -= 2*blend
            velocity = [-2*slope, 0, 0, 0, 0, 0]
            arm.send_ctrl_pos_vel(tuple(map(math.radians, target)), tuple(map(math.radians, velocity)),
                                  gripper_raw_deg=gripper)
            time.sleep(.01)
        self.hold(.5, lambda: arm.send_ctrl_pos_vel(tuple(map(math.radians, target)), (0,)*6,
                                                   gripper_raw_deg=gripper))
        actual = arm.snapshot().joint_angles_deg
        error = max(abs(a-b) for a, b in zip(actual, target))
        if error > 1.5:
            raise RuntimeError(f"arm {index}: position/velocity tracking error {error:.3f} degrees")
        self.report["arms"][-1]["position_velocity_error_deg"] = error

        self.begin("return_before_modes", index)
        self.hold(1.2, lambda: command(origin))
        joint_errors = []
        for joint in range(6):
            self.begin(f"joint{joint+1}_positive_2deg", index)
            target = origin.copy()
            target[joint] += 2.0
            self.hold(1.0, lambda: command(target))
            error = max(abs(a-b) for a, b in zip(arm.snapshot().joint_angles_deg, target))
            if error > 1.5:
                raise RuntimeError(f"arm {index}: joint {joint+1} tracking error {error:.3f} degrees")
            joint_errors.append(error)
            self.hold(.8, lambda: command(origin))
        self.report["arms"][-1]["per_joint_error_deg"] = joint_errors

        self.begin("continuous_tracking_15s", index)
        started = time.monotonic()
        while time.monotonic()-started < 15:
            self.monitor()
            target = origin.copy()
            target[0] += 2*math.sin(math.pi*(time.monotonic()-started))
            command(target)
            time.sleep(.01)
        self.hold(1.0, lambda: command(origin))
        self.begin("gravity", index)
        arm.gravity()
        self.wait(lambda s: s[index].mode == 3, 1, "gravity mode not reached")
        self.hold(.3)
        self.begin("damping", index)
        arm.damping()
        self.wait(lambda s: s[index].mode == 2, 1, "damping mode not reached")
        self.hold(.3)
        self.begin("reset", index)
        arm.reset()
        self.wait(lambda s: s[index].robot_state == 7, 1, "reset not accepted")
        self.wait(lambda s: s[index].robot_state == 4 and s[index].mode == 2, 5, "reset not complete")
        error = max(abs(a-b) for a, b in zip(arm.snapshot().joint_angles_deg, RESET))
        if error > 1.5:
            raise RuntimeError(f"arm {index}: reset tracking error {error:.3f} degrees")
        self.report["arms"][-1]["reset_error_deg"] = error
        self.begin("stop", index)
        arm.stop()
        self.wait(lambda s: s[index].mode == 0, 1, "stop mode not reached")
        self.report["arms"][-1]["completed"] = True

    def execute(self):
        self.io = None if self.simulation else CANMotorIO(
            self.channels, transport_config=self.transport_config, collect_performance=self.recorder is not None)
        self.link = (LocalLink.simulated() if self.simulation else LocalLink(
            self.io, configurations=[ArmConfig() for _ in self.channels],
            collect_performance=self.recorder is not None, collect_trace=self.recorder is not None))
        try:
            if self.io is not None:
                self.io.open()
            self.link.open()
            if self.recorder is not None:
                self.recorder.attach(self.link, self.io)
            self.arms = [self.link.arm(i) for i in range(self.arm_count)]
            for arm in self.arms:
                arm.connect(timeout=3)
            snapshots = self.wait(lambda s: all(x.robot_state == 2 and x.motor_lost_mask == 0 for x in s),
                                  3, "motors not online")
            self.initial = [tuple(s.joint_angles_deg) for s in snapshots]
            self.report["initial_joint_angles_deg"] = self.initial
            if any(abs(a-b) > 20 for angles in self.initial for a, b in zip(angles, RESET)):
                raise RuntimeError("initial pose outside initialization tolerance")
            self.begin("connected_stop_stream")
            self.hold(1)
            for index in range(self.arm_count):
                self.run_arm(index)
            self.begin("per_arm_watchdog", 0)
            self.arms[0].start()
            self.wait(lambda s: s[0].mode == 1, 1, "watchdog test run not reached")
            self.omit_heartbeat.add(0)
            self.wait(lambda s: s[0].mode == 2, 3.6, "watchdog damping not reached")
            if self.arm_count > 1 and self.arms[1].snapshot().mode != 0:
                raise RuntimeError("watchdog changed the other arm mode")
            self.omit_heartbeat.clear()
            self.arms[0].keepalive()
            self.arms[0].stop()
            self.wait(lambda s: all(x.mode == 0 for x in s), 1, "final stop not reached")
            self.report["final_stop_snapshots"] = [dataclasses.asdict(arm.snapshot()) for arm in self.arms]
            self.begin("complete")
            self.report["sequence_complete"] = True
        except Exception as error:
            self.report["error"] = str(error)
            self.report["failure_snapshots"] = [dataclasses.asdict(s) if s is not None else None
                for arm in getattr(self, "arms", ()) for s in (arm.snapshot(),)]
            with self.link._step_lock:
                self.report["failure_requests"] = [dataclasses.asdict(r) for r in self.link.last_result.motor_requests]
            self.report["failed_stage"] = self.stage
            self.record("stage", name="failed", stage=self.stage, error=str(error))
        finally:
            self.report["duration_s"] = time.monotonic()-self.origin
            self.report["link_stats"] = self.link.stats
            try:
                self.link.close()
            except Exception as error:
                self.report["shutdown_error"] = str(error)
                self.report["sequence_complete"] = False
            if self.io is not None:
                try:
                    self.io.close()
                except Exception as error:
                    self.report["can_close_error"] = str(error)
                    self.report["sequence_complete"] = False
                self.report["io_stats"] = self.io.stats()
            if self.samples:
                self.report["last_periodic_snapshot"] = self.samples[-1]["arms"]
            self.output.write_text(json.dumps(self.report, indent=2, default=str)+"\n")
            self.telemetry.close()
            print(json.dumps({key: value for key, value in self.report.items()
                              if key not in ("events", "last_periodic_snapshot", "final_stop_snapshots", "failure_snapshots", "failure_requests")}, default=str), flush=True)
        return self.report["sequence_complete"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--allow-motion", action="store_true", help="允许初始化、夹爪找零、小幅运动和复位")
    parser.add_argument("--interface", default="slcan")
    parser.add_argument("--transport-config")
    parser.add_argument("--log-dir", help="性能日志根目录")
    parser.add_argument("--channel", action="append", help="按 arm_id 顺序指定一条或两条通道")
    parser.add_argument("--output", help="JSON 报告路径")
    args = parser.parse_args()
    if not args.simulate and not args.allow_motion:
        parser.error("实机流程需要 --allow-motion")
    if not args.simulate and not 1 <= len(args.channel or ()) <= 2:
        parser.error("实机流程需要一条或两条 --channel")
    channels = [CANChannel(i, channel, interface=args.interface)
                for i, channel in enumerate(args.channel or ())]
    log.configure(level="ERROR")
    from arx_x5_control.sdk_adapter.telemetry import RuntimeRecorder
    with RuntimeRecorder(args) as recorder:
        result = FullFlow(args.simulate, channels=channels, output=args.output,
                          transport_config=args.transport_config, recorder=recorder).execute()
    print(json.dumps({"performance_log": str(recorder.directory), "recorder_error": recorder.error}))
    raise SystemExit(0 if result and recorder.error is None else 1)
