import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from arx_x5_control import log
from arx_x5_control.performance import Distribution, PerformanceProbe
from arx_x5_control.sdk_adapter.telemetry import LinuxCPU, RuntimeRecorder
from arx_x5_control.sdk_adapter.tui import open_session, parse_options


class PerformanceTests(unittest.TestCase):
    def test_histograms_merge_exact_totals_and_bound_quantiles(self):
        first, second = Distribution(), Distribution()
        for value in (0, 1, 9999, 10000):
            first.add(value)
        second.add(15_000_000)
        first.merge(json.loads(json.dumps(second.snapshot())))
        result = first.snapshot()
        self.assertEqual(result["count"], 5)
        self.assertEqual(result["sum_ns"], 15_020_000)
        self.assertEqual(result["max_ns"], 15_000_000)
        self.assertEqual(result["p50_upper_ns"], 10_000)
        self.assertIsNone(result["p99_upper_ns"])
        self.assertEqual(result["overflow_count"], 1)

    def test_window_swap_preserves_cross_window_period(self):
        probe = PerformanceProbe()
        probe.tick("period", 100)
        probe.tick("period", 250)
        probe.count("frames", 3)
        probe.peak("queue", 7)
        before = probe.snapshot()
        probe.tick("period", 350)
        after = probe.snapshot()
        self.assertEqual(before["timings"]["period"]["max_ns"], 150)
        self.assertEqual(after["timings"]["period"]["max_ns"], 100)
        self.assertEqual(before["counters"]["frames"], 3)
        self.assertEqual(after["counters"], {})
        self.assertEqual(before["peaks"]["queue"], 7)
        probe.reset_clock()
        probe.tick("period", 0)
        self.assertEqual(probe.snapshot()["timings"], {})

    def test_cpu_stat_handles_spaces_and_parentheses_in_name(self):
        fields = ["S"]+["0"]*36
        fields[11], fields[12], fields[19], fields[36] = "7", "5", "99", "3"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"stat"
            path.write_text("100 (a task) name) "+" ".join(fields))
            result = LinuxCPU._stat(path)
        self.assertEqual(result["name"], "a task) name")
        self.assertEqual(result["ticks"], 12)
        self.assertEqual(result["start_ticks"], 99)
        self.assertEqual(result["last_cpu"], 3)

    def test_session_saves_metrics_events_cpu_and_final_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            options = parse_options(["--simulate", "--log-dir", directory])
            log.configure(stream=None)
            with RuntimeRecorder(options) as recorder:
                with open_session(options, recorder) as session:
                    session.info("记录测试事件")
                    deadline = time.monotonic()+1.3
                    while time.monotonic() < deadline:
                        session.tick(time.monotonic())
                        time.sleep(.01)
                    self.assertIsNone(recorder.error)
                    session.link.check_health()
            self.assertIsNone(recorder.error)
            path = recorder.directory
            summary = json.loads((path/"summary.json").read_text())
            metadata = json.loads((path/"metadata.json").read_text())
            metrics = [json.loads(line) for line in (path/"metrics.jsonl").read_text().splitlines()]
            states = [json.loads(line) for line in (path/"states.jsonl").read_text().splitlines()]
            events = [json.loads(line) for line in (path/"events.jsonl").read_text().splitlines()]
            self.assertEqual(summary["status"], "complete")
            self.assertIsNone(summary["error"])
            self.assertEqual(summary["dropped_log_records"], 0)
            self.assertGreater(summary["timing_totals"]["host.control.period"]["count"], 100)
            self.assertGreater(summary["timing_totals"]["tui.ui.period"]["count"], 30)
            self.assertTrue(any(row["kind"] == "control" for row in events))
            self.assertIn("记录测试事件", (path/"ui.log").read_text())
            self.assertTrue((path/"runtime.log").read_text())
            self.assertGreater(len(states), 5)
            self.assertEqual(len(states[-1]["arms"]), 2)
            samples = [row for row in metrics if row["kind"] == "sample"]
            self.assertIsNotNone(samples[0]["cpu"]["host_cpu_percent"])
            self.assertTrue(samples[0]["cpu"]["processes"][0]["threads"])
            self.assertEqual(metadata["schema_version"], 1)
            self.assertEqual((Path(directory)/"latest").resolve(), path)

    def test_writer_failure_does_not_interrupt_controller(self):
        with tempfile.TemporaryDirectory() as directory:
            options = parse_options(["--simulate", "--log-dir", directory])
            with RuntimeRecorder(options) as recorder:
                with open_session(options, recorder) as session:
                    with patch.object(recorder, "_write", side_effect=OSError("disk full")):
                        recorder.emit("test", {})
                        deadline = time.monotonic()+1
                        while recorder.error is None and time.monotonic() < deadline:
                            recorder.emit("test", {})
                            time.sleep(.01)
                    self.assertEqual(recorder.error, "disk full")
                    session.tick(time.monotonic())
                    session.link.check_health()
                    self.assertTrue(session._recording_error_shown)
            self.assertFalse(recorder._thread.is_alive())


if __name__ == "__main__":
    unittest.main()
