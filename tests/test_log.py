"""验证包级日志语言切换、隔离、控制事件和命令行行为。"""

import ast
from contextlib import redirect_stdout
import io
import json
import logging
import os
from pathlib import Path
import re
from string import Formatter
import subprocess
import sys
import threading
import unittest

from arx_x5_control import Event, RobotState, ZeroResult, ZeroState, log
from arx_x5_control.log_messages import MESSAGES, REASON_KEYS

ROOT = Path(__file__).resolve().parents[1]


class PackageLogTests(unittest.TestCase):
    def setUp(self):
        previous = log.get_config()
        self.stream = io.StringIO()
        log.configure(language="zh", level="DEBUG", stream=self.stream)
        self.addCleanup(log.configure, language=previous.language, level=previous.level, stream=None)

    def test_cached_loggers_and_background_thread_follow_package_language(self):
        logger = log.get_logger("arx_x5_control.test.worker")
        ready, proceed, done = threading.Event(), threading.Event(), threading.Event()

        def worker():
            logger.info("sdk_demo.initializing")
            ready.set()
            if proceed.wait(2):
                logger.info("sdk_demo.initializing")
            done.set()

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        try:
            self.assertTrue(ready.wait(2))
            log.configure(language="en")
            proceed.set()
            self.assertTrue(done.wait(2))
        finally:
            proceed.set()
            thread.join(2)
        lines = self.stream.getvalue().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertIn("双臂握手完成", lines[0])
        self.assertIn("Both arms connected", lines[1])

    def test_namespace_config_and_handlers_do_not_affect_other_packages(self):
        import sdk.i18n
        before_sdk = sdk.i18n.get_lang()
        root = logging.getLogger()
        root_state = (root.level, tuple(root.handlers))
        other = logging.getLogger("another_independent_package")
        other_state = (other.level, tuple(other.handlers), other.propagate)
        log.configure(language="en", level="WARNING")
        self.assertEqual(sdk.i18n.get_lang(), before_sdk)
        self.assertEqual((root.level, tuple(root.handlers)), root_state)
        self.assertEqual((other.level, tuple(other.handlers), other.propagate), other_state)
        for name in ("sdk", "another_independent_package", "arx_x5_control_other", ""):
            with self.subTest(name=name), self.assertRaises(ValueError):
                log.get_logger(name)

    def test_reconfiguration_does_not_duplicate_logs_and_respects_level(self):
        logger = log.get_logger("arx_x5_control.test")
        for _ in range(5):
            log.configure(language="zh", level="WARNING", stream=self.stream)
        logger.info("runtime.started")
        logger.warning("event.watchdog", arm_id=1)
        lines = self.stream.getvalue().splitlines()
        self.assertEqual(len(lines), 1)
        self.assertIn("臂 1 命令超时", lines[0])
        log.configure(stream=None)
        logger.error("cli.failed", error="test")
        self.assertEqual(self.stream.getvalue().splitlines(), lines)

    def test_invalid_configuration_is_atomic(self):
        before = log.get_config()
        for kwargs in ({"language": "fr"}, {"level": "VERBOSE"}, {"level": 123}, {"stream": object()}):
            with self.subTest(kwargs=kwargs), self.assertRaises((TypeError, ValueError)):
                log.configure(**kwargs)
            self.assertEqual(log.get_config(), before)
        log.get_logger().info("runtime.started")
        self.assertIn("控制循环已启动", self.stream.getvalue())

    def test_catalog_keys_placeholders_and_event_reasons_are_complete(self):
        self.assertEqual(set(MESSAGES["zh"]), set(MESSAGES["en"]))
        formatter = Formatter()
        for key, chinese in MESSAGES["zh"].items():
            with self.subTest(key=key):
                english = MESSAGES["en"][key]
                zh_fields = {field for _, field, _, _ in formatter.parse(chinese) if field}
                en_fields = {field for _, field, _, _ in formatter.parse(english) if field}
                self.assertEqual(zh_fields, en_fields)
                self.assertFalse(re.search(r"[\u4e00-\u9fff]", english))
        # 核心保留原事件数据，保证每个固定拒绝/中止原因都有对应的翻译。
        for path in (ROOT / "sdk/arx_x5_control/arm.py", ROOT / "sdk/arx_x5_control/system.py"):
            for node in ast.walk(ast.parse(path.read_text())):
                if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                    continue
                if node.func.attr == "reject":
                    self.assertIn(node.args[2].value, REASON_KEYS)
                elif node.func.attr == "_abort_auto":
                    self.assertIn(node.args[1].value, REASON_KEYS)

    def test_control_events_translate_without_mutating_machine_data(self):
        logger = log.get_logger("arx_x5_control.test")
        events = [Event(0, 20, "state", state.name) for state in RobotState]
        events += [Event(0, 20, "rejected", "initialize: " + reason) for reason in REASON_KEYS]
        events += [Event(1, 30, kind) for kind in (
            "connected", "connection_required", "watchdog", "initialized",
            "initialization_failed", "reset_complete", "gravity_error", "shutdown")]
        events += [Event(1, 40, "auto_aborted", "motor feedback lost"),
                   Event(1, 40, "reference_error"),
                   Event(1, 40, "dynamics_error", "ANGLE_RANGE"),
                   Event(1, 40, "dynamics_error", "INVALID_ARGUMENT"),
                   Event(1, 40, "dynamics_error", "NONFINITE_RESULT"),
                   Event(1, 50, "calibrated", zero_state=ZeroState.DONE,
                         zero_results=tuple(ZeroResult), elapsed_ms=1234)]
        before = tuple(events)
        logger.events(events)
        chinese = self.stream.getvalue()
        self.assertIn("命令队列已满", chinese)
        self.assertIn("成功", chinese)
        self.assertNotIn("requires NORMAL", chinese)
        self.stream.seek(0)
        self.stream.truncate()
        log.configure(language="en")
        logger.events(events)
        english = self.stream.getvalue()
        self.assertIn("command queue full", english)
        self.assertIn("elapsed 1234 ms", english)
        self.assertFalse(re.search(r"[\u4e00-\u9fff]", english))
        self.assertEqual(tuple(events), before)

    def test_log_record_contains_package_key_language_and_original_event(self):
        records = []

        class Capture(logging.Handler):
            def emit(self, record):
                records.append(record)

        handler = Capture()
        owner = logging.getLogger("arx_x5_control")
        owner.addHandler(handler)
        self.addCleanup(owner.removeHandler, handler)
        event = Event(0, 0, "state", "WAIT_INIT")
        log.get_logger("arx_x5_control.test").event(event)
        record, = records
        self.assertEqual(record.package, "arx_x5_control")
        self.assertEqual(record.language, "zh")
        self.assertEqual(record.message_key, "event.state")
        self.assertIs(record.control_event, event)

    def test_json_data_and_existing_demo_emit_callback_remain_usable(self):
        from arx_x5_control.__main__ import demo
        lines = []
        result = demo(seconds=0.006, emit=lines.append)
        decoded = [json.loads(line) for line in lines]
        self.assertEqual(decoded[-1], result)
        self.assertTrue(any(item.get("kind") == "connected" for item in decoded[:-1]))
        output = io.StringIO()
        with redirect_stdout(output):
            log.write_data({"state": RobotState.NORMAL, "sequence_complete": True})
        self.assertEqual(json.loads(output.getvalue()), {"state": 4, "sequence_complete": True})

    def test_all_runtime_output_uses_package_log(self):
        # 防止后续代码绕过专属日志库使用 print、标准 logging 或直接写终端。
        for path in (ROOT / "sdk/arx_x5_control").glob("*.py"):
            if path.name == "log.py":
                continue
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                with self.subTest(file=path.name, line=getattr(node, "lineno", 0)):
                    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                        self.assertNotIn(node.func.id, ("print",))
                    if isinstance(node, ast.Import):
                        self.assertFalse(any(n.name in ("logging", "warnings") for n in node.names))
                    if isinstance(node, ast.ImportFrom):
                        self.assertNotIn(node.module, ("logging", "warnings"))
                    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
                        self.assertFalse(node.value.id == "sys" and node.attr in ("stdout", "stderr"))


class LogCLITests(unittest.TestCase):
    def run_cli(self, module="arx_x5_control", args=(), **env):
        environment = dict(os.environ)
        environment.pop(log.LANGUAGE_ENV, None)
        environment.pop(log.LEVEL_ENV, None)
        environment.update(env)
        return subprocess.run([sys.executable, "-m", module, *args], cwd=ROOT,
                              env=environment, capture_output=True, text=True, timeout=10)

    def test_cli_defaults_to_chinese_and_supports_environment_and_override(self):
        for args, env, expected in (
            ((), {}, "等待初始化"),
            ((), {log.LANGUAGE_ENV: "en"}, "awaiting initialization"),
            (("--lang", "zh"), {log.LANGUAGE_ENV: "en"}, "等待初始化"),
        ):
            with self.subTest(args=args, env=env):
                result = self.run_cli(args=("--seconds", "0.006", *args), **env)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(expected, result.stderr)
                self.assertFalse(json.loads(result.stdout)["sequence_complete"])
        result = self.run_cli(args=("--seconds", "0.006", "--log-level", "ERROR"))
        self.assertEqual(result.stderr, "")
        self.assertEqual(result.returncode, 0)

    def test_cli_errors_and_sdk_entry_help_use_log_language(self):
        result = self.run_cli(args=("--lang", "en", "--seconds", "-1"))
        self.assertEqual(result.returncode, 2)
        self.assertIn("Invalid command-line arguments", result.stderr)
        self.assertNotIn("Traceback", result.stderr)
        result = self.run_cli("arx_x5_control.sdk_adapter.demo", ("--lang", "en", "--help"))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("local control core", result.stdout)
        self.assertNotRegex(result.stdout, r"[\u4e00-\u9fff]")

    def test_import_has_no_runtime_output(self):
        result = subprocess.run([sys.executable, "-c",
                                 "import arx_x5_control.log; import arx_x5_control.sdk_adapter.link"],
                                cwd=ROOT, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((result.stdout, result.stderr), ("", ""))


if __name__ == "__main__":
    unittest.main()
