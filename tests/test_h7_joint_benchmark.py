"""Native checks of the workload/statistics used by the DWT firmware test."""
from pathlib import Path
import subprocess
import tempfile
import unittest

from ruckig_test_support import ruckig_library
from test_arm_framework import FLAGS, INCLUDES, ROOT


class H7BenchmarkTests(unittest.TestCase):
    def test_workload_and_statistics(self):
        with tempfile.TemporaryDirectory(prefix="h7-dwt-test-") as directory:
            executable = Path(directory) / "test"
            sources = ["tests/c_h7_joint_benchmark.c", "App/gimbal_cmd.c",
                       "libs/controller/adrc.c", "libs/common/common.c",
                       "libs/move_plan/curves/quintic_spline.c"]
            result = subprocess.run([
                "cc", *FLAGS, "-DH7_JOINT_BENCHMARK=1", "-UNDEBUG",
                "-ffunction-sections", "-fdata-sections",
                *["-I" + str(ROOT / p) for p in INCLUDES + ["App/test", "App/profiles"]],
                *[str(ROOT / p) for p in sources], ruckig_library(),
                "-Wl,--gc-sections", "-lstdc++", "-lm", "-o", str(executable)
            ], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
