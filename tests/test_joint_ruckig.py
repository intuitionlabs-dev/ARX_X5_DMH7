"""Native tests for ordinary joint position control (no SDK required)."""
from pathlib import Path
import subprocess
import tempfile
import unittest

from ruckig_test_support import ruckig_library
from test_arm_framework import FLAGS, INCLUDES, ROOT


class JointRuckigTests(unittest.TestCase):
    def test_firmware_reference_transitions_and_constraints(self):
        with tempfile.TemporaryDirectory(prefix="h7-joint-ruckig-") as temp:
            executable = Path(temp) / "joint-control"
            sources = ["tests/c_ruckig_joint_control.c", "libs/common/common.c"]
            result = subprocess.run([
                "cc", *FLAGS, "-ffunction-sections", "-fdata-sections", "-UNDEBUG",
                *["-I" + str(ROOT / p) for p in INCLUDES],
                *[str(ROOT / p) for p in sources], ruckig_library(),
                "-Wl,--gc-sections,--wrap=malloc,--wrap=calloc,--wrap=realloc,--wrap=free,--wrap=ruckig_c_update",
                "-lstdc++", "-lm", "-o", str(executable)
            ], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
