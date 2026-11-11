"""Build the actual Ruckig C library once for host firmware tests."""
import atexit
from functools import lru_cache
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
JOINT_POST_PROCESS_DIR = "App/joint_post_process"
JOINT_POST_PROCESS_SOURCES = (
    f"{JOINT_POST_PROCESS_DIR}/joint_post_process.c",
    f"{JOINT_POST_PROCESS_DIR}/algorithms/ruckig_post_process.c",
    f"{JOINT_POST_PROCESS_DIR}/algorithms/quintic_post_process.c",
)


@lru_cache(maxsize=1)
def ruckig_library():
    folder = tempfile.TemporaryDirectory(prefix="h7-ruckig-")
    atexit.register(folder.cleanup)
    for command in (
        ["cmake", "-S", str(ROOT / "libs/ruckig"), "-B", folder.name,
         "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"],
        ["cmake", "--build", folder.name, "-j", "4"],
    ):
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
    return str(Path(folder.name) / "libruckig_c.a")
