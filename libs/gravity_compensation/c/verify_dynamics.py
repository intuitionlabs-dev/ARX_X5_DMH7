#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""验证 C 逆动力学与原始 Orocos KDL 的一致性。"""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import xml.etree.ElementTree as ET

SOURCE = Path(__file__).resolve().parent
PACKAGE = SOURCE.parent
sys.path.insert(0, str(PACKAGE.parent))
from gravity_compensation.solver import MODEL_FILES, GravityCompensator

Float6, Float3 = C.c_float * 6, C.c_float * 3


def vector(element, attribute):
    return list(map(float, element.get(attribute, "0 0 0").split())) if element is not None else [0.0]*3


def kdl_input(name, gravity, cases):
    root = ET.parse(PACKAGE / "models" / MODEL_FILES[name]).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    rows = []
    for i in range(1, 7):
        joint = joints[f"joint{i}"]
        origin = joint.find("origin")
        inertial = links[joint.find("child").get("link")].find("inertial")
        inertial_origin = inertial.find("origin")
        values = (vector(origin, "xyz") + vector(origin, "rpy")
                  + vector(joint.find("axis"), "xyz")
                  + [float(inertial.find("mass").get("value"))]
                  + vector(inertial_origin, "xyz") + vector(inertial_origin, "rpy")
                  + [float(inertial.find("inertia").get(key))
                     for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")])
        rows.append(" ".join(map(str, values)))
    rows.extend([" ".join(map(str, gravity)), str(len(cases))])
    rows.extend(" ".join(map(str, q + qd + qdd)) for q, qd, qdd in cases)
    return "\n".join(rows) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path("build/dynamics-host"))
    parser.add_argument("--reference", type=Path, default=Path("build/kdl-reference/kdl_dynamics_reference"))
    parser.add_argument("--samples", type=int, default=1000)
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("samples 必须大于零")
    build = args.build_dir.resolve()
    build.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(SOURCE / "generate_dynamics_data.py"), "--check"], check=True)
    library = build / "x5_dynamics.so"
    subprocess.run(["cc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Wdouble-promotion", "-Wfloat-conversion", "-pedantic", "-fPIC", "-shared",
                    str(SOURCE / "x5_dynamics.c"), str(SOURCE / "x5_gravity.c"),
                    "-lm", "-o", str(library)], check=True)
    lib = C.CDLL(str(library))
    ptr = C.POINTER(C.c_float)
    lib.x5_dynamics_compute.argtypes = [C.c_int, ptr, ptr, ptr, ptr, C.c_int, ptr]
    lib.x5_gravity_compute.argtypes = [C.c_int, ptr, ptr, C.c_int, ptr]

    def calculate(model, q, qd, qdd, gravity=(0, 0, 0), mode=0):
        out = Float6()
        g = None if gravity is None else Float3(*gravity)
        status = lib.x5_dynamics_compute(model, Float6(*q), Float6(*qd), Float6(*qdd), g, mode, out)
        assert status == 0, status
        return tuple(out)

    rng = random.Random(20261017)
    zero = (0.0,)*6
    poses = [zero, (math.pi/2,)*6, (-math.pi/2,)*6, (128.0,)*6, (-128.0,)*6]
    poses += [tuple(rng.uniform(-math.pi, math.pi) for _ in range(6)) for _ in range(args.samples)]
    cases = []
    for pose in poses:
        q = tuple(Float6(*pose))
        qd = tuple(Float6(*(rng.uniform(-3, 3) for _ in range(6))))
        qdd = tuple(Float6(*(rng.uniform(-70, 70) for _ in range(6))))
        cases.extend([(q, zero, zero), (q, qd, zero), (q, zero, qdd), (q, qd, qdd)])
    max_error = max_gravity_error = 0.0
    comparisons = 0
    for model, name in enumerate(MODEL_FILES):
        for gravity in ((0, 0, -9.81), (2, -3, -8), (0, 0, 0), (0, 0, 9.81)):
            gravity = tuple(Float3(*gravity))
            result = subprocess.run([str(args.reference.resolve())],
                input=kdl_input(name, gravity, cases), text=True, capture_output=True, check=True)
            expected_rows = [tuple(map(float, line.split())) for line in result.stdout.splitlines()]
            assert len(expected_rows) == len(cases)
            for (q, qd, qdd), expected in zip(cases, expected_rows):
                raw = calculate(model, q, qd, qdd, gravity)
                sdk = calculate(model, q, qd, qdd, gravity, 1)
                g = Float6()
                assert lib.x5_gravity_compute(model, Float6(*q), Float3(*gravity), 0, g) == 0
                for j, (actual, reference) in enumerate(zip(raw, expected)):
                    error = abs(actual-reference)
                    assert error < 5e-5 + 4e-6*abs(reference), (name, q, qd, qdd, j, error)
                    max_error = max(max_error, error)
                    assert abs(sdk[j] - (reference + ((0.8 if j < 3 else 1.32)-1)*g[j])) < 5e-5 + 4e-6*abs(reference)
                    if qd == zero and qdd == zero:
                        max_gravity_error = max(max_gravity_error, abs(actual-g[j]))
                        assert abs(actual-g[j]) < 1e-5
                comparisons += 2
        for q in poses[:20]:
            columns = [calculate(model, q, zero, tuple(float(i == j) for i in range(6)))
                       for j in range(6)]
            for i in range(6):
                for j in range(6):
                    assert abs(columns[j][i]-columns[i][j]) < 1e-6
            for _ in range(10):
                v = tuple(rng.uniform(-1, 1) for _ in range(6))
                assert sum(v[i]*columns[j][i]*v[j] for i in range(6) for j in range(6)) > 0
        assert calculate(model, zero, zero, zero, None) == calculate(model, zero, zero, zero, (0, 0, -9.81))

    baseline = Float6(*([123.0]*6))
    requests = [(-1, Float6(), Float6(), Float6(), None, 0),
                (3, Float6(), Float6(), Float6(), None, 0),
                (0, None, Float6(), Float6(), None, 0),
                (0, Float6(), None, Float6(), None, 0),
                (0, Float6(), Float6(), None, None, 0),
                (0, Float6(), Float6(), Float6(), None, 2),
                (0, Float6(), Float6(), Float6(), Float3(math.nan, 0, 0), 0)]
    for component in range(3):
        for value in (math.nan, math.inf, -math.inf):
            arrays = [Float6(), Float6(), Float6()]
            arrays[component][0] = value
            requests.append((0, *arrays, None, 0))
    requests += [(0, Float6(129, 0, 0, 0, 0, 0), Float6(), Float6(), None, 0),
                 (0, Float6(), Float6(*([3e38]*6)), Float6(), None, 0)]
    for request in requests:
        assert lib.x5_dynamics_compute(*request, baseline) != 0
        assert tuple(baseline) == (123.0,)*6
    assert lib.x5_dynamics_compute(0, Float6(), Float6(), Float6(), None, 0, None) != 0
    report = {"kdl_version": "1.5.1", "models": list(MODEL_FILES),
              "comparisons": comparisons, "max_error_nm": max_error,
              "max_static_error_nm": max_gravity_error,
              "mass_matrix_symmetry_and_positive_energy": True,
              "invalid_inputs_preserve_output": True, "hardware_tested": False}
    (build / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
