#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""从仓库内的 X5 URDF 生成连杆动力学常量。"""
import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parent))
from gravity_compensation.solver import MODEL_FILES, GravityCompensator
from gravity_compensation.spatial import multiply, rpy_rotation, transpose


def inertia_in_link_frame(inertial):
    origin = inertial.find("origin")
    rpy = tuple(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else (0, 0, 0)
    values = inertial.find("inertia").attrib
    xx, yy, zz, xy, xz, yz = (float(values[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
    inertia = ((xx, xy, xz), (xy, yy, yz), (xz, yz, zz))
    rotation = rpy_rotation(rpy)
    return multiply(multiply(rotation, inertia), transpose(rotation))


def initializer(value):
    if isinstance(value, (tuple, list)):
        return "{" + ", ".join(initializer(x) for x in value) + "}"
    if not math.isfinite(value):
        raise ValueError("惯量参数必须为有限数")
    literal = format(value, ".17g")
    if "." not in literal and "e" not in literal:
        literal += ".0"
    return literal + "f"


def generate():
    lines = ["/* SPDX-License-Identifier: LGPL-2.1-or-later */",
             "/* 由 generate_dynamics_data.py 从 URDF 生成；单位为 kg、m、kg·m²。 */",
             "#ifndef X5_DYNAMICS_DATA_H", "#define X5_DYNAMICS_DATA_H",
             "static const link_data x5_dynamic_links[3][6] = {"]
    for name, filename in MODEL_FILES.items():
        root = ET.parse(PACKAGE / "models" / filename).getroot()
        links = {link.get("name"): link for link in root.findall("link")}
        chain = GravityCompensator.from_model(name).segments
        if len(chain) != 6 or any(s.joint_type != "revolute" for s in chain):
            raise ValueError("动力学模型要求六个旋转关节")
        lines.append("    { /* " + name + " */")
        for segment in chain:
            inertia = inertia_in_link_frame(links[segment.link_name].find("inertial"))
            fields = (segment.mass, segment.com, segment.xyz, segment.axis,
                      (segment.rotation,), (inertia,))
            lines.append("        " + initializer(fields) + ",")
        lines.append("    },")
    return "\n".join(lines + ["};", "#endif", ""])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    destination = Path(__file__).with_name("x5_dynamics_data.h")
    text = generate()
    if args.check:
        if destination.read_text() != text:
            raise SystemExit("动力学常量与 URDF 不一致")
        print("三种 X5 动力学模型常量与 URDF 一致")
    else:
        destination.write_text(text)
