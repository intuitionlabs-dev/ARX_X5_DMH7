#!/usr/bin/env python3

"""从仓库内的 URDF 生成 X5 / Metal 连杆动力学常量。"""
import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parent))
from gravity_compensation.solver import MODEL_FILES
from gravity_compensation.spatial import multiply, rpy_rotation, transpose
from gravity_compensation.urdf import load_chain
from gravity_compensation import parameters as metal_parameters


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


def generate(robot="x5"):
    if robot not in ("x5", "metal"):
        raise ValueError("robot must be x5 or metal")
    models = (MODEL_FILES if robot == "x5" else
              {name: name + ".urdf" for name in ("metal_with_gripper", "metal_no_gripper")})
    guard = robot.upper() + "_DYNAMICS_DATA_H"
    lines = [
             "/* 由 generate_dynamics_data.py 从 URDF 生成；单位为 kg、m、kg·m²。 */",
             "#ifndef " + guard, "#define " + guard,
             f"static const link_data {robot}_dynamic_links[{len(models)}][6] = {{"]
    for name, filename in models.items():
        root = ET.parse(PACKAGE / "models" / filename).getroot()
        links = {link.get("name"): link for link in root.findall("link")}
        chain = load_chain(PACKAGE / "models" / filename,
                           tip="Link6" if robot == "metal" else "link6")
        if len(chain) != 6 or any(s.joint_type != "revolute" for s in chain):
            raise ValueError("动力学模型要求六个旋转关节")
        lines.append("    { /* " + name + " */")
        for segment in chain:
            inertia = inertia_in_link_frame(links[segment.link_name].find("inertial"))
            fields = (segment.mass, segment.com, segment.xyz, segment.axis,
                      (segment.rotation,), (inertia,))
            lines.append("        " + initializer(fields) + ",")
        lines.append("    },")
    lines.append("};")
    if robot == "metal":
        lines.append("/* Metal 补偿系数来自 parameters.py；摩擦速度限值 0 表示不裁剪。 */")
        for name in ("GRAVITY_SCALES", "CORIOLIS_SCALES", "FRICTION_COEFFICIENTS",
                     "FRICTION_SPEED_LIMITS"):
            values = getattr(metal_parameters, name)
            if len(values) != 6:
                raise ValueError(f"{name} 必须包含六个关节参数")
            values = tuple(0.0 if v is None else v for v in values)
            lines.append(f"static const metal_real {name.lower()}[6] = {initializer(values)};")
    return "\n".join(lines + ["#endif", ""])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("x5", "metal"), default="x5")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    destination = Path(__file__).with_name(args.robot + "_dynamics_data.h")
    text = generate(args.robot)
    if args.check:
        if destination.read_text() != text:
            raise SystemExit("动力学常量与 URDF 不一致")
        print(f"{args.robot} 动力学常量与 URDF、补偿参数一致")
    else:
        destination.write_text(text)
