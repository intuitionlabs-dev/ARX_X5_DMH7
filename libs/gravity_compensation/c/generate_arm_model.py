#!/usr/bin/env python3

"""从 URDF 生成串联旋转关节模型，并合并固定连杆惯量。"""
import argparse
import math
from pathlib import Path
import re
import sys

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parent))
from gravity_compensation.urdf import load_chain
from gravity_compensation.spatial import add, multiply, rotate, transpose
from gravity_compensation.c.generate_dynamics_data import initializer

IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
ZERO = (0, 0, 0)


def parallel(mass, point):
    squared = sum(v*v for v in point)
    return tuple(tuple(mass*((squared if i == j else 0)-point[i]*point[j])
                       for j in range(3)) for i in range(3))


def merge(row, mass, com, inertia):
    total = row[0] + mass
    center = tuple((row[0]*a+mass*b)/total for a, b in zip(row[1], com)) if total else ZERO
    pa, pb, pc = parallel(row[0], row[1]), parallel(mass, com), parallel(total, center)
    row[5] = tuple(tuple(row[5][i][j]+inertia[i][j]+pa[i][j]+pb[i][j]-pc[i][j]
                         for j in range(3)) for i in range(3))
    row[0], row[1] = total, center


def generate(path, base, tip, symbol):
    if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", symbol):
        raise ValueError("模型名称必须是 C 标识符")
    rows, rotation, origin = [], IDENTITY, ZERO
    for segment in load_chain(path, base, tip):
        origin = add(origin, rotate(rotation, segment.xyz))
        rotation = multiply(rotation, segment.rotation)
        if segment.joint_type == "fixed":
            if rows:
                com = add(origin, rotate(rotation, segment.com))
                inertia = multiply(multiply(rotation, segment.inertia), transpose(rotation))
                merge(rows[-1], segment.mass, com, inertia)
        elif segment.joint_type in ("revolute", "continuous"):
            rows.append([segment.mass, segment.com, origin, segment.axis, rotation, segment.inertia])
            rotation, origin = IDENTITY, ZERO
        else:
            raise ValueError("动力学模型要求旋转关节")
    if not rows:
        raise ValueError("动力学模型必须包含旋转关节")
    lines = [
             '/* URDF 连杆惯量，单位 kg、m、kg·m²。 */', '#pragma once',
             '#include "arm_dynamics.h"', f'static const Arm_link {symbol}[] = {{']
    for mass, com, xyz, axis, rotation, inertia in rows:
        if not all(math.isfinite(v) for v in (mass, *com, *xyz, *axis)):
            raise ValueError("模型数据必须是有限数")
        lines.append('    '+initializer((mass, com, xyz, axis, (rotation,), (inertia,)))+',')
    return '\n'.join(lines+['};', ''])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--base', default='base_link')
    parser.add_argument('--tip')
    parser.add_argument('--symbol', default='arm_links')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        for name, filename, tip in (('metal', 'metal_with_gripper.urdf', 'Link6'),
                                     ('x5', 'x5.urdf', 'link6')):
            (args.output_dir/(name+'_model.h')).write_text(
                generate(PACKAGE/'models'/filename, 'base_link', tip, name+'_links'))
    else:
        if not args.urdf or not args.tip or not args.output:
            parser.error('必须指定 --urdf、--tip 和 --output')
        args.output.write_text(generate(args.urdf, args.base, args.tip, args.symbol))


if __name__ == '__main__':
    main()
