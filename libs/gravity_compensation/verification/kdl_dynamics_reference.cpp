// SPDX-License-Identifier: LGPL-2.1-or-later
// 使用原始 KDL RNE 和完整 URDF 惯量验证 C 实现。
#include "chainidsolver_recursive_newton_euler.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>

static double read_scalar() {
    double value;
    if (!(std::cin >> value)) throw std::runtime_error("输入数据不完整");
    return value;
}

static KDL::Vector read_vector() {
    const double x = read_scalar(), y = read_scalar(), z = read_scalar();
    return KDL::Vector(x, y, z);
}

static KDL::Rotation read_rotation() {
    const auto rpy = read_vector();
    return KDL::Rotation::RPY(rpy.x(), rpy.y(), rpy.z());
}

int main() {
    try {
        KDL::Chain chain;
        for (int i = 0; i < 6; ++i) {
            const auto xyz = read_vector();
            const auto rotation = read_rotation();
            const auto axis = read_vector();
            const double mass = read_scalar();
            const auto com = read_vector();
            const auto inertial_rotation = read_rotation();
            const double xx = read_scalar(), yy = read_scalar(), zz = read_scalar();
            const double xy = read_scalar(), xz = read_scalar(), yz = read_scalar();
            const KDL::RotationalInertia inertia(xx, yy, zz, xy, xz, yz);
            const auto rotated = inertial_rotation
                * KDL::RigidBodyInertia(0, KDL::Vector::Zero(), inertia);
            const KDL::Joint joint("joint" + std::to_string(i), xyz, rotation * axis,
                KDL::Joint::RotAxis);
            chain.addSegment(KDL::Segment("link" + std::to_string(i), joint,
                KDL::Frame(rotation, xyz),
                KDL::RigidBodyInertia(mass, com, rotated.getRotationalInertia())));
        }
        const auto gravity = read_vector();
        KDL::ChainIdSolver_RNE solver(chain, gravity);
        KDL::JntArray q(6), qd(6), qdd(6), tau(6);
        const KDL::Wrenches external(6, KDL::Wrench::Zero());
        const auto cases = static_cast<unsigned>(read_scalar());
        std::cout << std::setprecision(17);
        for (unsigned row = 0; row < cases; ++row) {
            for (auto* state : {&q, &qd, &qdd})
                for (unsigned j = 0; j < 6; ++j) (*state)(j) = read_scalar();
            if (solver.CartToJnt(q, qd, qdd, external, tau) != 0)
                throw std::runtime_error("KDL RNE 求解失败");
            for (unsigned j = 0; j < 6; ++j)
                std::cout << (j ? " " : "") << tau(j);
            std::cout << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
