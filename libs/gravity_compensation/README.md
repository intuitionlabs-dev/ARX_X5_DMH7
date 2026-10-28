# 六轴重力与逆动力学计算

本目录提供 Metal 和 ARX X5 的离线力矩计算。H7 固件调用 Metal C 接口，PC 后端使用 SDK 中对应型号的 Python 实现。计算模块只处理关节空间动力学，电机通信、模式切换和输出限幅由控制层负责。

| 型号 | C 接口 | 模型 | 厂家补偿 |
| --- | --- | --- | --- |
| Metal | [metal_dynamics.h](c/metal_dynamics.h) | 带夹爪、无夹爪 | 重力缩放、科氏缩放、速度相关摩擦 |
| ARX X5 | [x5_dynamics.h](c/x5_dynamics.h)、[x5_gravity.h](c/x5_gravity.h) | 2023、master、2025 | 重力缩放 |

两种型号均基于递归牛顿欧拉算法。输入为六轴关节位置、速度、加速度，单位分别为 rad、rad/s、rad/s²；输出为六轴力矩，单位 N·m。基座重力默认为 `(0, 0, -9.81)` m/s²，倾斜安装时需设置基座坐标系中的重力向量。模型不包含用户额外安装的工具或负载。

## Metal H7 控制链

```text
命令层五次样条 → 参考位置 / 参考速度 / 参考加速度
                             ↓          ↓
关节反馈位置 ──────────→ Metal 逆动力学 → 力矩前馈
                             ↓
                         电机 MIT 控制
```

默认使用 `METAL_MODEL_WITH_GRIPPER`。夹爪作为末端负载计入六轴模型，夹爪开合通道不单独参与六轴逆动力学。无夹爪负载使用 `METAL_MODEL_NO_GRIPPER`；模型、补偿开关及基座重力在 [gimbal_arm_def.h](../../App/inc/gimbal_arm_def.h) 中配置。

| 模式 | 动力学输入 | 力矩输出 |
| --- | --- | --- |
| RUN | 反馈位置、命令层参考速度和加速度 | 惯性 + 缩放科氏 + 缩放重力 + 摩擦 |
| GRAVITY | 反馈位置，速度和加速度按零处理 | 缩放重力 |
| DAMPING / STOP | 不求解 | 动力学前馈为零 |

普通位置目标和自动复位使用五次样条的速度、加速度。`0x17` 位置与速度前馈包中的速度由上位机提供，参考加速度为零。求解失败时应用层清空补偿并进入 STOP；状态码保存在 `Gimbal.dynamics_status`。

Metal SDK 模式的力矩为：

```text
tau = M(q) * qdd_ref + sC * C(q, qd_ref) * qd_ref + sG * G(q) + F(qd_ref)
```

`sG`、`sC` 按轴逐项相乘；惯性项不缩放。摩擦项为系数乘参考速度，J3、J4 仅对摩擦计算中的速度限幅。

| 参数 | J1 | J2 | J3 | J4 | J5 | J6 |
| --- | --- | --- | --- | --- | --- | --- |
| 重力系数 `sG` | 1.2 | 1.15 | 1.1 | 1.15 | 1 | 1 |
| 科氏系数 `sC` | 1.1 | 1.15 | 1 | 1 | 1.1 | 1.1 |
| 摩擦系数 N·m·s/rad | 0.15 | 0.3025 | 0.52 | 0.42 | 0.015 | 0.015 |
| 摩擦速度绝对限值 rad/s | 不裁剪 | 不裁剪 | 0.8 | 1.2 | 不裁剪 | 不裁剪 |

参数集中在 [parameters.py](parameters.py)，与 SDK Metal 参数一致。C 表由本目录 URDF 和这些参数生成，运行时不解析 URDF。仓库根目录执行：

```bash
python3 libs/gravity_compensation/c/generate_dynamics_data.py --robot metal --check
# 修改模型或补偿参数后生成 C 表
python3 libs/gravity_compensation/c/generate_dynamics_data.py --robot metal
```

C API 的使用、错误码和编译要求见 [C 接口说明](c/README.md)。模型源文件及许可证见 [来源说明](PROVENANCE.md)。主机数值对照和固件构建不能替代实机跟踪、力矩方向及周期耗时验证。

## ARX X5 独立接口

本目录的 Python `GravityCompensator` 和 `DynamicsCompensator` 提供 X5 离线计算，型号为 `2023`、`master`、`2025`。运行时只依赖 Python 标准库：

```python
from libs.gravity_compensation import DynamicsCompensator

solver = DynamicsCompensator.from_model("2023")
torque = solver.torques((0, .4, .8, 0, 0, 0), (0,) * 6, (0, 1, 0, 0, 0, 0))
```

X5 SDK 模式只缩放重力项，J1～J3 为 0.8，J4～J6 为 1.32；`sdk_scale=False` 返回物理力矩。输入无效时抛出 `DynamicsError`，错误状态为 `INVALID_ARGUMENT`、`ANGLE_RANGE` 或 `NONFINITE_RESULT`。

X5 C 常量校验命令：

```bash
python3 libs/gravity_compensation/c/generate_dynamics_data.py --robot x5 --check
python3 libs/gravity_compensation/c/generate_model_data.py --check
```

`reference/orocos_kdl` 和 `verification` 提供独立的 KDL 数值参考。构建参考程序时需要 C++ 编译器和 Eigen，控制固件及 Python 离线计算不依赖这些构建产物。

## 来源与许可证

算法采用 LGPL-2.1-or-later，见 [LICENSE](LICENSE)。Metal 模型和厂家参数采用 MakerMods MIT 许可，见 [METAL_LICENSE](models/METAL_LICENSE)；X5 模型采用 ARXrobotics BSD-3-Clause 许可，见 [models/LICENSE](models/LICENSE)。完整来源及计算边界见 [PROVENANCE.md](PROVENANCE.md)。
