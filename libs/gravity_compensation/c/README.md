# C 重力与逆动力学接口

计算接口使用 C99 和单精度 `float`，输入关节位置、速度、加速度，输出关节力矩。求解过程使用栈和只读常量，不分配堆内存，不读写电机或通信设备。H7 固件使用 Metal 接口。

## Metal

| 文件 | 作用 |
| --- | --- |
| [metal_dynamics.h](metal_dynamics.h) | 类型、型号、错误码、计算接口 |
| [metal_dynamics.c](metal_dynamics.c) | 重力及完整逆动力学求解 |
| [metal_dynamics_data.h](metal_dynamics_data.h) | 两种负载模型的质量、几何、惯量与补偿系数 |

将 `metal_dynamics.c` 加入工程并链接数学库 `libm`，头文件放在包含路径中。固件运行不需要 Python、ROS、KDL 或 Eigen。

```c
#include <stddef.h>
#include "metal_dynamics.h"

int main(void) {
    float q[6] = {0.0f, -0.4f, 0.8f, 0.0f, 0.0f, 0.0f};
    float qd[6] = {0.0f};
    float qdd[6] = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float torque[6];

    int status = metal_dynamics_compute(METAL_MODEL_WITH_GRIPPER,
        q, qd, qdd, NULL, METAL_GRAVITY_SDK, torque);
    if (status != METAL_DYNAMICS_OK) return 1;
    /* torque 为 J1～J6 的力矩前馈，单位 N·m。 */
    return 0;
}
```

| 参数 | 约定 |
| --- | --- |
| `model` | `METAL_MODEL_WITH_GRIPPER` 或 `METAL_MODEL_NO_GRIPPER` |
| `q[6]` | J1～J6 反馈位置，rad |
| `qd[6]` / `qdd[6]` | 参考速度 rad/s / 参考加速度 rad/s² |
| `gravity_base[3]` | 基座重力 m/s²；`NULL` 表示 `(0, 0, -9.81)` |
| `mode` | `METAL_GRAVITY_PHYSICAL` 或 `METAL_GRAVITY_SDK` |
| `torque[6]` | 输出 N·m，与输入数组分开分配 |

物理模式输出 `M(q)qdd + C(q,qd)qd + G(q)`。SDK 模式分别缩放重力和科氏项，并叠加摩擦补偿，惯性项保持物理值。具体系数见 [动力学参数表](../README.md)。

`metal_gravity_compute(model, q, gravity_base, mode, torque)` 提供静态重力接口，等价于速度和加速度均为零的逆动力学。SDK 模式下此接口只产生缩放后的重力项。

| 返回码 | 含义 |
| --- | --- |
| `METAL_DYNAMICS_OK`，0 | 输出有效 |
| `METAL_DYNAMICS_INVALID_ARGUMENT`，-1 | 型号、模式、指针或有限数检查失败 |
| `METAL_DYNAMICS_ANGLE_RANGE`，-2 | 任一位置的绝对值超过 128 rad |
| `METAL_DYNAMICS_NONFINITE_RESULT`，-3 | 计算结果不是有限数 |

失败时输出数组保持原值，调用者须检查状态码。H7 应用层会清空前馈并进入 STOP。128 rad 是求解器数值范围，机械臂软限位由命令层独立执行。

## 模型和参数生成

模型链为 `base_link → Link6`，均包含六个旋转关节。夹爪负载由 Link6 的惯性参数表示；不单独求解夹爪开合。固定旋转、轴向、质心和惯量直接来自 URDF。

在仓库根目录执行：

```bash
# 检查 C 表与本地 URDF、parameters.py 是否一致
python3 libs/gravity_compensation/c/generate_dynamics_data.py --robot metal --check

# 修改模型或 parameters.py 后重新生成
python3 libs/gravity_compensation/c/generate_dynamics_data.py --robot metal
```

生成器仅依赖 Python 标准库和本目录的解析代码。模型和参数不从网络读取，也不修改 SDK。

## H7 集成

[gimbal_arm_def.h](../../../App/inc/gimbal_arm_def.h) 提供编译期配置：

| 配置 | 默认值 |
| --- | --- |
| `JOINT_DYNAMICS_COMP_ENABLE` | 1 |
| `JOINT_DYNAMICS_COMP_MODEL` | `METAL_MODEL_WITH_GRIPPER` |
| `JOINT_DYNAMICS_GRAVITY_MODE` | `METAL_GRAVITY_SDK` |
| `JOINT_DYNAMICS_GRAVITY_BASE` | `NULL` |

[gimbal_arm.c](../../../App/gimbal_arm.c) 将关节角度从 deg 转为 rad，将命令层参考速度、加速度转为 rad/s、rad/s²。RUN 使用完整前馈，GRAVITY 使用静态重力，STOP / DAMPING 的动力学前馈为零。普通位置目标和自动复位使用五次样条导数；`0x17` 使用外部速度，参考加速度为零。

计算频率由 CmdTask 决定，为 500 Hz。CAN 命令发送频率为 J1 500 Hz、其他电机 250 Hz，两者分别调度。编译时应保持 NaN / Inf 检查有效，不启用 `-ffast-math`。MCU 上的实际周期耗时需结合任务调度测量。

## ARX X5 独立接口

| 功能 | 源文件 | 模型 |
| --- | --- | --- |
| 静态重力 | `x5_gravity.c`、`x5_gravity_data.h` | 2023 / master / 2025 |
| 完整逆动力学 | `x5_dynamics.c`、`x5_dynamics_data.h`，同时链接 `x5_gravity.c` | 2023 / master / 2025 |
| 厂家电机参数 | `x5_sdk_parameters.c` | 见 [参数说明](../sdk_parameters/README.md) |

接口分别见 [x5_gravity.h](x5_gravity.h)、[x5_dynamics.h](x5_dynamics.h)。X5 的 SDK 模式仅对重力项施加 `[0.8, 0.8, 0.8, 1.32, 1.32, 1.32]` 系数。静态重力入口无需 libm，完整逆动力学需要链接 libm。

算法许可证见 [LICENSE](../LICENSE)，模型和厂家参数的来源及许可证见 [PROVENANCE.md](../PROVENANCE.md)。
