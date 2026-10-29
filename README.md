# H7 与 PC 控制工程

本仓库包含 H7 固件、PC 控制程序和 SDK。
各组件的文档位于对应目录。

## H7 控制框架

H7 使用统一的六轴加单电机夹爪控制流程。
`enum Arm_Motor_Index` 是轴数的唯一配置入口。
轴数必须等于 6，否则编译报错。
每台臂独立选择机型配置和 CAN 总线。
机型配置包含电机、控制参数、限位、夹爪映射和初始化策略。
CMake 在构建前从 URDF 生成动力学模型。

内置配置包括 metalarm 与 ARX-X5。
默认两台臂均使用 metalarm。
混合机型构建示例：

```bash
cmake --preset Debug -DH7_ARM_A_PROFILE=arx_x5 -DH7_ARM_B_PROFILE=metalarm
cmake --build --preset Debug
```

控制核心使用配置的关节数。
现有通信协议使用六关节和独立夹爪槽位。
协议转换集中在 `App/arm_protocol.c`。
适配接口和参数见 [H7 机械臂适配](App/profiles/README.md)。

## 组件与 SDK

| 组件 | 目录 | 文档入口 |
| --- | --- | --- |
| H7 固件 | `App/`、`Bsp/`、`Core/` | [通信协议](libs/frame/protocol_doc.md) |
| Metal PC 控制程序 | `sdk/metal_arm_control/` | [使用说明](sdk/metal_arm_control/README.md) |
| ARX X5 PC 控制程序 | `sdk/arx_x5_control/` | [使用说明](sdk/arx_x5_control/README.md) |
| SDK | `sdk/` | [使用说明](sdk/README.md)、[API](sdk/API.md)、[配置](sdk/config/README.md) |
| USB-CAN 传输子包 | 各 PC 控制包的 `usb2can/` | [Metal](sdk/metal_arm_control/usb2can/README.md)、[ARX X5](sdk/arx_x5_control/usb2can/README.md) |
| 动力学库 | `libs/gravity_compensation/` | [使用说明](libs/gravity_compensation/README.md) |

SDK 作为 Git 子模块管理。
在仓库根目录初始化：

```bash
git submodule update --init --recursive sdk
```

PC 控制程序的 SDK 接入说明位于
[Metal 接入说明](sdk/metal_arm_control/SDK_LOCAL.md)和 [ARX X5 接入说明](sdk/arx_x5_control/SDK_LOCAL.md)。

在 H7 工程根目录安装 PC 控制包：

```bash
python3 -m pip install './sdk[can]'
```

SDK 仓库包含独立构建配置、控制代码和动力学模型。
集成测试在安装后运行：

```bash
python3 -m unittest discover -s tests -v
```
