# H7 与 PC 控制工程

本仓库包含 H7 固件、PC 控制程序和 SDK。
各组件的文档位于对应目录。

## H7 电机层

每台臂使用七个达妙 MIT 电机。`arm_id=0` 使用 CAN1，`arm_id=1` 使用 CAN2，各总线为 1 Mbps。
参数定义在 [gimbal_arm_def.h](App/inc/gimbal_arm_def.h)，电机创建及反馈回调位于 [gimbal_arm.c](App/gimbal_arm.c)。

| 电机 | TX / RX ID | Kp / Kd | 速度编码量程 rad/s | 力矩编码量程 N·m | 控制频率 |
| --- | --- | --- | --- | --- | --- |
| J1 | 1 / 0x11 | 200 / 3 | ±10 | ±30 | 500 Hz |
| J2 | 2 / 0x12 | 500 / 5 | ±10 | ±120 | 250 Hz |
| J3 | 3 / 0x13 | 400 / 5 | ±10 | ±30 | 250 Hz |
| J4 | 4 / 0x14 | 200 / 2 | ±10 | ±30 | 250 Hz |
| J5 | 5 / 0x15 | 20 / 0.1 | ±30 | ±20 | 250 Hz |
| J6 | 6 / 0x16 | 20 / 0.1 | ±30 | ±20 | 250 Hz |
| 夹爪 | 7 / 0x17 | 20 / 0.1 | ±30 | ±20 | 250 Hz |

位置编码量程为 ±6.28 rad，主机级联 PID 为零。电机通过分组错开发送，每条总线每毫秒两帧常规控制；周期使能另占发送名额。应用层位置、速度反馈以原始载荷的 65535 / 4095 刻度换算，目标角使用原始输出轴坐标。DM 驱动内部反馈字段使用驱动自身的刻度。

**当前 H7 电机层面向 Metal；命令层软限位、复位姿态、夹爪初始化流程及动力学模型为 X5 配置，完整的 Metal 整机运动控制尚未就绪。** PC 后端的独立功能见 SDK 文档。

## 组件与 SDK

| 组件 | 目录 | 文档入口 |
| --- | --- | --- |
| H7 固件 | `App/`、`Bsp/`、`Core/` | [通信协议](libs/frame/protocol_doc.md) |
| PC 控制程序 | `sdk/arx_x5_control/` | [使用说明](sdk/arx_x5_control/README.md) |
| SDK | `sdk/` | [使用说明](sdk/README.md)、[API](sdk/API.md)、[配置](sdk/config/README.md) |
| USB-CAN 传输子包 | `sdk/arx_x5_control/usb2can/` | [接口与配置](sdk/arx_x5_control/usb2can/README.md) |
| 动力学库 | `libs/gravity_compensation/` | [使用说明](libs/gravity_compensation/README.md) |

SDK 作为 Git 子模块管理。
在仓库根目录初始化：

```bash
git submodule update --init --recursive sdk
```

PC 控制程序的 SDK 接入说明位于
[sdk/arx_x5_control/SDK_LOCAL.md](sdk/arx_x5_control/SDK_LOCAL.md)。

在 H7 工程根目录安装 PC 控制包：

```bash
python3 -m pip install './sdk[can]'
```

SDK 仓库包含独立构建配置、控制代码和动力学模型。
集成测试在安装后运行：

```bash
python3 -m unittest discover -s tests -v
```
