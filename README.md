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

## H7 软限位与初始化

软限位定义在 [gimbal_cmd_def.h](App/inc/gimbal_cmd_def.h)，与 SDK 的 Metal 参数一致。已初始化电机的命令角度按下表限幅。

| 轴 | 下限（deg） | 上限（deg） | 复位目标（deg） |
| --- | --- | --- | --- |
| J1 | -160 | 160 | 0 |
| J2 | -180 | 0 | 0 |
| J3 | 0 | 180 | 0 |
| J4 | -123 | 81 | 0 |
| J5 | -85 | 85 | 0 |
| J6 | -145 | 145 | 0 |
| 夹爪 | 0 | 约 137.50987（2.4 rad） | 保持已有目标；初始化时取入口反馈角 |

1. `0x02 ARM_INIT` 在七个电机在线、遥控器离线且六轴距全零姿态均不超过 20° 时受理；入口为待初始化状态，或正常运行的 STOP / DAMPING 模式。
2. 初始化记录本台夹爪的当前反馈角，六轴执行 2 秒五次样条复位。夹爪保持入口角度；电机零点保持厂家配置。
3. 复位结束后夹爪标记为已初始化，进入正常运行的 DAMPING 模式。遥控器接管或电机掉线会中止初始化，返回入口状态。
4. `0x18 ARM_RESET` 在正常运行时执行相同的六轴复位，保持已有夹爪目标，完成后进入 DAMPING；入口不要求在全零姿态附近。

**当前 H7 动力学仍使用 X5 模型，完整的 Metal 整机运动控制尚未就绪。** PC 后端的独立功能见 SDK 文档。

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
