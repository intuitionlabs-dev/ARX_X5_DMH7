# ARX X5 H7 与 PC Python 控制程序

本仓保留原 H7 固件，并提供 `arx_x5_control`：将 H7 **实际接入机械臂运行链路**的控制逻辑移植为 PC Python 程序。源码基线为 [`21b97f9`](https://github.com/Crystal-1438/ARX_X5_DMH7/tree/21b97f9)。

Python 版本支持双臂，每臂 6 关节及 1 夹爪，包含总模式状态机、初始化、复位轨迹、夹爪找零、电机零点校准、PID、TD、静态重力补偿、速度/粘滞前馈、软限位及掉线处理。

**不含遥控器逻辑。通信仅定义接口**：上层输入 `Command`，电机输入 `MotorFeedback`，输出 `MotorRequest`、`ArmFeedback` 和 `Event`。新增 `LocalLink` 可将现有 `sdk.Arm` API 直接接入控制核心；电机通信仍由后续同设备 Python 程序通过 `MotorIO` 对接，本地后端不实现 CAN、UART、USB、完整字节帧、CRC、socket 或进程间通信。

完整的源文件对应关系、原参数和行为修正见 [迁移审计](docs/PC_PORT.md)，字段及调用契约见 [Python 接口](docs/PC_API.md)，现有 SDK 程序的接入方法见 [SDK 本地后端](docs/SDK_LOCAL.md)。

## 运行

Python 3.10 或更高版本；运行时只需要 Python 标准库。直接在仓库根目录执行：

```bash
python3 -m arx_x5_control --demo
```

默认执行 12 秒虚拟时间的双臂模拟：握手、初始化、夹爪找零、位置运动、重力模式、阻尼模式、复位。实际执行速度通常快于虚拟时间。结果 JSON 中的 `sequence_complete: true` 表示整个示例结束。

```bash
# 按墙钟时间播放同一模拟过程
python3 -m arx_x5_control --demo --realtime --seconds 12

# 可选：安装为 Python 包
# 安装包包含 SDK，先初始化仓库中固定版本的子模块。
git submodule update --init --recursive sdk
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/arx-x5-control --demo

# 单元测试、故障测试与原 C 算法对比
python3 -m unittest discover -s tests -v
```

示例只使用内存中的测试电机模型，不访问硬件。模拟器用于验证状态与接口，未模拟真实惯性、摩擦、负载或稳定性。

## SDK 接入

```bash
# 初始化 sdk 子模块后，运行 SDK 驱动的双臂示例。
python3 -m arx_x5_control.sdk_demo
```

```python
import sdk
from arx_x5_control import log
from arx_x5_control.sdk_link import LocalLink

log.configure(language="zh")
with LocalLink.simulated() as link:  # 实际电机程序改为 LocalLink(motor_io)
    arm = sdk.Arm(0, link)
    arm.connect()
    log.write_data(arm.snapshot())
```

现有的 `init_arm()`、`send_ctrl_pos_vel()`、模式切换、复位、设零和反馈接口继续使用 `sdk.Arm`；SDK 关节单位为 rad/rad/s，夹爪标定后使用 m。完整初始化及保活示例、`MotorIO` 两个接口、退出和异常处理见 [SDK 接入文档](docs/SDK_LOCAL.md)。

## 集成入口

```python
from arx_x5_control import (
    ArmConfig, Command, CommandKind, ControlSystem, MotorFeedback, log,
)

controller = ControlSystem([ArmConfig()])  # 单臂；不传配置时默认双臂

# MotorFeedback 的角度为 rad，速度为 rad/s；这里模拟初始反馈。
feedback = tuple(
    MotorFeedback(arm_id=0, motor_index=i, timestamp_ms=0,
                  position=0.0, velocity=0.0, temperature=25.0)
    for i in range(7)
)
result = controller.step(
    0,
    commands=[Command(0, CommandKind.CONNECT)],
    motor_feedback=feedback,
)

# 后续通信程序负责交付 result.motor_requests，回传下一批真实反馈。
log.write_data({"state": result.feedback[0].state.name})  # WAIT_INIT

# shutdown 返回的停机请求也必须由通信程序交付。
final_requests = controller.shutdown().motor_requests
```

`ControlSystem.step()` 是确定性控制入口，应由一个线程以约 1 ms 周期调用。支持注入时间，便于回放和测试。`runtime.run()` 提供使用 `time.monotonic_ns()` 的循环，并通过 [ControlIO](arx_x5_control/interfaces.py) 调用通信适配器。没有绑定任何传输方式。

上层关节角使用 **deg**，速度使用 **deg/s**；电机接口使用 **rad、rad/s、N·m**。两种坐标单位不能混用。正常使能顺序是 `CONNECT → WAIT_INIT → INITIALIZE → NORMAL/DAMPING → START → POSITION`。初始化需要全部电机在线，且六个关节距复位角各不超过 20°。

## 包专属日志和语言

`arx_x5_control` 内的运行日志统一使用自己的 `log` 库，默认中文；整个包的已有日志对象和后台线程可通过一处配置切换语言：

```python
from arx_x5_control import log
log.configure(language="zh", level="INFO")
log.configure(language="en")  # 下一条日志开始使用英文
```

命令行使用 `--lang zh/en`、`--log-level INFO`，或设置 `ARX_X5_CONTROL_LOG_LANGUAGE`、`ARX_X5_CONTROL_LOG_LEVEL`。人类可读日志写入标准错误流，最终 JSON 结果写入标准输出。独立包分别拥有自己的日志库与配置，本次配置不会影响 SDK 或其他包。完整约定见 [日志文档](docs/LOGGING.md)。

## 验证范围

57 项测试覆盖双臂初始化及复位、校准反馈/ACK、超时、掉线、模式切换、限位、发送频率、输入验证与退出停机，以及 SDK 接入和包级日志语言切换。原 C 算法对比会直接编译仓库的 PID、TD、样条与重力源码；需要主机 C 编译器 `cc`，没有编译器时仅这 4 项测试跳过。测试 SDK 接入前需初始化 `sdk` 子模块。

程序已通过本地模拟和独立安装包验证。尚未接入实机。普通 PC Python 不保证硬实时；漏拍计数可读，调度延迟不会通过突发重放旧控制周期来补偿。保留的控制增益来自 H7，实机验证需要使用实际 PC 调度和通信延迟。

## 文件

| 位置 | 用途 |
| --- | --- |
| `arx_x5_control/system.py` | 单线程调度、双臂隔离、输入输出与退出 |
| `arx_x5_control/arm.py` | 总模式状态机、坐标转换、重力与电机参考量 |
| `arx_x5_control/command.py` | 软限位、TD、复位、夹爪找零 |
| `arx_x5_control/motor.py` | 电机反馈、PID 级联、限幅、启停及更新周期 |
| `arx_x5_control/calibration.py` | ENCOS/达妙设零状态机 |
| `arx_x5_control/numerics.py`、`trajectory.py` | 数值算法与多轴轨迹 |
| `arx_x5_control/config.py` | 标定量及 H7 默认参数 |
| `arx_x5_control/types.py`、`interfaces.py` | 与后续通信程序对接的类型及协议接口 |
| `arx_x5_control/runtime.py` | PC 单调时钟循环 |
| `arx_x5_control/log.py`、`log_messages.py` | 本包专属日志、全包配置及中英文词条 |
| `arx_x5_control/sdk_link.py`、`sdk_demo.py` | SDK 本地后端及完整双臂示例 |
| `arx_x5_control/simulation.py`、`__main__.py` | 无硬件模拟及命令行入口 |
| `libs/gravity_compensation/` | 复用仓库已有纯 Python 重力模型与 URDF |
| `tests/` | 行为、故障、C/Python 数值对比测试 |

重力模块及模型沿用各自原许可证和来源文件，安装包包含对应许可证。没有修改 H7 固件源码和 SDK 子模块；SDK 接入层复用固定版本的 SDK，控制核心及原始 `--demo` 入口仍可独立运行。
