# 通过 SDK 调用 PC Python 控制核心

`arx_x5_control.sdk_link.LocalLink` 实现了现有 `sdk.Link` 的调用接口。向 `sdk.Arm` 传入此对象后，`connect()`、初始化、模式切换、位置/速度控制、复位、设零、反馈读取及等待方法都会进入本机 `ControlSystem`。

```text
业务程序 → sdk.Arm（rad / rad/s / m）
                   ↓
              LocalLink（参数转换、命令队列、反馈分发）
                   ↓
              ControlSystem（状态机、轨迹、重力、电机控制）
                   ↓
              MotorIO（MotorRequest / MotorFeedback）
                   ↓
              后续同设备 Python 电机程序
```

SDK 的 `Arm`、单位换算和夹爪标定代码直接复用仓库固定的 `sdk` 子模块，提交为 `bdbfeb68f752c2141c9440a4913d4756b9647c05`。本次无需修改子模块。

## 运行完整示例

Python 3.10 及以上。在仓库根目录执行：

```bash
git submodule update --init --recursive sdk
python3 -m arx_x5_control.sdk_demo

# 本包所有运行日志切换为英文。
python3 -m arx_x5_control.sdk_demo --lang en
```

若 GitHub SSH 访问不可用，可用 HTTPS 初始化相同子模块：

```bash
git -c url.https://github.com/.insteadOf=git@github.com: submodule update --init --recursive sdk
```

示例使用 SDK 完成双臂握手、初始化、夹爪找零、位置/速度下发、重力模式、阻尼模式及复位，按实际时间运行约 10 秒。末尾输出 `sequence_complete: true`，随后交付停机请求。它使用 `SimulatedMotors` 测试电机，控制算法运行的是 `ControlSystem`。

安装包同时包含控制核心及固定版本 SDK，无需安装 pyserial：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/arx-x5-sdk-demo
```

从源码构建前必须初始化 `sdk` 子模块。安装 wheel 后不再依赖仓库路径。

## 现有 SDK 程序如何接入

原先使用 `sdk.Link()` 的位置改为 `LocalLink(motor_io)`，其他 `sdk.Arm` 调用保持一致。无硬件时显式选择 `LocalLink.simulated()`：

```python
import sdk
from arx_x5_control import ArmConfig, log
from arx_x5_control.sdk_link import LocalLink
from arx_x5_control.sdk_demo import wait_arms

# 单臂示例；不传 configurations 时默认双臂。
log.configure(language="zh")
with LocalLink.simulated(configurations=[ArmConfig()]) as link:
    arm = sdk.Arm(0, link)             # 也可使用 link.arm(0)
    arm.connect()                    # 打开本地链路、握手、等待反馈
    wait_arms(link, [arm], lambda a: a.robot_state == sdk.board_defs.STATE_WAIT_INIT)
    arm.init_arm()
    wait_arms(link, [arm], lambda a: a.robot_state == sdk.board_defs.STATE_NORMAL)
    arm.start()
    wait_arms(link, [arm], lambda a: a.mode == sdk.board_defs.MODE_RUN)
    arm.send_ctrl_pos_vel(
        arm.joint_angles,            # 6 个关节角，rad
        (0.0,) * 6,                   # 6 个关节速度，rad/s
        gripper_raw_deg=-20.0,        # 未标定夹爪时可显式使用原始 deg
    )
    arm.wait_feedback(timeout=1)
    log.write_data(arm.snapshot())
# 离开 with 时，控制核心生成并交付停机请求。
```

夹爪完成 SDK 的两点标定后，可以继续使用 `gripper_m=...`，由 `sdk.GripperCalibStore` 和 `sdk.GripperCalibration` 换算为电机角。未标定时，米制控制仍按 SDK 原有行为报错。`joint_angles`、`joint_velocities` 和 `snapshot()` 仍提供 SI 单位；`arm.feedback` 是 SDK 的原始 deg 数据视图。

双臂共用一个 `LocalLink`，每台臂使用独立 `sdk.Arm`，并分别保活。`Arm.close()` 不关闭外部传入的共享链路；统一使用 `link.close()` 或 `with` 管理生命周期。

## 电机侧只实现两个接口

通过 `LocalLink(motor_io, configurations=...)` 传入满足 `MotorIO` 协议的对象：

```python
from arx_x5_control import MotorFeedback, MotorRequest

class MyMotorIO:
    def poll_feedback(self, now_ms: int) -> tuple[MotorFeedback, ...]:
        # 从后续 Python 电机程序取走新反馈；没有新数据时返回空元组。
        # 不得重复返回旧反馈，也不得将旧反馈的采样时间改成 now_ms。
        raise NotImplementedError

    def write_motor_requests(self, requests: tuple[MotorRequest, ...]) -> None:
        # 顺序交付控制、启停、设零请求；无法接收时抛出异常。
        # 包括 close() 时发出的 SOFT_STOP 和 DISABLE。
        raise NotImplementedError
```

两个方法都必须非阻塞，在控制所属线程中依次执行。反馈关节电机角为 rad、速度为 rad/s，输出转矩为 N·m；字段定义见 [Python 接口文档](PC_API.md)。设备打开、设备关闭、请求交付和设零保护间隔由电机适配器负责。

后台模式中 `now_ms` 是从 `LocalLink` **对象构造时刻**起经过的毫秒数；电机采样时间必须转换到同一时钟。`reopen()` 保留适配器对象和时钟起点。不要使用 Unix 时间戳，也不要在重开时把适配器时间清零。

后端未实现 CAN、串口、USB、socket、共享内存或进程通信。它在内存中转换 `sdk.Arm` 生成的 payload 参数，不生成完整协议帧或计算 CRC；`inject_bytes()` 明确拒绝输入完整字节帧。后续通信可以全部放在 `MotorIO` 中。

## SDK 方法映射

| SDK 方法 | 控制核心输入或输出 |
| --- | --- |
| `connect()`、`send_version()`、`keepalive()` | 校验 SDK 版本，提交 `CONNECT`；重复握手刷新该臂保活时间 |
| `init_arm()` | `INITIALIZE`，保留位姿、在线和状态门槛 |
| `set_zero()` | `CALIBRATE`，完成事件转换为 `sdk.SetZeroAck` |
| `start()` / `stop()` / `damping()` / `gravity()` | 对应模式命令 |
| `send_ctrl_pos()` | `POSITION`，关节 rad 转 deg，保留夹爪标定换算 |
| `send_ctrl_pos_vel()` | `POSITION_VELOCITY`，速度 rad/s 转 deg/s |
| `reset()` | `RESET`，执行复位轨迹 |
| `feedback`、`snapshot()`、`wait_feedback()`、`wait_for()` | 从对应臂的 SDK 通道读取控制核心反馈 |
| `last_setzero_ack`、`wait_setzero_ack()` | 读取该臂本轮设零的状态、7 个电机结果及耗时 |
| `send_raw()` | 支持上述命令的 SDK payload；未知类型、非法长度、非有限数及版本不符直接报错 |

`force=True` 仅跳过 SDK 的客户端预检，控制核心的状态和电机条件仍然生效。发送成功表示进入命令队列，执行结果应通过反馈或事件确认。每台臂队列容量为 7，满时抛出 `sdk.LinkError`，不会覆盖未执行的命令。

## 调度、退出和故障

- `open()` 启动目标周期为 1 ms 的线程，命令每 2 ms 处理，反馈约每 4 ms 发布；PC Python 不保证硬实时。
- 不自动发送保活。每台臂需在 3 s 内收到有效命令，反馈输出本身不算保活。等待初始化或设零时也要持续发送 `keepalive()`；示例的 `wait_arms()` 已处理此事。
- `close()` 停止控制线程并交付所有电机的停机请求；`reopen()` 新建控制器，清除旧反馈、旧设零结果和未执行命令，需要重新握手与初始化。
- 适配器抛异常时，后端记录故障并尝试交付停机请求。通过 `check_health()`、后续发送或 `wait_feedback()` 可取得 `sdk.LinkError`；`stats["link_err"]` 可读原因。SDK 的 `wait_for()` 和 `wait_setzero_ack()` 沿用原有轮询行为，应用长时间等待时也应调用 `check_health()`。
- 若停机请求也交付失败，`close()` 会报错。若适配器阻塞，超过 2 秒仍未退出会报错，并拒绝在旧线程运行时再开控制线程。适配器必须遵守非阻塞约定。
- `link.stats` 增加漏拍、旧反馈、核心拒绝命令、各臂队列长度和事件覆盖计数。`n_tx/n_rx` 为逻辑消息数量；没有真实串口状态包，`link_status` 为 `None`，CRC 计数为零。
- `drain_events()` 取走最近的控制事件，最多保留 512 条，历史覆盖数量记录在 `event_overruns` 中。`last_result` 提供最近一次控制结果；后台运行期间不得直接修改 `controller`。

## 确定性测试与回放

设置 `background=False` 后，使用调用者提供的严格递增毫秒时钟推进：

```python
from arx_x5_control.sdk_link import LocalLink
from arx_x5_control import log

log.configure(language="zh")
with LocalLink.simulated(background=False) as link:
    arm = link.arm(0)
    arm.send_version()
    link.step(0)
    log.write_data(arm.snapshot())
    arm.init_arm()
    for now_ms in range(1, 8001):
        if now_ms % 500 == 0:
            arm.keepalive()
        link.step(now_ms)
    log.write_data(arm.snapshot())
```

手动模式下，SDK 的阻塞等待不会自动推进控制时钟。可先提交命令再调用 `step()`，或让另一个所属线程持续推进。时间必须在 `reopen()` 前后保持递增。

后端运行日志使用 `arx_x5_control.log`，通过 `log.configure(language="zh"/"en")` 切换全包语言；不改变 SDK 自己的语言设置。配置、输出目标和事件数据的区别见 [日志文档](LOGGING.md)。

## 验证

```bash
python3 -m unittest discover -s tests -v
python3 -m sdk.tui --selftest
python3 -m arx_x5_control.sdk_demo
```

新增集成测试通过真实 `sdk.Arm` 覆盖双臂初始化、模式和复位、位置/速度单位、米制夹爪标定、设零结果、反馈在线掩码、保活超时、非法命令、队列容量、线程故障及退出停机。SDK 控制台的 `ConsoleSession` 也可以传入 `LocalLink`；现有 `python -m sdk.tui` 命令行仍使用它原有的串口或模拟后端。

以上验证为软件和测试电机验证，尚未接入实机。
