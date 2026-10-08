# 上位机 SDK 手册

通过串口 / USB 控制 CtrBoard-H7 上的 ARX X5 双臂。**先看第 5 节和第 10 节** —— 板子的闸门
是**静默丢弃**的，不知道这件事会花掉你一下午。

- 协议版本：`Intuition-ARX-X5-2023-v0.2`（版本码 `0xF91DD3C9`）。板子只认这一个码，
  对不上就**永久锁死、一帧不回**（要复位板子），所以别手改。
- 一台板子两块板卡、**两条 CAN 总线、两台臂**，但上位机只有**一条链路**（USB 或 RS485）。
- 单位：**对外全部 SI**（关节 rad / rad·s⁻¹、夹爪 m、温度 ℃）。线上是 deg，SDK 替你换。

## 1. 安装

Python 3.8+，跑真机只需要 pyserial：

```bash
pip install -r sdk/requirements.txt      # 就一行 pyserial>=3.5
```

`import sdk`、协议层、离线自测**不 import pyserial**（只有 `Link.open()` 才 import），
所以没有它的机器照样能跑自测、也能写只碰协议层的代码。仓库没有打包脚手架，
从**仓库根目录** `import sdk` 即可。

## 2. 快速开始

```python
import sdk, time

arm = sdk.Arm(0)                 # arm_id 0 = CAN1 上的 A 臂，1 = CAN2 上的 B 臂
arm.connect()                    # 开串口 → 发版本包 → 等这一台的第一帧反馈
print(arm.robot_state_name, arm.joint_angles)    # rad

arm.wait_for(lambda a: a.robot_state == 2)       # 等到「待初始化」
arm.init_arm()                                   # 0x02：复位 + 夹爪找零（先看第 5 节的闸门！）
arm.wait_for(lambda a: a.robot_state == 4)
arm.start()                                      # 进 run 档，控制帧只在 run 档被接受

while True:
    arm.send_ctrl_pos(arm.joint_angles, gripper_m=0.0)   # 照抄当前角度、夹爪闭合
    time.sleep(0.01)                                     # 这一帧本身就是保活
```

两台臂（**必须共享一个 `Link`**）：

```python
link = sdk.Link(); link.open()
a, b = sdk.Arm(0, link), sdk.Arm(1, link)
a.connect(); b.connect()
```

帧接口一个不落都在 `Arm` 上，签名和闸门见第 7 节。

## 3. 单位约定

| 量 | SDK 对外 | 线上 |
|---|---|---|
| 关节角 | **rad** | deg |
| 关节角速度 | **rad/s** | deg/s |
| 夹爪开口 | **m**（要标定，见第 6 节） | — |
| 夹爪原始角 | `raw_gripper_angle_deg` / `gripper_raw_deg=` | deg |
| 温度 | ℃ | ℃（uint8） |

**原始夹爪角是唯一的例外**：它是**电机的原始角**，双向都不做 `pos_to_angle` 换算
（那几张表只有 6 个关节有），所以想要"米"就只能标定。`snapshot()` 是**同一帧**里换算好的
一致视图；单独读 `arm.joint_angles` 再读 `arm.mode` 是两次读，中间可能夹进一帧新的。

## 4. 会话流程

```
上电 ──> 等待握手(0) ──版本码对上──> 离线等待(1) ──> 待初始化(2)
                                        ↑                  │ 0x02
            通信离线(5) ←──500ms无帧──┐  │                  ↓
                                        │  └────────── 初始化中(3)
             正常运行(4) ──0x18──> 复位中(7)                  │
                 ↑  ↑                                          ↓
              run/damping/stop/gravity(4 个 Arm_mode)  ←── 正常运行(4)
```

- **只有收到第一帧 0x80 才算握手成功**（板子握手前一帧不发）—— `connect()` 等的就是它。
- **上电后的常规路径**：`state 0 → 1 → 2`。要到 `2` 才能 `init_arm()`。
- `0x02` 跑完 → `正常运行(4)`，夹爪也顺便找完零。
- **链路断了不要紧**（拔线、串口崩）：板子 500 ms 收不到有效帧 → `通信离线(5)` + 切阻尼；
  链路回来后**落到 `离线等待(1)`**，也就是**要重新 `init_arm()`**，不会自己回到 4。

## 5. 闸门：板子默默丢弃你的帧

**板子对所有命令的回绝方式都是"当没收到"** —— 没有 NACK、没有错误码、`send_*` 也照样返回
成功。`Arm` 的每个命令方法在发包**之前**做同样的预检，不通过就抛 `GateRejected` 并说清缺
哪一条；`force=True` 跳过预检（**只跳过客户端预检，跳不过板子**）。预检先看反馈帧的年纪，
超过 100 ms 直接抛 `FeedbackStale`，绝不拿旧状态猜。

| 命令 | 板子侧闸门（`App/robot_state.c`） |
|---|---|
| `init_arm()` `0x02` | `state == 待初始化(2)` **或** (`state == 正常运行(4)` 且 `mode ∈ {stop, damping}`)，**且** 7 台电机全在线，**且** 6 关节（不含夹爪）与复位位姿偏差 ≤ 20° |
| `set_zero()` `0x10` | `state == 待初始化(2)` |
| `start/gravity/stop/damping` `0x12..0x15` | `state == 正常运行(4)` |
| `send_ctrl_pos/pos_vel` `0x16/0x17` | `state == 正常运行(4)` **且** `mode == run(1)` |
| `reset()` `0x18` | `state == 正常运行(4)` **且** 7 台全在线（**故意不查姿态** —— 臂歪着才要收） |

**上面每一条还都要求「遥控器离线」。** 而遥控器在线状态**不在任何反馈帧里**，所以
**客户端预检查不到这一条** —— 唯一的表现是"发了没反应"或者"`mode` 被遥控器覆盖掉了"。
遥控器在你手上时，先把遥控器关掉。

`0x02` 的复位位姿是 `board_defs.RESET_POSE_DEG = (0, 0.5, 0.5, 0, 0, 0)` 度，
容差 `ROBOT_INIT_POSE_TOL_DEG = 20`。用 `arm.init_arm()` 之前自己看一眼 `arm.joint_angles`。

## 6. 夹爪标定

夹爪是 **DM4310、CAN_ID 8、MIT 模式**。线上的 `gripper_angle` 是**电机原始角（deg）**，
而**仓库里没有任何"多少度 = 多少毫米"的规格**，所以米数只能实测：

```bash
python3 -m sdk.calibrate_gripper --arm-id 0 --full-open-m 0.085
```

工具会连上、显示状态，然后 `d` 切阻尼（或 `x` 失能）**用手把夹爪掰到两端**，各按一次键记
下来，回车存进 `sdk/config/gripper_calib.json`：

- `FULL_OPEN` 是**你预先配置的参数**（完全张开时的开口距离，米，>0），代码里不写死量程。
- `0` 记「这里 = 0 m」，`f` 记「这里 = FULL_OPEN」，之后按这两点线性映射。
  **方向和尺度完全由这两个点决定**，工具不预设哪端是张开 —— 装反了也不用改代码。
- 也可以用 `r` 进 run 档后 `←`/`→` **点动**（按原始角的方向，不是"张开/闭合"的方向），
  但 run 档整条臂都使能着，手不好够的时候再用。

**必须在一次成功的初始化之后标定**：夹爪的零点是行程中点（∓142°），而那个零点只有找零
才会建立 —— 在那之前原始角的零点是没有含义的。工具会查 `motor_inited` 里夹爪那一位并在
开头警告。**之后再跑任何 `0x10` 设零或再找零，零点会挪，标定作废要重标。**

没标定的时候 `arm.gripper_opening` **抛 `GripperNotCalibrated`**（不是返回一个猜的数），
而 `arm.raw_gripper_angle_deg` / `gripper_raw_deg=` 一直可用。标定文件是本机实测数据，
仓库 `.gitignore` 忽略 `*.json`，**不要提交**。

## 7. 帧接口参考

`↑` 上位机发，`↓` 板子发。payload 全是小端。头部 6 字节 =
`AA 55 | type_id | datalen | arm_id | crc8`，尾 2 字节 crc16；整帧长 = `8 + datalen`。
`crc8` = CRC-8/MAXIM 盖头 5 字节，`crc16` = 反射多项式 `0x8408`（初值 `0xFFFF`）盖头+payload。

| tid | 方向 | payload | 线上单位 | SDK 方法 |
|---|---|---|---|---|
| `0x01` | ↑ | 4 B：u32 版本码 | — | `send_version()` / `keepalive()` |
| `0x02` | ↑ | 0 | — | `init_arm()` |
| `0x10` | ↑ | 0 | — | `set_zero()` |
| `0x11` | ↓ | 12 B：u8 state + u8[7] result + u32 用时 ms | — | `wait_setzero_ack()` / `last_setzero_ack` |
| `0x12` | ↑ | 0 | — | `start()`（run 档） |
| `0x13` | ↑ | 0 | — | `gravity()` |
| `0x14` | ↑ | 0 | — | `stop()`（**失能、松劲**） |
| `0x15` | ↑ | 0 | — | `damping()` |
| `0x16` | ↑ | 28 B：f32[6] 关节角 + f32 夹爪角 | deg | `send_ctrl_pos()` |
| `0x17` | ↑ | 52 B：f32[6] 关节角 + f32[6] 关节角速度 + f32 夹爪角 | deg, deg/s | `send_ctrl_pos_vel()` |
| `0x18` | ↑ | 0 | — | `reset()` |
| `0x80` | ↓ | 63 B：4×u8 + f32[6] + f32[6] + f32 + u8[7] | deg, deg/s, ℃ | `arm.feedback` / `snapshot()` |
| 其它 | ↑ | 任意 | — | `send_raw(tid, payload)` —— **总逃逸口**，无预检无换算 |

`0x80` 的 4 个 u8 依次是 `mode` / `robot_state` / `motor_lost_mask` / `motor_inited_mask`，
掩码 **bit0..5 = j1..j6、bit6 = 夹爪**。`0x80` 板子侧**实测 250 Hz**。

几个容易踩的点：

- `0x14 stop()` 是**失能**（臂会松劲、靠重力掉），不是"停在这不动"。
- `0x18 reset()` 走一段 2 s 的轨迹回复位位姿，**不做夹爪找零**，跑完停在**阻尼**。
- `0x10 set_zero()` **覆盖电机的原零点**（把"当前位姿"定义成零点）；只管 j1~j6，
  夹爪不在默认参与集里。**没有即时回执**，结果走几秒后的 `0x11`。
- `0x11` 的 `result = 1（未确认）` **不是故障**：它表示"帧发了、位置本来就在容差内"，
  臂本来就停在零位上时必然是它。

## 8. 多臂

```python
link = sdk.Link(); link.open()
a, b = sdk.Arm(0, link), sdk.Arm(1, link)     # 或 link.arm(0) / link.arms()
```

- **必须共享一个 `Link`**。两个进程、或一个进程里两个 `Link` 抢同一个 tty，会把帧拆散
  （`device reports readiness to read but returned no data`）。
- 帧头的 `arm_id` 决定给哪一台：`0` = CAN1 = A 臂，`1` = CAN2 = B 臂。**没有广播**，
  越界的 `arm_id` 板子**直接丢**（SDK 侧也丢并计数 `link.stats["n_bad_arm"]`）。
- 一个 `Link` 可以同时驱动两台，各自的反馈槽、序号、`wait_*` 完全独立。

## 9. 线程与回执

- `Link` 开一根 **RX 后台线程**：`read → FrameParser → 按 arm_id 分发`。**RX 线程里不调
  用户回调**（慢回调会饿死收帧），你只能主动读或 `wait_*` 轮询。
- 每臂一把锁，反馈整对象替换（`Feedback` 是 frozen dataclass），所以拿到手的那份永远不会
  被改写。但**两次独立的读可能跨帧** —— 要严格一致就用 `snapshot()`。
- 发帧持一把**写锁**（pyserial 的 `write` 多线程不保证整帧原子）。
- **保活不自动发**：SDK 不起保活线程。你的周期控制帧本身就是保活；其余场景自己
  `keepalive()`。**板子自己发的 `0x80` 不算"收到的帧"，不喂狗。**
- `0x11` 没有请求 id，只能按序号等：`set_zero()` 会记下发送前的序号，
  `wait_setzero_ack()` 等序号更大的那一帧（默认 25 s，超时抛 `TimeoutError`）。
- 读写失败后属性会停在旧值 —— 用 `link_up` / `feedback_age` / `feedback_ok` 判活，
  可以 `link.reopen()`。

## 10. 注意事项

1. **静默丢弃，没有 NACK。** 没有异常 ≠ 板子受理了。`send_*` 只保证字节写出去了，
   受理与否看**下一帧 0x80**（`robot_state` / `mode` 变没变）。`force=True` 不是"一定成功"，
   是"我不管预检了"。
2. **遥控器在线状态查不到**（不在任何反馈帧里），但它是**每一条命令**的隐含前提。
   发了没反应、或者 `mode` 被莫名其妙覆盖 → 先确认遥控器关了。
3. **不要绕过 `Arm` 直接 `send_raw()` 发已知的帧** —— 预检是唯一能提前告诉你"这条过不了"
   的东西。`send_raw` 是给板子加了新包时用的。
4. **500 ms 隐式保活**：任何有效帧都算，包括 `0x01`。断了落到**离线等待(1)**，
   **必须重新初始化**，不会自己回 `正常运行`。
5. **切档会作废上一帧控制指令**（板子侧 `Ctrl_Invalidate`，防止把切档前的旧目标灌进位置环
   猛拽一下）。`start()` 之后要**重新发** `0x16/0x17`。
6. **软限位要自己夹。** 板子那边确实会 `fclip`，但**只对 `motor_inited` 为真的电机生效**
   —— 没初始化过的电机不受保护，而且是**静默**夹的，你从期望角上看不出来。
   用 `board_defs.clamp_axis(axis, value)`（返回 `(值, 是否夹过)`）。限位：
   min `(-115, 0.5, 0.5, -90, -90, -112.87)`°、max `(90, 206.26, 171.88, 90, 90, 119.17)`°。
7. **夹爪没有力矩/力控命令** —— `0x16/0x17` 只带角度。夹爪软限位 ∓142° 也**只在
   `motor_inited[GRIPPER]` 为真时（即找零成功后）才生效**。
8. **`0x10` 会覆盖原零点**，跑之前先把臂摆到基准位姿；它也会让夹爪的标定作废。
9. **`0x14 stop()` 是失能**，臂会靠重力掉。
10. **温度区分不了"真 0 ℃"和"没数据"**（板子把 NAN 写成 0）。
11. **协议里没有递增计数/时间戳，检测不了丢帧**（半双工 RS485 碰撞会丢 ~20%）。
    SDK 只能按到达时间判陈旧（`feedback_age`）。
12. **口子按 USB 身份挑**：板子 USB CDC `0483:5740` 优先，其次 CH340 适配器 `1a86`
    （RS485），**J-Link 的 SEGGER VCOM（`1366`）会被拒绝**。`list_links()` 看现状。
    别按 `/dev/ttyACM*` 这种名字写死 —— 换一次适配器就失效。
13. **标定文件不要提交**，那是本机的实测数据。

## 11. 离线自测

```bash
python3 -m sdk.selftest          # 不需要硬件、不需要 pyserial
```

151 项：CRC 参考值 + 位运算参考实现逐字节比表、整帧字节锚点、编解码往返、切帧重同步、
按 `arm_id` 分流、单位往返、夹爪标定模型、**闸门矩阵**（逐条断言该挡的挡了、该发的发了什么）、
以及**与固件头文件对拍**（码表、限位、状态个数、`Arm_mode` 顺序 —— 仓库在的时候才跑，
所以 `sdk/` 被拷出去单用也不会失败）。改协议或改板侧常量之后先跑它。

## 12. 文件清单

| 文件 | 职责 |
|---|---|
| `__init__.py` | 包入口，只 re-export；`import sdk` 不碰 pyserial、不起线程 |
| `errors.py` | 异常层级（单独一个文件，免得各模块互相 import） |
| `protocol.py` | 协议层：CRC / FNV / 码表 / 组帧 / `FrameParser` / payload 编解码（**线上单位**） |
| `board_defs.py` | 板侧常量镜像：软限位、复位位姿、状态 / 模式名字表 |
| `units.py` | deg↔rad 策略 |
| `link.py` | 串口 + RX 线程 + 按 `arm_id` 分流 + 写锁 + 口子挑选 |
| `arm.py` | `Arm`：全部帧接口 + 闸门预检 + 单位换算（**SI**） |
| `gripper.py` | 夹爪两点标定：m ↔ 原始 deg 的映射 + JSON 存取 |
| `calibrate_gripper.py` | 交互式标定工具 |
| `selftest.py` | 离线自测 |
| `config/` | 运行期数据（`gripper_calib.json`），由工具自动创建 |

异常层级（`errors.py`）：根是 `ArError`，下面分 `LinkError`（`NotConnected` 是它的子类）、
`ProtocolError`、`GateRejected`、`FeedbackStale`、`GripperNotCalibrated`、`CalibrationError`。
`GateRejected` 的消息里会写清楚缺哪一条闸门，直接 `print` 就能给人看。
