# H7 → PC Python 迁移审计

审计基线：主仓 `21b97f9`。沿 `main.c → freertos.c → Cmd_Loop/MotorTask/MonitorTask/Moveplan_Task` 跟踪实际调用链；行为以 C 实现为准，不以存在差异的注释为准。Python 包没有导入遥控器或原通信 SDK。

## 源码覆盖

| H7 源码 | 执行内容 | Python 对应 |
| --- | --- | --- |
| `Core/Src/main.c`、`freertos.c` | 电机/臂实例创建、任务时序 | `ControlSystem`、`runtime.run`；硬件启动代码排除 |
| `App/arm.c`、`inc/arm.h` | 两台独立的 Arm 及状态数据 | `Arm`，默认两实例 |
| `App/cmd.c` | 命令 → 状态机 → 命令处理 → 驱动层 | `ControlSystem.step`；删除 `Rc_Loop` |
| `App/robot_state.c` | 握手、8 个总状态、初始化/复位/校准、控制输入、3 s 看门狗、250 Hz 反馈 | `Arm.handle/state_step/snapshot` |
| `App/gimbal_cmd.c` | 模式处理、目标跟随、输入源、软限位、TD、轨迹/外部速度选择 | `CommandLayer`；所有 Rc_Online/rc 分支排除 |
| `App/arm_auto.c` | 2 s 复位、夹爪闭合检测/退回中点/设零/稳定等待 | `CommandLayer._reset/_gripper_zero` |
| `App/arm_zero.c` | 参与掩码、失能确认、逐电机设零、ENCOS ACK 与回读、DM 唤醒/重试/超时 | `ZeroCalibration` |
| `App/gimbal_arm.c` | 电机配置、首次就绪、掉线停机、反馈坐标转换、重力/速度/粘滞前馈、模式增益 | `Arm`、`Motor`、`ArmConfig` |
| `App/inc/robot_def.h` | 模式/索引/命令/反馈类型 | `types.py`，保留状态数值 |
| `App/inc/gimbal_arm_def.h` | 电机方向、减速比、零点、力矩系数、阻尼、重力参数 | `config.py` |
| `App/inc/gimbal_cmd_def.h` | 关节/夹爪软限位与规划速度前馈 | `config.py`、`CommandLayer` |
| `libs/controller/pid.c` | 实际使用的 PID_POSITION | `PID`，包含积分分离、限幅及原微分保持行为 |
| `libs/controller/adrc.c` | 独立 TD：fst/fsg/TDFunction_independent | `TrackingDifferentiator` |
| `libs/controller/controller.c` | PID 位置→速度级联 | `Motor.control`，前馈力矩在驱动端与速度 PID 输出相加 |
| `libs/move_plan/curves/*.c` | 一次、三次、五次多项式 | `Spline` |
| `libs/move_plan/move_plan/*.c` | 临时/多段规划、同步组、航点速度/加速度估计 | `Trajectory`、`PlanGroup`；Python 对象代替堆池 |
| `libs/move_plan/hal_move_plan.c` | 500 Hz 规划更新 | 命令周期中更新，按绝对经过时间采样 |
| `libs/gravity_compensation/c/x5_gravity.c` | X5 静态重力力矩 | 复用已有 `GravityCompensator`；直接对比原 C |
| `libs/dm_motor/DM_motor.c` | 反馈状态/多圈统计、模式控制、每 75 ms 轮流使能、量程限制 | `Motor`、`MotorBank`；字节解码/发送替换为接口 |
| `libs/dm_motor/dm_mit_control.c` | PID + deg→rad + 多圈补偿 + 力矩前馈 | `Motor.control` |
| `libs/encos_motor/ENCOS_motor.c` | 字段新鲜度、错误门控、启用时清积分、设零静默及 ACK | `Motor`、`MotorBank`、`ZeroCalibration` |
| `libs/encos_motor/encos_mixed_control.c` | 混合控制参考量 | `Motor.control` |
| `libs/monitor/monitor.c` | 10 ms 倒计数电机在线判定、设零期间保活 | `Motor.monitor_deadline_ms/keep_alive`，按原 10 ms 网格计算 |
| `libs/can/can_config.h` | 电机发送频率和相位 | `TX_GROUPS` 和到期调度 |
| `App/inc/temperature_protect.h` | 温度线性降额辅助函数 | `temperature_coefficient`，与固件一样未接入输出 |

实际位置环 `PID_POSITION`、级联控制及独立 TD 已移植。仓库内的 PID_DELTA/PID_COMP_POSITION/PID_FUZZY、完整 ADRC/ESO、SMC、LQR、MRAC、MPC、basic_controller 没有被当前机械臂实例选用，不属于 H7 当前控制链路，未作为新的运行模式加入。`gimbal_arm.c` 中的旧低通滤波/堵转检测函数没有调用方，也没有加入运行链路。

原工程未实现正/逆运动学、笛卡尔位姿控制、全动力学惯性/科里奥利补偿或负载估计，Python 也不把这些功能标记为已实现。`ik_status/is_load` 原本只是占位字段。当前运动输入是六个关节角及夹爪角。

排除的通信内容包括 `libs/frame`、`soft_crc`、`uart`、`usb`、`can` 的编解码/收发/硬件总线健康恢复、`encos_protocol.c` 的字节协议及反馈路由实现。控制所需的启停、设零、ACK、时限与字段有效性保留在语义接口层。原 0x81 的 CRC/字节丢弃统计属于传输层；PC 核心只提供命令拒绝、状态迁移、控制计数、过期反馈及调度漏拍计数。

整个 `libs/rc` 及 App 中的遥控器优先级、开关模式、遥杆控制、遥控器中断自动动作逻辑全部排除。BMI088/Mahony、显示、蜂鸣器、相机、Flash 测试、板级中断/时钟与工程回环测试不参与当前机械臂闭环，不移植到 PC 控制器。

## 固定参数与时序

| 电机 | ID | 类型 | 更新频率/相位 | 内部 kp / kd | 速度/力矩量程 |
| --- | --- | --- | --- | --- | --- |
| j1 | 1 | ENCOS | 500 Hz，0/2/4/6 ms | 0 / 35 | ±18 rad/s，±30 N·m |
| j2 | 2 | ENCOS | 250 Hz，1/5 ms | 0 / 50 | 同上 |
| j3 | 4 | ENCOS | 250 Hz，1/5 ms | 0 / 50 | 同上 |
| j4 | 5 | 达妙 | 250 Hz，2/6 ms | 50 / 0.8 | ±45 rad/s，±10 N·m |
| j5 | 6 | 达妙 | 250 Hz，2/6 ms | 25 / 0.8 | 同上 |
| j6 | 7 | 达妙 | 250 Hz，3/7 ms | 10 / 1 | 同上 |
| 夹爪 | 8 | 达妙 | 250 Hz，3/7 ms | 8 / 0.1 | 同上 |

相位按 8 ms 重复，两臂相同。电机位置输出量程均为 ±12.5 rad。达妙每 75 ms 轮流发送一个 ENABLE，一轮四台约 300 ms。原 CAN 的 200 μs 帧间间隔、每拍最多 4 帧和 FIFO 恢复是传输实现，不在 PC 控制算法内忙等。

位置 PID `(kp,ki,kd)`：j1 和 j4～夹爪为 `(6,.08,50)`，j2/j3 为 `(4,0,50)`。积分上限 200，位置环输出上限 1000 deg/s，j1～j3 积分范围 1°，其余积分范围 0，因此实际上不积分。速度 PID 增益全部为 0；其原输出上限前三关节 12，其余 10。实际力矩来自重力/粘滞/夹爪前馈及速度 PID 输出，再按电机量程限制。

**PID 没有 dt**，增益取决于调用次数；未把所有电机 PID 改为 1 kHz。原 `PID_Calc` 在误差不变且已有微分量较大时保留上一微分，Python 保留并单独测试。

TD 每 2 ms 更新：`r=[1000,700,700,2000,2000,2000]`、`h=.002`、`h0=.004`。轨迹执行时输出位置、速度、加速度，并同步 TD 状态以平滑退出。加速度在原驱动中没有惯性前馈消费路径，Python 也只保留规划结果。

关节角软限位：最小 `[-115,.5,.5,-90,-90,-112.87]`，最大 `[90,206.26,171.88,90,90,119.17]`。夹爪找零后限制 `[-142,142]`。复位目标 `[0,.5,.5,0,0,0]`，五次样条 2 s。

夹爪找零：每控制周期推进 .2°，目标超前限制 20°，闭合阶段前馈 .5 N·m；每 200 ms 统计移动量，连续 3 个窗口移动 <.5° 认定抵达机械限位；反向退 142°，到达误差 ≤4° 后设零，等待 400 ms。闭合/退回/设零上限分别 10/6/12 s。退回阶段没有 .5 N·m 前馈，保持原代码行为。

电机校准：默认六关节，夹爪由专门找零流程单独参与。达妙失能确认条件为状态码 0 或静默 300 ms；失能确认上限 1500 ms，每 100 ms 重试。设零后等待 20 ms，必要时使能唤醒再等 100 ms；每台上限 1000 ms。ENCOS 设零后静默 501 ms，ACK 等待 1500 ms，每台上限 2500 ms，无位置时每 120 ms 尝试软停机以获取反馈。零位容差 .01 rad，总流程上限 15 s。

ENCOS 在线监控重载 10 个 10 ms 周期，达妙为 2 个周期。实际掉线边界受监控相位影响，分别约 90～100 ms 与 10～20 ms。设零期间按 H7 行为临时保活本臂全部监控，防止维护期间因不发送常规帧而被误判掉线。

默认重力模型为 2023，基座重力 `(0,0,-9.81)`，SDK 比例 `[.8,.8,.8,1.32,1.32,1.32]`。方向/减速比/力矩系数/速度前馈系数默认均为 1，粘滞系数默认全 0；允许逐臂使用 `ArmConfig` 覆盖。

## 保留的特殊行为

- STOP 是软停机输出；达妙仍发送 ENABLE 保活。`shutdown` 另外产生达妙 DISABLE。
- DAMPING 全部位置环关闭，电机 kp=0、kd 使用上表；没有重力力矩。
- GRAVITY 六关节 kp/kd=0，只输出重力；夹爪保留普通控制增益，参考位置跟随反馈。
- 模式切换清除上层目标，STOP/被动模式让 TD 跟随反馈，避免恢复时使用陈旧位置目标。
- 前六个 `motor_initialized` 位每拍置位，夹爪单独维护；它们不是在线标志。
- DM 安装的是原始角度反馈回调，并在发指令时减去多圈偏移；保留这个实现，不把备用的多圈反馈路径当成实际配置。
- 温度只回传。原 `gimbal_arm.c` 温度降额调用已注释，Python 未启用未标定的温度限流。

## 为 PC 明确修正的行为

1. 删除全部遥控器接管条件。连接和输入来源只有 Python 命令接口。
2. 不保留 STM32 DMA、FreeRTOS、字节帧和 CAN 队列；改为显式时间、输入数据、输出请求。握手版本为 Python API 版本，不是原固件帧版本哈希。
3. 输入先验证维度、臂号与有限数，拒绝未来时间并忽略过期反馈；不把无效输入带入 PID/TD。
4. 固件规划器在最后段到期时先增加索引、下一拍可能在边界检查前访问不存在的段。Python 按绝对时间选段，并明确输出终点；线性样条也限制在段内。
5. 初始化期间电机掉线或通信超时会取消轨迹与未完成校准。夹爪找零失败不发送初始化成功，返回初始化入口状态；原固件只看自动模式是否结束，可能无条件进入 NORMAL。
6. 校准超时/取消时对临时唤醒的达妙补发 DISABLE，未完成电机结果设为 TIMEOUT，避免留下已唤醒状态或 NONE 终态。
7. ENCOS 启用时只复位 PID 历史，不把已设置的被动模式零增益临时覆盖成正常模式增益。
8. PC 错过更新相位时，下次调度执行一次到期电机更新，不反复补执行旧周期。夹爪超时按真实经过时间计时，目标推进仍每次 .2°，不会因一次延迟而跳跃一大段目标。
9. 反馈在本次命令处理后产生；原 H7 反馈任务与电机任务交错，可能显示前一拍状态。双线程时序被确定的单线程顺序代替。Python 使用双精度和标准三角函数，不追求浮点位级相同。
10. 温度保留可选浮点值，未知为 None，不压缩为 uint8；传输相关的 CRC、丢字节统计留给未来适配器。

这些差异已通过接口说明和测试明确，不提供保留越界或错误初始化成功行为的兼容开关。

## 验证证据与限制

`python3 -m unittest discover -s tests -v`：35 项测试通过，包括直接编译原 C 的 4 项数值测试，无跳过。随机种子固定，测试过程中生成的动态库位于临时目录；运行 Python 控制器不需要 C 动态库。

| 对比对象 | 输入规模 | 最大允许绝对差 |
| --- | --- | --- |
| PID | 7 组配置，各 340 次含阶跃、保持、随机参考/反馈 | .002，位置环输出单位 deg/s |
| TD | 3 组 r，各 3000 次正负阶跃及收敛 | .002°、.02°/s |
| 一次/三次/五次样条 | 各 200 个随机非零端点速度/加速度案例 | .02，分别按位置/速度/加速度单位 |
| 重力 | 3 模型 × 2 重力向量 × 300 位姿 × 2 模式 = 3600 案例 | 1e-5 N·m |

行为测试包括全六关节校准、两臂完整初始化及复位、找零失败、ACK 缺失/失败/成功但无位置、设零唤醒超时、失能确认失败、双臂隔离、每臂看门狗、软限位、速度前馈、模式清目标、发送频率、漏拍、输入原子验证和异常退出停机。

`python3 -m arx_x5_control --demo --seconds 12` 完成两臂流程，最终两臂均 NORMAL/DAMPING，夹爪均已初始化。wheel 已在独立虚拟环境、仓库目录之外以隔离模式加载并运行。没有做实机闭环验证，也没有把测试电机模型作为动力学或控制稳定性证据。
