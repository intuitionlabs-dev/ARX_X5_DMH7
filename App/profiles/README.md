# H7 机械臂适配

## 术语和单位

| 术语 | 含义 |
| --- | --- |
| 机型配置 | `Arm_profile` 定义的机构和控制参数 |
| 电机适配器 | `Arm_motor_driver` 定义的驱动操作 |
| 关节坐标 | 控制核心使用的关节角，单位 deg |
| 电机坐标 | 驱动控制器使用的输出轴角，单位 deg |
| URDF | 机器人模型文件，提供几何和惯量 |

动力学接口使用 rad、rad/s、rad/s² 和 N·m。
基座重力使用 m/s²。
夹爪协议字段使用 deg。

## 配置入口

| 文件 | 配置内容 |
| --- | --- |
| `../inc/robot_def.h` | `enum Arm_Motor_Index` |
| `metal.c` | metalarm 的电机、关节和初始化策略 |
| `arx_x5.c` | ARX-X5 的电机、关节和初始化策略 |
| `profile_constants.h` | 内置机型共用的编译期控制和初始化常量 |
| `../arm_board.c` | 每台臂的机型和 CAN 总线绑定 |
| `CMakeLists.txt` | 机型注册和模型生成规则 |
| `../../libs/gravity_compensation/models/` | URDF 文件 |

`Arm_Motor_Index` 是轴数的唯一配置入口。
关节枚举位于 `ARM_JOINT_NUM` 前面。
`GRIPPER` 和 `ARM_MOTOR_NUM` 随关节数派生。
控制数组、轨迹工作区和设零选择表使用派生尺寸。
H7 固件仅支持六关节和一个夹爪电机。
`ARM_JOINT_NUM` 必须等于 6，否则编译报错。
每台臂可以选择不同的机型配置。
配置表的关节数必须与枚举一致。
共用常量用于静态配置初始化。
机型内重复的电机量程和编码常量定义在各自配置文件中。

默认两台臂均使用 metalarm。
下列命令选择 ARX-X5 与 metalarm：

```bash
cmake --preset Debug -DH7_ARM_A_PROFILE=arx_x5 -DH7_ARM_B_PROFILE=metalarm
cmake --build --preset Debug
```

`H7_ARM_A_PROFILE` 和 `H7_ARM_B_PROFILE` 是 CMake 缓存项。
`arm_board.c` 绑定 `arm_id` 与 CAN 总线。
每台臂独占一条 CAN 总线。

## 控制流程

`Robot_State_Loop()` 处理协议和状态迁移。
`Gimbal_Cmd_Update()` 处理输入、轨迹和软限位。
自动动作通过 MovePlan 临时规划组生成关节位置、速度和加速度。
`Arm_Reset()` 使用 `switch/case` 推进复位步骤。
复位目标和持续时间由机型配置提供。
`Gimbal_Update()` 处理反馈、动力学和电机参考值。
`MotorTask` 运行驱动发送和设零状态机。
控制周期为 2 ms。
电机发送周期为 1 ms。
各电机的 `tx_group` 决定发送频率和相位。

控制核心通过 `Arm_motor_driver` 操作电机。
`Gimbal.motors[]` 按枚举顺序保存实例。
每台臂独立持有命令、反馈和工作区。
初始化时分配对象。
周期控制使用已有工作区。
核心在临界区内发布参考值和增益。
动力学计算位于临界区外。

## 电机配置

`Arm_motor_config` 的数组顺序与枚举一致。
最后一个元素配置单电机夹爪。

| 字段 | 单位或取值 | 含义 |
| --- | --- | --- |
| `driver` | 适配器地址 | 电机操作接口 |
| `model` | 字符串 | 电机型号或协议标识 |
| `tx_id`、`rx_id` | CAN ID | 控制与反馈标识符 |
| `tx_group` | 8 bit 掩码 | 控制帧发送相位 |
| `position_max` | rad | 位置编码半量程 |
| `speed_max` | rad/s | 速度编码半量程 |
| `torque_max` | N·m | 力矩编码半量程 |
| `kp_max`、`kd_max` | 驱动协议量程 | 增益编码上限 |
| `position_pid`、`speed_pid` | `PID_Config` | 主机位置环和速度环参数 |
| `kp`、`kd` | 驱动协议增益 | 运行模式增益 |
| `damping_kd` | 驱动协议增益 | 阻尼模式增益 |
| `position_divisor`、`speed_divisor` | 编码分母 | DM 反馈换算参数 |
| `cmd_multiturn` | 0 / 1 | DM 目标是否补偿累计圈数 |
| `current_max` | A | ENCOS 电流编码半量程 |
| `torque_constant` | N·m/A | ENCOS 电流到力矩的换算系数 |

metalarm 使用七个 DM MIT 电机。
ARX-X5 的前三轴使用 ENCOS。
ARX-X5 的其余关节和夹爪使用 DM MIT。
编码量程必须与驱动器配置一致。

`Arm_Profile_Validate()` 校验配置。
校验包括尺寸、有限值、限位、ID 和发送预算。
每个相位为维护命令预留一帧。
帧数必须满足 `CAN_TX_MAX_FRAMES_PER_TICK`。
配置错误时，创建函数返回 `NULL`。
电机掉线时，核心按配置进入 STOP。

## 关节配置

| 字段 | 单位或取值 | 含义 |
| --- | --- | --- |
| `direction` | -1 / 1 | 关节与电机的方向关系 |
| `reduction` | 正数 | 电机角与关节角的比例 |
| `position_zero_deg` | deg | 关节零角对应的电机角 |
| `minimum_deg`、`maximum_deg` | deg | 关节软限位 |
| `reset_deg` | deg | 复位目标 |
| `torque_scale` | 比例系数 | 关节力矩到电机输出的标定系数 |
| `viscous_damping` | N·m·s/rad | 粘性摩擦补偿系数 |
| `velocity_scale` | 比例系数 | 电机速度前馈系数 |
| `plan_velocity_scale` | 比例系数 | 自动轨迹速度前馈系数 |
| `td_r` | 跟踪微分器参数 | 目标跟踪参数 |
| `absolute_encoder` | 0 / 1 | 在线时能否直接建立位置有效状态 |

位置换算为：

```text
motor_deg = joint_deg × direction × reduction + position_zero_deg
joint_deg = (motor_deg - position_zero_deg) × direction / reduction
```

## 夹爪映射

`Arm_gripper_mapping` 定义协议角与电机角的双向映射。
`cmd[2]` 和 `motor_deg[2]` 提供两个标定点。
映射支持方向反转和比例缩放。
`to_motor` 与 `from_motor` 支持自定义映射。
自定义映射必须成对提供。
`context` 指向映射参数。
`minimum` 和 `maximum` 限制协议角。
`torque_scale` 换算夹爪力矩。

两个内置机型使用角度恒等映射。
metalarm 的夹爪范围为 0～2.4 rad。
ARX-X5 的夹爪范围为 -142～142 deg。

## 初始化策略

`Arm_profile.initialize` 指定初始化回调。
回调在 CmdTask 内执行。
`start=1` 启动一次初始化。
后续周期使用 `start=0` 推进状态。
回调返回 0 表示等待。
回调返回 1 表示完成。
回调返回负值表示失败。
回调必须非阻塞。
`cancel_initialize` 处理初始化退出。
该回调可释放自定义策略状态。

`Arm_Initialize_Hold()` 执行关节复位。
该策略保持入口夹爪角。
metalarm 使用此策略。

`Arm_Initialize_Home()` 先执行关节复位。
随后夹爪寻找机械限位并退回配置距离。
夹爪到位后请求设零并等待复核。
ARX-X5 使用此策略。
搜索力矩、速度、停滞判据和超时由 `init` 配置。

遥控器接管和电机掉线会中止初始化。
成功后进入正常运行的 DAMPING 模式。
失败后返回初始化入口状态。
`initialization_context` 可保存自定义策略状态。

设零由 MotorTask 执行。
`Arm_zero.selection[]` 按枚举选择参与电机。
`selection_valid=0` 选择全部关节。
`selection_valid=1` 使用显式选择表。
发布选择表后设置 `request=1`。
设零期间暂停该臂常规发送。
DM 适配器使用位置反馈复核。
ENCOS 适配器等待确认和总线保护间隔。

## URDF 和动力学

CMake 在编译前生成模型头文件。
生成文件位于构建目录的 `arm_models/`。
导出器接收 URDF、基座、末端和 C 标识符。
它沿指定串联链提取旋转关节。
它将固定连杆惯量合并到相邻活动连杆。
模型关节顺序必须与配置数组一致。

```bash
python3 libs/gravity_compensation/c/generate_arm_model.py \
  --urdf /绝对路径/robot.urdf \
  --base base_link --tip tool_link \
  --symbol robot_links --output /绝对路径/robot_model.h
```

`Arm_dynamics_model` 持有连杆数据和补偿系数。
`Arm_Dynamics_Compute()` 接收模型关节数。
工作区由每台臂按枚举尺寸分配。
RUN 模式计算完整逆动力学前馈。
GRAVITY 模式计算静态重力补偿。
动力学错误使该臂进入 STOP。
`dynamics=NULL` 关闭动力学前馈。

## 协议边界

`arm_protocol.c` 集中处理线格式转换。
位置报文为 28 字节。
位置速度报文为 52 字节。
反馈报文为 63 字节。
夹爪使用协议第七个电机槽位。
六个关节与协议槽位按顺序对应。
编译期检查保证关节数等于 6。
本地复位流程仍覆盖全部配置关节。
SDK 使用现有协议。

## 适配步骤

1. 确认 `Arm_Motor_Index` 包含六个关节。
2. 提供六轴机型配置数组。
3. 配置电机型号、ID、量程和发送相位。
4. 配置关节参数和夹爪映射。
5. 在 `arm_register_profile()` 注册机型和 URDF。
6. 选择初始化回调。
7. 配置板级绑定。
8. 构建固件并运行主机测试。
9. 在硬件上验证方向、限位和初始化动作。

机型配置和回调必须在实例使用期间有效。
适配使用新的电机协议时，实现 `Arm_motor_driver`。
适配器负责协议校验、状态查询和设零语义。

## 验证

```bash
python3 -m unittest discover -s tests -p 'test_arm_framework.py' -v
python3 -m unittest discover -s tests -p 'test_metal_motors.py' -v
python3 -m unittest discover -s tests -p 'test_metal_initialization.py' -v
```

主机测试覆盖六轴控制流程。
编译测试确认其他轴数触发编译错误。
测试检查协议缓冲区、双臂隔离和初始化策略。
动力学测试对照两种机型的现有 C 实现。
电机测试使用真实 DM 与 ENCOS 驱动。
机械动作和实时周期需要硬件验证。
