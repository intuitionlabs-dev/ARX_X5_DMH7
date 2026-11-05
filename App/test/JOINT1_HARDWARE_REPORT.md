# A 臂 JOINT1 实机诊断报告

Flash 逐字节校验和 STOP 状态 CPU 基准通过。STOP 校准窗口内，外部测得 Cmd 为 499.978 Hz，超期计数增量为 0。A 臂标准初始化因夹爪设零复核超时中止，整套测试未完成。

## 固件与实机配置

| 项目 | 实测或构建记录 |
| --- | --- |
| 固件提交 | `4c838441cf8510a41a165e2c067780397de9ea7f` |
| SDK 提交 | `9d14e5fe73bc705f38bb7cca52c8339d801c4b8d` |
| ELF SHA-256 | `c02066e33603d2a6a5b5106aefebfeb620d51c656ec9df08fccf7a619991d461` |
| MCU、构建 | STM32H723VGT6，GCC 10.3.1，Debug `-O0`，诊断开启 |
| A/B 臂 | 双 ARX-X5，Flash 中两个臂的 profile 指针均指向 `ARM_PROFILE_ARX_X5` |
| Flash 校验 | 从 ELF 提取的两个连续 Flash `PT_LOAD` 段共 540,384 字节，运行中经 SWD 回读一致 |
| 主机验证 | Ruckig CTest 2/2 通过，诊断测试 9/9 通过，含实际 FP32 预演与 ASan/UBSan 固件保护测试 |

J-Link 直接加载 ELF 时，两处对齐填充共 8 字节保持擦除值。实机测量使用与 ELF 加载段逐字节一致的连续镜像。完整校验在测量前执行。

## STOP 状态的时钟与 CPU 基准

CPU 基准含时钟校准共用时 **5.993 s**，预算为 8 s。基准启动时两臂均为 STOP，诊断保护关闭。

| 计数器 | 对 PC 单调时钟的拟合速率 | 拟合残差 RMS |
| --- | ---: | ---: |
| DWT CYCCNT | 479,982,079 cycle/s | 291,337 cycle |
| HAL | 999.982 ms/s | 0.760 ms |
| RTOS | 999.955 tick/s | 0.568 tick |
| Cmd | 499.978 次/s | 0.284 次 |

校准窗口为 5.279 s，SWD 读数的最大时间包络为 0.819 ms，时钟快照在 Cmd 结束时发布。按 Cmd 速率折算的平均更新间隔为 2.00009 ms。窗口内 `deadline_misses` 增量为 0。该结果支持本机 STOP 工况下 500 Hz 的更新频率。运动工况的逐周期均值、P99、最大值及抢占情况需用运动采样判定。

| 场景 | 样本数 | 平均 µs | 最大 µs | 单轴 500 Hz 占比 | 六轴线性估计 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 标量基线 | 128 | 0.263 | 0.263 | 0.0131% | 0.0788% |
| FP32 五次曲线求值 | 128 | 0.771 | 0.788 | 0.0385% | 0.2313% |
| Ruckig 缓存采样 | 128 | 28.296 | 28.355 | 1.4148% | 8.4888% |
| Ruckig 强制重规划 | 128 | 406.419 | 406.732 | 20.3209% | 121.9256% |

DWT 计时读数开销为 23 cycle（约 0.048 µs），表中耗时保留该开销。五次曲线场景只测固定曲线的 q/v/a 求值。六轴列按六个相同实例、每轴每周期执行对应场景估算。六轴每周期都重规划的估算开销超过 2 ms，实际占用取决于各轴的重规划比例和输入。

## 运行参数与运动测试状态

STOP 元数据中的 JOINT1 软限位为 −115° 至 +90°，方向和减速比均为 1，零点为 0°，速度倍率和规划速度倍率均为 1。Ruckig 周期为 2 ms，速度、加速度和加加速度上限分别为 400°/s、2500°/s² 和 10000°/s³。外层位置 PID 的 Kp/Ki/Kd 为 6/0/50，输出上限为 1000。MIT Kp/Kd 为 0/35。完整运行快照包含各轴限位、控制器配置、反馈偏置、保护阈值、时钟寄存器及构建标识。[运行元数据](joint1_evidence/debug-stop/firmware.json)

目标板的 USB CDC 标识与 SWD 读取的芯片 UID 一致。保持 J-Link 后台连接时，SDK 收到 ARX-X5 版本 `0xF81DD236`，A 臂七个电机均在线。标准初始化依次进入关节复位、夹爪寻限位、回退和设零复核。SWD 在 5.93 s 内取得 2,521 条初始化监测记录。采样时遥控器离线，电机掉线位均为 0。设零结果仅包含夹爪，`disable_confirmed=1`，`zero_state=6`（`ARM_ZERO_TIMEOUT`），夹爪结果码为 4（`ARM_ZERO_RES_TIMEOUT`），耗时 1,501 ms。末帧夹爪反馈为 0.984°，DM 驱动要求命令后的新反馈在 ±0.01 rad（约 ±0.573°）内。固件记录了复核超时，实际总线上的设零命令及反馈时序仍待确认。[设零结果](joint1_evidence/debug-init/gripper-zero-result.json)

A 臂未达到 `STATE_NORMAL` 和全电机初始化的运动入口条件。JOINT1 的测试中心、实际幅度、`0x16` 样本、周期 P99 和最大值、参考积分比例、反馈相位和时差、ENCOS 类型和反馈年龄、PID 恒等式残差及测试阻尼退出结果均无实测值。已完成的测量用时 5.993 s，整套 30 s 测试预算的验收待完成。当前 J-Link 操作范围为 MCU 运行时读取内存。交接脚本的运动采样需要通过 SWD 写入采集请求和保护控制字段。

## 测试证据

- 仓库中的 STOP 证据包括[时钟原始读数](joint1_evidence/debug-stop/clock_raw.json)、[时钟拟合结果](joint1_evidence/debug-stop/clock_summary.json)和[基准原始汇总](joint1_evidence/debug-stop/benchmark.json)。
- 本机归档 `build/joint1-hardware-evidence.tar.gz` 包含 ELF、map、编译命令、构建和烧录日志、Flash 比对记录、离线测试日志、时钟和 CPU 实测 JSON，以及 SDK 与 SWD 初始化监测原始记录。SHA-256 为 `6e6af2cde9ac195416dad5d8dedcc87ce90ad4678e8822e8e4bcd06b870d0660`。
