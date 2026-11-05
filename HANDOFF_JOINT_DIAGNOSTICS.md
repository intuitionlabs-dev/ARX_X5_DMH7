# JOINT1 时间基准、相位与 Ruckig CPU 测试交接

接手 agent：请在本地 H7 实机完成下面的测试，保存原始数据、生成报告并提交到
`feat/joint1-timing-diagnostics`。用户已授权测试运动，机械臂周围已清空；所有目标和
运动路径须在当前固件的软限位内。无需重复询问是否允许这些测试。

本分支从 `feat/ruckig-c-joint-control` 最新提交 `257463f` 创建。
本次仅在 `App/test` 和 `Core/Src/freertos.c` 增加测试实现；未改 CMake、SDK 子模块、
生产控制算法、机型参数、任务周期或启动流程。测试宏默认关闭，上电不会自动运动。
本交接编写环境没有 H7/J-Link，因此下面的实机结果尚未取得。

**最新约束：默认 JOINT1 为中心 ±45°（最多往返 90°），按运行软限位缩小；整套测量
累计预算 30 秒。** CPU 阶段上限 8 秒，运动阶段含校准、采样、读取缓冲和退出上限
22 秒，其中预留 4 秒用于退出。正常设备预计总计约 18～23 秒，以实机 timing 记录为准。
烧录、Flash/ELF 核对、原有初始化属于测试前准备，最终绘图在测量结束后进行。
不再先跑 5°/20°，不重复两遍，也不要求在同一轮中重烧两个优化等级。
超时会取消测试并报告“未完成”，不能把超时终止当成全部通过。

## 要回答的问题

历史截图来自 A 臂 JOINT2、ENCOS、纯位置 `0x16`。用户确认 Cmd FPS 为 500，Timeline
倍率均为 1。截图估算 `Δq_ref / ∫v_ff dt ≈ 0.765`，而反馈位置/速度的积分比例约为 1；
不能据此认定真实 Cmd 频率只有 380 Hz。截图速度过零点还可能是反馈**提前**，需要
同步原始数据确认。`257463f` 已把 `g_*` / `g_ruckig_*` 切到 JOINT1，并提高轨迹上限，
不要混用旧截图的关节、参数和新 ELF。

| 待验证项 | 证据 / 结果字段 |
| --- | --- |
| 固件、机型、零点和倍率是否一致 | Flash 对 ELF 逐字节校验；`firmware.json`、每次 `metadata.json` |
| HAL 所谓 500 Hz 是否等于实际 500 Hz | PC `perf_counter` 对照 DWT、HAL、RTOS、Cmd 计数；`clock_summary.json` |
| 周期抖动、超时和抢占 | 每周期 DWT、Cmd wall time、`deadline_misses`、采样 hook 最大耗时 |
| 位置、速度是否来自同一条 Ruckig 参考 | post q/v/a、输入 current、输出 q/v、time、status、new_calculation；连续性和积分比例 |
| 是否混入 RC、自动规划或 `0x17` | 每条记录的 flags；`normal_position_samples` 必须等于样本数 |
| 位置环是否正在抵消速度前馈 | 同次电机计算的 `ref_speed = speed_comp + pid_out`，并记录 error、d_error、MIT kp/kd |
| 反馈是否提前/滞后，以及时间差 | 位置、速度的独立相位拟合；正数=反馈滞后，负数=反馈提前 |
| 位置、速度是否同批更新 / 陈旧 | ENCOS type、valid、last_rx/position/speed_ms、raw 与 controller 匹配标志 |
| Ruckig 的计算开销 | 停止模式下的独立线程基准：不调用 Ruckig、FP32 五次曲线求值、缓存采样、重新规划 |

## 环境和构建

先保留本地无关改动，再拉取任务分支。不要重置用户工作区或 force-push。

```bash
git fetch origin
git switch feat/joint1-timing-diagnostics
git pull --ff-only
git submodule update --init sdk
git rev-parse HEAD
git submodule status sdk
```

需要 ARM GCC、CMake、Ninja、PC 本机 C/C++ 编译器、Python，以及已安装的 SEGGER
J-Link 驱动/共享库。Python 依赖：

```bash
python -m pip install numpy matplotlib pyelftools pylink-square pyserial
```

例子选择 A=ARX-X5、B=Metal；**B 必须保持实际接线机型的配置**。先用与问题固件相同的
优化等级复现。仓库 Debug 为 `-O0`，Release 为 `-Os`，不是 `-O3`。

```bash
cmake -S . -B build/joint-diag-debug -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -DH7_ARM_A_PROFILE=arx_x5 -DH7_ARM_B_PROFILE=metalarm \
  -DCMAKE_C_FLAGS=-DTEST_JOINT_DIAGNOSTICS_ENABLE=1
cmake --build build/joint-diag-debug -j 6
```

用本地现有烧录方式烧录 `build/joint-diag-debug/CtrBoard-H7_ALL.elf`，保存 ELF、map、
`compile_commands.json` 和构建日志。仅在机械臂静止、原有安全停止条件成立时重启/烧录。
运行测试时关闭 Ozone/J-Link Commander 等占用探针的程序，取消断点；采集工具只附加到
已运行的 MCU，不调用 halt/reset/go。接上后若 MCU 已暂停，脚本直接退出。

PC 路径预演必须使用**本分支的实际 FP32 C 库**，不能用 PyPI 的其他版本。
Linux / WSL 的本机库构建：

```bash
cmake -S libs/ruckig -B build/joint-diag-native \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DRUCKIG_C_BUILD_TESTS=ON
cmake --build build/joint-diag-native -j 6
ctest --test-dir build/joint-diag-native --output-on-failure
c++ -shared -o build/joint-diag-native/libruckig_diag.so \
  -Wl,--whole-archive build/joint-diag-native/libruckig_c.a -Wl,--no-whole-archive
python -m unittest discover -s App/test -p test_joint_diag.py -v
```

Windows 实机若 Python 在 Windows，需用同架构 MinGW 本机构建，不能加载 WSL `.so`。
在 MinGW 环境中用上面的 native CMake 命令加 `-G Ninja -DCMAKE_C_COMPILER=gcc
-DCMAKE_CXX_COMPILER=g++`，链接命令改为 `g++ -shared -o build/joint-diag-native/ruckig_diag.dll
-Wl,--export-all-symbols -Wl,--whole-archive build/joint-diag-native/libruckig_c.a
-Wl,--no-whole-archive`，把相应 MinGW runtime DLL 所在目录加入 PATH。
下面的 `--ruckig-lib` 随之改为该 DLL；`--port` 改为实际 COM 口。
主机原生保护测试使用 `cc` 和 ASan/UBSan，推荐在 Linux/WSL 跑；实机脚本本身跨平台。

## 先完成只读校验和 CPU 基准

启动后等至少 3 秒。芯片默认 `STM32H723ZG`，若本地探针数据库使用其他准确名称，
通过 `--device` 指定；多探针用 `--serial <J-Link序列号>`。每次 `--out` 必须是新目录。

```bash
python App/test/joint_diag_run.py inspect \
  --elf build/joint-diag-debug/CtrBoard-H7_ALL.elf --out build/j1-inspect-debug
python App/test/joint_diag_run.py bench \
  --elf build/joint-diag-debug/CtrBoard-H7_ALL.elf --out build/j1-bench-debug
```

`bench` 自带 5.3 秒时钟校准，30 秒方案不要额外执行独立的 `clock` 命令。
`inspect/clock/bench` 不发送 SDK 控制/保活。应在初始化前、两臂均为 `arm_stop` 的状态下
执行；CPU 基准最好在电机动力电源关闭、仅 H7 供电时做。不要在承重运动期间切 STOP
来满足前提。基准会检查两臂均停止且 test guard 未启用，否则拒绝。

`bench` 每种情况测 128 次，共 512 次，**只在每次调用期间关中断**，每次调用后让出 CPU。
单次超过标称 1 ms 会终止并记录 error=4。它测的是隔离后的算法 CPU 周期，不能用于
同时评估实时调度抖动。DWT 不被测试代码清零，以免破坏 CAN 的 TX 间隔计时。

`benchmark.json` 的 `cost` 顺序：

1. passthrough：不调用 Ruckig 的标量基线；不向电机发送未经规划的位置。
2. FP32 五次曲线 q/v/a 求值：归一化固定曲线，不含系数重算，不代表旧控制链总成本。
3. Ruckig cached：已规划轨迹内部采样，不把长时间停在终点的开销当作运动开销。
4. Ruckig replan：从静止到 ±45° 的强制重规划，使用运行配置的 v/a/j 限制。

已输出 mean/max µs 和单轴 500 Hz 百分比：`mean_us × 500 / 1e6 × 100`。
六轴百分比只是六个相同实例的线性估计；两个机械臂、不同输入、重规划比例和中断负载
不能由这个数直接代替。记录计时读数自身开销；不要把 Cmd wall time（含抢占）写成
Ruckig 独占 CPU 时间。可另按本次记录的重规划比例加权 cached/replan 成本，但须标为
估计，本基准的重新规划输入并非每次实测输入的完整回放。

## 实机运动测试

CPU 基准结束后按现有流程连接电机、完成正常握手/初始化，使 A 臂进入 `STATE_NORMAL`，
七个电机全在线、均初始化完成、各轴基本静止。遥控器必须离线。脚本不强制跳过状态闸门，
不自动执行设零、夹爪寻零或复位，也不修改 PID/速度倍率。

启动前保留当前控制姿态，关闭占用 SDK 串口的其他程序。只运行下面一轮：

```bash
python App/test/joint_diag_run.py run \
  --elf build/joint-diag-debug/CtrBoard-H7_ALL.elf --port /dev/ttyACM0 \
  --ruckig-lib build/joint-diag-native/libruckig_diag.so \
  --amplitude 45 --out build/j1-debug-main
```

默认就是 `--amplitude 45`；实际幅度始终按运行软限位缩小并保留 15°，峰峰值最多 90°。
位置不足以容纳至少 3° 测试时脚本拒绝；不要改软限位绕过它。
需要换起始姿态时，沿用正常 SDK 位置控制，把 JOINT1 缓慢移到范围中部，其余轴保持，
确认静止后再开始测试。

脚本行为：

- 使用实时元数据中的限位，在 PC 上用本库分别按 250/380/500/750 次每秒调用预演
  路径，Ruckig 自身 dt 保持 2 ms。每个参考采样必须离软限位至少 10°。
- 测试中心取 JOINT1 当前反馈。其他关节及夹爪保持开始时目标；通过 SDK 的
  `send_ctrl_pos()` 以约 100 Hz 发送 **`0x16`**，H7 仍按原 500 Hz 控制。
- 先保持 0.25 秒并做 5.3 秒 PC/DWT/HAL/RTOS/Cmd 时钟校准，然后依次做 hold 0.5 秒、
  steps 2.45 秒、smooth 2.45 秒，各一次。steps 在 0.15/0.90/1.65 秒变为 +A/−A/0；
  smooth 在 0.15/0.80/1.55/2.25 秒以零端点速度、加速度连接 0/+A/−A/0。
  每段结束主动冻结缓冲，不再等待长窗口或重复采集。
- 全程检查反馈新鲜度、在线/初始化状态、模式和目标限位。固件额外检查 JOINT1
  反馈距限位 5°、参考软限位、位置误差 20°、反馈有效性与 50 ms 超时；异常锁存
  `guard_fault`，通过现有阻尼增益处理 A 臂各轴。这个反馈保护不是物理硬限位。
- 固件另有 18 秒保护计时，按 DWT 周期累加并处理回绕；PC 校准后更新实际 cycle_hz。
  即使 PC 运动循环被阻塞，正常运行的固件也会到期锁存 guard_fault=5 并进入阻尼。
- 正常结束与异常退出均尝试发送标准阻尼命令，确认后关闭 guard。通信已断时仍依赖
  原固件掉线策略，保留异常和 guard 状态供诊断。不要切换到 `force=True` 重试。
- 绘图在测试结束、确认阻尼后执行，不让 matplotlib 启动耗时触发通信超时。

本轮只测当前问题所用的编译等级。若后续需要比较 Release，应作为另一轮独立测量，
重新烧录/初始化后使用新的输出目录，不把额外一轮计入本次 30 秒方案。
核对 `benchmark.json.measurement_s` 与 `timing.json.measurement_and_cleanup_s` 之和
不超过 30 秒，并确认 `measurements_complete`、`damping_confirmed` 均为 true。
记录超时或失败原因，不通过缩小余量、修改参数或反复重跑掩盖超时。

## 数据解释与判定

每个 capture 最多 1280 条、不循环覆盖；500 Hz 时容量约 2.56 秒，短段主动提前冻结。
头文件中的 state=3
意味着写入已冻结且必要的 D-cache clean 已完成，可在 MCU 继续运行时读出。
状态、请求、时钟控制结构在 DTCM；大样本数组在 RAM_D1。不要在运动中暂停 CPU 导出。

`samples.bin` 是原始 ABI，`samples.csv` 是展开后的同一数据，`metadata.json` 是该次
运行配置。`commands.json` 保存 PC 发命令时间，`clock_raw.json` 保存时钟拟合原始读数，
`analysis.json` 和 `waveform.png` 是计算结果。离线重算：

```bash
python App/test/joint_diag_run.py analyze --out build/j1-debug-main
```

Cmd 行的 `cycles` 是该轮开始，`wall_cycles` 到该轮结束，HAL/RTOS 在结束时读取；
位置与速度是该轮 post-process 输出；真实参考发布发生在 start/end 区间内，故相位
结果仍有这一段执行时长的不确定性。`motor_*` 是最近一次电机控制计算的完整快照，
有自己的 cycles/seq/反馈时刻；**不能把新 Cmd 的 q 和旧 motor 的 v 配成同一时刻**。
图中电机曲线使用其自己的时间。motor seq 重复表示保持上次快照；跳号会在报告中计数，
低频反馈积分仍可参考，但需结合实际间隔评估。Motor flags bit17 表示计算发生于
Cmd 执行期间，`cmd_seq` 仅标记所在轮，不能据此断言已经使用该轮的新参考。

raw/controller 匹配标志 bit16 为零时，可能在驱动复制反馈后又收到了新 CAN 帧，
不能把新的 last_* 时间戳归给旧 controller 数据。即使匹配，时间戳也只有 HAL 毫秒
分辨率，且相同数值不保证同帧。type/valid 与两个更新时间用于判断反馈更新路径。
`tx_*` 是编码前目标，**不是确认已发到总线的抓包**。若需判断 CAN 排队、驱动器
内部滤波或更细的反馈时延，继续用本地已有 CAN 分析仪/示波器采集总线，不能从这里
直接下定论；本次没有擅自占用未知 GPIO 引脚。

按下面顺序得出结论：

1. Flash 校验、控制来源和 runtime 参数先通过。`valid_for_phase_conclusion=false`
   时保留数据并解释原因，不把它写成正常位置模式的结论。
2. 对照外部 `cmd_count_per_s`、`hal_ms_per_s`、`rtos_tick_per_s`、`cycles_per_s`。
   若 Cmd/HAL 都按相同比例偏慢，则优先查时钟源/计时基准；HAL≈1000、Cmd≈380
   且丢周期增加时，才有调度超时的直接证据。PC 读数误差含 SWD 往返窗口和至多一个
   Cmd 周期，关注 `max_read_bracket_ms` 与拟合残差，不声称微秒级外部相位标定。
3. 动态测试看 `reference_integral.scale` 和 `reference_integral_nominal_2ms.scale`。
   前者使用外部校准时间，后者强制每次更新 2 ms；两者差异可区分时间推进和数值参考
   问题。比例应接近 1；约 0.765 是显著异常。hold 的积分拟合不可用于判定。
4. 看 `post_vs_ruckig_*`、输入/上次输出连续性、`motor_reference_integral` 和实际
   velocity_scale，定位差异出在求解器、后处理、发布/更新频率还是坐标换算。
5. 检查 `speed_sum_identity_max_error`、`position_error_identity_max_error`（浮点
   残差通常应远小于 0.01），比较 PID 总输出、`Kp*error` 和 `Kd*d_error`。
   **当前 KD 非零**，不能只用 Kp 解释 PID；d_error 是每次计算的误差差，不除以 dt。
   MIT kp/kd 是电机内部另一层控制，也要记录。速度反馈应同时与 FF 和 `FF+PID`
   比较，不能把正常闭环修正当作电机跟踪失败。
6. 相位采用固定重叠窗、允许幅值和偏置不同，搜索 ±150 ms，输出符号约定为
   正=反馈落后、负=反馈提前。`at_search_edge=true` 或残差大时不要直接相信该单一
   数字，结合原始曲线及重复测试；不会根据一次截图替代测量。

## 已完成的验证与已知上游问题

本地已完成诊断开启的 Debug/Release 交叉编译，以及默认关闭时的 Debug 构建。
诊断开启时 RAM_D1 约 217,760 B / 320 KiB，DTCM 约 61.8 KiB；没有链接溢出。
新增离线测试覆盖所有 C/Python ABI 字段、相位符号、积分比例、外部计时/回绕、
限位/轨迹端点、真实 FP32 预演，以及实际固件 callback/冻结/保护锁存逻辑。
后者使用虚拟寄存器和 ASan/UBSan，还验证 18 秒保护跨多次 CYCCNT 回绕，以及缩短后的
四组 128 次 CPU 基准与取消操作。另有累计 30 秒预算检查；这些不等于已通过实机。

现有回归测试在**未修改的 `257463f` 基线**也存在：

- `test_joint_ruckig` 和 `test_arm_framework.test_six_axis_core` 链接失败：新 RC
  按键逻辑引用 `Etc_Get`，旧 host 测试构建没有链接对应实现。
- `test_c_parity.test_auto_and_passive_sources_against_python` 在第 163 行调用 C
  测试入口时崩溃。不能把此次全量回归标成通过；本分支未修改这些生产/测试文件。
- Release 还有既有 Ruckig Step2 `std::sort` 的数组边界告警及其他既有告警；这次
  并未改 vendor 算法。当前单轴更新不走 Step2，但报告须保留构建日志。

## 本地 agent 的交付

写 `App/test/JOINT1_HARDWARE_REPORT.md`，至少包括：固件 commit/ELF SHA256、编译等级、
实际 A/B 配置、测试姿态/幅度、全部 runtime 参数、外部测得的四个计数速率、周期
均值/P99/最大值、deadline misses、积分比例、相位符号/时差、反馈类型/年龄、PID
恒等式残差，以及 CPU 基准各场景的 mean/max µs 和百分比。明确哪些证据支持哪项
解释，哪些尚不能确认；没有硬件结果时不要填写推测值。

保留每次原始 bin/CSV/JSON、图、ELF 和日志，报告注明保存路径。`build/` 与 JSON
默认被 gitignore 忽略；按实际交付需要显式添加选定的测试证据或提供可访问的归档，
不要误提交串口凭据或其他项目文件。每完成一组测试及时提交报告并推送本任务分支，
核实 push 成功；不要擅自修改控制参数来使结果“通过”。

探针接口依据：[PyLink 官方 API](https://pylink.readthedocs.io/en/latest/pylink.html)。
使用其 SWD 连接与后台 memory_read32/memory_write32；运行中是否能稳定访问应以实际
探针/目标表现为准，读取失败时中止测试，不自动暂停目标重试。
