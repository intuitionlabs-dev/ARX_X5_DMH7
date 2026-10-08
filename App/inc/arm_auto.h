#pragma once

/* 机械臂「一键模式」的动作本体 —— 从源工程 APP/gimbal_cmd/arm_auto.c 搬来的**框架**，
 * 本工程实现了两个动作：**一键复位**(Arm_Reset) 和 **一键夹爪找零**(Arm_Gripper_Zero)。
 *
 * 框架长什么样：每个动作都是一个**非阻塞的多步状态机**，靠 Gimbal_cmd 里三个字段驱动 ——
 *   auto_mode       当前在跑哪个动作(arm_auto_none = 没在跑)
 *   last_auto_mode  上一拍的值；两者不等就说明"刚进入这个动作"，把 auto_step 清零重来
 *   auto_step       走到第几步
 * 每一步要么"起一段规划"(plan_temp)，要么"等这段规划跑完"(group_is_cplt)，
 * 最后用 default 分支把自己退出(auto_mode = arm_auto_none)。所以整个动作是每拍推一格、
 * 不阻塞、不占任何延时 —— 它跑在 CmdTask 的 500 Hz 里，和摇杆点动是同一个任务。
 *
 * 规划怎么送到电机：起规划之后**每拍**由 set_motor_ref() 把规划器的当前点写进
 * ctrl_src.joint_src，同时把速度/加速度写进 obj->plan_joint_vel/acc[]；gimbal_cmd.c 的
 * Plan_Running_Check() 发现规划在跑就把 plan_running 置 1，Arm_Ref_Set() 于是绕过关节角
 * 微分器(TD)、直接用规划器给的量和原始目标角。规划结束那一拍 plan_running 回 0、TD 已经
 * 被 joint_angle_td_follow() 对齐过了，所以不会有跳变。
 *
 * 谁触发：遥控器上**没有**触发入口(只有 3 个摇杆 + swA，而 swA 已经被启停开关占了)，
 * 所以 auto_mode 由上层写 —— 现在是 App/robot_state.c 的初始化流程在写
 * （`gimbal_cmd->auto_mode = arm_auto_reset`，跑完它自己回到 arm_auto_none，再写
 * arm_auto_gripper_zero）。要用调试器手工触发，先按 nm 取 gimbal_cmd 指针读出结构体
 * 基址再写偏移(见 memory arm-auto-framework)。
 *
 * 注意：**arm_stop 档位下 Stop_Mode_Update() 每拍都会把 auto_mode 清 0**，所以要么把
 * rc.sw_a 打在 arm_run 档再写，要么写进去后尽快切到 arm_run。
 */

#include "gimbal_cmd.h"

/* 一键复位的轨迹时长，s。源工程是 reset_time_calc() 按腕点位置的多项式算出来的
 * (0.5~1.5 左右)，本工程没有正运动学，而且复位是**一段**五次样条直接从当前位姿走到
 * 复位姿态、没有联动限位要绕，所以固定。 */
#define ARM_AUTO_RESET_TIME_S 2

/* 复位姿态 = **预设初始化位置**，deg，6 个关节。
 *
 * 两个用法的同一份数据：
 *   - Arm_Reset() 拿它当轨迹终点；
 *   - App/robot_state.c 拿它当"收到初始化命令时臂该在哪儿"的基准 —— 偏差超过
 *     ROBOT_INIT_POSE_TOL_DEG 就拒绝这次初始化（防止上位机在臂还歪着的时候让它动）。
 *
 * 定义在 arm_auto.c（.rodata，只读）。 */
extern const float ARM_RESET_ANGLE[ARM_JOINT_NUM];

/* 一键复位：从当前位姿起一段 ARM_AUTO_RESET_TIME_S 的轨迹，走到 ARM_RESET_ANGLE，
 * 跑完自己退出(auto_mode 回 arm_auto_none)。每拍调一次(CmdTask 500 Hz)，
 * 由 gimbal_cmd.c 的 Auto_Mode_Update() 在 auto_mode == arm_auto_reset 时调。 */
void Arm_Reset(Gimbal_cmd* obj);

/* 一键夹爪找零：**推到抵住机械限位 → 往张开方向退回半个行程 → 把那里设成零点**。
 *
 *   推：每拍把夹爪目标角往**闭合**方向推一小步(最多领先反馈 20°)、叠一个闭合力矩前馈，
 *       同时看夹爪的**反馈角在 0.2 s 的窗口里动了没有**；连续 3 个窗口没动就认定抵住了
 *       机械限位。判据只用反馈角、不用力矩/电流 —— 与电机型号和 kp 无关。
 *   退：从那个**反馈角**往回退 (GRIPPER_ANGLE_MAX - GRIPPER_ANGLE_MIN)/2 = 半个行程。
 *   设零：把 App/arm_zero.c 那个设零状态机叫起来，用 gimbal->zero.participate_override 把这一轮
 *       **缩到夹爪一台** —— 达妙 0xFE 和常规控制帧互斥、只能在 MotorTask 里发，所以这一步
 *       不自己发帧。代价是这一轮整条 CAN 安静 0.1~1.5 s(别的电机收不到常规帧)。
 *
 * 设完之后夹爪的零点在行程中点上，所以闭合端读 +142、张开端读 -142 ——
 * gimbal_cmd_def.h 的 GRIPPER_ANGLE_MIN/MAX 就是按这个口径写的。
 *
 * 夹爪的"已初始化"标志(motor_inited[GRIPPER])由这个动作**自己维护**：起手置 0(零点正在被
 * 改写，之前的口径作废)、设零成功才置 1。置 1 之后 0x80 反馈帧的 motor_inited_mask 会报
 * 夹爪已初始化，gimbal_cmd.c 的 ±142 夹爪软限位也开始生效。
 *
 * 调试器里可读的跨拍状态在 `Gimbal_cmd::gz`(定义见 App/inc/gimbal_cmd.h)：读
 * `g_arm[N].gimbal_cmd->gz.stall_angle`(认到的机械限位角)、`->gz.back_target`(退到哪儿)、
 * `->gz.zero_result`(那一轮设零的结果码，见 arm_zero.h)。**per 台**：两台臂各跑各的找零，
 * 各有各的进度。
 * **第二次跑 gz_zero_result 会是 1(UNCONFIRMED)**：那时夹爪本来就在中点上(读数≈0)，
 * "写没写进去"分辨不出来，arm_zero 如实不谎报成功 —— 不是故障。
 *
 * 退出路径：设完零点 / **推开超时 10 s** / 退回超时 6 s / 等设零超时 12 s / 夹爪掉线 /
 * 当前不在 arm_run 档(被动模式下位置环是关的，"反馈不动"会变成必然，会误判成找到限位)。
 * 超时都是为了不把 robot_state 的初始化流程(它等着 auto_mode 回 arm_auto_none)拖死。 */
void Arm_Gripper_Zero(Gimbal_cmd* obj);
