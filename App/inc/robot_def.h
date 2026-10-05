#pragma once

/* 六轴机械臂(+夹爪)的枚举 / 结构体定义。
 *
 * 从 ec_code/engineer-wheel-leg-v-1/APP/robot_def.h 里挑出来的**臂相关部分**——
 * 那个文件同时装着底盘、裁判系统、UI、以太网一大堆东西，本工程用不到，没搬。
 *
 * 这个头里**只有类型，没有常量数组**，所以谁 include 都安全。板级标定量(电机方向、
 * 减速比、零点、前馈系数)在 App/inc/gimbal_arm_def.h，那个是"每个包含它的编译单元
 * 各得一份副本"，只给 gimbal_arm.c 用。
 *
 * 机械臂的指令来源：源工程走 pub_sub(register_pub / register_sub)，本工程没有这个
 * 框架，改由命令层(App/gimbal_cmd.c)每拍直接写 Gimbal.arm_control_data。
 */

#include <stdint.h>

/* 源工程里由 config/global_config.h 提供 */
#define MAYBE_UNUSED __attribute__((unused))
/* 源工程里 C++ 下展开成 constexpr、C 下展开成 static const */
#define CONSTEXPR static const

/* 机械臂任一电机掉线是否停机 */
#define ANY_MOTOR_LOST_STOP_ENABLE 1

/* 机械臂工作模式 —— 描述的是**电机怎么出力**，不是机器人"现在在干什么"。
 * "现在该干什么"是更高一层的 Robot_state（App/inc/robot_state.h），它通过
 * gimbal_cmd->mode 下发到这里的 mode 上。两层别混：同一个 robot_state_normal
 * 既可以配 arm_damping（正常运行、但默认阻尼）也可以配 arm_run。
 *
 * 四个值的物理含义（实现见 gimbal_arm.c 的 set_pid_* 与 Set_Motor_ref）：
 *   arm_stop             电机失能，松劲
 *   arm_run              正常位控：位置环 + 速度环，带重力/粘滞前馈
 *   arm_damping          使能但位置环全关，只留关节阻尼 kd —— 被动拖拽/掉线兜底
 *   arm_only_gravity_comp 使能但位置环全关、kd 也给 0，只重力前馈顶住自重
 *
 * arm_stop=0 / arm_run=1 **不能动**：JLink 上那些看模式的老偏移、老的
 * `if (mode == arm_stop)` 判断都依赖这两个值，所以新值只能往后加。 */
typedef enum { arm_stop = 0, arm_run, arm_damping, arm_only_gravity_comp } Arm_mode;

/* 机械臂一键模式（源工程 robot_def.h 里的 Arm_auto_mode）。
 * 那边一共 12 个：reset / stretch / store / upstairs / protect / pop /
 * fetch_down / fetch_up / first_ore / gripper_down / turn_around，本工程只做**复位**
 * 和**夹爪找零**。
 * 剩下的那些要么依赖 Plan_pkg(存矿/弹矿/取家矿)、要么依赖底盘(上台阶)、要么依赖图传
 * (一键转身)，本工程一个都没有，所以只留名字在这儿备查。
 * 枚举值和源工程一致（arm_auto_reset 在那边也是 1），以后按名字补回来不会串值。
 * arm_auto_gripper_zero 在源工程里叫 arm_auto_gripper_down，本工程改了名：这边只做
 * "夹爪撞到限位后把那一点设成零点"，不做源工程那套"往下压一颗矿石"。 */
typedef enum {
    arm_auto_none = 0,     // 待命
    arm_auto_reset,        // 复位
    arm_auto_gripper_zero, // 夹爪找零（**尚是空壳**，见 arm_auto.c）
    ARM_AUTO_MODE_NUM,
} Arm_auto_mode;

/* 机械臂电机索引。GRIPPER 故意和 ARM_JOINT_NUM 同值：它是第 7 个电机，但不属于 6 个
 * 关节，所以以关节数结尾的循环不会碰到它，以电机数结尾的循环会。 */
// NOLINTBEGIN
enum Arm_Motor_Index {
    JOINT1 = 0,
    JOINT2,
    JOINT3,
    JOINT4,
    JOINT5,
    JOINT6,
    ARM_JOINT_NUM,            // 6，关节数
    GRIPPER = ARM_JOINT_NUM,  // 6，夹爪
    ARM_MOTOR_NUM             // 7，电机总数
};
// NOLINTEND

/* 云台 cmd -> arm：指令 */
typedef struct {
    Arm_mode mode;
    float joint_angle[ARM_JOINT_NUM];  // 关节角度，deg
    float joint_vel[ARM_JOINT_NUM];    // 关节速度，deg/s
    float joint_acc[ARM_JOINT_NUM];    // 关节加速度
    float gripper_angle;    // 夹爪开合角，deg，范围 GRIPPER_ANGLE_MIN..MAX(-284..0)
} Arm_control_data;

/* arm -> 云台 cmd：反馈 */
typedef struct {
    Arm_mode mode;
    uint8_t is_motor_lost[ARM_MOTOR_NUM];  // 电机掉线标志
    uint8_t motor_inited[ARM_MOTOR_NUM];   // 初始化状态，0 未初始化 / 1 已初始化
    float joint_angle[ARM_JOINT_NUM];      // 关节角度，deg
    float joint_vel[ARM_JOINT_NUM];        // 关节角速度，deg/s（上位机规划速度前馈的起点）
    float gripper_angle;                   // 夹爪实际角，deg
    uint8_t motor_temp[ARM_MOTOR_NUM];     // 电机温度，℃
} Arm_feedback_data;
