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

/* ==================== 一块板子上有几台机械臂 ====================
 *
 * 本工程可以**同时驱动两台完全同构的机械臂**（同样 6 关节 + 夹爪、同型号同量程同零点，
 * 只有走哪路 CAN 不同），每台一份完整的状态机 + 命令层 + 驱动层实例。
 *
 * **臂号就是 CAN 总线索引**：臂 A 走 CAN1(索引 0)，臂 B 走 CAN2(索引 1)。不另编一套号
 * 是为了少一层映射 —— 帧里的 arm_id 直接用这个值，电机的 config.bsp_can_index 也是它。
 *
 * 这个值也是**上线协议的 arm_id 的取值范围**（见 libs/frame/protocol_doc.md）：
 * 帧里的 arm_id 必须 < ARM_NUM，越界的帧被丢弃。 */
#define ARM_NUM 2

/* 臂号。值 == CAN 索引，见上。 */
typedef enum { ARM_A = 0, ARM_B = 1 } Arm_Index;

/* 一台臂的聚合对象(驱动 + 命令层 + 总模式状态机)，定义在 App/inc/arm.h。
 * 这里只做**前向声明**：命令层(gimbal_cmd.h)要拿 `Arm*` 当反指针找自己的模式源，但它
 * 不能 include arm.h —— arm.h 反过来要 include gimbal_cmd.h(要 Gimbal_cmd 的完整定义)，
 * 互相包含只会让其中一个拿到不完整的类型。字段用它当**指针**，前向声明就够。 */
typedef struct Arm Arm;

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

/* 机械臂自动动作，由总模式状态机调度。 */
typedef enum {
    arm_auto_none = 0,     // 待命
    arm_auto_reset,        // 六轴复位
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
    float joint_acc[ARM_JOINT_NUM];    // 关节加速度，deg/s²
    float gripper_angle;    // 夹爪角，deg，范围见 gimbal_cmd_def.h
    float gripper_torque;    // 夹爪力矩，N·m
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
