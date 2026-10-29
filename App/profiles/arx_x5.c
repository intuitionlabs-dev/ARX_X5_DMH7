#include "arm_profile.h"
#include "arm_initialization.h"
#include "profile_constants.h"
#include "can_config.h"
#include <stdbool.h>
#include "x5_model.h"

/* ARX-X5 电机的编码量程和反馈换算分母。 */
#define ARX_X5_MOTOR_POSITION_MAX_RAD 12.5f
#define ARX_X5_ENCOS_SPEED_MAX_RAD_S 18.0f
#define ARX_X5_DM_SPEED_MAX_RAD_S 45.0f
#define ARX_X5_ENCOS_TORQUE_MAX_NM 30.0f
#define ARX_X5_DM_TORQUE_MAX_NM 10.0f
#define ARX_X5_MOTOR_POSITION_DIVISOR 65536.0f
#define ARX_X5_MOTOR_SPEED_DIVISOR 4096.0f

/* ARX-X5 腕部关节和夹爪的对称角度范围。 */
#define ARX_X5_WRIST_LIMIT_DEG 90.0f
#define ARX_X5_GRIPPER_LIMIT_DEG 142.0f

/* 按枚举顺序配置 ARX-X5 的六个关节和夹爪电机。 */
static const Arm_motor_config motors[] = { // NOLINT(readability-identifier-naming)
    /* 关节 1 电机的驱动、总线标识、编码量程和模式增益。 */
    {.driver=&ARM_DRIVER_ENCOS, .model="ENCOS Type4",
        .tx_id=1, .rx_id=1, .tx_group=CAN_TX_500HZ_G0,
        .position_max=ARX_X5_MOTOR_POSITION_MAX_RAD,
        .speed_max=ARX_X5_ENCOS_SPEED_MAX_RAD_S,
        .torque_max=ARX_X5_ENCOS_TORQUE_MAX_NM,
        .kp_max=ARM_PROFILE_ENCOS_KP_MAX,
        .kd_max=ARM_PROFILE_ENCOS_KD_MAX,
        .current_max=ARM_PROFILE_MOTOR_CURRENT_MAX_A,
        .position_divisor=ARX_X5_MOTOR_POSITION_DIVISOR,
        .speed_divisor=ARX_X5_MOTOR_SPEED_DIVISOR,
        .cmd_multiturn=true,
        /* 配置该电机的位置环和速度环 PID。 */
        .position_pid={
            .KP=6.0f,
            .KI=0.0f,
            .KD=50.0f,
            .error_max=200.0f,
            .outputMax=1000.0f,
            .Irange=1.0f,
        },
        .speed_pid={
            .KP=0.0f,
            .KI=0.0f,
            .KD=0.0f,
            .error_max=0.0f,
            .outputMax=12.0f,
            .Irange=0.0f,
        },
        .kp=0.0f, .kd=35.0f, .damping_kd=35.0f},
    /* 关节 2 电机的驱动、总线标识、编码量程和模式增益。 */
    {.driver=&ARM_DRIVER_ENCOS, .model="ENCOS Type4",
        .tx_id=2, .rx_id=2, .tx_group=CAN_TX_250HZ_G1,
        .position_max=ARX_X5_MOTOR_POSITION_MAX_RAD,
        .speed_max=ARX_X5_ENCOS_SPEED_MAX_RAD_S,
        .torque_max=ARX_X5_ENCOS_TORQUE_MAX_NM,
        .kp_max=ARM_PROFILE_ENCOS_KP_MAX,
        .kd_max=ARM_PROFILE_ENCOS_KD_MAX,
        .current_max=ARM_PROFILE_MOTOR_CURRENT_MAX_A,
        .position_divisor=ARX_X5_MOTOR_POSITION_DIVISOR,
        .speed_divisor=ARX_X5_MOTOR_SPEED_DIVISOR,
        .cmd_multiturn=true,
        /* 配置该电机的位置环和速度环 PID。 */
        .position_pid={
            .KP=4.0f,
            .KI=0.0f,
            .KD=50.0f,
            .error_max=200.0f,
            .outputMax=1000.0f,
            .Irange=1.0f,
        },
        .speed_pid={
            .KP=0.0f,
            .KI=0.0f,
            .KD=0.0f,
            .error_max=0.0f,
            .outputMax=12.0f,
            .Irange=0.0f,
        },
        .kp=10.0f, .kd=50.0f, .damping_kd=50.0f},
    /* 关节 3 电机的驱动、总线标识、编码量程和模式增益。 */
    {.driver=&ARM_DRIVER_ENCOS, .model="ENCOS Type4",
        .tx_id=4, .rx_id=4, .tx_group=CAN_TX_250HZ_G1,
        .position_max=ARX_X5_MOTOR_POSITION_MAX_RAD,
        .speed_max=ARX_X5_ENCOS_SPEED_MAX_RAD_S,
        .torque_max=ARX_X5_ENCOS_TORQUE_MAX_NM,
        .kp_max=ARM_PROFILE_ENCOS_KP_MAX,
        .kd_max=ARM_PROFILE_ENCOS_KD_MAX,
        .current_max=ARM_PROFILE_MOTOR_CURRENT_MAX_A,
        .position_divisor=ARX_X5_MOTOR_POSITION_DIVISOR,
        .speed_divisor=ARX_X5_MOTOR_SPEED_DIVISOR,
        .cmd_multiturn=true,
        /* 配置该电机的位置环和速度环 PID。 */
        .position_pid={
            .KP=4.0f,
            .KI=0.0f,
            .KD=50.0f,
            .error_max=200.0f,
            .outputMax=1000.0f,
            .Irange=1.0f,
        },
        .speed_pid={
            .KP=0.0f,
            .KI=0.0f,
            .KD=0.0f,
            .error_max=0.0f,
            .outputMax=12.0f,
            .Irange=0.0f,
        },
        .kp=10.0f, .kd=50.0f, .damping_kd=50.0f},
    /* 关节 4 电机的驱动、总线标识、编码量程和模式增益。 */
    {.driver=&ARM_DRIVER_DM, .model="DM MIT",
        .tx_id=5, .rx_id=0, .tx_group=CAN_TX_250HZ_G2,
        .position_max=ARX_X5_MOTOR_POSITION_MAX_RAD,
        .speed_max=ARX_X5_DM_SPEED_MAX_RAD_S,
        .torque_max=ARX_X5_DM_TORQUE_MAX_NM,
        .kp_max=ARM_PROFILE_DM_KP_MAX,
        .kd_max=ARM_PROFILE_DM_KD_MAX,
        .current_max=ARM_PROFILE_MOTOR_CURRENT_MAX_A,
        .position_divisor=ARX_X5_MOTOR_POSITION_DIVISOR,
        .speed_divisor=ARX_X5_MOTOR_SPEED_DIVISOR,
        .cmd_multiturn=true,
        /* 配置该电机的位置环和速度环 PID。 */
        .position_pid={
            .KP=6.0f,
            .KI=0.08f,
            .KD=50.0f,
            .error_max=200.0f,
            .outputMax=1000.0f,
            .Irange=0.0f,
        },
        .speed_pid={
            .KP=0.0f,
            .KI=0.0f,
            .KD=0.0f,
            .error_max=0.0f,
            .outputMax=10.0f,
            .Irange=0.0f,
        },
        .kp=50.0f, .kd=0.8f, .damping_kd=0.8f},
    /* 关节 5 电机的驱动、总线标识、编码量程和模式增益。 */
    {.driver=&ARM_DRIVER_DM, .model="DM MIT",
        .tx_id=6, .rx_id=0, .tx_group=CAN_TX_250HZ_G2,
        .position_max=ARX_X5_MOTOR_POSITION_MAX_RAD,
        .speed_max=ARX_X5_DM_SPEED_MAX_RAD_S,
        .torque_max=ARX_X5_DM_TORQUE_MAX_NM,
        .kp_max=ARM_PROFILE_DM_KP_MAX,
        .kd_max=ARM_PROFILE_DM_KD_MAX,
        .current_max=ARM_PROFILE_MOTOR_CURRENT_MAX_A,
        .position_divisor=ARX_X5_MOTOR_POSITION_DIVISOR,
        .speed_divisor=ARX_X5_MOTOR_SPEED_DIVISOR,
        .cmd_multiturn=true,
        /* 配置该电机的位置环和速度环 PID。 */
        .position_pid={
            .KP=6.0f,
            .KI=0.08f,
            .KD=50.0f,
            .error_max=200.0f,
            .outputMax=1000.0f,
            .Irange=0.0f,
        },
        .speed_pid={
            .KP=0.0f,
            .KI=0.0f,
            .KD=0.0f,
            .error_max=0.0f,
            .outputMax=10.0f,
            .Irange=0.0f,
        },
        .kp=25.0f, .kd=0.8f, .damping_kd=0.8f},
    /* 关节 6 电机的驱动、总线标识、编码量程和模式增益。 */
    {.driver=&ARM_DRIVER_DM, .model="DM MIT",
        .tx_id=7, .rx_id=0, .tx_group=CAN_TX_250HZ_G3,
        .position_max=ARX_X5_MOTOR_POSITION_MAX_RAD,
        .speed_max=ARX_X5_DM_SPEED_MAX_RAD_S,
        .torque_max=ARX_X5_DM_TORQUE_MAX_NM,
        .kp_max=ARM_PROFILE_DM_KP_MAX,
        .kd_max=ARM_PROFILE_DM_KD_MAX,
        .current_max=ARM_PROFILE_MOTOR_CURRENT_MAX_A,
        .position_divisor=ARX_X5_MOTOR_POSITION_DIVISOR,
        .speed_divisor=ARX_X5_MOTOR_SPEED_DIVISOR,
        .cmd_multiturn=true,
        /* 配置该电机的位置环和速度环 PID。 */
        .position_pid={
            .KP=6.0f,
            .KI=0.08f,
            .KD=50.0f,
            .error_max=200.0f,
            .outputMax=1000.0f,
            .Irange=0.0f,
        },
        .speed_pid={
            .KP=0.0f,
            .KI=0.0f,
            .KD=0.0f,
            .error_max=0.0f,
            .outputMax=10.0f,
            .Irange=0.0f,
        },
        .kp=10.0f, .kd=1.0f, .damping_kd=1.0f},
    /* 夹爪电机的驱动、总线标识、编码量程和模式增益。 */
    {.driver=&ARM_DRIVER_DM, .model="DM MIT",
        .tx_id=8, .rx_id=0, .tx_group=CAN_TX_250HZ_G3,
        .position_max=ARX_X5_MOTOR_POSITION_MAX_RAD,
        .speed_max=ARX_X5_DM_SPEED_MAX_RAD_S,
        .torque_max=ARX_X5_DM_TORQUE_MAX_NM,
        .kp_max=ARM_PROFILE_DM_KP_MAX,
        .kd_max=ARM_PROFILE_DM_KD_MAX,
        .current_max=ARM_PROFILE_MOTOR_CURRENT_MAX_A,
        .position_divisor=ARX_X5_MOTOR_POSITION_DIVISOR,
        .speed_divisor=ARX_X5_MOTOR_SPEED_DIVISOR,
        .cmd_multiturn=true,
        /* 配置该电机的位置环和速度环 PID。 */
        .position_pid={
            .KP=6.0f,
            .KI=0.08f,
            .KD=50.0f,
            .error_max=200.0f,
            .outputMax=1000.0f,
            .Irange=0.0f,
        },
        .speed_pid={
            .KP=0.0f,
            .KI=0.0f,
            .KD=0.0f,
            .error_max=0.0f,
            .outputMax=10.0f,
            .Irange=0.0f,
        },
        .kp=8.0f, .kd=0.1f, .damping_kd=0.1f},
};

/* 配置 ARX-X5 的关节方向、限位、复位角和跟踪参数。 */
static const Arm_joint_config joints[] = { // NOLINT(readability-identifier-naming)
    /* 关节 1 的坐标换算、角度范围和轨迹参数。 */
    {.direction=ARM_PROFILE_FORWARD_DIRECTION,
        .reduction=ARM_PROFILE_DIRECT_REDUCTION,
        .minimum_deg=-115.0f,
        .maximum_deg=90.0f,
        .reset_deg=0.0f,
        .torque_scale=ARM_PROFILE_DEFAULT_TORQUE_SCALE,
        .velocity_scale=ARM_PROFILE_DEFAULT_VELOCITY_SCALE,
        .plan_velocity_scale=ARM_PROFILE_DEFAULT_PLAN_VELOCITY_SCALE,
        .td_r=1000.0f,
        .absolute_encoder=true},
    /* 关节 2 的坐标换算、角度范围和轨迹参数。 */
    {.direction=ARM_PROFILE_FORWARD_DIRECTION,
        .reduction=ARM_PROFILE_DIRECT_REDUCTION,
        .minimum_deg=0.5f,
        .maximum_deg=206.26f,
        .reset_deg=0.5f,
        .torque_scale=ARM_PROFILE_DEFAULT_TORQUE_SCALE,
        .velocity_scale=ARM_PROFILE_DEFAULT_VELOCITY_SCALE,
        .plan_velocity_scale=ARM_PROFILE_DEFAULT_PLAN_VELOCITY_SCALE,
        .td_r=4000.0f,
        .absolute_encoder=true},
    /* 关节 3 的坐标换算、角度范围和轨迹参数。 */
    {.direction=ARM_PROFILE_FORWARD_DIRECTION,
        .reduction=ARM_PROFILE_DIRECT_REDUCTION,
        .minimum_deg=0.5f,
        .maximum_deg=171.88f,
        .reset_deg=0.5f,
        .torque_scale=ARM_PROFILE_DEFAULT_TORQUE_SCALE,
        .velocity_scale=ARM_PROFILE_DEFAULT_VELOCITY_SCALE,
        .plan_velocity_scale=ARM_PROFILE_DEFAULT_PLAN_VELOCITY_SCALE,
        .td_r=4000.0f,
        .absolute_encoder=true},
    /* 关节 4 的坐标换算、角度范围和轨迹参数。 */
    {.direction=ARM_PROFILE_FORWARD_DIRECTION,
        .reduction=ARM_PROFILE_DIRECT_REDUCTION,
        .minimum_deg=-ARX_X5_WRIST_LIMIT_DEG,
        .maximum_deg=ARX_X5_WRIST_LIMIT_DEG,
        .reset_deg=0.0f,
        .torque_scale=ARM_PROFILE_DEFAULT_TORQUE_SCALE,
        .velocity_scale=ARM_PROFILE_DEFAULT_VELOCITY_SCALE,
        .plan_velocity_scale=ARM_PROFILE_DEFAULT_PLAN_VELOCITY_SCALE,
        .td_r=2000.0f,
        .absolute_encoder=true},
    /* 关节 5 的坐标换算、角度范围和轨迹参数。 */
    {.direction=ARM_PROFILE_FORWARD_DIRECTION,
        .reduction=ARM_PROFILE_DIRECT_REDUCTION,
        .minimum_deg=-ARX_X5_WRIST_LIMIT_DEG,
        .maximum_deg=ARX_X5_WRIST_LIMIT_DEG,
        .reset_deg=0.0f,
        .torque_scale=ARM_PROFILE_DEFAULT_TORQUE_SCALE,
        .velocity_scale=ARM_PROFILE_DEFAULT_VELOCITY_SCALE,
        .plan_velocity_scale=ARM_PROFILE_DEFAULT_PLAN_VELOCITY_SCALE,
        .td_r=2000.0f,
        .absolute_encoder=true},
    /* 关节 6 的坐标换算、角度范围和轨迹参数。 */
    {.direction=ARM_PROFILE_FORWARD_DIRECTION,
        .reduction=ARM_PROFILE_DIRECT_REDUCTION,
        .minimum_deg=-112.87f,
        .maximum_deg=119.17f,
        .reset_deg=0.0f,
        .torque_scale=ARM_PROFILE_DEFAULT_TORQUE_SCALE,
        .velocity_scale=ARM_PROFILE_DEFAULT_VELOCITY_SCALE,
        .plan_velocity_scale=ARM_PROFILE_DEFAULT_PLAN_VELOCITY_SCALE,
        .td_r=2000.0f,
        .absolute_encoder=true},
};

/* 逐轴配置重力补偿标定系数。 */
static const float gravity_scale[] = {0.8f, 0.8f, 0.8f, 1.32f, 1.32f, 1.32f}; // NOLINT(readability-identifier-naming)

/* 将 URDF 模型数据与机型补偿参数关联。 */
static const Arm_dynamics_model dynamics = { // NOLINT(readability-identifier-naming)
    .joint_count=sizeof(x5_links)/sizeof(x5_links[0]), .links=x5_links,
    .gravity_scale=gravity_scale,
};

/* 汇总 ARX-X5 机型参数和初始化操作。 */
const Arm_profile ARM_PROFILE_ARX_X5 = {
    .name="ARX-X5",
    .joint_count=sizeof(joints)/sizeof(joints[0]), .motor_count=sizeof(motors)/sizeof(motors[0]),
    .motors=motors, .joints=joints,
    /* 配置单电机夹爪的角度映射、力矩比例和限位。 */
    .gripper={
        .cmd=ARM_PROFILE_GRIPPER_IDENTITY_POINTS,
        .motor_deg=ARM_PROFILE_GRIPPER_IDENTITY_POINTS,
        .torque_scale=ARM_PROFILE_DEFAULT_TORQUE_SCALE,
        .minimum=-ARX_X5_GRIPPER_LIMIT_DEG, .maximum=ARX_X5_GRIPPER_LIMIT_DEG},
    /* 配置复位时长、设零时限和夹爪初始化判据。 */
    .init={
        .reset_time_s=ARM_PROFILE_RESET_TIME_S,
        .pose_tolerance_deg=ARM_PROFILE_INIT_POSE_TOLERANCE_DEG,
        .zero_total_ms=ARM_PROFILE_ZERO_TOTAL_TIMEOUT_MS,
        .zero_motor_ms=ARM_PROFILE_ZERO_MOTOR_TIMEOUT_MS,
        .zero_disable_timeout_ms=ARM_PROFILE_ZERO_DISABLE_TIMEOUT_MS,
        .zero_disable_retry_ms=ARM_PROFILE_ZERO_DISABLE_RETRY_MS,
        .gripper_home_torque=ARM_PROFILE_GRIPPER_HOME_TORQUE_NM,
        .gripper_speed_deg_s=ARM_PROFILE_GRIPPER_SPEED_DEG_S,
        .gripper_backoff_deg=ARM_PROFILE_GRIPPER_BACKOFF_DEG,
        .gripper_follow_error_deg=ARM_PROFILE_GRIPPER_FOLLOW_ERROR_DEG,
        .gripper_backoff_tolerance_deg=ARM_PROFILE_GRIPPER_BACKOFF_TOLERANCE_DEG,
        .gripper_settle_ms=ARM_PROFILE_GRIPPER_SETTLE_MS,
        .stall_tolerance_deg=ARM_PROFILE_GRIPPER_STALL_TOLERANCE_DEG,
        .stall_window_ms=ARM_PROFILE_GRIPPER_STALL_WINDOW_MS,
        .stall_windows=ARM_PROFILE_GRIPPER_STALL_WINDOWS,
        .seek_timeout_ms=ARM_PROFILE_GRIPPER_SEEK_TIMEOUT_MS,
        .backoff_timeout_ms=ARM_PROFILE_GRIPPER_BACKOFF_TIMEOUT_MS},
    /* 配置动力学模型、基座重力和位置参考时长。 */
    .dynamics=&dynamics, .gravity={0,0,ARM_PROFILE_BASE_GRAVITY_Z_M_S2},
    .quintic_duration_s=ARM_PROFILE_QUINTIC_DURATION_S,
    /* 登记初始化取消和推进回调。 */
    .cancel_initialize=Arm_Initialization_Cancel,
    .initialize=Arm_Initialize_Home,
    /* 配置掉线保护和遥控器关节映射。 */
    .stop_on_motor_lost=true,
    .rc_joint={JOINT1,JOINT2,JOINT3},
    .rc_speed_deg_s=ARM_PROFILE_RC_SPEED_DEG_S,
};
