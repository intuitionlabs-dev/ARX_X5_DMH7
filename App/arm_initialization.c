#include "arm_initialization.h"
#include <math.h>
#include "main.h" // IWYU pragma: keep
#include <string.h>

/* 清空初始化状态并启动关节复位。 */
static void reset_begin(Gimbal_cmd* cmd) {
    memset(&cmd->initialization, 0, sizeof(cmd->initialization));
    cmd->auto_step = 0;
    cmd->auto_failed = 0;
    cmd->auto_mode = arm_auto_reset;
}

/* 执行关节复位并保持夹爪入口位置。 */
int Arm_Initialize_Hold(Gimbal_cmd* cmd, uint8_t start) {
    /* 收到启动标志时初始化复位流程。 */
    if (start) { reset_begin(cmd); return 0; }
    /* 将自动规划失败传递给上层状态机。 */
    if (cmd->auto_failed) return -1;
    /* 等待关节复位结束。 */
    if (cmd->auto_mode != arm_auto_none) return 0;
    /* 关节复位结束后标记夹爪位置有效。 */
    cmd->gimbal->arm_feedback_data.motor_inited[GRIPPER] = 1;
    return 1;
}

/* 发布夹爪电机坐标目标和搜索力矩。 */
static void home_reference(Gimbal_cmd* cmd, float motor_target, float torque) {
    cmd->initialization.home_active = 1;
    cmd->initialization.home_target_deg = motor_target;
    cmd->initialization.home_torque = torque;
    cmd->gimbal->arm_control_data.gripper_angle =
        Arm_Gripper_From_Motor(cmd->gimbal->profile, motor_target);
    cmd->gimbal->arm_control_data.gripper_torque = torque;
}

/* 执行关节复位、夹爪寻限位和设零。 */
int Arm_Initialize_Home(Gimbal_cmd* cmd, uint8_t start) {
    /* 收到启动标志时初始化复位流程。 */
    if (start) { reset_begin(cmd); return 0; }
    /* 将自动规划失败传递给上层状态机。 */
    if (cmd->auto_failed) return -1;
    Gimbal* g = cmd->gimbal;
    const Arm_init_policy* policy = &g->profile->init;
    const uint32_t now = HAL_GetTick();
    /* 读取夹爪反馈并计算本周期目标增量。 */
    const float position = g->motors[GRIPPER].control->fdb_position;
    const float step = policy->gripper_speed_deg_s*ARM_CONTROL_PERIOD_S;
    /* 关节复位完成后初始化夹爪搜索状态。 */
    if (cmd->initialization.step == 0) {
        /* 等待关节复位结束。 */
        if (cmd->auto_mode != arm_auto_none) return 0;
        g->arm_feedback_data.motor_inited[GRIPPER] = 0;
        cmd->initialization.start_ms = cmd->initialization.window_ms = now;
        cmd->initialization.window_position = position;
        cmd->initialization.backoff_target = position;
        cmd->initialization.step = 1;
    }
    /* 按夹爪初始化步骤推进非阻塞动作。 */
    switch (cmd->initialization.step) {
    /* 递增夹爪目标并检测机械限位。 */
    case 1:
        cmd->initialization.backoff_target = fminf(position+policy->gripper_follow_error_deg,
            cmd->initialization.backoff_target+step);
        home_reference(cmd, cmd->initialization.backoff_target, policy->gripper_home_torque);
        /* 到达检测周期后更新停滞计数。 */
        if (now-cmd->initialization.window_ms >= policy->stall_window_ms) {
            /* 位移小于容差时累计停滞窗口。 */
            if (fabsf(position-cmd->initialization.window_position) < policy->stall_tolerance_deg)
                ++cmd->initialization.stall_windows;
            /* 检测到有效位移时清除停滞累计。 */
            else cmd->initialization.stall_windows = 0;
            cmd->initialization.window_ms = now;
            cmd->initialization.window_position = position;
        }
        /* 连续停滞达到阈值后转入回退。 */
        if (cmd->initialization.stall_windows >= policy->stall_windows) {
            cmd->initialization.backoff_target = position-policy->gripper_backoff_deg;
            cmd->initialization.start_ms = now;
            cmd->initialization.step = 2;
        } else if (now-cmd->initialization.start_ms >= policy->seek_timeout_ms) return -1; /* 搜索超过配置时限时报告失败。 */
        break;
    /* 夹爪按配置速度退回设定距离。 */
    case 2: {
        const float current = Arm_Gripper_To_Motor(g->profile, g->arm_control_data.gripper_angle);
        home_reference(cmd, fmaxf(cmd->initialization.backoff_target, current-step), 0);
        /* 回退到位后仅发布夹爪设零请求。 */
        if (fabsf(position-cmd->initialization.backoff_target) <= policy->gripper_backoff_tolerance_deg) {
            memset(g->zero.selection, 0, sizeof(g->zero.selection));
            g->zero.selection[GRIPPER] = g->zero.selection_valid = 1;
            __atomic_thread_fence(__ATOMIC_RELEASE);
            g->zero.request = 1;
            cmd->initialization.start_ms = now;
            cmd->initialization.step = 3;
        } else if (now-cmd->initialization.start_ms >= policy->backoff_timeout_ms) return -1; /* 回退超过配置时限时报告失败。 */
        break;
    }
    /* 保持夹爪位置并等待设零结果。 */
    case 3:
        home_reference(cmd, position, 0);
        /* 设零退出后校验结果并开始稳定计时。 */
        if (!Arm_Zero_Busy(g)) {
            const uint8_t result = g->zero.result[GRIPPER];
            /* 设零状态或结果异常时报告失败。 */
            if (g->zero.state != ARM_ZERO_DONE || (result != ARM_ZERO_RES_OK
                && result != ARM_ZERO_RES_UNCONFIRMED)) return -1;
            cmd->initialization.start_ms = now;
            cmd->initialization.step = 4;
        } else if (now-cmd->initialization.start_ms >= policy->zero_total_ms+1000u) return -1; /* 设零等待超过总时限及保护余量时报告失败。 */
        break;
    /* 保持夹爪位置直至稳定时间结束。 */
    case 4:
        home_reference(cmd, position, 0);
        /* 稳定时间满足后标记夹爪初始化完成。 */
        if (now-cmd->initialization.start_ms >= policy->gripper_settle_ms) {
            g->arm_feedback_data.motor_inited[GRIPPER] = 1;
            return 1;
        }
        break;
    /* 步骤索引异常时报告初始化失败。 */
    default:
        return -1;
    }
    return 0;
}

/* 停止初始化输出并使夹爪目标跟随反馈。 */
void Arm_Initialization_Cancel(Gimbal_cmd* cmd) {
    cmd->initialization.home_active = 0;
    cmd->auto_mode = arm_auto_none;
    cmd->auto_step = 0;
    cmd->gimbal->arm_control_data.gripper_torque = 0;
    cmd->gimbal->arm_control_data.gripper_angle =
        cmd->gimbal->arm_feedback_data.gripper_angle;
}
