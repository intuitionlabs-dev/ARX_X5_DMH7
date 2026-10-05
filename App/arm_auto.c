#include "arm_auto.h"

#include <string.h> /* memcpy */

#include "gimbal_cmd_def.h" /* JOINT_ANGLE_MIN/MAX(拼复位姿态用的软限位) */
#include "move_plan.h"      /* move_plan_run / quintic_spline / Move_plan */

/* ===================== 相对于源工程 arm_auto.c 砍掉了什么 =====================
 *
 * 源工程那个文件 784 行、12 个一键动作，本工程**只留框架 + 一键复位**。逐个交代：
 *
 *   Arm_Reset                 搬来了，但**改成一段轨迹**。源工程是「j3 先抬出干涉区 →
 *                             复位 j4~j6 → 再整体复位 j1~j6」三段，每段之间还插一个
 *                             "等规划跑完"的空步。本工程的臂是典型 6 轴臂、**只有单关节
 *                             限位、没有联动限位**，没有要绕的干涉区，所以直接一段五次
 *                             样条从当前位姿走到 RESET_ANGLE，时长固定(见 arm_auto.h
 *                             的 ARM_AUTO_RESET_TIME_S)。
 *   is_motor_inplace()        ★ **源工程里也是零调用方**(标着 MAYBE_UNUSED 的到位检查)
 *
 * 框架本身(状态机骨架 / plan_temp / set_motor_ref / group_is_cplt / clear_qd_qdd)
 * 一字不改地留着 —— 以后要加动作就照 Arm_Reset 这副骨架抄。
 */

CONSTEXPR float RESET_ANGLE[ARM_JOINT_NUM] = {
    0, JOINT_ANGLE_MIN[JOINT2], JOINT_ANGLE_MIN[JOINT3], 0, 0, 0};

/* 起一段临时规划：从**当前反馈关节角**走到 end，用五次样条，时长 t 秒。
 * Plan_Group_Temp_Calc_And_Run() 内部给每根轴各建一个临时规划，所以一次调用要吃掉
 * ARM_JOINT_NUM 个临时规划名额(池子 MAX_TEMP_PLAN_NUM = 30，规划跑完由 500 Hz 的
 * Move_Plan_Update() 回收 —— 那个任务没跑的话，第一轮之后再也规划不出来)。 */
static void plan_temp(Gimbal_cmd* obj, const float end[ARM_JOINT_NUM], float t) {
    float start_pos[ARM_JOINT_NUM];
    float end_pos[ARM_JOINT_NUM];
    /* 两份都拷进局部数组：起点那份是不想动反馈数组，终点那份是因为接口参数是
     * float[] 非 const，直接把 static const 的 RESET_ANGLE 传进去会丢限定符。 */
    memcpy(start_pos, obj->gimbal->arm_feedback_data.joint_angle, sizeof(start_pos));
    memcpy(end_pos, end, sizeof(end_pos));
    Plan_Group_Temp_Calc_And_Run(obj->temp_group, (uint8_t)ARM_JOINT_NUM, start_pos, end_pos,
        t, quintic_spline);
}

/* 把规划组的当前点搬进控制源：位置写 joint_src，速度/加速度写 plan_joint_vel/acc[]。
 * gimbal_cmd.c 的 Ctrl_Src_Calc() 会把 joint_src 解算成 joint_angle_solved，
 * Arm_Ref_Set() 在 plan_running 时直接用它(不过关节角微分器)。 */
static void set_motor_ref(Gimbal_cmd* obj, Move_plan** group) {
    if (group == NULL) return;
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        if (group[i] == NULL) return;
    }
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->ctrl_src.joint_src.joint_angle[i] = group[i]->current_pos;
        obj->plan_joint_vel[i] = group[i]->current_vel;
        obj->plan_joint_acc[i] = group[i]->current_acc;
    }
}

/* 规划跑完那一拍清零速度和加速度，免得规划结束后前馈还挂着一个末速度。 */
static void clear_qd_qdd(Gimbal_cmd* obj) {
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->plan_joint_vel[i] = 0;
        obj->plan_joint_acc[i] = 0;
    }
}

/* 规划组是否已经跑完。本工程只用**同步**规划组(每根轴同时起同时停)，所以看第一根轴
 * 的状态就够了。status 不是 move_plan_run 有两种情况：还没起(Run 之前)和已经跑完，
 * 两种都算"没在跑"。 */
static uint8_t group_is_cplt(Move_plan** group) {
    if (group == NULL || group[0] == NULL) return 0;
    return (group[0]->status != move_plan_run) ? 1 : 0;
}

void Arm_Reset(Gimbal_cmd* obj) {
    /* 刚进入这个模式(上一拍还不是它)：从头开始，别接着上次的步数跑 */
    if (obj->last_auto_mode != obj->auto_mode) {
        obj->auto_step = 0;
    }

    obj->ctrl_type = arm_ctrl_joint;  // 复位用关节空间控制

    switch (obj->auto_step) {
    case 0:  // 起一段从当前位姿到复位姿态的规划
        plan_temp(obj, RESET_ANGLE, ARM_AUTO_RESET_TIME_S);
        obj->auto_step++;
        break;
    case 1:  // 每拍把规划器的当前点喂进控制源，等这段规划跑完
        set_motor_ref(obj, obj->temp_group);
        if (group_is_cplt(obj->temp_group)) {
            clear_qd_qdd(obj);
            obj->auto_step++;
        }
        break;
    default:  // 跑完(或步数越界)：退出这个模式，交回给上层
        obj->auto_mode = arm_auto_none;
        obj->auto_step = 0;
        break;
    }
}
