#include "arm_auto.h"
#include <math.h>           /* fabsf(夹爪找零判定用) */
#include <string.h>         /* memcpy */
#include "arm_zero.h"       /* 夹爪设零那一轮(arm_zero_request / _state / 参与集覆盖) */
#include "DM_motor.h"       /* 夹爪的掉线看门狗(DM_motor::monitor) */
#include "gimbal_cmd_def.h" /* JOINT_ANGLE_MIN/MAX(拼复位姿态)、GRIPPER_ANGLE_*(夹爪找零) */
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

/* 复位姿态（也是"预设初始化位置"）。
 *
 * **原来这里是 CONSTEXPR(=static const)**，现在去掉了 static：总模式状态机
 * (App/robot_state.c) 拿它当"初始化前该在哪儿"的基准，收到初始化命令帧时要拿当前反馈角
 * 和它逐个比，偏差太大就拒绝。声明在 arm_auto.h 的 extern，与 C 相比 `const` 全局在
 * .rodata(Flash)里，只读、线程安全。
 *
 * 值是用户定的口径：j2/j3 压到各自的**软限位下限**（最收拢的姿态），其余回 0。
 * 比源工程那套"j2 打满 206°"温和得多。 */
const float ARM_RESET_ANGLE[ARM_JOINT_NUM] = {
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
    Plan_Group_Temp_Calc_And_Run(obj->temp_group, (uint8_t)ARM_JOINT_NUM, start_pos,
        end_pos, t, quintic_spline);
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
        plan_temp(obj, ARM_RESET_ANGLE, ARM_AUTO_RESET_TIME_S);
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

/* ===================== 一键夹爪找零 =====================
 *
 * 三步：**推到抵住机械限位 → 往张开方向退回半个行程 → 把那里设成零点**。
 * 设完之后零点就在行程中点上：闭合端读 +142、张开端读 -142(见 gimbal_cmd_def.h 的
 * GRIPPER_ANGLE_MIN/MAX —— 这两个数就是跟着这件事定的)。
 *
 * 第一步 找限位。驱动：每拍把目标角往**闭合**方向推一小步，再叠一个闭合方向的力矩前馈
 *       (gripper_torque -> 电机的 output_comp，见 gimbal_arm.c 的 Set_Motor_ref())。
 *   判定：**只看反馈角在窗口内动了没有**，不看力矩/电流。顶住之后位置环的误差会一直挂着
 *       (电机照旧出力)，而反馈角停在原地 —— 两者一比就分得出来；而且这套判据**和电机型号、
 *       kp 调成多少都无关**；力矩和电流在"顶住"和"还在加速"两种情况下都可能是大值，反而
 *       分不开。要**连续 3 个小窗都没动**才认(0.6 s)，是为了扛两件事：起动的静摩擦(头一下
 *       推不动)和编码器噪声 —— 真顶住了会一直顶住，静摩擦只影响头几十毫秒。
 *
 * 第二步 退回中点。基准用**反馈角**、不用目标角：目标角为了压住限位一直领先反馈(见
 *       GRIPPER_ZERO_LEAD_DEG)，拿它当基准会偏出去。
 *
 * 第三步 设零。零点得写进电机里(达妙 0xFE)，而那条帧和常规控制帧互斥、只能在 MotorTask
 *       里发 —— 所以这一步**不自己发帧**，而是把 arm_zero 那个已经验过的设零状态机叫起来，
 *       用 arm_zero_participate_override 把这一轮**缩到夹爪一台**(不缩的话它会把 j1~j6
 *       一起失能，臂当场松掉)。代价：这一轮里整条 CAN 是安静的(见 arm_zero.h)，别的电机
 *       收不到常规帧，实测 0.1~1.5 s。
 *
 * 退出：设完零点 / 超时 / 夹爪掉线 / 不在 arm_run 档 —— 都走同一套收尾再回 arm_auto_none。
 *   **超时是必须的**：robot_state 的初始化流程第二步就等着这个模式回到 none，卡在这儿等于
 *   整机上电流程卡住。 */

/* ---- 找零的整定参数 ---- */
#define GRIPPER_ZERO_TICK_MS 2u        /* 本层节拍：CmdTask 500 Hz(见 freertos.c 的 StartCmdTask) */
#define GRIPPER_ZERO_TORQUE 0.5f       /* 推的时候叠的闭合力矩前馈，N·m(夹爪 max_torque = 10) */
#define GRIPPER_ZERO_STEP_DEG 0.2f     /* 每拍挪一小步：0.2 / 2ms = 100 deg/s */
/* 目标角最多领先反馈多少 deg。两个作用：
 *   ① 顶住限位之后位置环的误差被卡在这儿 —— 压住限位的力矩就是有界的
 *      (kp * 20deg + 力矩前馈 ≈ 1.2 N·m)。不管的话，0.6 s 的判定窗口里目标角会一路领先
 *      60°，顶住限位的力矩越顶越大。
 *   ② 第二步退的时候目标角和反馈差 142°，不限住的话位置环一上来就出 ~5 N·m，会甩着
 *      夹爪撞到张开端。
 * 它同时是这里**唯一的限位**：gimbal_cmd.c 的 Motor_Angle_Limit() 对夹爪的 ±142 软限位
 * 只在 motor_inited[GRIPPER] 为真时才卡，而找零的 case 0 **故意**先把那个标志清 0
 * (见那里的注释：这一轮本来就要越过旧口径的边界)，所以找零期间软限位一定是不生效的。 */
#define GRIPPER_ZERO_LEAD_DEG 20.0f
#define GRIPPER_ZERO_STALL_MS 200u     /* 判定小窗的长度，ms */
#define GRIPPER_ZERO_STALL_MOVE 0.5f   /* 小窗里反馈角挪动小于这个值 = 没动，deg */
#define GRIPPER_ZERO_STALL_WIN 3u      /* 连续几个小窗没动才认(3 x 200ms = 0.6 s) */
/* 退多少 = **半个行程**，退到行程中点。写成"两个软限位之差的一半"而不是 142，是为了让
 * "零点在行程中点"这件事自己说话 —— 两个软限位就是绕中点对称的。 */
#define GRIPPER_ZERO_BACK_DEG ((GRIPPER_ANGLE_MAX - GRIPPER_ANGLE_MIN) / 2.0f)
#define GRIPPER_ZERO_ARRIVE_DEG 2.0f  /* 退到中点的判据，deg */
#define GRIPPER_ZERO_PUSH_TIMEOUT_MS 10000u    /* 满行程 284 deg 按 100 deg/s 约 3 s，留足余量 */
#define GRIPPER_ZERO_BACK_TIMEOUT_MS 6000u     /* 退半个行程约 1.4 s */
#define GRIPPER_ZERO_SETZERO_TIMEOUT_MS 12000u /* 等设零那一轮；arm_zero 自己 8 s 封顶 */
#define GRIPPER_ZERO_SETTLE_MS 400u   /* 设零之后等常规帧恢复(见 case 4)，ms */
#define GRIPPER_ZERO_STEP_END 5u      /* 收尾步号：走到它就退出这个模式 */

/* 找零的跨拍状态。放文件级 static 而不塞进 Gimbal_cmd —— 那个结构体的偏移调试器里存着，
 * 不想动(和它末尾那段注释同一个理由)；每次进这个动作在 case 0 里全部重置。 */
static float gz_win_pos;      /* 当前小窗开始时的反馈角 */
static float gz_stall_angle;  /* 认到"顶住了"那一刻的反馈角 = 机械限位；调试器里可读 */
static float gz_back_target;  /* 退到哪儿(绝对角) = gz_stall_angle - 半个行程 */
static uint16_t gz_win_ms;    /* 当前小窗已经量了多少 ms */
static uint16_t gz_total_ms;  /* 推开阶段一共过了多少 ms */
static uint16_t gz_back_ms;   /* 退回阶段一共过了多少 ms */
static uint16_t gz_zero_ms;   /* 等设零那一轮等了多久，ms */
static uint16_t gz_settle_ms; /* 设零之后等常规帧恢复了多久，ms */
static uint8_t gz_stall_win;  /* 已经连续几个小窗没动 */
static uint8_t gz_zero_sent;  /* 设零请求已经发出去了 */
/* 设零那一轮的结果码(Arm_zero_result)，调试器里读：
 *   2=OK(位置被拉到 0) 1=UNCONFIRMED(本来就在容差内，写没写进去看不出来) 4=超时 5=没参与。
 * **第二次跑会看到 1**：那时夹爪就在行程中点上(读数≈0)，本来就分辨不出来。 */
static uint8_t gz_zero_result;

/* 把夹爪目标角往 dir(+1 = 闭合 / -1 = 张开)方向挪一步，但最多领先反馈
 * GRIPPER_ZERO_LEAD_DEG —— 理由见那个宏。领先够了就不动，等反馈跟上来。 */
static void gripper_zero_step(Gimbal_cmd* obj, float dir) {
    float cmd = obj->gimbal->arm_control_data.gripper_angle;
    float fdb = obj->gimbal->arm_feedback_data.gripper_angle;
    if ((cmd - fdb) * dir >= GRIPPER_ZERO_LEAD_DEG) return;
    obj->gimbal->arm_control_data.gripper_angle = cmd + dir * GRIPPER_ZERO_STEP_DEG;
}

/* 这一步还能不能继续。夹爪掉线 → 反馈是死的；不在 arm_run 档 → Set_Motor_ref() 的被动
 * 分支把夹爪的位置环关掉、目标角和力矩根本到不了电机 —— 两种都会让"反馈不动"变成必然，
 * 于是被误判成"找到限位"。停机档位走不到这儿(Stop_Mode_Update 每拍把 auto_mode 清 0)，
 * 阻尼/仅重力才会。
 *
 * 这里**不查 motor_inited[GRIPPER]**：找零自己就是那个标志的唯一维护者(case 0 置 0、
 * case 4 置回 1)，拿它当门槛等于"没初始化过就永远不许初始化"，自锁。 */
static uint8_t gripper_zero_can_run(Gimbal_cmd* obj) {
    if (obj->gimbal->arm_feedback_data.is_motor_lost[GRIPPER]) return 0;
    if (obj->mode != arm_run) return 0;
    return 1;
}

/* 喂一次夹爪的掉线看门狗。设零那一轮总线是故意安静的，夹爪失能后还可能不再发反馈
 * (实测见 arm_zero.c 的文件头)，20 ms 的看门狗会把它判成掉线 —— 而 gimbal_arm.c 的
 * Motor_Lost_Handle() **只要有一台掉线就把 arm_mode 打成 arm_stop**，整个臂当场松掉。
 * arm_zero.c 里的 keep_monitors_alive() 是同一招，但它只管设零进行中的那几拍，设零一结束
 * 就轮到这儿接上。 */
static void gripper_zero_watchdog(Gimbal_cmd* obj) {
    DM_motor* g = obj->gimbal->gripper;
    if (g && g->monitor) g->monitor->reset(g->monitor);
}

/* 收尾：目标角对齐到反馈(退出后位置环会拿它当指令，不对齐就是一次猛拽)、清掉找零力矩、
 * 把这轮留下的参与集覆盖清干净，然后跳到收尾步；下一拍自然走 default 退出，和 Arm_Reset
 * 一个路子。 */
static void gripper_zero_finish(Gimbal_cmd* obj) {
    obj->gimbal->arm_control_data.gripper_angle =
        obj->gimbal->arm_feedback_data.gripper_angle;
    obj->gimbal->arm_control_data.gripper_torque = 0;
    arm_zero_participate_override = 0;
    obj->auto_step = GRIPPER_ZERO_STEP_END;
}

/* 一键夹爪找零。 */
void Arm_Gripper_Zero(Gimbal_cmd* obj) {
    if (obj->last_auto_mode != obj->auto_mode) {
        obj->auto_step = 0;
    }

    obj->ctrl_type = arm_ctrl_joint;

    switch (obj->auto_step) {
    case 0:  // 起手
        /* 先把夹爪标回"未初始化"。找零会把电机的零点整个换掉 —— 在它跑完之前，"夹爪已
         * 初始化"这句话不成立：
         *   - 0x80 反馈帧的 motor_inited_mask 会如实把这一位报 0(见 robot_state.c)；
         *   - gimbal_cmd.c 的夹爪软限位只在 motor_inited[GRIPPER] 为真时才卡，标 0 之后
         *     这一轮的目标角不被程序限位挡着 —— 找零本来就要越过它(旧口径下推到底的角
         *     和新口径的 ±142 是两套数)。
         * 配套的另一半在 case 4：设零成功(OK / UNCONFIRMED)时才置回 1。两句是一对，
         * 改一个要一起看 —— 只留置 0 的话夹爪永远"未初始化"、软限位永远不生效。 */
        obj->gimbal->arm_feedback_data.motor_inited[GRIPPER] = 0;
        arm_zero_participate_override = 0;  // 上一次没走完留下的覆盖，清掉
        /* 目标角先对齐反馈(免得从别处的旧目标慢慢爬过来，甚至先反向开到底) */
        obj->gimbal->arm_control_data.gripper_angle =
            obj->gimbal->arm_feedback_data.gripper_angle;
        obj->gimbal->arm_control_data.gripper_torque = GRIPPER_ZERO_TORQUE;
        gz_win_pos = obj->gimbal->arm_feedback_data.gripper_angle;
        gz_stall_angle = gz_win_pos;
        gz_back_target = gz_win_pos;
        gz_win_ms = 0;
        gz_total_ms = 0;
        gz_back_ms = 0;
        gz_zero_ms = 0;
        gz_settle_ms = 0;
        gz_stall_win = 0;
        gz_zero_sent = 0;
        gz_zero_result = ARM_ZERO_RES_NONE;
        obj->auto_step++;
        break;

    case 1: {  // 往闭合方向推，同时看反馈角跟不跟
        float fdb = obj->gimbal->arm_feedback_data.gripper_angle;
        if (!gripper_zero_can_run(obj)) {  // 推也白推，或者"不动"说明不了任何事
            gripper_zero_finish(obj);
            break;
        }
        /* 推：目标角加一步(领先量见 GRIPPER_ZERO_LEAD_DEG)，力矩前馈每拍都得重写 ——
         * Set_Ctrl_Default() 在 Auto_Mode_Update() 之前已经把它清 0 了。 */
        gripper_zero_step(obj, +1.0f);
        obj->gimbal->arm_control_data.gripper_torque = GRIPPER_ZERO_TORQUE;

        /* 判定：每 0.2 s 看一次"这一窗里反馈角挪了多少" */
        gz_total_ms += GRIPPER_ZERO_TICK_MS;
        gz_win_ms += GRIPPER_ZERO_TICK_MS;
        if (gz_win_ms >= GRIPPER_ZERO_STALL_MS) {
            if (fabsf(fdb - gz_win_pos) < GRIPPER_ZERO_STALL_MOVE) {
                gz_stall_win++;  // 这一窗没动
            } else {
                gz_stall_win = 0;  // 还在动，重新数
            }
            gz_win_pos = fdb;
            gz_win_ms = 0;
        }
        if (gz_stall_win >= GRIPPER_ZERO_STALL_WIN) {  // 顶住了：这个角就是机械限位
            gz_stall_angle = fdb;
            gz_back_target = gz_stall_angle - GRIPPER_ZERO_BACK_DEG;
            obj->auto_step++;  // -> 2 退回中点
            break;
        }
        if (gz_total_ms >= GRIPPER_ZERO_PUSH_TIMEOUT_MS) {  // 推不出来也别把初始化流程拖死
            gripper_zero_finish(obj);
            break;
        }
        break;
    }

    case 2: {  // 往张开方向退回行程中点
        float fdb = obj->gimbal->arm_feedback_data.gripper_angle;
        if (!gripper_zero_can_run(obj)) {
            gripper_zero_finish(obj);
            break;
        }
        gripper_zero_step(obj, -1.0f);
        /* 到目标就夹住(领先量那条只保证"不超速"，不管目标在哪) */
        if (obj->gimbal->arm_control_data.gripper_angle < gz_back_target)
            obj->gimbal->arm_control_data.gripper_angle = gz_back_target;

        gz_back_ms += GRIPPER_ZERO_TICK_MS;
        if (fabsf(fdb - gz_back_target) <= GRIPPER_ZERO_ARRIVE_DEG) {
            obj->auto_step++;  // -> 3 设零
            break;
        }
        if (gz_back_ms >= GRIPPER_ZERO_BACK_TIMEOUT_MS) {
            /* 退不到位(夹住东西了/被挡住)：**不设零**。这时设零会把零点写在行程中点以外
             * 的地方，整条夹爪的角度口径就废了 —— 如实收工，让上层自己决定怎么办。 */
            gripper_zero_finish(obj);
            break;
        }
        break;
    }

    case 3:  // 设零：叫起 arm_zero，只让它动夹爪
        /* 目标角贴着反馈。这一句有两层用法：
         *   - 设零之前/期间：目标角 = 退到位那个角，在老口径里是对的(请求被消费之前常规帧
         *     还在发，不能先改成别的值)；
         *   - 设零落地之后：反馈角自己就跳到 0 了(零点从出厂位置挪到行程中点，读数整体平移)，
         *     这一句于是自动把目标角跟着改到新口径 —— 不动的话位置环会拿着 142° 的误差，
         *     常规帧一恢复就猛拽。每拍都对齐，中间最多差一拍。
         * 力矩前馈清零：设零期间电机可能被 arm_zero 唤醒(读完关回去那种)，别让它顶着找零力矩。 */
        obj->gimbal->arm_control_data.gripper_angle =
            obj->gimbal->arm_feedback_data.gripper_angle;
        obj->gimbal->arm_control_data.gripper_torque = 0;

        if (!gz_zero_sent) {
            /* 只让夹爪参与(不这么做 arm_zero 会把 7 台全失能)。**先发请求**：请求被消费
             * 之前常规帧还在发，那时目标角还是老口径的"退到位"值、是对的。 */
            arm_zero_participate_override = (uint8_t)(1u << GRIPPER);
            arm_zero_request = 1;
            gz_zero_sent = 1;
        }
        gripper_zero_watchdog(obj);
        gz_zero_ms += GRIPPER_ZERO_TICK_MS;

        /* 结束判据 = "请求被消费了(arm_zero_request 由 start() 清 0) 且状态机落到
         * DONE/TIMEOUT"。只判状态不行：上一轮(比如 JLink 触发的整臂设零)留下的 DONE 会
         * 一直挂在那儿，刚进这一步就会被当成"跑完了"。 */
        if ((!arm_zero_request
                && (arm_zero_state == ARM_ZERO_DONE || arm_zero_state == ARM_ZERO_TIMEOUT))
            || gz_zero_ms >= GRIPPER_ZERO_SETZERO_TIMEOUT_MS) {
            gz_zero_result = arm_zero_result[GRIPPER];
            obj->auto_step++;  // -> 4 收尾(那一轮还在跑也由 case 4 等它)
            break;
        }
        break;

    case 4:  // 收尾：等常规帧恢复，再把目标角对到新口径上
        obj->gimbal->arm_control_data.gripper_angle =
            obj->gimbal->arm_feedback_data.gripper_angle;
        obj->gimbal->arm_control_data.gripper_torque = 0;
        gripper_zero_watchdog(obj);
        if (Arm_Zero_Busy()) {  // 设零那一轮还没跑完(走了超时兜底那条路才会到这)
            gz_settle_ms = 0;
            break;
        }
        /* 设零刚结束时夹爪还是失能态：控制帧要等 libs/dm_motor 那个 75ms 轮转的使能帧
         * (DMmotor_Calc_Send 顶部)转回来才重新生效，所以多等一会儿再把这一轮交出去 ——
         * 反正这点时间不影响什么，而"设零完就退出去"会让后面每一拍都在一个还没恢复的
         * 夹爪上做闭环。 */
        gz_settle_ms += GRIPPER_ZERO_TICK_MS;
        if (gz_settle_ms >= GRIPPER_ZERO_SETTLE_MS) {
            /* 设零成功了才宣布"夹爪已初始化"，和 case 0 那句置 0 配套。
             * 两种结果算成功：
             *   OK(2)          —— 位置被拉到 0，零点确实写进去了；
             *   UNCONFIRMED(1) —— 本来就在读数 0 附近，分辨不出写没写进去。**第二次跑找零
             *                     就是这种**：夹爪已经在中点上，零点没动，读数照样是对的。
             * 其余(超时/失败/没走到)都留 0 —— 那时候夹爪的读数口径是可疑的，别让上位机
             * 和软限位以为它已经标定好。
             * 置 1 之后两个后果：0x80 反馈帧的 motor_inited_mask 报 1；gimbal_cmd.c 的
             * 夹爪软限位(±142)开始真的卡目标角 —— 那正是找零想要的口径。 */
            if (gz_zero_result == ARM_ZERO_RES_OK
                || gz_zero_result == ARM_ZERO_RES_UNCONFIRMED) {
                obj->gimbal->arm_feedback_data.motor_inited[GRIPPER] = 1;
            }
            gripper_zero_finish(obj);
        }
        break;

    default:  // 跑完(或步数越界)：退出这个模式，交回给上层
        obj->auto_mode = arm_auto_none;
        obj->auto_step = 0;
        break;
    }
}
