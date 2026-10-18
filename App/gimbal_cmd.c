#include "gimbal_cmd.h"
#include <stdlib.h>         /* malloc */
#include <string.h>         /* memcpy / memset */
#include "arm.h"            /* Arm —— 反指针取本台的模式源(obj->arm->arm_mode) */
#include "arm_auto.h"       /* Arm_Reset() 等一键模式的动作本体 */
#include "common.h"         /* fclip */
#include "gimbal_cmd_def.h" /* JOINT_ANGLE_MIN/MAX、PLAN_SPEED_COMP_COEF */
#include "move_plan.h"      /* move_plan_run(判规划是否在跑) */
#include "plan_group.h"     /* Plan_Group_Temp_Create() */
#include "rc.h"
#include "robot_state.h" /* 串口总模式状态机：离线时的模式来源 + 控制帧来源 */

/* CmdTask 控制周期，单位为 s。 */
#define GIMBAL_CMD_PERIOD_S 0.002f

/* 命令层对象挂在 App/inc/arm.h 的 `Arm` 上(g_arm[N].gimbal_cmd)，一台臂一个 ——
 * 共用一份的话，第二台建完就把第一台的 TD/目标值/一键模式进度整个覆盖掉。 */

/* 遥控器是否在线 —— 判据就一句：100 ms 内收到过遥控帧(见 libs/rc 的 is_Offline)。
 * 抽成函数是因为它在本文件里要用两处(模式判定、run 档挑输入源)，两处必须同源；
 * App/robot_state.c 里也用同一个判据(那边自己 include rc.h，不跨模块耦合)。 */
static uint8_t Rc_Online(void) { return (uint8_t)(!is_Offline(rc.monitor)); }

/* 私有函数(顶层) */
// NOLINTBEGIN
static void Stop_Mode_Update(Gimbal_cmd* obj);     // 停机模式
static void Passive_Mode_Update(Gimbal_cmd* obj);  // 被动模式(阻尼 / 仅重力补偿)
static void Run_Mode_Update(Gimbal_cmd* obj);      // 运行模式(遥控器点动 或 串口控制帧)
static void Set_Ctrl_Default(Gimbal_cmd* obj);     // 部分控制默认值设置
static void Auto_Mode_Update(Gimbal_cmd* obj);     // 一键模式分派
static void Plan_Running_Check(Gimbal_cmd* obj);   // 规划运行状态检查
static void Ctrl_Src_Calc(Gimbal_cmd* obj);        // 解算各数据源
static void Motor_Angle_Limit(Gimbal_cmd* obj);    // 软限位
static void Ctrl_Post_Process(Gimbal_cmd* obj);    // 控制量后处理
static void Arm_Ref_Set(Gimbal_cmd* obj);          // 机械臂控制值设置
// NOLINTEND
/* 私有函数(非顶层) */
static void joint_src_follow(Gimbal_cmd* obj);  // 关节控制源跟随反馈
static void joint_angle_td_follow(Gimbal_cmd* obj, float ref[ARM_JOINT_NUM]);  // TD 直接跟随

Gimbal_cmd* Gimbal_Cmd_Create(Arm* arm) {
    /* 命令层是挂在机械臂驱动上的：臂本身、或者它的驱动没建起来，就什么都不用做。
     * (驱动建不起来通常是堆不够，见 Gimbal_Create() 的说明。) */
    if (arm == NULL || arm->gimbal == NULL) return NULL;

    Gimbal_cmd* obj = (Gimbal_cmd*)malloc(sizeof(Gimbal_cmd));
    if (obj == NULL) return NULL;
    memset(obj, 0, sizeof(Gimbal_cmd));
    obj->arm = arm;
    obj->gimbal = arm->gimbal;
    /* ctrl_type 是 arm_ctrl_joint(枚举值 0)，memset 之后就是它，不用再写 */

    /* 关节角跟踪微分器初始化。*/
    const float td_r[ARM_JOINT_NUM] = {
        /* 400 */ 1000,
        4000,
        4000,
        2000,
        2000,
        2000,
    };
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        TD_Config td_config = {
            .r = td_r[i],
            .h = GIMBAL_CMD_PERIOD_S,
            .h0 = GIMBAL_CMD_PERIOD_S * 2,
            .J = 0,  // 独立td用不到
        };
        TD_Init(&obj->joint_angle_td[i], &td_config);
    }

    /* 一键模式的临时规划组：ARM_JOINT_NUM 根轴各一个名额，一次 plan_temp() 就占满。
     * 规划本体是 malloc 出来的，跑完由 500 Hz 的 Move_Plan_Update() 回收
     * (Core/Src/freertos.c 的 HAL_Moveplan_Thread_Create())。 */
    obj->temp_group = Plan_Group_Temp_Create((uint8_t)ARM_JOINT_NUM);
    return obj;
}

void Gimbal_Cmd_Update(Gimbal_cmd* obj) {
    if (obj == NULL || obj->gimbal == NULL) return;

    /* ---- 模式判定 ----
     * 两个来源，**遥控器优先**：
     *
     *   遥控器在线  → 完全照旧：sw_a 定 arm_run/arm_stop。这是"覆盖所有串口指令"的
     *                 落点之一(另一个是下面 run 档挑输入源) —— 在线时遥控器眼里
     *                 就像串口不存在。注意在线时**永远到不了被动模式**：那两档遥控器
     *                 上没法选。
     *   遥控器离线  → 交给串口总模式状态机(App/robot_state.c)决定，它给什么就是什么。
     *
     * 总模式状态机**照常在跑**(握手/反馈/状态迁移都不受遥控器影响)，只是它的输出在
     * 这里被盖掉。这样不需要在状态机里维护"冻结/解冻"的第二套状态。 */
    /* 离线那一支取自**本台臂**的模式字段：总模式状态机是按臂各跑一份的(每臂各自握手、
     * 各自看门狗)，A 台掉线进了 damping 不该把 B 台也带成 damping。 */
    obj->mode = Rc_Online() ? ((rc.sw_a == 1) ? arm_run : arm_stop) : obj->arm->arm_mode;
    /* 控制类型：源工程有 关节 / 笛卡尔 / 自定义腕点 三种(靠遥控器或图传切)，本工程
     * 只剩下关节空间这一种(见 gimbal_cmd.h 的 Arm_ctrl_type)。 */
    obj->ctrl_type = arm_ctrl_joint;

    /* 控制默认值设置 */
    Set_Ctrl_Default(obj);

    /* 不同模式控制。
     * 三个分支按"电机怎么出力"分，不是按"机器人现在在干什么"分 —— 后者是
     * robot_state 的事，到这里已经变成上面的 obj->mode 了。 */
    switch (obj->mode) {
    case arm_run:
        Run_Mode_Update(obj);
        break;
    case arm_damping:
    case arm_only_gravity_comp:
        Passive_Mode_Update(obj);
        break;
    case arm_stop:
    default:
        Stop_Mode_Update(obj);
        break;
    }

    /* 一键模式(和模式无关，无条件跑一遍分派；臂停了由 Stop_Mode_Update() 把它清 0)。
     * 位置和源工程一致：模式分派之后、控制源解算之前 —— 一键模式写的是 ctrl_src，
     * 后面的 Ctrl_Src_Calc/Motor_Angle_Limit 要能看见它。 */
    Auto_Mode_Update(obj);

    /* 规划运行状态检查：plan_running 决定后面 Arm_Ref_Set() 走规划那一支还是 TD 那一支，
     * 所以必须放在 Ctrl_Post_Process()/Arm_Ref_Set() 之前。 */
    Plan_Running_Check(obj);

    /* 控制源解算 */
    Ctrl_Src_Calc(obj);

    /* 软限位 */
    Motor_Angle_Limit(obj);

    /* 控制量后处理(跟踪微分器) */
    Ctrl_Post_Process(obj);

    /* 机械臂控制值设置 */
    Arm_Ref_Set(obj);

    /* 收尾：一键模式的"上一拍模式"在这里更新 —— 每个动作都靠
     * last_auto_mode != auto_mode 判"刚进入"，所以必须放在**所有**动作函数调完之后，
     * 而且每拍都要更新(动作里自己改 auto_mode 也在这之后才被记下来)。 */
    obj->last_auto_mode = obj->auto_mode;

    /* 到此为止 arm_control_data 已经写好了 —— 上面每一步都是**直接写** obj->gimbal 里
     * 那一份，没有中间副本要发。
     * 源工程这里是 Send_Data()：发板间 socket、拓展板 uart，再 publish 到
     * ARM_CONTROL_TOPIC 给机械臂那块板。本工程机械臂驱动就在同一个任务里，不用发，
     * 调用方接着调 Gimbal_Update() 就会读到(顺序见 App/cmd.c 的 Cmd_Loop())。 */
}

static void Stop_Mode_Update(Gimbal_cmd* obj) {
    // 停止
    obj->gimbal->arm_control_data.mode = arm_stop;

    /* 一键模式直接取消(源工程同款)。副作用要知道：**停机档位下 auto_mode 每拍都被清 0**，
     * 所以要用一键复位必须把 rc.sw_a 打在 arm_run 档，否则写进去也活不过一拍。
     *
     * 串口初始化流程(App/robot_state.c)不受这个影响：它在 robot_state_initing 期间
     * 把 Arm_mode 定成 arm_run，压根不会走到 Stop_Mode_Update()。真正的风险是**初始化
     * 途中遥控器上线** —— 那时模式被遥控器打成 arm_stop/arm_run，一旦停在 stop 档，
     * 这行就会把正在跑的复位打断。robot_state 对这种情况的处理是"判定复位中止、
     * 退回 wait_init"，**不能当成复位已完成**(见 robot_state.c 里 initing 的说明)。 */
    obj->auto_mode = arm_auto_none;
    obj->auto_step = 0;

    /* 停机控制量保护：目标跟随反馈，保持命令与反馈一致，防止切回运行时跳变。
     * 顺带还有个"自播种"的效果：joint_src 一直跟着反馈走，所以将来第一次进 arm_run 时
     * joint_src 已经是当前位姿，不会从 0 猛拽回去。(源工程对应的是
     * arm_ref_synced_flag + 上电默认折叠姿态那一套，本工程不需要。) */
    memcpy(obj->gimbal->arm_control_data.joint_angle,
        obj->gimbal->arm_feedback_data.joint_angle,
        sizeof(obj->gimbal->arm_feedback_data.joint_angle));
    memset(obj->gimbal->arm_control_data.joint_vel, 0,
        sizeof(obj->gimbal->arm_control_data.joint_vel));
    memset(obj->gimbal->arm_control_data.joint_acc, 0,
        sizeof(obj->gimbal->arm_control_data.joint_acc));

    /* 控制源跟随。
     * 源工程还有 cartesian_src_follow() / custom_selfsaving_src_follow()(各自跟反馈的
     * 末端位姿、腕点)，那两个数据源本工程没有。 */
    joint_src_follow(obj);

    /* 关节角跟踪微分器跟随：直接对齐，不要"跟"出速度来 */
    joint_angle_td_follow(obj, obj->gimbal->arm_feedback_data.joint_angle);

    // 夹爪跟随
    obj->gimbal->arm_control_data.gripper_angle =
        obj->gimbal->arm_feedback_data.gripper_angle;
    obj->gimbal->arm_control_data.gripper_torque = 0;
}

/* 被动模式：阻尼 / 仅重力补偿。
 *
 * 这两个模式下**电机是使能的**(见 gimbal_arm.c 的使能分支：只有 arm_stop 失能)，
 * 但位置环被关了，力矩只由 kd(阻尼)或重力前馈(仅重力)出 —— 具体在 gimbal_arm.c 的
 * set_pid_damping() / set_pid_zero() / Set_Motor_ref() 的被动分支里。
 *
 * 这一层要做的事就一件：**把目标值对齐到反馈**，别让位置环"虽然关了但目标还挂着旧值"
 * —— 位置环关了不代表 ref_position 不重要，`ref_speed = pid_pos.output + speed_comp`
 * 里 pid_pos.output 因为 config 全零确实是 0，但如果哪天配置改回来，或者某个驱动的
 * 位置环不是纯 PID，一个陈旧目标就会当场变成力矩。做法和 Stop_Mode_Update() 一模一样。
 *
 * **和 Stop_Mode_Update 唯一的区别：这里不清 auto_mode。** 因为初始化流程
 * (robot_state_initing)期间臂就是 arm_run，而一键动作跑完前后可能经过被动模式；
 * 清了 auto_mode 会把正在跑的复位拦腰打断。 */
static void Passive_Mode_Update(Gimbal_cmd* obj) {
    obj->gimbal->arm_control_data.mode = obj->mode;

    /* 目标跟随反馈 —— 和 Stop_Mode_Update() 里那段同款，理由见那里的注释
     * (防跳变 + 自播种：切回 arm_run 时 joint_src 已经在当前位姿上)。 */
    memcpy(obj->gimbal->arm_control_data.joint_angle,
        obj->gimbal->arm_feedback_data.joint_angle,
        sizeof(obj->gimbal->arm_feedback_data.joint_angle));
    memset(obj->gimbal->arm_control_data.joint_vel, 0,
        sizeof(obj->gimbal->arm_control_data.joint_vel));
    memset(obj->gimbal->arm_control_data.joint_acc, 0,
        sizeof(obj->gimbal->arm_control_data.joint_acc));

    joint_src_follow(obj);
    joint_angle_td_follow(obj, obj->gimbal->arm_feedback_data.joint_angle);

    // 夹爪跟随
    obj->gimbal->arm_control_data.gripper_angle =
        obj->gimbal->arm_feedback_data.gripper_angle;
    obj->gimbal->arm_control_data.gripper_torque = 0;
}

/* 运行模式。输入源有**两个**，按遥控器是否在线二选一：
 *
 *   - **遥控器在线**：完全照旧 —— 三个摇杆各管一根轴，按**边沿**点动
 *     (Etc_Get：一直按着只动一次)，每拍 30 deg。串口控制帧在这条路上**被忽略**
 *     (连读都不读)，这就是"遥控器覆盖所有串口指令"的落点。
 *   - **遥控器离线**：用串口控制帧(App/robot_state.c 收的 0x16/0x17)写目标。
 *
 * 它是"运行模式"这一个大类的输入选择，不只是"遥控器模式"。
 *
 * 源工程这份逻辑是围着 DT7 写的：右拨杆三档选"停 / 控底盘 / 控云台"，左拨杆三档再选
 * 具体控哪几个关节，另外还有拨轮(一键模式)、四向按键(夹爪开合 / j6 重初始化)、摇杆点动。
 * 本工程的遥控器驱动是 libs/rc/(PWM 输入捕获，不是 DBUS/串口 DT7)，实测接线只有
 * 3 个比例轴 rc.rx / rc.ry / rc.lx 加一个二值开关 rc.sw_a，拨轮、四向按键、swB 都没接
 * (见 libs/rc/rc_config.h 那张接线表)，上游那套档位语义在这里铺不开 —— 所以遥控器这一支
 * 只留了最小的一套，j4~j6 没有手控入口。
 *
 * 注意分支里的 `if (rc.sw_a == 0) return;`：**只在遥控器在线时有意义**。它管的是
 * "sw_a 在 stop 档时不要动目标值"。因为模式判定那里 sw_a == 0 会把 mode 打成 arm_stop、
 * 根本走不到这个函数，所以这行其实是给"同一拍里模式刚变"留的保险，保留原样。 */
static void Run_Mode_Update(Gimbal_cmd* obj) {
    obj->gimbal->arm_control_data.mode = arm_run;

    if (Rc_Online()) {
        /* ---- 遥控器在线 ---- */
        if (rc.sw_a == 0) {
            return;
        }

        // // 右摇杆左右控制 j1，
        // if (Etc_Get(rc.trig.rx_left)) obj->ctrl_src.joint_src.joint_angle[JOINT1] += 30;
        // if (Etc_Get(rc.trig.rx_right)) obj->ctrl_src.joint_src.joint_angle[JOINT1] -= 30;
        // 右摇杆上下控制 j2
        // if (Etc_Get(rc.trig.ry_up)) obj->ctrl_src.joint_src.joint_angle[JOINT2] += 30;
        // if (Etc_Get(rc.trig.ry_down)) obj->ctrl_src.joint_src.joint_angle[JOINT2] -= 30;
        if (fabsf(rc.ry) > 0.1)
            obj->ctrl_src.joint_src.joint_angle[JOINT2] += fsgn(rc.ry) * 90 / 500.0;
        // // 左摇杆左右控制 j3
        // if (Etc_Get(rc.trig.lx_right)) obj->ctrl_src.joint_src.joint_angle[JOINT3] += 30;
        // if (Etc_Get(rc.trig.lx_left)) obj->ctrl_src.joint_src.joint_angle[JOINT3] -= 30;
        if (fabsf(rc.lx) > 0.1)
            obj->ctrl_src.joint_src.joint_angle[JOINT3] += fsgn(rc.lx) * 90 / 500.0;
        // 左摇杆左右控制gripper
        // if (fabsf(rc.lx) > 0.1)
        //     obj->gimbal->arm_control_data.gripper_angle += rc.lx * 90 / 500.0;
        return;
    }

    /* ---- 遥控器离线：串口控制帧 ----
     * 只认**标记为 valid 的最新一帧**。valid 由 robot_state 在每次模式/状态切换时清零，
     * 所以"run → stop → run"不会把停机前的旧目标重新灌进来把臂拽回去。
     *
     * 没有帧（或帧已失效）就什么都不做 —— 此时 joint_src 保持上一拍的值。这是有意的：
     * 上位机**周期性**下发，两帧之间本来就该保持；真正"上位机死了"由 robot_state 的
     * 3 s 超时判，判到就把模式切成 arm_damping，值就无所谓了。
     *
     * 位置走 joint_src，后面 Ctrl_Src_Calc → 软限位 → TD 照常过一遍（和遥控器点动
     * 走的是同一条路）；速度前馈不走这里，在 Arm_Ref_Set() 里单独覆盖。 */
    const Robot_ctrl_cmd* cmd = Robot_State_Ctrl(obj->arm);
    if (cmd == NULL || !cmd->valid) return;

    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->ctrl_src.joint_src.joint_angle[i] = cmd->joint_angle[i];
    }
    obj->gimbal->arm_control_data.gripper_angle = cmd->gripper_angle;
}

/* 一键模式分派(源工程 Gimbal_Cmd_Update() 里那个 switch (obj->auto_mode)，位置一样)。
 * 动作本体全在 App/arm_auto.c，这里只做"哪个模式调哪个函数"。
 *
 * 谁写 auto_mode：**遥控器写不了**(没有空闲按键)，由串口总模式状态机
 * (App/robot_state.c 的 robot_state_initing) 写 —— 初始化流程就靠这两步：
 * 先 arm_auto_reset、等它跑完回 none，再 arm_auto_gripper_zero、再等回 none。
 * 调试器也可以直接写(见 arm_auto.h 的说明)，但注意 **arm_stop 档位下
 * Stop_Mode_Update() 每拍都会把它清 0**。 */
static void Auto_Mode_Update(Gimbal_cmd* obj) {
    switch (obj->auto_mode) {
    case arm_auto_reset:
        Arm_Reset(obj);
        break;
    case arm_auto_gripper_zero:
        /* 推到抵住机械限位(判据是反馈角在窗口里不动) → 退回半个行程 → 在那里设零点；
         * 设零那一步是叫 arm_zero 跑一轮"只有夹爪"的，见 App/arm_auto.c。 */
        Arm_Gripper_Zero(obj);
        break;
        /* 源工程这里还有 10 个：arm_auto_stretch / store / upstairs / pop /
         * fetch_down / fetch_up / protect / first_ore / gripper_down / turn_around。
         * 本工程没做，枚举成员在 robot_def.h 里注释着，加的时候在这里补 case。 */
    case arm_auto_none:
    case ARM_AUTO_MODE_NUM:
    default:
        break;
    }
}

/* 规划运行状态检查：决定后面 Arm_Ref_Set() 走规划那一支还是 TD 那一支。
 * 源工程有两个来源 —— 临时规划组 obj->temp_group，和规划包 obj->pkg.plan_group_list
 * (存矿/弹矿/取家矿那些动作组)。本工程只有前者。 */
static void Plan_Running_Check(Gimbal_cmd* obj) {
    /* 当前没有一键模式 -> 认定没有规划在跑(源工程第一句就是这么写的，省得去翻规划表) */
    if (obj->auto_mode == arm_auto_none) {
        obj->plan_running = 0;
        return;
    }

    /* 同步规划组，每根轴同时起同时停，看第一根即可 */
    if (obj->temp_group != NULL && obj->temp_group[0] != NULL
        && obj->temp_group[0]->status == move_plan_run) {
        obj->plan_running = 1;
    } else {
        obj->plan_running = 0;
    }
}

static void Set_Ctrl_Default(Gimbal_cmd* obj) {
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->gimbal->arm_control_data.joint_vel[i] = 0;
        obj->gimbal->arm_control_data.joint_acc[i] = 0;
    }
    obj->gimbal->arm_control_data.gripper_torque = 0;
}

static void Ctrl_Src_Calc(Gimbal_cmd* obj) {  // 无论模式都解算各数据源
    /* 关节空间控制源：无需解算，ctrl_src.joint_src 里的就是目标值本身。
     *
     * 源工程这里还有两大段，都要 arm_kinematics_calc(本工程没移植)，按用户要求删了：
     *   - 笛卡尔空间控制源：Arm_Ikine_Calc() 把末端位姿解成关节角，再做球腕相邻角转换、
     *     两组解选最近的、奇异点处理；
     *   - 自定义控制器腕点控制源：Elbow_Ikine_Calc() 由腕点位置解前三个关节。 */

    /* 关节角度解算：根据控制模式选择控制源 */
    switch (obj->ctrl_type) {
    case arm_ctrl_joint:  // 关节空间控制
        memcpy(obj->ctrl_src.joint_angle_solved, obj->ctrl_src.joint_src.joint_angle,
            sizeof(obj->ctrl_src.joint_angle_solved));
        break;
        /* case arm_ctrl_cartesian:          笛卡尔空间控制 —— 不做 */
        /* case arm_ctrl_custom_selfsaving:  自定义控制器腕点控制 —— 不做 */
    }

    /* 控制源跟随：源工程那三行 if 的意思是"当前没在用的控制源跟着反馈走，防止切过去时
     * 跳变"。本工程只剩 arm_ctrl_joint 一个源，另外两个源没有，所以这里没有内容。 */

    /* 停机保护。判据是 `!= arm_run` 而不是 `== arm_stop`：**两个被动模式也要走这里**。
     * 被动模式下位置环是关的（gimbal_arm.c 的 set_pid_damping/set_pid_zero），目标值
     * 不会被位置环拿来算力矩，但把它对齐到反馈仍然是对的 —— 万一上位机在阻尼模式里
     * 发了几帧控制包，那些值会留在 joint_src 里，一切成 arm_run 就会被当成"目标"猛地
     * 执行。这里每拍覆盖成反馈就没有这个隐患。 */
    if (obj->mode != arm_run) {
        for (int i = 0; i < ARM_JOINT_NUM; i++) {
            obj->ctrl_src.joint_angle_solved[i] =
                obj->gimbal->arm_feedback_data.joint_angle[i];
        }
    }
}

static void Motor_Angle_Limit(Gimbal_cmd* obj) {
    // 仅对已初始化的电机进行软限位
    // 关节电机
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        if (!obj->gimbal->arm_feedback_data.motor_inited[i]) continue;
        obj->ctrl_src.joint_angle_solved[i] = fclip(obj->ctrl_src.joint_angle_solved[i],
            JOINT_ANGLE_MIN[i], JOINT_ANGLE_MAX[i]);
    }
    /* 源工程这里还挂着一大段"j1/j2 支架干涉航点"的处理(BRACKET_WAYPOINTS)，但它们全是
     * 注释掉的调试代码；再往下是夹爪限位 TODO。本工程夹爪的遥控控制没做，整块删了。 */

    // 反向传递限位：关节角控制源
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->ctrl_src.joint_src.joint_angle[i] = obj->ctrl_src.joint_angle_solved[i];
    }
    /* 源工程还有"笛卡尔空间控制源和自定义控制器控制源的反向传递限位"，两边都标着
     * TODO，本工程那两个源没有。 */

    // 夹爪
    if (obj->gimbal->arm_feedback_data.motor_inited[GRIPPER])
        obj->gimbal->arm_control_data.gripper_angle =
            fclip(obj->gimbal->arm_control_data.gripper_angle, GRIPPER_ANGLE_MIN,
                GRIPPER_ANGLE_MAX);
}

static void Ctrl_Post_Process(Gimbal_cmd* obj) {
    /* TD 提供目标位置、速度和跟踪加速度。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        // 使用控制源的关节角目标值作为微分器的输入
        obj->joint_angle_td[i].ref = obj->ctrl_src.joint_angle_solved[i];
        TDFunction_independent(&obj->joint_angle_td[i]);
    }

    /* 规划运行时，TD 位置跟随规划目标。 */
    if (obj->plan_running) {
        joint_angle_td_follow(obj, obj->ctrl_src.joint_angle_solved);
    }
}

static void Joint_Quintic_Ref_Update(Gimbal_cmd* obj, int joint) {
    Joint_quintic_ref* ref = &obj->joint_quintic_ref[joint];
    const float target = obj->ctrl_src.joint_angle_solved[joint];
    if (!ref->initialized) {
        ref->position = obj->gimbal->arm_feedback_data.joint_angle[joint];
        ref->velocity = 0.0f;
        ref->acc = 0.0f;
        ref->initialized = 1;
    }

    if (!ref->active || target != ref->target) {
        Quintic_spline_config config = {
            .x0 = 0.0f,
            .y0 = ref->position,
            .dy0 = ref->velocity,
            .ddy0 = ref->acc,
            .x1 = JOINT_QUINTIC_DURATION_S,
            .y1 = target,
            .dy1 = 0.0f,
            .ddy1 = 0.0f,
            .xmin = 0.0f,
            .xmax = JOINT_QUINTIC_DURATION_S,
        };
        Quintic_Spline_Reinit(&ref->curve, &config);
        ref->target = target;
        ref->elapsed = 0.0f;
        ref->active = 1;
    }

    ref->elapsed += GIMBAL_CMD_PERIOD_S;
    if (ref->elapsed >= JOINT_QUINTIC_DURATION_S) {
        ref->elapsed = JOINT_QUINTIC_DURATION_S;
        ref->position = ref->target;
        ref->velocity = 0.0f;
        ref->acc = 0.0f;
    } else {
        ref->position = Quintic_Spline_Calc_y(&ref->curve, ref->elapsed);
        ref->velocity = Quintic_Spline_Calc_dy(&ref->curve, ref->elapsed);
        ref->acc = Quintic_Spline_Calc_ddy(&ref->curve, ref->elapsed);
    }
}

static void Arm_Ref_Set(Gimbal_cmd* obj) {
    Arm_control_data* control = &obj->gimbal->arm_control_data;
    const Robot_ctrl_cmd* cmd = Robot_State_Ctrl(obj->arm);
    const uint8_t use_external_velocity =
        (obj->mode == arm_run && !Rc_Online() && cmd != NULL && cmd->valid && cmd->has_vel);

    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        float angle;
        float velocity;
        float acc;

        if (obj->plan_running) {
            angle = obj->ctrl_src.joint_angle_solved[i];
            velocity = obj->plan_joint_vel[i] * PLAN_SPEED_COMP_COEF[i];
            acc = obj->plan_joint_acc[i];
        } else if (use_external_velocity) {
            angle = obj->joint_angle_td[i].v1;
            velocity = cmd->joint_vel[i];
            acc = 0.0f;
        } else if (obj->mode == arm_run) {
            Joint_Quintic_Ref_Update(obj, i);
            angle = obj->joint_quintic_ref[i].position;
            velocity = obj->joint_quintic_ref[i].velocity;
            acc = obj->joint_quintic_ref[i].acc;
        } else {
            angle = obj->joint_angle_td[i].v1;
            velocity = obj->joint_angle_td[i].v2;
            acc = obj->joint_angle_td[i].v3;
        }

        if (obj->mode != arm_run) acc = 0.0f;
        if (!isfinite(angle) || !isfinite(velocity) || !isfinite(acc)) {
            obj->mode = arm_damping;  // 失效时切到被动模式，防止电机出力异常
            control->mode = obj->mode;
            for (int j = 0; j < ARM_JOINT_NUM; j++) {
                control->joint_angle[j] = obj->gimbal->arm_feedback_data.joint_angle[j];
                control->joint_vel[j] = 0.0f;
                control->joint_acc[j] = 0.0f;
                obj->joint_quintic_ref[j].initialized = 0;
                obj->joint_quintic_ref[j].active = 0;
            }
            return;
        }

        control->joint_angle[i] = angle;
        control->joint_vel[i] = velocity;
        control->joint_acc[i] = acc;
        if (obj->mode != arm_run || obj->plan_running || use_external_velocity) {
            Joint_quintic_ref* ref = &obj->joint_quintic_ref[i];
            if (obj->mode != arm_run) {
                ref->position = obj->gimbal->arm_feedback_data.joint_angle[i];
                ref->velocity = 0.0f;
                ref->acc = 0.0f;
            } else {
                ref->position = angle;
                ref->velocity = velocity;
                ref->acc = acc;
            }
            ref->initialized = 1;
            ref->active = 0;
        }
    }
}

static void joint_src_follow(Gimbal_cmd* obj) {
    // 关节角控制源跟随反馈，保持命令与反馈跟随，防止跳变
    memcpy(obj->ctrl_src.joint_src.joint_angle, obj->gimbal->arm_feedback_data.joint_angle,
        sizeof(obj->ctrl_src.joint_src.joint_angle));
}

static void joint_angle_td_follow(Gimbal_cmd* obj, float ref[ARM_JOINT_NUM]) {
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->joint_angle_td[i].ref = ref[i];
        obj->joint_angle_td[i].v1 = obj->joint_angle_td[i].ref;
        obj->joint_angle_td[i].v2 = 0;
        obj->joint_angle_td[i].v3 = 0;
    }
}
