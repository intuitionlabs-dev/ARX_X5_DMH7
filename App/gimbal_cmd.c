#include "gimbal_cmd.h"
#include <stdlib.h>         /* malloc */
#include <string.h>         /* memcpy / memset */
#include "arm_auto.h"       /* Arm_Reset() 等一键模式的动作本体 */
#include "common.h"         /* fclip */
#include "gimbal_cmd_def.h" /* JOINT_ANGLE_MIN/MAX、PLAN_SPEED_COMP_COEF */
#include "move_plan.h"      /* move_plan_run(判规划是否在跑) */
#include "plan_group.h"     /* Plan_Group_Temp_Create() */
#include "rc.h"

/* ===================== 相对于源工程砍掉了什么 =====================
 *
 * 源工程的 gimbal_cmd.c(1703 行) 是那台轮腿车的**底盘 + 机械臂协调层**，绑在一整套
 * 本工程没有的基础设施上。下面按源工程 Gimbal_Cmd_Update() 的骨架逐条对照，★ 是删掉的：
 *
 *   取得通信数据          ★ 板间 socket(和底盘 F7 换包)、图传(键鼠 + 自定义控制器)、
 *                           机械臂反馈订阅(pub_sub) —— 本工程是三块板合一
 *   电机掉线处理          ★ 上游在这里汇总 any_motor_lost 再报警 + 拉停。本工程
 *                           gimbal_arm.c 的 Motor_Lost_Handle() 每拍都把 arm_mode 打回
 *                           arm_stop 了，这里是重复；音乐蜂鸣器也没有
 *   机械臂目标同步处理    ★ 靠 arm_ref_synced_flag 把目标拽到折叠姿态。本工程的等价物
 *                           是 Stop_Mode_Update() 里"控制量跟随反馈"(见那里注释)
 *   Set_Ctrl_Default      保留(只剩关节速度/加速度置零)
 *   Arm_Fdb_Calc          ★ 要正运动学 Serial_Link_FKine()，arm_serial_link 没移植
 *   reset_time_calc       ★ 同上(要腕点位置)，而且只服务一键复位
 *   reset_j3_min_calc     ★ 同上(要末端/腕点位置)，只服务一键复位
 *   模式分派              ★ 图传键鼠模式 / DT7 遥控器模式。本工程换成"遥控器(rc)的
 *                           sw_a 启停 + 摇杆点动"，只剩 arm_stop / arm_run 两支
 *   Custom_Update         ★ 自定义控制器(图传链路)
 *   一键模式(arm_auto)    保留(**动作本体在 App/arm_auto.c**，本工程只做复位)，
 *                           这里只留分派 Auto_Mode_Update()
 *   Plan_Running_Check    保留(源工程两个来源，本工程只剩临时规划组 temp_group 那一个；
 *                           另一半在遍历 obj->pkg.plan_group_list，本工程没有 Plan_pkg)
 *   Ctrl_Src_Calc         保留(只剩关节控制源那一支)
 *   Motor_Angle_Limit     保留(单关节软限位)
 *   Ctrl_Post_Process     保留(TD，外加"规划在跑就跟着对齐")
 *   Arm_Ref_Set           保留(现在是两支：规划在跑用规划量，否则用 TD)
 *   Rfid_Handle           ★ RFID
 *   杂项(go_on/skip/温度报警) ★ 都靠上游那些输入；只留 last_auto_mode 的更新
 *   Send_Data             ★ 板间 socket + 拓展板 uart + pub_sub publish；本工程没有发送
 *                           这一步，直接写 gimbal->arm_control_data 就完了
 *
 * 另外这些字段/函数也是整块删的：
 *   - cartesian_src / custom_selfsaving_src(笛卡尔空间控制、自定义控制器腕点控制)
 *     —— 要 arm_kinematics_calc，按用户要求不做，枚举成员在 gimbal_cmd.h 里注释着
 *   - joint_td_accel[] + td_accel_estimate/td_accel_follow —— 本工程的 TD_t 只有 v1/v2，
 *     没有源工程那个 v3 和加速度估计器
 *   - 夹爪遥控控制、支架干涉航点(BRACKET_*)、J2_LOCK_MIN —— 都靠上游那些输入
 *   - Arm_Syn_Ref_Handle()/Arm_Joint_Send_Reinit_And_Sync() —— service 框架
 *   - 蜂鸣器 —— Bsp/inc/bsp_buzzer.h 只有 Init/On/Off/Toggle，没有能放音乐的
 */

/* 本层跑在 CmdTask 里，周期见 Core/Src/freertos.c 的 StartCmdTask(tick += 2U)。
 * 源工程是 APP_PERIOD_S，也是 0.002 —— TD 的 h/h0 是"每拍"的步长，任务周期改了这里
 * 必须跟着改，否则跟踪微分器的可行域整个变样。 */
#define GIMBAL_CMD_PERIOD_S 0.002f

/* 命令层对象，Gimbal_Cmd_Create() 之后有效 */
Gimbal_cmd* gimbal_cmd = NULL;

/* 私有函数(顶层) */
// NOLINTBEGIN
static void Stop_Mode_Update(Gimbal_cmd* obj);    // 停机模式
static void Remote_Mode_Update(Gimbal_cmd* obj);  // 遥控器控制模式(sw_a 启停 + 摇杆点动)
static void Set_Ctrl_Default(Gimbal_cmd* obj);    // 部分控制默认值设置
static void Auto_Mode_Update(Gimbal_cmd* obj);    // 一键模式分派
static void Plan_Running_Check(Gimbal_cmd* obj);  // 规划运行状态检查
static void Ctrl_Src_Calc(Gimbal_cmd* obj);       // 解算各数据源
static void Motor_Angle_Limit(Gimbal_cmd* obj);   // 软限位
static void Ctrl_Post_Process(Gimbal_cmd* obj);   // 控制量后处理
static void Arm_Ref_Set(Gimbal_cmd* obj);         // 机械臂控制值设置
// NOLINTEND
/* 私有函数(非顶层) */
static void joint_src_follow(Gimbal_cmd* obj);  // 关节控制源跟随反馈
static void joint_angle_td_follow(Gimbal_cmd* obj, float ref[ARM_JOINT_NUM]);  // TD 直接跟随

Gimbal_cmd* Gimbal_Cmd_Create(void) {
    /* 命令层是挂在机械臂驱动上的，驱动没建起来就什么都不用做 */
    if (gimbal == NULL) return NULL;

    Gimbal_cmd* obj = (Gimbal_cmd*)malloc(sizeof(Gimbal_cmd));
    if (obj == NULL) return NULL;
    memset(obj, 0, sizeof(Gimbal_cmd));
    obj->gimbal = gimbal;
    /* ctrl_type 是 arm_ctrl_joint(枚举值 0)，memset 之后就是它，不用再写 */

    /* 关节角跟踪微分器初始化。*/
    const float td_r[ARM_JOINT_NUM] = {
        /* 400 */ 1000,
        700,
        700,
        4000,
        1000,
        4000,
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

    /* ---- 模式判定 ----*/
    obj->mode = arm_stop;
    if (is_Offline(rc.monitor))
        obj->mode = arm_stop;
    else if (rc.sw_a == 1)
        obj->mode = arm_run;
    else
        obj->mode = arm_stop;
    /* 控制类型：源工程有 关节 / 笛卡尔 / 自定义腕点 三种(靠遥控器或图传切)，本工程
     * 只剩下关节空间这一种(见 gimbal_cmd.h 的 Arm_ctrl_type)。 */
    obj->ctrl_type = arm_ctrl_joint;

    /* 控制默认值设置 */
    Set_Ctrl_Default(obj);

    /* 不同模式控制 */
    switch (obj->mode) {
    case arm_run:
        Remote_Mode_Update(obj);
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
     * 所以要用一键复位必须把 rc.sw_a 打在 arm_run 档，否则写进去也活不过一拍。 */
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
     * 末端位姿、腕点)，那两个数据源本工程没有，删了。 */
    joint_src_follow(obj);

    /* 关节角跟踪微分器跟随：直接对齐，不要"跟"出速度来 */
    joint_angle_td_follow(obj, obj->gimbal->arm_feedback_data.joint_angle);

    /* 源工程这里还做了：
     *   - 加速度估计器同步跟随(td_accel_follow)：本工程 TD_t 没有 v3，估计器整条链没搬；
     *   - plan_joint_vel/acc 置零：规划路径靠 arm_auto，删了；
     *   - gimbal_send_data.robot_mode / auto_mode / auto_step：底盘那一套，删了。 */
}

/* 遥控器控制模式。
 *
 * 源工程这份逻辑是围着 DT7 写的：右拨杆三档选"停 / 控底盘 / 控云台"，左拨杆三档再选
 * 具体控哪几个关节，另外还有拨轮(一键模式)、四向按键(夹爪开合 / j6 重初始化)、摇杆点动。
 *
 * 本工程的遥控器驱动是 libs/rc/(PWM 输入捕获，不是 DBUS/串口 DT7)，实测接线只有
 * 3 个比例轴 rc.rx / rc.ry / rc.lx 加一个二值开关 rc.sw_a，拨轮、四向按键、swB 都没接
 * (见 libs/rc/rc_config.h 那张接线表) —— 上游那套档位语义在这里铺不开。
 *
 * 所以这里**只做了最小的一套**：sw_a 当启停开关(见 Gimbal_Cmd_Update() 的模式分派)，
 * 三个摇杆各管一根轴 —— 右摇杆左右 j1、右摇杆上下 j2、左摇杆左右 j3，按**边沿**点动
 * (Etc_Get：一直按着只动一次)，每拍 30 deg。左摇杆上下、swB、拨轮、四向按键都没接线，
 * j4~j6 目前没有手控入口。
 *
 * 源工程那套档位语义(j1 0.15 / j2 0.3 / ... 的连续点动、拨轮键一键模式)在这里铺不开，
 * 但骨架(mode 分派 arm_stop / arm_run 两支)是照它留的，将来补功能不用动结构。 */
static void Remote_Mode_Update(Gimbal_cmd* obj) {
    obj->gimbal->arm_control_data.mode = arm_run;

    if (rc.sw_a == 0) {
        return;
    }

    // 右摇杆左右控制 j1，
    if (Etc_Get(rc.trig.rx_left)) obj->ctrl_src.joint_src.joint_angle[JOINT1] += 30;
    if (Etc_Get(rc.trig.rx_right)) obj->ctrl_src.joint_src.joint_angle[JOINT1] -= 30;
    // 右摇杆上下控制 j2
    if (Etc_Get(rc.trig.ry_up)) obj->ctrl_src.joint_src.joint_angle[JOINT2] += 30;
    if (Etc_Get(rc.trig.ry_down)) obj->ctrl_src.joint_src.joint_angle[JOINT2] -= 30;
    // 左摇杆左右控制 j3
    if (Etc_Get(rc.trig.lx_right)) obj->ctrl_src.joint_src.joint_angle[JOINT3] += 30;
    if (Etc_Get(rc.trig.lx_left)) obj->ctrl_src.joint_src.joint_angle[JOINT3] -= 30;
}

/* 一键模式分派(源工程 Gimbal_Cmd_Update() 里那个 switch (obj->auto_mode)，位置一样)。
 * 动作本体全在 App/arm_auto.c，这里只做"哪个模式调哪个函数"。
 *
 * **触发没接**：这个 switch 只认 obj->auto_mode 的值，而本工程没有任何地方去写它
 * (遥控器没有空闲按键)，所以正常跑起来恒是 arm_auto_none、什么都不做。要跑一键复位
 * 得由上层输入或者调试器直接写 obj->auto_mode = arm_auto_reset —— 注意 arm_stop 档位下
 * Stop_Mode_Update() 每拍都会把它清 0。 */
static void Auto_Mode_Update(Gimbal_cmd* obj) {
    switch (obj->auto_mode) {
    case arm_auto_reset:
        Arm_Reset(obj);
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
     * 跳变"。本工程只剩 arm_ctrl_joint 一个源，另外两个源已经删了，所以这里没有内容。 */

    /* 停机保护 */
    if (obj->mode == arm_stop) {
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
        obj->gimbal->arm_control_data.gripper_oc_angle =
            fclip(obj->gimbal->arm_control_data.gripper_oc_angle, GRIPPER_ANGLE_MIN,
                GRIPPER_ANGLE_MAX);
}

static void Ctrl_Post_Process(Gimbal_cmd* obj) {
    /* 关节角跟踪微分器：用解算出来的目标角当输入，输出的 v1/v2 就是限了速度和加速度
     * 之后的位置/速度，Arm_Ref_Set() 直接拿去用(规划没在跑的时候)。
     *
     * 源工程这里还有 td_accel_estimate()：由 TD 微分 + 低通滤波估计加速度，写进
     * joint_td_accel[i].v3。本工程的 TD_t 里根本没有 v3 这个字段，那套估计器没搬。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        // 使用控制源的关节角目标值作为微分器的输入
        obj->joint_angle_td[i].ref = obj->ctrl_src.joint_angle_solved[i];
        TDFunction_independent(&obj->joint_angle_td[i]);
    }

    /* 规划在跑的时候让 TD 直接对齐规划值（v1 = ref、v2 清零）。
     * Arm_Ref_Set() 这时不用 TD 的输出，所以这里喂的是**给规划结束那一拍准备的**：
     * 规划一停、plan_running 回 0，下一拍就切回用 TD 的 v1/v2 —— 如果 TD 还停在起规划
     * 之前的旧位置，切换瞬间目标值会跳回去。先对齐就没有这个跳变。
     * （源工程这里还跟着对齐加速度估计器 td_accel_follow()，本工程没有那个估计器。） */
    if (obj->plan_running) {
        joint_angle_td_follow(obj, obj->ctrl_src.joint_angle_solved);
    }
}

static void Arm_Ref_Set(Gimbal_cmd* obj) {
    /* 使用后处理的目标值：关节位置、速度和加速度(源工程还有关节力矩前馈，本工程
     * 在 gimbal_arm.c 的 Set_Motor_ref() 里算)。
     *
     * 分两支，和源工程一致：
     *   - **规划没在跑**：用 TD 的 v1/v2。加速度源工程用 joint_td_accel[i].v3
     *     (TD 微分 + LPF 估计)，本工程没有那个估计器，恒置 0 —— 反正 gimbal_arm.c
     *     现在也没读 joint_acc。
     *   - **规划在跑**(一键模式)：位置用 ctrl_src.joint_angle_solved(规划器给的**原始**
     *     值，故意不过 TD —— 轨迹本身就是五次样条，再叠一层 TD 反而把加减速削掉)，
     *     速度用规划器的速度乘 PLAN_SPEED_COMP_COEF(j1 那一路要压到 0.7，理由见
     *     gimbal_cmd_def.h)，加速度直接用规划器的。 */
    if (!obj->plan_running) {
        for (int i = 0; i < ARM_JOINT_NUM; i++) {
            obj->gimbal->arm_control_data.joint_angle[i] = obj->joint_angle_td[i].v1;
            obj->gimbal->arm_control_data.joint_vel[i] = obj->joint_angle_td[i].v2;
            obj->gimbal->arm_control_data.joint_acc[i] = 0;
        }
    } else {
        for (int i = 0; i < ARM_JOINT_NUM; i++) {
            obj->gimbal->arm_control_data.joint_angle[i] = obj->ctrl_src.joint_angle_solved[i];
            obj->gimbal->arm_control_data.joint_vel[i] =
                obj->plan_joint_vel[i] * PLAN_SPEED_COMP_COEF[i];
            obj->gimbal->arm_control_data.joint_acc[i] = obj->plan_joint_acc[i];
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
    }
}
