#include "arm_zero.h"

#include <math.h> /* fabsf() */

#include "main.h"       /* HAL_GetTick() */
#include "gimbal_arm.h" /* gimbal：7 个电机的指针都挂在它身上(见 Gimbal_t) */
#include "DM_motor.h"   /* DMmotor_Disable / Enable / Save_Zero / errcode / fdb_position */
#include "ENCOS_motor.h" /* ENCOSmotor_Disable / Save_Zero / zero_ack_* */

/* 为什么跑在 MotorTask、为什么设零期间要把两条总线全关掉：见 arm_zero.h 的文件头。 */

/* ================= 实测记录(2026-10-04，4 台达妙在机上，1kHz 状态机) =================
 *
 * 1) 0xFE(设零)本身是有效的：发过之后 4 台的位置都变成 ≈0(夹爪从 −1.32 跳到 −0.0004)，
 *    断电重上仍在。且**失能状态下才写得进去**(达妙协议要求)，本模块先失能是对的。
 *
 * 2) 失能确认(0xFD 之后)：j4/j5/夹爪 的 errcode 在 130~350ms 内变成 0(=失能)，
 *    j6(DM2325) 更慢(>480ms)。**更要命的是失能后不是所有电机都继续发反馈**：
 *    j5 收到 0xFD 之后就彻底不发了，motor_fps.now 永远停在失能前那一刻、errcode 也
 *    永远停在 1 —— 只看 errcode 会一直等不到确认。所以确认判据是"errcode==0 或
 *    已经安静 ≥300ms 二者之一"，见 dm_disabled_confirmed()。
 *
 * 3) 因此**检查设零是否成功**也不能死等反馈：失能后不发反馈的电机(m5 那种)在设零后
 *    同样不会汇报，只能像达妙上位机那样**发一次使能帧(0xFC)把它唤醒**，读到新位置之后
 *    再发 0xFD 关回去。见 check_current() 里达妙那一支的两条路。
 *
 * 4) 总线是故意安静的，而电机失能后可能不再发反馈 → 20ms 的掉线看门狗会把它们全判掉线。
 *    keep_monitors_alive() 在设零期间主动喂狗，理由见那里的注释。
 * ================================================================================= */

/* ---------- 时间参数 ---------- */
/* 失能确认窗口。要等所有参与电机都"确认失能"，早到就早走。实测最慢的 j6 也要 >480ms，
 * 加上"安静 300ms 也算失能"这条判据，1500ms 足够。 */
#define ARM_ZERO_DISABLE_CONFIRM_MS 1500u
/* 等确认期间每 100ms 重发一遍失能帧("不成功就反复")。 */
#define ARM_ZERO_DISABLE_RETRY_MS 100u
/* 失能帧发出后安静这么久没新反馈，就认为它已经失能(失能后不汇报的固件)。 */
#define ARM_ZERO_DISABLE_SILENT_MS 300u

/* 失能确认要不要真的等(见上)。改成 0 = 发完失能帧等固定时间就往下走，是明确的退让，
 * 只在这套判据完全不成立时才用。 */
#define ARM_ZERO_DISABLE_NEED_CONFIRM 1
/* NEED_CONFIRM 为 0 时用的固定等待时间 */
#define ARM_ZERO_DISABLE_FALLBACK_MS 100u

/* 达妙：设零帧发出后先等这么久再看反馈(反馈 500Hz，20ms 至少 10 帧) */
#define ARM_ZERO_DM_VERIFY_DELAY_MS 20u
/* 达妙：发使能帧唤醒之后等它把反馈发回来 */
#define ARM_ZERO_DM_WAKE_MS 100u
/* 达妙单台反复重发的上限(从进入这台算起) */
#define ARM_ZERO_DM_PER_MOTOR_MS 1000u
/* 达妙位置容差(rad)。12.5rad/65536 ≈ 0.00038rad 一个码，0.01rad 已经很宽。 */
#define ARM_ZERO_DM_POS_TOL_RAD 0.01f

/* ENCOS 单台上限。库内部：一次设零后总线要静默 501ms(ENCOS_SETTING_GUARD_MS)才能再发，
 * ACK 超时判据也是 1000ms(ENCOS_ZERO_ACK_TIMEOUT_MS)，重发一次的周期就 ≥1s，这里要更大。 */
#define ARM_ZERO_ENCOS_PER_MOTOR_MS 1500u

/* 整体上限 */
#define ARM_ZERO_TOTAL_MS 8000u

/* ---------- 反馈(全局，JLink 直接读) ---------- */
volatile uint8_t arm_zero_request = 0;
volatile uint8_t arm_zero_state = ARM_ZERO_IDLE;
volatile uint8_t arm_zero_result[ARM_MOTOR_NUM] = {0};
volatile uint8_t arm_zero_retry[ARM_MOTOR_NUM] = {0};
volatile uint32_t arm_zero_elapsed_ms = 0;
volatile uint8_t arm_zero_disable_confirmed = 0;
uint8_t arm_zero_encos_enable = 0;
/* 参与集覆盖：**非 0 时这一轮只让这些位参与**(bit = Arm_Motor_Index)，0 = 照旧(达妙 4 台
 * 恒参与、j1~j3 看 arm_zero_encos_enable)。一键夹爪找零(App/arm_auto.c)用它把一轮设零
 * 缩到夹爪一台 —— "设零帧和常规控制帧互斥"这条没变(那一轮照样整条总线不发帧)，但 j4~j6
 * 不再被失能，臂不会当场松掉。
 * 它是**一次性**的：start() 把它锁进 s_participate_mask 之后就地清 0(和 arm_zero_request
 * 被消费一个路子)，所以不会漏下"以后每轮都只设夹爪"这种状态。 */
uint8_t arm_zero_participate_override = 0;
/* 哪些电机需要在"检查"阶段先被使能帧唤醒才读得到位置(bit = Arm_Motor_Index)。
 * 实测 j5 失能后不再发反馈，就会落到这一位；全是 0 说明 4 台失能后都还继续汇报。 */
volatile uint8_t arm_zero_wake_mask = 0;

/* ---------- 内部状态(静态零初始化即可) ---------- */
static uint32_t s_start_ms;  /* 本轮开始时刻 */
static uint32_t s_phase_ms;  /* 当前阶段(等失能确认 / 当前这一台)的起点 */
static uint32_t s_send_ms;   /* 当前这一台最近一次"发出的那一帧"的时刻 */
static uint32_t s_retry_ms;  /* 最近一次重发失能帧的时刻 */
static uint8_t s_index;      /* 正在处理的电机，ARM_MOTOR_NUM 表示跑完了 */
static uint8_t s_dm_woken;   /* 当前这台达妙是不是已经被使能帧唤醒过(读完要关回去) */
static float s_before_pos;   /* 发设零帧之前的位置：分辨"真被拉到 0"还是"本来就在 0" */
/* 本轮的参与集，bit = Arm_Motor_Index，**在 start() 里按当时的 arm_zero_encos_enable 锁定**。
 * 不能每次现问那个全局：判定成功/超时是在 finish() 里做的，若运行途中有人把开关翻回 0，
 * 现问就会把已经记成"超时"的 ENCOS 当成"本来就不参与"，于是本该报 TIMEOUT 的一轮被判成
 * DONE —— 实测就是这么撞出来的。 */
static uint8_t s_participate_mask;

/* ---------- 电机取用 ---------- */
/* 7 台电机都挂在 Gimbal 上(见 gimbal_arm.h)，用索引拿。拿不到(创建失败)返回 NULL。 */
static DM_motor* dm_at(Gimbal* g, uint8_t i) {
    switch (i) {
    case JOINT4: return g->joint4;
    case JOINT5: return g->joint5;
    case JOINT6: return g->joint6;
    case GRIPPER: return g->gripper;
    default: return NULL;
    }
}

static ENCOS_motor* encos_at(Gimbal* g, uint8_t i) {
    switch (i) {
    case JOINT1: return g->joint1;
    case JOINT2: return g->joint2;
    case JOINT3: return g->joint3;
    default: return NULL;
    }
}

/* 本轮这台参不参与。看的是 start() 锁下来的 s_participate_mask，不是实时的开关。 */
static uint8_t participates(uint8_t i) {
    return (uint8_t)((s_participate_mask >> i) & 1u);
}

/* 从 from 开始找下一台参与的电机，返回 ARM_MOTOR_NUM 表示没有了 */
static uint8_t next_participant(uint8_t from) {
    while (from < ARM_MOTOR_NUM && !participates(from)) from++;
    return from;
}

/* 这一台从 since(含)起发过反馈吗。motor_fps.now 是"最后一帧反馈的到达时刻"
 * (FrameRateStatistics() 每帧都刷它)，时钟回绕按有符号差值判。
 *
 * **必须是"含同一毫秒"(>=)，不能写成严格大于**：2026-10-04 实测这几台是"收一帧回一帧"
 * 的问答式，我们发出去的帧和它回来的反馈落在**同一个 ms**里(motor_fps.now == 发帧时刻)。
 * 写成 > 会把这个有效反馈判掉，然后因为没有新帧可等而反复重发 —— 实测就是这样让 j4 白
 * 重发 8 次然后报超时的。代价是"发帧前一瞬间刚到的那一帧"也可能被当成新帧，但那一帧
 * 携带的是**设零之前**的位置：位置若在容差内，结论只会是"无法确认"(UNCONFIRMED)，
 * 若在容差外则判失败重发 —— 两个方向都不会谎报成功，所以这个口径是安全的。 */
static uint8_t dm_reported_since(DM_motor* dm, uint32_t since) {
    return (int32_t)((uint32_t)dm->motor_fps.now - since) >= 0;
}

/* ---------- 步骤 ---------- */
static void disable_all(Gimbal* g) {
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        if (!participates(i)) continue;
        DM_motor* dm = dm_at(g, (uint8_t)i);
        if (dm) {
            DMmotor_Disable(dm);
            continue;
        }
        ENCOS_motor* ec = encos_at(g, (uint8_t)i);
        if (ec) ENCOSmotor_Disable(ec);
    }
}

/* 失能确认(达妙)。判据见文件头实测记录第 2 条：errcode==0，或者已经安静够久。 */
static uint8_t dm_disabled_confirmed(DM_motor* dm, uint32_t now) {
    if (dm->errcode == dm_err_disable) return 1;
    return (uint32_t)(now - (uint32_t)dm->motor_fps.now) >= ARM_ZERO_DISABLE_SILENT_MS;
}

/* 失能确认：只对达妙逐台判。ENCOS 的 errcode 是故障码、没有"我失能了"的反馈位，
 * 所以参与了也只算它一票通过 —— 等 CAN3 通了再回来看要不要补判据。 */
static uint8_t all_disabled(Gimbal* g, uint32_t now) {
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        if (!participates(i)) continue;
        DM_motor* dm = dm_at(g, (uint8_t)i);
        if (dm == NULL) continue; /* 对象都没建起来，判不了 */
        if (!dm_disabled_confirmed(dm, now)) return 0;
    }
    return 1;
}

/* 设零期间总线是故意安静的，电机失能后还可能不再发反馈 —— 这会让 20ms 的掉线看门狗
 * 把它们全判成掉线。掉线本身无所谓(arm_mode 本来就在 arm_stop)，但 gimbal_arm.c 的
 * Motor_Lost_Handle() 会把 motor_inited[JOINT6] 清 0，而唯一能把它置回去的
 * Arm_Auto_Init() 在 j1~j3 掉线时根本跑不起来 —— 一次设零之后 j6 就永远是"未初始化"了。
 * 所以设零期间主动喂狗：这份静默是我们自己造成的，不该记成掉线。 */
static void keep_monitors_alive(Gimbal* g) {
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        DM_motor* dm = dm_at(g, (uint8_t)i);
        if (dm && dm->monitor) {
            dm->monitor->reset(dm->monitor);
            continue;
        }
        ENCOS_motor* ec = encos_at(g, (uint8_t)i);
        if (ec && ec->monitor) ec->monitor->reset(ec->monitor);
    }
}

/* 本轮结束：扫描每台的结果决定 DONE 还是 TIMEOUT(两种情况都要给反馈)。
 * force_bad=1 用于"失能没确认"和"整体超时"这两条没有单台结果的失败路径。 */
static void finish(uint32_t now, uint8_t force_bad) {
    uint8_t bad = force_bad;
    for (int i = 0; i < ARM_MOTOR_NUM && !bad; i++) {
        if (!participates((uint8_t)i)) continue;
        if (arm_zero_result[i] == ARM_ZERO_RES_TIMEOUT ||
            arm_zero_result[i] == ARM_ZERO_RES_FAIL ||
            arm_zero_result[i] == ARM_ZERO_RES_NONE)
            bad = 1;
    }
    arm_zero_elapsed_ms = (uint32_t)(now - s_start_ms);
    arm_zero_state = bad ? ARM_ZERO_TIMEOUT : ARM_ZERO_DONE;
}

/* 这一台完事，换下一台；没有下一台就结束 */
static void advance(Gimbal* g, uint32_t now) {
    s_index = next_participant((uint8_t)(s_index + 1));
    s_phase_ms = now;
    s_dm_woken = 0;
    if (s_index >= ARM_MOTOR_NUM) {
        finish(now, 0);
        return;
    }
    arm_zero_state = ARM_ZERO_SEND;
}

/* 重发当前这台的设零帧(达妙)。若刚唤醒过，先补一帧失能把它关回去。 */
static void dm_resend_zero(DM_motor* dm, uint32_t now) {
    if (s_dm_woken) {
        DMmotor_Disable(dm);
        s_dm_woken = 0;
    }
    DMmotor_Save_Zero(dm);
    arm_zero_retry[s_index]++;
    s_send_ms = now;
    /* s_before_pos 不更新：它代表"这台设零之前本来是多少" */
}

/* 达妙：读位置判成败。成功要顺手把唤醒过的电机再关回去。 */
static void dm_evaluate(Gimbal* g, DM_motor* dm, uint32_t now) {
    if (fabsf(dm->fdb_position) <= ARM_ZERO_DM_POS_TOL_RAD) {
        if (fabsf(s_before_pos) <= ARM_ZERO_DM_POS_TOL_RAD) {
            /* 设零前本来就在容差内：前后都是 ~0，**无法分辨**这一帧到底写进去没有。
             * 如实记 UNCONFIRMED，不谎报成功。 */
            arm_zero_result[s_index] = ARM_ZERO_RES_UNCONFIRMED;
        } else {
            /* 明显被拉到了 0 —— 设零确实生效了 */
            arm_zero_result[s_index] = ARM_ZERO_RES_OK;
            /* 设零后编码器读数整体平移，累计圈数 round 是相对旧零点的，按新零点归零。
             * 这里在 MotorTask 里写、CAN 中断里也会写 round，是个宽宏大量的竞争
             * (设零期间本来就没有常规控制)。 */
            dm->round = 0;
            dm->real_fdb_position = dm->fdb_position;
        }
        if (s_dm_woken) {
            DMmotor_Disable(dm); /* 读完关回去，保持"全失能" */
            s_dm_woken = 0;
        }
        advance(g, now);
        return;
    }
    dm_resend_zero(dm, now); /* 没到 0：重发 */
}

/* ② 给当前这台发设零帧。一次只发一台：ENCOS 有">500ms 总线指令间隔"的硬要求，
 * 达妙一次一台也无害。 */
static void send_current(Gimbal* g, uint32_t now) {
    DM_motor* dm = dm_at(g, s_index);
    if (dm) {
        s_before_pos = dm->fdb_position;
        s_dm_woken = 0;
        DMmotor_Save_Zero(dm);
        s_send_ms = now;
        arm_zero_state = ARM_ZERO_CHECK;
        return;
    }
    ENCOS_motor* ec = encos_at(g, s_index);
    if (ec == NULL) {
        arm_zero_result[s_index] = ARM_ZERO_RES_FAIL; /* 对象没建起来，这台的活干不了 */
        advance(g, now);
        return;
    }
    ENCOS_Result r = ENCOSmotor_Save_Zero(ec);
    if (r == ENCOS_BUSY) {
        /* 还在上一台设零帧之后的总线静默期(501ms)里，下一拍再试 */
        arm_zero_state = ARM_ZERO_SEND;
        return;
    }
    s_send_ms = now;
    arm_zero_state = ARM_ZERO_CHECK;
}

/* ③ 检查当前这一台成没成；不成就在单台窗口内反复重发 */
static void check_current(Gimbal* g, uint32_t now) {
    DM_motor* dm = dm_at(g, s_index);
    uint32_t limit = dm ? ARM_ZERO_DM_PER_MOTOR_MS : ARM_ZERO_ENCOS_PER_MOTOR_MS;
    if ((uint32_t)(now - s_phase_ms) >= limit) {
        /* 反复试到超时。达妙"本来就在 0 附近、写没写进去看不出来"的那种，上面已经判过
         * UNCONFIRMED 了，走到这里是真的没拉到 0。 */
        if (arm_zero_result[s_index] != ARM_ZERO_RES_FAIL)
            arm_zero_result[s_index] = ARM_ZERO_RES_TIMEOUT;
        advance(g, now);
        return;
    }

    if (dm == NULL) {
        /* ENCOS：库自己收 0x7FF 的应答(ENCOS_motor.c 的 RxCallBack)，我们只看结果 */
        ENCOS_motor* ec = encos_at(g, s_index);
        if (ec == NULL) {
            arm_zero_result[s_index] = ARM_ZERO_RES_FAIL;
            advance(g, now);
            return;
        }
        if (ec->zero_ack_received) {
            if (ec->zero_ack_success) {
                arm_zero_result[s_index] = ARM_ZERO_RES_OK;
                advance(g, now);
            } else {
                /* 协议明确回了"失败"：记下来再发一遍(Save_Zero 会清掉上次的 ACK 标志) */
                arm_zero_result[s_index] = ARM_ZERO_RES_FAIL;
                if (ENCOSmotor_Save_Zero(ec) == ENCOS_OK) {
                    arm_zero_retry[s_index]++;
                    s_send_ms = now;
                }
            }
        }
        return;
    }

    /* 达妙：没有 ACK，只能看反馈。两条路(见文件头实测记录第 3 条)：
     *   A. 失能后**仍然**发反馈的固件(实测 j4)：等一帧新反馈直接读位置；
     *   B. 失能后**不再**发反馈的固件(实测 j5)：没反馈可读，先发一帧使能(0xFC)把它
     *      唤醒，读到新位置之后再发 0xFD 关回去 —— 这也是达妙上位机设完零点的做法。 */
    if (s_dm_woken == 0) {
        if ((uint32_t)(now - s_send_ms) < ARM_ZERO_DM_VERIFY_DELAY_MS) return;
        if (dm_reported_since(dm, s_send_ms)) { /* A 路 */
            dm_evaluate(g, dm, now);
            return;
        }
        DMmotor_Enable(dm); /* B 路：唤醒 */
        s_dm_woken = 1;
        arm_zero_wake_mask |= (uint8_t)(1u << s_index);
        s_send_ms = now; /* 复用这个时间戳当"唤醒帧发出时刻" */
        return;
    }
    /* 唤醒帧已经发出去了，等它把反馈发回来 */
    if ((uint32_t)(now - s_send_ms) < ARM_ZERO_DM_WAKE_MS) return;
    if (!dm_reported_since(dm, s_send_ms)) {
        /* 唤醒也没换来反馈：整台重来(先补失能，再重发设零帧) */
        dm_resend_zero(dm, now);
        return;
    }
    dm_evaluate(g, dm, now);
}

/* 开始一轮 */
static void start(Gimbal* g, uint32_t now) {
    /* 先定参与集(见 s_participate_mask 的注释)：达妙 4 台恒参与，ENCOS 那 3 台看开关。
     * 这一轮从头到尾都按这个集子走，中途翻开关不影响本轮。 */
    s_participate_mask = arm_zero_participate_override;
    /* 覆盖是**一次性**的：锁进 s_participate_mask 之后就地清掉(和 arm_zero_request 被消费
     * 一个路子)。这样即使触发的那个上层动作被外部中途打断(比如遥控器上线，robot_state
     * 会把 auto_mode 直接清掉，那边的收尾代码就跑不到了)，也不会留下一个"以后每次设零
     * 都只设夹爪"的雷。 */
    arm_zero_participate_override = 0;
    if (s_participate_mask == 0) { /* 覆盖为 0 = 照旧(整臂) */
        for (int i = 0; i < ARM_MOTOR_NUM; i++) {
            if (i <= JOINT3 && !arm_zero_encos_enable) continue; /* j1~j3 占位：只记 SKIP */
            s_participate_mask |= (uint8_t)(1u << i);
        }
    }
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        arm_zero_result[i] = participates((uint8_t)i) ? ARM_ZERO_RES_NONE : ARM_ZERO_RES_SKIP;
        arm_zero_retry[i] = 0;
    }
    s_start_ms = now;
    s_phase_ms = now;
    s_retry_ms = now;
    s_index = next_participant(0);
    s_dm_woken = 0;
    s_before_pos = 0.0f;
    arm_zero_elapsed_ms = 0;
    arm_zero_disable_confirmed = 0;
    arm_zero_wake_mask = 0;

    disable_all(g);                         /* ① 所有参与电机失能 */
    arm_zero_state = ARM_ZERO_WAIT_DISABLE; /* ② 等确认 */
}

uint8_t Arm_Zero_Busy(void) {
    switch ((Arm_zero_state)arm_zero_state) {
    case ARM_ZERO_DISABLE:
    case ARM_ZERO_WAIT_DISABLE:
    case ARM_ZERO_SEND:
    case ARM_ZERO_CHECK:
        return 1;
    default:
        return 0;
    }
}

void Arm_Zero_Loop(void) {
    /* gimbal 是 CmdTask 里建的(Gimbal_Create() 依赖 CAN 已初始化，见 gimbal_arm.h)。
     * MotorTask 里读这个指针：指针写是原子的，最坏情况也只是这一拍拿到 NULL —— 那样
     * 请求留着不消费，下一拍再来。 */
    Gimbal* g = gimbal;
    uint32_t now = HAL_GetTick();

    if (!Arm_Zero_Busy()) {
        /* 空闲或上一轮已结束：只有请求来了才开新一轮(所以写 1 可以重复触发) */
        if (!arm_zero_request || g == NULL) return;
        arm_zero_request = 0;
        start(g, now);
        return;
    }

    if (g == NULL) {
        /* 理论上不会：状态机是 gimbal 存在之后才起来的 */
        finish(now, 1);
        return;
    }

    keep_monitors_alive(g);

    /* 整体兜底 */
    if ((uint32_t)(now - s_start_ms) >= ARM_ZERO_TOTAL_MS) {
        finish(now, 1);
        return;
    }

    switch ((Arm_zero_state)arm_zero_state) {
    case ARM_ZERO_WAIT_DISABLE:
#if ARM_ZERO_DISABLE_NEED_CONFIRM
        if (all_disabled(g, now)) {
            arm_zero_disable_confirmed = 1;
            s_index = next_participant(0);
            s_phase_ms = now;
            if (s_index >= ARM_MOTOR_NUM) {
                finish(now, 0);
                break;
            }
            arm_zero_state = ARM_ZERO_SEND;
        } else if ((uint32_t)(now - s_phase_ms) >= ARM_ZERO_DISABLE_CONFIRM_MS) {
            /* 等不到失能确认：**不往下发设零帧**(电机可能还带着力矩)。
             * 如实报超时，arm_zero_disable_confirmed 留在 0 供排查。 */
            finish(now, 1);
        } else if ((uint32_t)(now - s_retry_ms) >= ARM_ZERO_DISABLE_RETRY_MS) {
            s_retry_ms = now;
            disable_all(g); /* 没确认就再发一遍 */
        }
#else
        /* 退让路径：不判确认，发完等固定时间就走 */
        if ((uint32_t)(now - s_phase_ms) >= ARM_ZERO_DISABLE_FALLBACK_MS) {
            arm_zero_disable_confirmed = 1;
            s_index = next_participant(0);
            s_phase_ms = now;
            if (s_index >= ARM_MOTOR_NUM) {
                finish(now, 0);
                break;
            }
            arm_zero_state = ARM_ZERO_SEND;
        }
#endif
        break;

    case ARM_ZERO_SEND:
        send_current(g, now);
        break;

    case ARM_ZERO_CHECK:
        check_current(g, now);
        break;

    default: /* DONE / TIMEOUT / IDLE 不该走到这里 */
        break;
    }
}
