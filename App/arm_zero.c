#include "arm_zero.h"

#include <math.h> /* fabsf() */

#include "main.h"       /* HAL_GetTick() */
#include "gimbal_arm.h" /* Gimbal：7 个电机的指针 + 本轮设零的全部状态都在它身上 */
#include "DM_motor.h"   /* DMmotor_Disable / Enable / Save_Zero / errcode / fdb_position */
#include "ENCOS_motor.h" /* ENCOSmotor_Disable / Save_Zero / zero_ack_* */

/* 为什么跑在 MotorTask、为什么设零期间要把两条总线全关掉：见 arm_zero.h 的文件头。 */

/* ================= 设零行为依据(实测) =================
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
 *
 * 5) ENCOS 设零复核：**0x7FF 的应答只跟电机自己有关** —— 它写完才回，迟到量实测
 *    0.4~5s 不等，而且连续反复触发会越来越慢(同一块板连着十来轮实测：0.43 → 0.6 → 2.7
 *    → 4.9s)。往总线上顶帧(0xE0 位置查询、零增益停止帧两种都试过)不能让 ACK 提前到：
 *    实测有电机在自己的第一条顶帧之前就回了 ACK。所以**不能死等 ACK**。
 *    驱动这一侧是干净的：5s 那一轮里 j3 的 `zero_pending` 从头到尾是 1，ACK 一到就被收下
 *    (pend 1→0、rcv/ok 同时置 1) —— 慢的是电机，不是我们的收帧或判据。
 *    ⇒ 两条：① 等 ACK 设上限 ARM_ZERO_ENCOS_ACK_WAIT_MS(1500ms)，等够还不来就**退让
 *    到"只看设零后的新位置"**(和达妙那四台同一套判据、同一容差)，并把这台记进
 *    ack_missing_mask —— 死等它就会跟着电机的迟到量一起退化成假超时。② 顶帧仍然要发，
 *    但作用只有一条：**要一帧设零后的新位置** —— ENCOS 是问答式的，不主动要就只能等它
 *    自愿发(它确实也会自愿发，只是不保证时机)。
 * ============================================================================== */

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
/* 位置容差(rad)。12.5rad/65536 ≈ 0.00038rad 一个码，0.01rad 已经很宽。
 * 达妙和 ENCOS 同量纲(ENCOS_motor::fdb_position 也是 rad、量程同样 ±12.5rad)，
 * 所以这个容差两边通用。 */
#define ARM_ZERO_POS_TOL_RAD 0.01f

/* ENCOS：等 0x7FF 应答的上限。**这是个退让阈值**，见 check_current 的 ENCOS 支 ——
 * 实测 ACK 迟到量 0.4~5s 不等、反复触发还会越来越慢(文件头第 5 条)，而死等它会把"零点
 * 其实写好了"的电机报成假超时。所以等够这么久还不来就退让到"只看设零后的新位置"
 * (和达妙同一套判据)。1500ms 是实测正常值(0.4~0.6s)的两倍半；退让本身不损失可信度
 * —— 位置那一关照样卡着容差。 */
#define ARM_ZERO_ENCOS_ACK_WAIT_MS 1500u
/* ENCOS 单台上限。复核一走完就 advance，所以这是**上限不是延时**：正常一台 ≈ ACK(或退让
 * 阈值) + 一次 501ms 闸之后的位置应答 ≈ 0.6~2.1s。留到 2500 = 让退让路径(1500)之后还有
 * 501ms 的指令闸 + 一帧应答的余量，不至于擦边判超时。
 * (库内部的两个数：设零后总线静默 501ms = ENCOS_SETTING_GUARD_MS；待 ACK 超时 1000ms =
 *  ENCOS_ZERO_ACK_TIMEOUT_MS —— 后者只在驱动的 Send_Step 被调到时才推进，而设零期间这条
 *  总线的 Send_Step 被门关掉了，所以那个 1000ms 不会把我们的 zero_pending 清掉。) */
#define ARM_ZERO_ENCOS_PER_MOTOR_MS 2500u
/* ENCOS 复核时"顶一帧"之间的最小间隔(见 check_current 里的说明)。设零后头 501ms 里发什么
 * 都过不了驱动 bus_busy() 那把闸，所以这个数只决定 501ms 之后的节奏 —— 空闲期本来就是
 * 500Hz 发同一形状的停止帧，不必更保守。 */
#define ARM_ZERO_ENCOS_POKE_MS 120u

/* 整体上限。最坏情况：失能确认 1500ms + 3 台 ENCOS 各吃满 2500ms + 3 台达妙各 1000ms
 * = 12000ms —— 得罩得住，否则每台的复核还没走完就被整体超时砍掉(那才是真正看不出问题的
 * 那种失败)。**别忘了前面那段失能确认**：只算单台窗口的话正好也是 12000，擦边，所以要再
 * 留 3s 余量。 */
#define ARM_ZERO_TOTAL_MS 15000u

/* j1~j3 是否真的参与(0=占位，只标记 SKIP)。**不是状态、是策略开关**，所以留在全局：
 * 它描述的是"这块板子上 ENCOS 那条路能不能用"，和具体哪台臂无关(两台臂同构)。
 * 触发那一轮开始时会被**锁进** Arm_zero::participate_mask，运行途中翻它不影响本轮。
 *
 * 置 1 的前提是 j1~j3 的 ENCOS 在两条总线上正常跑(标称 250Hz 反馈、motor_inited 对应位
 * 为 1)。 */
uint8_t arm_zero_encos_enable = 1;

/* ---------- 其余状态全在 `Gimbal::zero` 里 ---------- */
/* 设零状态**必须按台分开**：两台臂共用一份的话，A 台设到一半、B 台一触发就把进度整个
 * 覆盖。所以统一收进 Arm_zero 结构体(定义与字段说明见 App/inc/arm_zero.h)、挂在 Gimbal
 * 上，本文件里每一处状态访问都长成 `g->zero.xxx`，并且**每个函数都得有那个 g**。 */

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

/* 本轮这台参不参与。看的是 start() 锁下来的 zero.participate_mask，不是实时的开关。 */
static uint8_t participates(Gimbal* g, uint8_t i) {
    return (uint8_t)((g->zero.participate_mask >> i) & 1u);
}

/* 从 from 开始找下一台参与的电机，返回 ARM_MOTOR_NUM 表示没有了 */
static uint8_t next_participant(Gimbal* g, uint8_t from) {
    while (from < ARM_MOTOR_NUM && !participates(g, from)) from++;
    return from;
}

/* 这一台从 since(含)起发过反馈吗。motor_fps.now 是"最后一帧反馈的到达时刻"
 * (FrameRateStatistics() 每帧都刷它)，时钟回绕按有符号差值判。
 *
 * **必须是"含同一毫秒"(>=)，不能写成严格大于**：这几台是"收一帧回一帧"的问答式，
 * 我们发出去的帧和它回来的反馈落在**同一个 ms**里(motor_fps.now == 发帧时刻)。写成 >
 * 会把这个有效反馈判掉，然后因为没有新帧可等而反复重发、最后报超时。代价是"发帧前一
 * 瞬间刚到的那一帧"也可能被当成新帧，但那一帧
 * 携带的是**设零之前**的位置：位置若在容差内，结论只会是"无法确认"(UNCONFIRMED)，
 * 若在容差外则判失败重发 —— 两个方向都不会谎报成功，所以这个口径是安全的。 */
static uint8_t dm_reported_since(DM_motor* dm, uint32_t since) {
    return (int32_t)((uint32_t)dm->motor_fps.now - since) >= 0;
}

/* ---------- 步骤 ---------- */
static void disable_all(Gimbal* g) {
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        if (!participates(g, i)) continue;
        DM_motor* dm = dm_at(g, (uint8_t)i);
        if (dm) {
            DMmotor_Disable(dm);
            continue;
        }
        ENCOS_motor* ec = encos_at(g, (uint8_t)i);
        if (ec) ENCOSmotor_Disable(ec);
    }
}

/* 失能确认(达妙)。判据见文件头第 2 条：errcode==0，或者已经安静够久。 */
static uint8_t dm_disabled_confirmed(DM_motor* dm, uint32_t now) {
    if (dm->errcode == dm_err_disable) return 1;
    return (uint32_t)(now - (uint32_t)dm->motor_fps.now) >= ARM_ZERO_DISABLE_SILENT_MS;
}

/* 失能确认：只对达妙逐台判。ENCOS 的 errcode 是故障码、没有"我失能了"的反馈位，
 * 所以参与了也只算它一票通过 —— 等 CAN3 通了再回来看要不要补判据。 */
static uint8_t all_disabled(Gimbal* g, uint32_t now) {
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        if (!participates(g, i)) continue;
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
static void finish(Gimbal* g, uint32_t now, uint8_t force_bad) {
    uint8_t bad = force_bad;
    for (int i = 0; i < ARM_MOTOR_NUM && !bad; i++) {
        if (!participates(g, (uint8_t)i)) continue;
        if (g->zero.result[i] == ARM_ZERO_RES_TIMEOUT ||
            g->zero.result[i] == ARM_ZERO_RES_FAIL ||
            g->zero.result[i] == ARM_ZERO_RES_NONE)
            bad = 1;
    }
    g->zero.elapsed_ms = (uint32_t)(now - g->zero.start_ms);
    g->zero.state = bad ? ARM_ZERO_TIMEOUT : ARM_ZERO_DONE;
}

/* 这一台完事，换下一台；没有下一台就结束 */
static void advance(Gimbal* g, uint32_t now) {
    g->zero.index = next_participant(g, (uint8_t)(g->zero.index + 1));
    g->zero.phase_ms = now;
    g->zero.dm_woken = 0;
    if (g->zero.index >= ARM_MOTOR_NUM) {
        finish(g, now, 0);
        return;
    }
    g->zero.state = ARM_ZERO_SEND;
}

/* 重发当前这台的设零帧(达妙)。若刚唤醒过，先补一帧失能把它关回去。 */
static void dm_resend_zero(Gimbal* g, DM_motor* dm, uint32_t now) {
    if (g->zero.dm_woken) {
        DMmotor_Disable(dm);
        g->zero.dm_woken = 0;
    }
    DMmotor_Save_Zero(dm);
    g->zero.retry[g->zero.index]++;
    g->zero.send_ms = now;
    /* g->zero.before_pos 不更新：它代表"这台设零之前本来是多少" */
}

/* 达妙：读位置判成败。成功要顺手把唤醒过的电机再关回去。 */
static void dm_evaluate(Gimbal* g, DM_motor* dm, uint32_t now) {
    if (fabsf(dm->fdb_position) <= ARM_ZERO_POS_TOL_RAD) {
        if (fabsf(g->zero.before_pos) <= ARM_ZERO_POS_TOL_RAD) {
            /* 设零前本来就在容差内：前后都是 ~0，**无法分辨**这一帧到底写进去没有。
             * 如实记 UNCONFIRMED，不谎报成功。 */
            g->zero.result[g->zero.index] = ARM_ZERO_RES_UNCONFIRMED;
        } else {
            /* 明显被拉到了 0 —— 设零确实生效了 */
            g->zero.result[g->zero.index] = ARM_ZERO_RES_OK;
            /* 设零后编码器读数整体平移，累计圈数 round 是相对旧零点的，按新零点归零。
             * 这里在 MotorTask 里写、CAN 中断里也会写 round，是个宽宏大量的竞争
             * (设零期间本来就没有常规控制)。 */
            dm->round = 0;
            dm->real_fdb_position = dm->fdb_position;
        }
        if (g->zero.dm_woken) {
            DMmotor_Disable(dm); /* 读完关回去，保持"全失能" */
            g->zero.dm_woken = 0;
        }
        advance(g, now);
        return;
    }
    dm_resend_zero(g, dm, now); /* 没到 0：重发 */
}

/* ② 给当前这台发设零帧。一次只发一台：ENCOS 有">500ms 总线指令间隔"的硬要求，
 * 达妙一次一台也无害。 */
static void send_current(Gimbal* g, uint32_t now) {
    DM_motor* dm = dm_at(g, g->zero.index);
    if (dm) {
        g->zero.before_pos = dm->fdb_position;
        g->zero.dm_woken = 0;
        DMmotor_Save_Zero(dm);
        g->zero.send_ms = now;
        g->zero.state = ARM_ZERO_CHECK;
        return;
    }
    ENCOS_motor* ec = encos_at(g, g->zero.index);
    if (ec == NULL) {
        g->zero.result[g->zero.index] = ARM_ZERO_RES_FAIL; /* 对象没建起来，这台的活干不了 */
        advance(g, now);
        return;
    }
    /* 记下"设零前是多少"：判成败要用它区分"真的被拉到 0"和"本来就在 0"(同达妙那条路)。
     * Save_Zero 会清掉 feedback_valid 的 POSITION 位，但 fdb_position 里的**值**还是最后
     * 收到的那一帧 —— 设零期间臂是失能静止的，这个值就是设零前的位姿。 */
    g->zero.before_pos = ec->fdb_position;
    ENCOS_Result r = ENCOSmotor_Save_Zero(ec);
    if (r == ENCOS_BUSY) {
        /* 还在上一台设零帧之后的总线静默期(501ms)里，下一拍再试 */
        g->zero.state = ARM_ZERO_SEND;
        return;
    }
    g->zero.send_ms = now;
    g->zero.state = ARM_ZERO_CHECK;
}

/* ③ 检查当前这一台成没成；不成就在单台窗口内反复重发 */
static void check_current(Gimbal* g, uint32_t now) {
    DM_motor* dm = dm_at(g, g->zero.index);
    uint32_t limit = dm ? ARM_ZERO_DM_PER_MOTOR_MS : ARM_ZERO_ENCOS_PER_MOTOR_MS;
    if ((uint32_t)(now - g->zero.phase_ms) >= limit) {
        /* 反复试到超时。达妙"本来就在 0 附近、写没写进去看不出来"的那种，上面已经判过
         * UNCONFIRMED 了，走到这里是真的没拉到 0。
         * ENCOS：ACK 报了成功、只是**位置没来得及核对上**的话不能报"超时/失败" ——
         * 电机那边很可能已经写好了(实测就是这样)，如实记"已发未确认"。 */
        ENCOS_motor* ecto = dm ? NULL : encos_at(g, g->zero.index);
        if (g->zero.result[g->zero.index] != ARM_ZERO_RES_FAIL)
            g->zero.result[g->zero.index] =
                (ecto != NULL && ecto->zero_ack_received && ecto->zero_ack_success)
                ? ARM_ZERO_RES_UNCONFIRMED
                : ARM_ZERO_RES_TIMEOUT;
        advance(g, now);
        return;
    }

    if (dm == NULL) {
        /* ENCOS：库自己收 0x7FF 的应答(ENCOS_motor.c 的 RxCallBack)。但 **ACK 只说明算命令
         * 收到了，不等于零点真写进去了** —— 库自己的 README 也写着"失败/超时不要假定零点
         * 设置成功；核对反馈后由应用决定是否重试"。所以 ACK 报成功之后还要再核一帧位置。
         *
         * "新"位置怎么判：驱动在**发出设零帧时**和**收到 ACK 时**都把 feedback_valid 的
         * POSITION 位清掉了(ENCOS_motor.c 的 Save_Zero / RxCallBack)，而它只被"解出一帧
         * 真位置"重新置起。所以这一位重新立起来 ⟺ 设零之后的第一帧位置反馈到了 ——
         * 不用自己记时间戳，也不会把设零前那帧老位置当成新位置读进来。 */
        ENCOS_motor* ec = encos_at(g, g->zero.index);
        if (ec == NULL) {
            g->zero.result[g->zero.index] = ARM_ZERO_RES_FAIL;
            advance(g, now);
            return;
        }
        /* ---- ACK 等它自己来；**别在这儿发帧去顶** ----
         * 见文件头第 5 条：ACK 的到达时刻只跟电机那边写完没有有关(实测 0.4~5s 不等，反复
         * 触发还会更慢)，位置查询(0xE0)和零增益停止帧都顶不动它 —— 实测发了帧遥测是回来了，
         * ACK 照样压着。设零期间这条总线是我们自己关着门的(见 arm_zero.h 文件头)，但**关门
         * 不影响 ACK** —— 它本来就是"写完了自己回来"的。
         * 等有上限：等够 ARM_ZERO_ENCOS_ACK_WAIT_MS 还不来就走下面的退让路(见判位置那段)。 */
        if (!ec->zero_ack_received) {
            if ((uint32_t)(now - g->zero.send_ms) < ARM_ZERO_ENCOS_ACK_WAIT_MS) return;
        } else if (!ec->zero_ack_success) {
            /* 协议明确回了"失败"：记下来再发一遍(Save_Zero 会清掉上次的 ACK 标志) */
            g->zero.result[g->zero.index] = ARM_ZERO_RES_FAIL;
            if (ENCOSmotor_Save_Zero(ec) == ENCOS_OK) {
                g->zero.retry[g->zero.index]++;
                g->zero.send_ms = now;
            }
            return;
        }
        if (!(ec->feedback_valid & ENCOS_HAS_POSITION)) {
            /* 等**设零之后的新位置**。POSITION 位在发设零帧和收 ACK 时都被驱动清掉了
             * (ENCOS_motor.c 的 Save_Zero / RxCallBack)，只有"解出一帧真位置"才重新立起 ——
             * 所以这一位立起来 ⟺ 我们要的那帧新位置到了，不会把设零前的老位置当新位置用。
             * (走退让路时 ACK 没来、没人清这一位，但 Save_Zero 已经清过一次了，同样只有
             *  设零后的真位置才立得起来。)
             * 怎么要：ENCOS 是问答式的，得主动顶一帧。**发的是零增益停止帧**
             * (ENCOSmotor_Disable() → send_stop() → type 0 混控帧，和空闲期 500Hz 扫描发的同一
             * 形状，停着的电机收到它不会动，也不会碰零点)。电机收到它回一帧 type 1 遥测，
             * 里面就带着位置。
             * 不必每拍都发：501ms 的总线指令闸(驱动 bus_busy())会挡掉太密的，这里再按
             * ARM_ZERO_ENCOS_POKE_MS 排一下节奏。发不出去(BUSY)就下一拍再来 —— 下一拍的
             * retry_ms 会重新起算，不会因为忙就空转到窗口尽头。 */
            if ((uint32_t)(now - g->zero.retry_ms) >= ARM_ZERO_ENCOS_POKE_MS) {
                g->zero.retry_ms = now; /* 借它当"上次顶帧"的时间戳 */
                ENCOSmotor_Disable(ec);
            }
            return;
        }
        /* ---- 判位置(两条路合流：ACK 确认过 / 等够退让) ----
         * 判据和达妙那条路**逐字相同**(用户对达妙那四台就是定的这个，容差也一样)：
         *   位置出容差 → 失败 + 重发；位置在容差内、设零前也在容差内 → 无法分辨，记"已发未确认"；
         *   位置在容差内、设零前在容差外 → 真的被拉到 0 了，记成功。
         * 没有 ACK 就走到这儿的，记进 ack_missing_mask：这一位让"ACK 确认过的成功"和
         * "只看位置的成功"在结果里分得开，不混成一个 OK(见 arm_zero.h 里那个字段的说明)。 */
        if (!ec->zero_ack_received)
            g->zero.ack_missing_mask |= (uint8_t)(1u << g->zero.index);
        if (fabsf(ec->fdb_position) <= ARM_ZERO_POS_TOL_RAD) {
            g->zero.result[g->zero.index] =
                fabsf(g->zero.before_pos) <= ARM_ZERO_POS_TOL_RAD
                    ? ARM_ZERO_RES_UNCONFIRMED
                    : ARM_ZERO_RES_OK;
            advance(g, now);
            return;
        }
        /* 位置没到零点(ACK 说成功 / 没等到 ACK 都算)：如实记失败并重发一次(重发受 501ms
         * 总线静默期约束，Save_Zero 忙的话这个 result 就留在 FAIL 上，由整体/单台超时收尾)。 */
        g->zero.result[g->zero.index] = ARM_ZERO_RES_FAIL;
        if (ENCOSmotor_Save_Zero(ec) == ENCOS_OK) {
            g->zero.retry[g->zero.index]++;
            g->zero.send_ms = now;
        }
        return;
    }

    /* 达妙：没有 ACK，只能看反馈。两条路(见文件头实测记录第 3 条)：
     *   A. 失能后**仍然**发反馈的固件(实测 j4)：等一帧新反馈直接读位置；
     *   B. 失能后**不再**发反馈的固件(实测 j5)：没反馈可读，先发一帧使能(0xFC)把它
     *      唤醒，读到新位置之后再发 0xFD 关回去 —— 这也是达妙上位机设完零点的做法。 */
    if (g->zero.dm_woken == 0) {
        if ((uint32_t)(now - g->zero.send_ms) < ARM_ZERO_DM_VERIFY_DELAY_MS) return;
        if (dm_reported_since(dm, g->zero.send_ms)) { /* A 路 */
            dm_evaluate(g, dm, now);
            return;
        }
        DMmotor_Enable(dm); /* B 路：唤醒 */
        g->zero.dm_woken = 1;
        g->zero.wake_mask |= (uint8_t)(1u << g->zero.index);
        g->zero.send_ms = now; /* 复用这个时间戳当"唤醒帧发出时刻" */
        return;
    }
    /* 唤醒帧已经发出去了，等它把反馈发回来 */
    if ((uint32_t)(now - g->zero.send_ms) < ARM_ZERO_DM_WAKE_MS) return;
    if (!dm_reported_since(dm, g->zero.send_ms)) {
        /* 唤醒也没换来反馈：整台重来(先补失能，再重发设零帧) */
        dm_resend_zero(g, dm, now);
        return;
    }
    dm_evaluate(g, dm, now);
}

/* 开始一轮 */
static void start(Gimbal* g, uint32_t now) {
    /* 先定参与集(见 g->zero.participate_mask 的注释)：达妙 4 台恒参与，ENCOS 那 3 台看开关。
     * 这一轮从头到尾都按这个集子走，中途翻开关不影响本轮。 */
    g->zero.participate_mask = g->zero.participate_override;
    /* 覆盖是**一次性**的：锁进 g->zero.participate_mask 之后就地清掉(和 g->zero.request 被消费
     * 一个路子)。这样即使触发的那个上层动作被外部中途打断(比如遥控器上线，robot_state
     * 会把 auto_mode 直接清掉，那边的收尾代码就跑不到了)，也不会留下一个"以后每次设零
     * 都只设夹爪"的雷。 */
    g->zero.participate_override = 0;
    if (g->zero.participate_mask == 0) { /* 覆盖为 0 = 默认集：六个关节 */
        for (int i = 0; i < ARM_MOTOR_NUM; i++) {
            if (i <= JOINT3 && !arm_zero_encos_enable) continue; /* j1~j3 占位：只记 SKIP */
            /* 夹爪**不在默认集里**：它的"零"是行程中点(∓142)，只能靠"推到限位→退回半个
             * 行程"那个找零流程得到(见 arm_auto.c 的夹爪找零)，把当前开口设成零是错的。
             * 要设夹爪就显式写 participate_override 点名(那条路本来就这么用)。 */
            if (i == GRIPPER) continue;
            g->zero.participate_mask |= (uint8_t)(1u << i);
        }
    }
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        g->zero.result[i] = participates(g, (uint8_t)i) ? ARM_ZERO_RES_NONE : ARM_ZERO_RES_SKIP;
        g->zero.retry[i] = 0;
    }
    g->zero.start_ms = now;
    g->zero.phase_ms = now;
    g->zero.retry_ms = now;
    g->zero.index = next_participant(g, 0);
    g->zero.dm_woken = 0;
    g->zero.before_pos = 0.0f;
    g->zero.elapsed_ms = 0;
    g->zero.disable_confirmed = 0;
    g->zero.wake_mask = 0;
    g->zero.ack_missing_mask = 0;

    disable_all(g);                         /* ① 所有参与电机失能 */
    g->zero.state = ARM_ZERO_WAIT_DISABLE; /* ② 等确认 */
}

uint8_t Arm_Zero_Busy(Gimbal* g) {
    /* g 为 NULL(驱动没建起来)时算"不忙"：没有电机对象就没有帧要发，让它照常走。 */
    if (g == NULL) return 0;
    switch ((Arm_zero_state)g->zero.state) {
    case ARM_ZERO_DISABLE:
    case ARM_ZERO_WAIT_DISABLE:
    case ARM_ZERO_SEND:
    case ARM_ZERO_CHECK:
        return 1;
    default:
        return 0;
    }
}

void Arm_Zero_Loop(Gimbal* g) {
    /* g 是 CmdTask 里建的(Gimbal_Create() 依赖 CAN 已初始化，见 gimbal_arm.h)，MotorTask
     * 每拍读它。指针写是原子的，最坏情况也只是这一拍拿到 NULL —— 那样**什么都不做**：
     * 请求留着不消费，下一拍再来(状态也全在 g 里，没有 g 就没有状态可推进)。 */
    if (g == NULL) return;
    uint32_t now = HAL_GetTick();

    if (!Arm_Zero_Busy(g)) {
        /* 空闲或上一轮已结束：只有请求来了才开新一轮(所以写 1 可以重复触发) */
        if (!g->zero.request) return;
        g->zero.request = 0;
        start(g, now);
        return;
    }

    keep_monitors_alive(g);

    /* 整体兜底 */
    if ((uint32_t)(now - g->zero.start_ms) >= ARM_ZERO_TOTAL_MS) {
        finish(g, now, 1);
        return;
    }

    switch ((Arm_zero_state)g->zero.state) {
    case ARM_ZERO_WAIT_DISABLE:
#if ARM_ZERO_DISABLE_NEED_CONFIRM
        if (all_disabled(g, now)) {
            g->zero.disable_confirmed = 1;
            g->zero.index = next_participant(g, 0);
            g->zero.phase_ms = now;
            if (g->zero.index >= ARM_MOTOR_NUM) {
                finish(g, now, 0);
                break;
            }
            g->zero.state = ARM_ZERO_SEND;
        } else if ((uint32_t)(now - g->zero.phase_ms) >= ARM_ZERO_DISABLE_CONFIRM_MS) {
            /* 等不到失能确认：**不往下发设零帧**(电机可能还带着力矩)。
             * 如实报超时，g->zero.disable_confirmed 留在 0 供排查。 */
            finish(g, now, 1);
        } else if ((uint32_t)(now - g->zero.retry_ms) >= ARM_ZERO_DISABLE_RETRY_MS) {
            g->zero.retry_ms = now;
            disable_all(g); /* 没确认就再发一遍 */
        }
#else
        /* 退让路径：不判确认，发完等固定时间就走 */
        if ((uint32_t)(now - g->zero.phase_ms) >= ARM_ZERO_DISABLE_FALLBACK_MS) {
            g->zero.disable_confirmed = 1;
            g->zero.index = next_participant(g, 0);
            g->zero.phase_ms = now;
            if (g->zero.index >= ARM_MOTOR_NUM) {
                finish(g, now, 0);
                break;
            }
            g->zero.state = ARM_ZERO_SEND;
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
