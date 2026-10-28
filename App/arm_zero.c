#include "arm_zero.h"
#include "gimbal_arm.h"
#include "main.h"
#include <math.h>

/* Metal 六关节顺序设零；显式 participate_override 可选择参与电机。
 * 失能、等待反馈、必要时唤醒复核的状态推进由 MotorTask 执行。 */
#define ARM_ZERO_DISABLE_CONFIRM_MS 1500u
#define ARM_ZERO_DISABLE_RETRY_MS 100u
#define ARM_ZERO_DISABLE_SILENT_MS 300u
#define ARM_ZERO_DISABLE_NEED_CONFIRM 1
#define ARM_ZERO_DISABLE_FALLBACK_MS 100u
#define ARM_ZERO_DM_VERIFY_DELAY_MS 20u
#define ARM_ZERO_DM_WAKE_MS 100u
#define ARM_ZERO_DM_PER_MOTOR_MS 1000u
#define ARM_ZERO_POS_TOL_RAD 0.01f
#define ARM_ZERO_TOTAL_MS 15000u

/* ---------- 电机取用 ---------- */
/* 7 台电机都挂在 Gimbal 上(见 gimbal_arm.h)，用索引拿。拿不到(创建失败)返回 NULL。 */
static DM_motor* dm_at(Gimbal* g, uint8_t i) {
    switch (i) {
    case JOINT1: return g->joint1;
    case JOINT2: return g->joint2;
    case JOINT3: return g->joint3;
    case JOINT4: return g->joint4;
    case JOINT5: return g->joint5;
    case JOINT6: return g->joint6;
    case GRIPPER: return g->gripper;
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
    }
}

/* 失能确认(达妙)。判据见文件头第 2 条：errcode==0，或者已经安静够久。 */
static uint8_t dm_disabled_confirmed(DM_motor* dm, uint32_t now) {
    if (dm->errcode == dm_err_disable) return 1;
    return (uint32_t)(now - (uint32_t)dm->motor_fps.now) >= ARM_ZERO_DISABLE_SILENT_MS;
}

/* 按参与电机逐台确认失能。 */
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
    if (fabsf(Gimbal_Motor_Position_Rad(dm)) <= ARM_ZERO_POS_TOL_RAD) {
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

/* 给当前电机发设零帧，并锁存设零前的输出轴位置。 */
static void send_current(Gimbal* g, uint32_t now) {
    DM_motor* dm = dm_at(g, g->zero.index);
    if (dm == NULL) {
        g->zero.result[g->zero.index] = ARM_ZERO_RES_FAIL;
        advance(g, now);
        return;
    }
    g->zero.before_pos = Gimbal_Motor_Position_Rad(dm);
    g->zero.dm_woken = 0;
    DMmotor_Save_Zero(dm);
    g->zero.send_ms = now;
    g->zero.state = ARM_ZERO_CHECK;
}

/* ③ 检查当前这一台成没成；不成就在单台窗口内反复重发 */
static void check_current(Gimbal* g, uint32_t now) {
    DM_motor* dm = dm_at(g, g->zero.index);
    if (dm == NULL) {
        g->zero.result[g->zero.index] = ARM_ZERO_RES_FAIL;
        advance(g, now);
        return;
    }
    if ((uint32_t)(now - g->zero.phase_ms) >= ARM_ZERO_DM_PER_MOTOR_MS) {
        if (g->zero.result[g->zero.index] != ARM_ZERO_RES_FAIL)
            g->zero.result[g->zero.index] = ARM_ZERO_RES_TIMEOUT;
        advance(g, now);
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
    /* 本轮锁存参与集；默认六关节，显式覆盖可选择其他电机。 */
    g->zero.participate_mask = g->zero.participate_override;
    /* 覆盖是**一次性**的：锁进 g->zero.participate_mask 之后就地清掉(和 g->zero.request 被消费
     * 一个路子)。这样即使触发的那个上层动作被外部中途打断(比如遥控器上线，robot_state
     * 会把 auto_mode 直接清掉，那边的收尾代码就跑不到了)，也不会留下一个"以后每次设零
     * 都只设夹爪"的雷。 */
    g->zero.participate_override = 0;
    if (g->zero.participate_mask == 0) { /* 覆盖为 0 = 默认集：六个关节 */
        for (int i = 0; i < ARM_MOTOR_NUM; i++) {
            /* 默认保留夹爪厂家零点；显式设零需用 participate_override 选择夹爪。 */
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
