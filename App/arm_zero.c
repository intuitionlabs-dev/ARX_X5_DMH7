#include "arm_zero.h"
#include "gimbal_arm.h"
#include "main.h" // IWYU pragma: keep
#include <string.h>

/* 判断设零请求是否待处理或正在执行。 */
uint8_t Arm_Zero_Busy(Gimbal* g) {
    return g && (g->zero.request || (g->zero.state >= ARM_ZERO_DISABLE
        && g->zero.state <= ARM_ZERO_CHECK));
}

/* 向该臂全部电机发送失能命令。 */
static void zero_disable_all(Gimbal* g) {
    /* 逐电机调用驱动失能接口。 */
    for (int i = 0; i < ARM_MOTOR_NUM; ++i)
        g->motors[i].config->driver->disable(&g->motors[i]);
}

/* 汇总设零结果、总耗时及最终状态。 */
static void zero_finish(Gimbal* g, uint32_t now, uint8_t failed) {
    zero_disable_all(g);
    /* 检查所有参与电机的最终结果。 */
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        /* 将尚未完成的参与电机标记为超时。 */
        if (g->zero.participating[i] && g->zero.result[i] == ARM_ZERO_RES_NONE)
            g->zero.result[i] = ARM_ZERO_RES_TIMEOUT;
        /* 任一参与电机结果异常时汇总为失败。 */
        if (g->zero.participating[i] && g->zero.result[i] != ARM_ZERO_RES_OK
            && g->zero.result[i] != ARM_ZERO_RES_UNCONFIRMED) failed = 1;
    }
    g->zero.elapsed_ms = now-g->zero.start_ms;
    g->zero.state = failed ? ARM_ZERO_TIMEOUT : ARM_ZERO_DONE;
}

/* 跳过未参与电机并推进设零步骤。 */
static void zero_advance(Gimbal* g, uint32_t now) {
    /* 查找下一个参与设零的电机。 */
    while (g->zero.index < ARM_MOTOR_NUM && !g->zero.participating[g->zero.index])
        ++g->zero.index;
    g->zero.phase_ms = now;
    /* 全部电机处理结束后汇总结果。 */
    if (g->zero.index == ARM_MOTOR_NUM) zero_finish(g, now, 0);
    /* 仍有参与电机时转入命令发送步骤。 */
    else g->zero.state = ARM_ZERO_SEND;
}

/* 在电机任务中非阻塞推进设零流程。 */
void Arm_Zero_Loop(Gimbal* g) {
    /* 控制对象无效时结束本次更新。 */
    if (!g) return;
    const uint32_t now = HAL_GetTick();
    /* 空闲时接收请求并初始化本轮设零状态。 */
    if (g->zero.state < ARM_ZERO_DISABLE || g->zero.state > ARM_ZERO_CHECK) {
        /* 没有设零请求时保持空闲。 */
        if (!g->zero.request) return;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        g->zero.request = 0;
        /* 根据选择表生成参与标志和初始结果。 */
        for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
            g->zero.participating[i] = g->zero.selection_valid ? g->zero.selection[i] : i != GRIPPER;
            g->zero.result[i] = g->zero.participating[i] ? ARM_ZERO_RES_NONE : ARM_ZERO_RES_SKIP;
        }
        g->zero.selection_valid = 0;
        memset(g->zero.selection, 0, sizeof(g->zero.selection));
        g->zero.start_ms = g->zero.phase_ms = g->zero.retry_ms = now;
        g->zero.index = 0;
        g->zero.disable_confirmed = 0;
        g->zero.elapsed_ms = 0;
        zero_disable_all(g);
        g->zero.state = ARM_ZERO_WAIT_DISABLE;
        return;
    }
    /* 设零期间维持电机在线监控计数。 */
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) g->motors[i].monitor->reset(g->motors[i].monitor);
    /* 总耗时超过配置时限后结束设零。 */
    if (now-g->zero.start_ms >= g->profile->init.zero_total_ms) {
        zero_finish(g, now, 1);
        return;
    }
    /* 等待全部电机失能确认。 */
    if (g->zero.state == ARM_ZERO_WAIT_DISABLE) {
        uint8_t stopped = 1;
        /* 逐电机检查停止状态。 */
        for (int i = 0; i < ARM_MOTOR_NUM; ++i)
            /* 任一电机仍在运行时继续等待。 */
            if (!g->motors[i].config->driver->stopped(&g->motors[i], now)) stopped = 0;
        /* 全部失能确认后进入逐电机设零。 */
        if (stopped) {
            g->zero.disable_confirmed = 1;
            zero_advance(g, now);
        } else if (now-g->zero.phase_ms >= g->profile->init.zero_disable_timeout_ms) zero_finish(g, now, 1); /* 失能等待超过配置时限时结束设零。 */
        /* 到达重试间隔后再次发送失能命令。 */
        else if (now-g->zero.retry_ms >= g->profile->init.zero_disable_retry_ms) {
            g->zero.retry_ms = now;
            zero_disable_all(g);
        }
        return;
    }
    Arm_motor* motor = &g->motors[g->zero.index];
    int result = 0;
    /* 调用当前电机的设零启动接口。 */
    if (g->zero.state == ARM_ZERO_SEND) {
        result = motor->config->driver->zero_start(motor, now);
        /* 启动成功后等待驱动复核。 */
        if (result > 0) {
            g->zero.state = ARM_ZERO_CHECK;
            return;
        }
    } else { /* 轮询当前电机的设零结果。 */
        result = motor->config->driver->zero_poll(motor, now);
        /* 记录成功结果并转向下一电机。 */
        if (result > 0) {
            g->zero.result[g->zero.index] = result == 1 ? ARM_ZERO_RES_OK : ARM_ZERO_RES_UNCONFIRMED;
            ++g->zero.index;
            zero_advance(g, now);
            return;
        }
    }
    /* 驱动失败或单电机超时后记录结果并继续。 */
    if (result < 0 || now-g->zero.phase_ms >= g->profile->init.zero_motor_ms) {
        motor->config->driver->disable(motor);
        g->zero.result[g->zero.index] = result < 0 ? ARM_ZERO_RES_FAIL : ARM_ZERO_RES_TIMEOUT;
        ++g->zero.index;
        zero_advance(g, now);
    }
}
