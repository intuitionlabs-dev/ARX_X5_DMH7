/* Frozen baseline from rebuild 3317d9b9dbe7afa5f0b0b182fe44cf94a8752ec7:
 * App/gimbal_cmd.c Joint_Quintic_Ref_Update and Arm_Ref_Set.
 * Adaptations: separate refs array, fixed original 0.16 s profile duration,
 * renamed entry point, inline original offline predicate. Algorithm unchanged.
 * Test-only: never linked into normal firmware. */
typedef struct {
    Quintic_spline curve;
    float target;
    float elapsed;
    float position;
    float velocity;
    float acc;
    uint8_t initialized;
    uint8_t active;
} Joint_quintic_ref;

static void Joint_Quintic_Ref_Update(Gimbal_cmd* obj, Joint_quintic_ref* refs, int joint) { // NOLINT(readability-identifier-naming)
    Joint_quintic_ref* ref = &refs[joint];
    const float target = obj->ctrl_src.joint_angle_solved[joint];
    /* 首次采样以当前反馈建立参考初值。 */
    if (!ref->initialized) {
        ref->position = obj->gimbal->arm_feedback_data.joint_angle[joint];
        ref->velocity = 0.0f;
        ref->acc = 0.0f;
        ref->initialized = 1;
    }

    /* 目标变化或轨迹未激活时重建五次曲线。 */
    if (!ref->active || target != ref->target) {
        /* 以当前参考导数和新目标构造曲线边界。 */
        Quintic_spline_config config = {
            .x0 = 0.0f,
            .y0 = ref->position,
            .dy0 = ref->velocity,
            .ddy0 = ref->acc,
            .x1 = 0.16f,
            .y1 = target,
            .dy1 = 0.0f,
            .ddy1 = 0.0f,
            .xmin = 0.0f,
            .xmax = 0.16f,
        };
        Quintic_Spline_Reinit(&ref->curve, &config);
        ref->target = target;
        ref->elapsed = 0.0f;
        ref->active = 1;
    }

    /* 按命令控制周期推进曲线时间。 */
    ref->elapsed += GIMBAL_CMD_PERIOD_S;
    /* 到达轨迹终点后保持目标并清零导数。 */
    if (ref->elapsed >= 0.16f) {
        ref->elapsed = 0.16f;
        ref->position = ref->target;
        ref->velocity = 0.0f;
        ref->acc = 0.0f;
    } else { /* 在轨迹区间内计算位置及一、二阶导数。 */
        ref->position = Quintic_Spline_Calc_y(&ref->curve, ref->elapsed);
        ref->velocity = Quintic_Spline_Calc_dy(&ref->curve, ref->elapsed);
        ref->acc = Quintic_Spline_Calc_ddy(&ref->curve, ref->elapsed);
    }
}

/* 按控制来源生成位置、速度和加速度参考。 */
static void Quintic_Reference(Gimbal_cmd* obj, Joint_quintic_ref* refs) {
    Arm_control_data* control = &obj->gimbal->arm_control_data;
    const Robot_ctrl_cmd* cmd = Robot_State_Ctrl(obj->arm);
    const uint8_t use_external_velocity =
        (obj->mode == arm_run && is_Offline(rc.monitor) && cmd != NULL && cmd->valid && cmd->has_vel);

    /* 逐轴选择参考来源并发布控制输出。 */
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        float angle;
        float velocity;
        float acc;

        /* 自动规划运行时使用规划位置和导数。 */
        if (obj->plan_running) {
            angle = obj->ctrl_src.joint_angle_solved[i];
            velocity = obj->plan_joint_vel[i] * obj->gimbal->profile->joints[i].plan_velocity_scale;
            acc = obj->plan_joint_acc[i];
        } else if (use_external_velocity) { /* 带速度协议使用跟踪位置和协议速度。 */
            angle = obj->joint_angle_td[i].v1;
            velocity = cmd->joint_vel[i];
            acc = 0.0f;
        } else if (obj->mode == arm_run) { /* 普通位置控制使用连续曲线参考。 */
            Joint_Quintic_Ref_Update(obj, refs, i);
            angle = refs[i].position;
            velocity = refs[i].velocity;
            acc = refs[i].acc;
        } else { /* 被动和停止模式使用跟踪器输出。 */
            angle = obj->joint_angle_td[i].v1;
            velocity = obj->joint_angle_td[i].v2;
            acc = obj->joint_angle_td[i].v3;
        }

        /* 被动和停止模式清零参考加速度。 */
        if (obj->mode != arm_run) acc = 0.0f;
        /* 参考值异常时切入阻尼并清除轨迹状态。 */
        if (!isfinite(angle) || !isfinite(velocity) || !isfinite(acc)) {
            obj->mode = arm_damping;
            control->mode = obj->mode;
            /* 逐轴恢复反馈位置并清零运动参考。 */
            for (int j = 0; j < ARM_JOINT_NUM; j++) {
                control->joint_angle[j] = obj->gimbal->arm_feedback_data.joint_angle[j];
                control->joint_vel[j] = 0.0f;
                control->joint_acc[j] = 0.0f;
                refs[j].initialized = 0;
                refs[j].active = 0;
            }
            return;
        }

        /* 将本轴选定的位置、速度和加速度写入执行目标。 */
        control->joint_angle[i] = angle;
        control->joint_vel[i] = velocity;
        control->joint_acc[i] = acc;
        /* 同步其他控制来源的状态以衔接位置轨迹。 */
        if (obj->mode != arm_run || obj->plan_running || use_external_velocity) {
            Joint_quintic_ref* ref = &refs[i];
            /* 被动和停止模式以反馈重置参考状态。 */
            if (obj->mode != arm_run) {
                ref->position = obj->gimbal->arm_feedback_data.joint_angle[i];
                ref->velocity = 0.0f;
                ref->acc = 0.0f;
            } else { /* 运行模式以实际发布参考建立衔接状态。 */
                ref->position = angle;
                ref->velocity = velocity;
                ref->acc = acc;
            }
            ref->initialized = 1;
            ref->active = 0;
        }
    }
}

