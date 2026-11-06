#include "cmd.h"

#include "rc.h"

#include "arm.h"
#include "gimbal_arm.h"
#include "gimbal_cmd.h"
#include "robot_state.h"

/* 更新输入与状态机后依次执行各臂控制。 */
void Cmd_Loop(void) {

    /* 更新遥控器通道和开关状态。 */
    Rc_Loop();

    /* 消费上位机报文并推进通信状态。 */
    Robot_State_Loop();

    /* 逐臂生成控制参考值并更新电机目标。 */
    for (uint8_t i = 0; i < (uint8_t)ARM_NUM; i++) {
        Arm* arm = &g_arm[i];
        /* 跳过控制对象尚未创建的机械臂。 */
        if (arm->gimbal == NULL || arm->gimbal_cmd == NULL) continue;

        /* 由命令层生成本周期关节与夹爪目标。 */
        Gimbal_Cmd_Update(arm->gimbal_cmd);

        /* 由执行层换算目标并发布电机参考值。 */
        Gimbal_Update(arm->gimbal);
    }
}
