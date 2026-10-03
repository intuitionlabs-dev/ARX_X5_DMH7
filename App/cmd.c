#include "cmd.h"

#include "rc.h" /* Rc_Loop()：遥控器驱动的服务，Cmd_Loop() 里带跑一遍 */

#include "gimbal_arm.h" /* gimbal / Gimbal_Update() */

void Cmd_Loop(void) {
    /* 遥控器输入先在这里服务：换算成 µs/频率、判丢信号、推逻辑层、喂看门狗。
     * 驱动本体在 libs/rc/ 里。 */
    Rc_Loop();

    /* 六轴机械臂云台。gimbal 对象在 StartCmdTask 进循环前由 Gimbal_Create()
     * 建好；创建失败(DMmotor_Create 返回 NULL)时它是 NULL，跳过。
     * 注意这里是 500 Hz / 2 ms，和源工程 ROBOT_CMD_TASK_DELAY 一致。 */
    if (gimbal != NULL) {
        Gimbal_Update(gimbal);
    }
}
