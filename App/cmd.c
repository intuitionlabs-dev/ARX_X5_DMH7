#include "cmd.h"

#include "rc.h" /* Rc_Loop()：遥控器驱动的服务，Cmd_Loop() 里带跑一遍 */

#include "gimbal_arm.h" /* gimbal / Gimbal_Update() */
#include "gimbal_cmd.h" /* gimbal_cmd / Gimbal_Cmd_Update() */

void Cmd_Loop(void) {
    /* 遥控器输入先在这里服务：换算成 µs/频率、判丢信号、推逻辑层、喂看门狗。
     * 驱动本体在 libs/rc/ 里。
     * 注意现在只是"服务"它 —— 值还没人用：遥控器→关节角的映射(输入逻辑)按用户要求
     * 先不做，见 App/gimbal_cmd.c 的 Remote_Mode_Update()。 */
    Rc_Loop();

    /* gimbal / gimbal_cmd 都在 StartCmdTask 进循环前建好；建失败(内存不够)时是 NULL */
    if (gimbal == NULL) return;

    /* 命令层：输入 → 控制源 → 软限位 → TD 平滑 → gimbal->arm_control_data。
     * 现在输入侧是空的(恒 arm_stop、目标跟随反馈)，但这条管线每拍都在跑。 */
    Gimbal_Cmd_Update(gimbal_cmd);

    /* 六轴机械臂云台。gimbal 对象在 StartCmdTask 里由 Gimbal_Create() 建好。
     *
     * **顺序不能反**：先 Gimbal_Cmd_Update()(里面把命令喂进去) 再 Gimbal_Update()(执行)。
     * 反过来的话命令要多延迟一拍。这里读到电机反馈天然是上一拍(2 ms 前)的，和源工程
     * 走一趟 pub_sub 的延迟是同一个量级，不是 bug。
     * 这里是 500 Hz / 2 ms，和源工程 ROBOT_CMD_TASK_DELAY 一致。 */
    Gimbal_Update(gimbal);
}
