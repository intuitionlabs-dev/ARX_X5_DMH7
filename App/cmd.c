#include "cmd.h"

#include "rc.h" /* Rc_Loop()：遥控器驱动的服务，Cmd_Loop() 里带跑一遍 */

#include "arm.h"           /* g_arm[] —— 两台臂(每台一个驱动 + 一个命令层) */
#include "gimbal_arm.h"    /* Gimbal_Update() */
#include "gimbal_cmd.h"    /* Gimbal_Cmd_Update() */
#include "robot_state.h"   /* Robot_State_Loop()：串口总模式状态机 */

void Cmd_Loop(void) {
    /* 遥控器输入先在这里服务：换算成 µs/频率、判丢信号、推逻辑层、喂看门狗。
     * 驱动本体在 libs/rc/ 里；rc 挂在 cmd 下(见 App/inc/cmd.h 的转发 include)，
     * 要读遥控器直接访问全局 rc 即可。
     * 注意现在只是"服务"它 —— 值还没人用：遥控器→关节角的映射(输入逻辑)按用户要求
     * 先不做，见 App/gimbal_cmd.c 的 Remote_Mode_Update()。 */
    Rc_Loop();

    /* 串口总模式状态机(App/robot_state.c)：收上位机的帧、跑状态迁移、发 250 Hz 反馈。
     * 放在 gimbal == NULL 判断**之前** —— 和 Rc_Loop() 一个道理：它是"输入服务"，
     * 而后面 Gimbal_Cmd_Update() 每拍都要读它给的模式(Robot_State_Arm_Mode())，
     * 遥控器在线时读它给的 rc_online 判据。gimbal 没建起来(内存不够)时它也照跑：
     * 握手、状态机、反馈帧都不依赖电机，方便单独排查串口链路。 */
    Robot_State_Loop();

    /* 逐台跑。两台臂的 gimbal / gimbal_cmd 都在 StartCmdTask 进循环前建好；
     * 建失败(内存不够)时是 NULL。**只跑建起来了的那些**：没有驱动就没有什么可执行，
     * 而且 Gimbal_Cmd_Update() 会顺着 arm->gimbal 去读反馈。
     *
     * 两台之间**没有任何共享状态**：模式各读各的(obj->arm->arm_mode)、控制帧各读各的
     * (Robot_State_Ctrl(obj->arm) 是按 arm_id 分流的)、遥控器是同一个物理信号(设计如此，
     * 两台同动)。所以这个循环里不需要任何顺序或互斥。 */
    for (uint8_t i = 0; i < (uint8_t)ARM_NUM; i++) {
        Arm* arm = &g_arm[i];
        if (arm->gimbal == NULL || arm->gimbal_cmd == NULL) continue;

        /* 命令层：输入 → 控制源 → 软限位 → TD 平滑 → gimbal->arm_control_data。
         * 现在输入侧是空的(恒 arm_stop、目标跟随反馈)，但这条管线每拍都在跑。 */
        Gimbal_Cmd_Update(arm->gimbal_cmd);

        /* 六轴机械臂云台。gimbal 对象在 StartCmdTask 里由 Gimbal_Create() 建好。
         *
         * **顺序不能反**：先 Gimbal_Cmd_Update()(里面把命令喂进去) 再 Gimbal_Update()(执行)。
         * 反过来的话命令要多延迟一拍。这里读到电机反馈天然是上一拍(2 ms 前)的，和源工程
         * 走一趟 pub_sub 的延迟是同一个量级，不是 bug。
         * 这里是 500 Hz / 2 ms，和源工程 ROBOT_CMD_TASK_DELAY 一致。 */
        Gimbal_Update(arm->gimbal);
    }
}
