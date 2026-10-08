#include "arm.h"

/* 两台臂的存储本体。**静态分配、开机即存在**(理由见 App/inc/arm.h 的文件头)：
 * 收包回调可能在 CmdTask 还没跑到 Gimbal_Create() 时就打进来，那时收包环必须已经在了；
 * 用数组而不是"指针 + 判空"就是想消掉这个中间状态。
 *
 * 这里是**零初始化**：gimbal / gimbal_cmd 都是 NULL，arm_mode 是 arm_stop(=0)。
 * 建对象由 Core/Src/freertos.c 的 StartCmdTask 负责(要在任务里建，栈的原因)。 */
Arm g_arm[ARM_NUM];
