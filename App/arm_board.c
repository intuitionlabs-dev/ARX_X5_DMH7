#include "arm_profile.h"

/* 未指定 A 臂机型时使用 metalarm。 */
#ifndef ARM_A_PROFILE
#define ARM_A_PROFILE ARM_PROFILE_METAL
#endif
/* 未指定 B 臂机型时使用 metalarm。 */
#ifndef ARM_B_PROFILE
#define ARM_B_PROFILE ARM_PROFILE_METAL
#endif

/* 将机械臂索引绑定到机型配置和 CAN 总线。 */
const Arm_binding ARM_BINDINGS[ARM_NUM] = {
    /* A 臂使用 CAN 总线索引 0。 */
    [ARM_A] = {&ARM_A_PROFILE, 0},
    /* B 臂使用 CAN 总线索引 1。 */
    [ARM_B] = {&ARM_B_PROFILE, 1},
};
