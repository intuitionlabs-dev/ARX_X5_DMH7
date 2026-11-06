#define main joint_reference_tests
#include "../../../tests/c_ruckig_joint_control.c"
#undef main

/* 检查串联关节的重力矩和无效输入保护。 */
static void dynamics_test(void) {
    Arm_link links[ARM_JOINT_NUM];
    Arm_dynamics_work work[ARM_JOINT_NUM];
    float q[ARM_JOINT_NUM] = {0}, zero[ARM_JOINT_NUM] = {0};
    float torque[ARM_JOINT_NUM];
    for (int i=0; i<ARM_JOINT_NUM; ++i)
        links[i] = (Arm_link){.mass=1, .com={.5f,0,0}, .origin={1,0,0},
            .axis={0,1,0}, .fixed_rotation={{{1,0,0},{0,1,0},{0,0,1}}}};
    const Arm_dynamics_model model = {.joint_count=ARM_JOINT_NUM, .links=links};
    assert(Arm_Dynamics_Compute(&model,q,zero,zero,NULL,1,torque,work,ARM_JOINT_NUM) == 0);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        const float length = ARM_JOINT_NUM-i;
        assert(fabsf(torque[i]+4.905f*length*length) < .005f);
    }
    q[0] = NAN;
    torque[0] = 123;
    assert(Arm_Dynamics_Compute(&model,q,zero,zero,NULL,1,torque,work,ARM_JOINT_NUM) < 0);
    assert(torque[0] == 123);
}

int main(void) {
    joint_reference_tests();
    dynamics_test();
    puts("关节轨迹、控制源切换、重力矩和异常保护验证通过");
    return 0;
}
