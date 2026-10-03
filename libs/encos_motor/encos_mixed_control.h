#pragma once

#include "ENCOS_motor.h"

/* ENCOS 电机"力位混合控制"的控制函数，给 ENCOSmotor_Create() 当第二个参数用。
 *
 * 对标 libs/dm_motor/dm_mit_control.c 的 DM_Mit_Control()，两者只差一处：达妙那边要把
 * controller 算出来的角度减掉 round*2*max_position 折回单圈，ENCOS 这边不减 —— 驱动
 * 根本不记圈数(ENCOSmotor_Reset_Round() 显式返回 ENCOS_UNSUPPORTED，README 也写明不做
 * 2π 回绕)，而它对标的 ARX X5 MotorType4 位置量程是 ±12.5 rad，本臂 j1~j3 的关节范围
 * 换算过去分别是 ±3.14/2.618、[-2.485,-2.42]、[2.615,2.669] rad，都在单圈内。
 *
 * **不能给 ENCOSmotor_Create() 传 NULL**：驱动的默认分支只写 ref_torque，不会把
 * controller 的 ref_position/ref_speed 转成电机目标值(那是"外部自己喂目标"的用法)，
 * 还会把 output_comp 前馈丢掉。 */
void ENCOS_Mixed_Control(ENCOS_motor* motor);
