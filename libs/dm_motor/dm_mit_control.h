#pragma once

#include "DM_motor.h"

/* 提供 DM MIT 控制回调，级联控制输出与力矩前馈共同生成目标。 */
void DM_Mit_Control(struct dm_motor_t* motor);
