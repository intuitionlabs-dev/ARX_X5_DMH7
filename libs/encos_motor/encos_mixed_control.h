#pragma once

#include "ENCOS_motor.h"

/* 提供 ENCOS 混合控制回调，将控制器参考值换算为协议单位。 */
void ENCOS_Mixed_Control(ENCOS_motor* motor);
