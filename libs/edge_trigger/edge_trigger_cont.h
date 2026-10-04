#pragma once

#include <stdint.h>
// clang-format off
typedef enum {
    cont_pos = 0,// 上升沿触发
    cont_neg // 下降沿触发
} Edge_continuous_type;
// clang-format on

// 带有滞回特性的连续量边沿触发器
typedef struct {
    int16_t value;  // 连续量
    uint8_t flag;  // 触发标志位,触发后置1,读取或反向触发后清0，与triggered无关
    uint8_t triggered; // 触发状态位,触发后置1,反向触发后清0，与flag无关
    Edge_continuous_type type;
    int16_t threshold_high; // 上阈值
    int16_t threshold_low;// 下阈值
} Edge_trigger_continuous;

Edge_trigger_continuous* Edge_Trigger_Continuous_Create(Edge_continuous_type type,
    int16_t threshold_low, int16_t threshold_high);
void Etc_Update(Edge_trigger_continuous* obj, int16_t new_value);
uint8_t Etc_Peek(Edge_trigger_continuous* obj);
uint8_t Etc_Get(Edge_trigger_continuous* obj);