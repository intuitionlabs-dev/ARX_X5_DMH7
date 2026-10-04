#pragma once
// #include <stdbool.h>
#include <stdint.h>

// clang-format off
typedef enum {
    bin_pos = 0,// 上升沿触发
    bin_neg // 下降沿触发
} Edge_binary_type;
// clang-format on

// 二值边沿触发器
typedef struct {
    uint8_t value;  // 二值量: 0或1
    uint8_t flag;  // 触发标志位,触发后置1,读取或反向触发后清0，与triggered无关
    uint8_t triggered;  // 触发状态位,触发后置1,反向触发后清0，与flag无关
    uint32_t refactory_ticks_ms;  // 不应期，在每次触发后的refactory_ticks_ms时间内不刷新flag,即使该时间内触发过也不会刷新flag。与triggered无关
    uint32_t last_trigger_ticks_ms;  // 上次触发时间戳，单位ms
    Edge_binary_type type;
} Edge_trigger_binary;

Edge_trigger_binary* Edge_Trigger_Binary_Create(Edge_binary_type type, uint32_t refactory_ticks_ms);
void Etb_Update(Edge_trigger_binary* obj, uint8_t new_value);
uint8_t Etb_Peek(Edge_trigger_binary* obj);
uint8_t Etb_Get(Edge_trigger_binary* obj);