#include "edge_trigger_bin.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_time.h"
#include "debug/debug_log.h"

Edge_trigger_binary* Edge_Trigger_Binary_Create(Edge_binary_type type,
    uint32_t refactory_ticks_ms) {
    if (type != bin_pos && type != bin_neg) {
        printf_log("%s: Invalid type\n", __func__);
        return NULL;
    }

    Edge_trigger_binary* obj = (Edge_trigger_binary*)RT_MALLOC(sizeof(Edge_trigger_binary));
    memset(obj, 0, sizeof(Edge_trigger_binary));

    obj->type = type;
    obj->refactory_ticks_ms = refactory_ticks_ms;
    obj->last_trigger_ticks_ms = BSP_sys_time_ms();
    switch (type) {
    case bin_pos:
        obj->value = 0;
        break;
    case bin_neg:
        obj->value = 1;
        break;
    }
    obj->flag = 0;

    return obj;
}

void Etb_Update(Edge_trigger_binary* obj, uint8_t new_value) {
    obj->value = new_value;

    switch (obj->type) {
    case bin_pos:
        if (obj->triggered == 0 && obj->value != 0
            && (BSP_sys_time_ms() - obj->last_trigger_ticks_ms) > obj->refactory_ticks_ms) {
            obj->flag = 1;
            obj->last_trigger_ticks_ms = BSP_sys_time_ms();  /* 记录触发时间 */
        } else if (obj->value == 0)  /* 只在值0时清除flag，保证flag在未被取走且值为1时保持有效 */
            obj->flag = 0;

        if (obj->value != 0)
            obj->triggered = 1;
        else
            obj->triggered = 0;

        break;
    case bin_neg:
        if (obj->triggered == 0 && obj->value == 0
            && (BSP_sys_time_ms() - obj->last_trigger_ticks_ms) > obj->refactory_ticks_ms) {
            obj->flag = 1;
            obj->last_trigger_ticks_ms = BSP_sys_time_ms();  /* 记录触发时间 */
        } else if (obj->value != 0)  /* 只在值非0时清除flag，保证flag在未被取走且值为0时保持有效 */
            obj->flag = 0;

        if (obj->value == 0)
            obj->triggered = 1;
        else
            obj->triggered = 0;

        break;
    }
}
/* 读取持续触发状态。 */
uint8_t Etb_Peek(Edge_trigger_binary* obj) { return obj->triggered; }
/* 读取并清除一次性边沿标志。 */
uint8_t Etb_Get(Edge_trigger_binary* obj) {
    uint8_t temp = obj->flag;
    obj->flag = 0;
    return temp;
}
