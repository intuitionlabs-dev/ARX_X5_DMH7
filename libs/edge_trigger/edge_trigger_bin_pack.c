#include "edge_trigger_bin_pack.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_time.h"
#include "debug/debug_log.h"

Edge_trigger_binary_pack* Edge_Trigger_Binary_Pack_Create(Edge_binary_type type, uint32_t refactory_ticks_ms, uint16_t units_cnt, uint8_t init_value) {
    if (type != bin_pos && type != bin_neg) {
        printf_log("%s: Invalid type\n", __func__);
        return NULL;
    }
    if (units_cnt == 0) {
        printf_log("%s: Invalid units_cnt %d\n", __func__, units_cnt);
        return NULL;
    }

    Edge_trigger_binary_pack* obj =
        (Edge_trigger_binary_pack*)RT_MALLOC(sizeof(Edge_trigger_binary_pack));
    memset(obj, 0, sizeof(Edge_trigger_binary_pack));

    obj->units =
        (Edge_trigger_binary_pack_unit*)RT_MALLOC(sizeof(Edge_trigger_binary_pack_unit)
            * units_cnt);

    obj->type = type;
    obj->refactory_ticks_ms = refactory_ticks_ms;
    obj->units_cnt = units_cnt;

    uint8_t init_triggered = 0;
    const uint32_t current_ticks_ms = BSP_sys_time_ms();
    // 根据初始值和类型确定初始触发状态位
    switch (type) {
    case bin_pos:
        init_triggered = init_value;
        break;
    case bin_neg:
        init_triggered = !init_value;
        break;
    }

    Edge_trigger_binary_pack_unit temp_unit = {
        .value = init_value,
        .flag = 0,  //无论初始值，初始时不触发
        .triggered = init_triggered,
        .last_trigger_ticks_ms = current_ticks_ms,
    };
    for (uint16_t i = 0; i < units_cnt; i++) {
        obj->units[i] = temp_unit;
    }

    return obj;
}

void Etbp_Update(Edge_trigger_binary_pack* obj, uint16_t index, uint8_t new_value) {
    // 不做越界检查，便于debug

    obj->units[index].value = new_value;
    const uint32_t current_ticks_ms = BSP_sys_time_ms();
    switch (obj->type) {
    case bin_pos:
        if (obj->units[index].value == 0)  //只在值0时清除flag，保证flag在未被取走且值为1时保持有效
            obj->units[index].flag = 0;
        else if (obj->units[index].triggered == 0 && obj->units[index].value == 1
            && (current_ticks_ms - obj->units[index].last_trigger_ticks_ms)
                > obj->refactory_ticks_ms) {
            obj->units[index].flag = 1;
            obj->units[index].last_trigger_ticks_ms = current_ticks_ms;  //记录触发时间
        }

        obj->units[index].triggered = obj->units[index].value;

        break;
    case bin_neg:
        if (obj->units[index].value == 1)  //只在值非0时清除flag，保证flag在未被取走且值为0时保持有效
            obj->units[index].flag = 0;
        else if (obj->units[index].triggered == 0 && obj->units[index].value == 0
            && (current_ticks_ms - obj->units[index].last_trigger_ticks_ms)
                > obj->refactory_ticks_ms) {
            obj->units[index].flag = 1;
            obj->units[index].last_trigger_ticks_ms = current_ticks_ms;  //记录触发时间
        }

        obj->units[index].triggered = !(obj->units[index].value);

        break;
    }
}

/**
 * @brief 查看触发状态位，但不获取触发标志位
 * 
 * @param obj 
 * @param index 触发器索引
 * @return uint8_t 0或1,表示是否触发
 */
uint8_t Etbp_Peek(Edge_trigger_binary_pack* obj, uint16_t index) {
    // 不做越界检查，便于debug
    return obj->units[index].triggered;
}

/**
 * @brief 获取并清除触发标志位
 * @note 意义是避免在更新频率低于检查标志位频率时多次执行变化沿逻辑
 * @param obj 
 * @param index 触发器索引
 * @return uint8_t 0或1,表示是否获取到触发标志位
 */
uint8_t Etbp_Get(Edge_trigger_binary_pack* obj, uint16_t index) {
    // 不做越界检查，便于debug
    uint8_t temp = obj->units[index].flag;
    obj->units[index].flag = 0;
    return temp;
}