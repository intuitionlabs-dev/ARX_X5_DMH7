/**
 * @file edge_trigger_bin_pack.h
 * @author 赵苏亚
 * @brief 二值边沿触发器组，针对同配置多触发器情景下的内存占用和效率进行优化，但逻辑与单个二值边沿触发器相同
 * @version 0.1
 * @date 2026-03-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include "edge_trigger_bin.h"
#pragma pack(1)
// 二值边沿触发器组的单元结构体
typedef struct {
    uint8_t value : 1;      // 二值量: 0或1
    uint8_t flag : 1;       // 触发标志位,触发后置1,读取或反向触发后清0，与triggered无关
    uint8_t triggered : 1;  // 触发状态位,触发后置1,反向触发后清0，与flag无关
    uint8_t _reserved : 5;           // 保留位，凑齐一个字节，避免对齐问题
    uint32_t last_trigger_ticks_ms;  // 上次触发时间戳，单位ms
} Edge_trigger_binary_pack_unit;
#pragma pack()

// 二值边沿触发器组
typedef struct {
    uint32_t refactory_ticks_ms;  // 不应期，在每次触发后的refactory_ticks_ms时间内不刷新flag,即使该时间内触发过也不会刷新flag。与triggered无关
    Edge_binary_type type;
    uint16_t units_cnt;  // 触发器数量，最大支持65535
    Edge_trigger_binary_pack_unit* units;  // 触发器单元数组，实际长度为units_cnt。由于不会动态变化且为了节省内存占用和提高效率，故不使用cvector
} Edge_trigger_binary_pack;

Edge_trigger_binary_pack* Edge_Trigger_Binary_Pack_Create(Edge_binary_type type, uint32_t refactory_ticks_ms, uint16_t units_cnt, uint8_t init_value);
void Etbp_Update(Edge_trigger_binary_pack* obj,uint16_t index, uint8_t new_value);
uint8_t Etbp_Peek(Edge_trigger_binary_pack* obj,uint16_t index);
uint8_t Etbp_Get(Edge_trigger_binary_pack* obj,uint16_t index);