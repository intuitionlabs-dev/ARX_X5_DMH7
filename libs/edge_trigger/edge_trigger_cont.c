#include "edge_trigger_cont.h"
#include <stdlib.h>
#include <string.h>
/* #include "debug/debug_log.h" */ /* 源工程这个头只为了 printf_log()，本工程没有这个
                                      * 模块；而且本工程的 printf 本身是坏的
                                      * (__io_putchar 没定义，调了会跳空指针，见
                                      * libs/move_plan 里同样处理)。下面两处非法参数
                                      * 改成静默 return NULL —— 参数都是编译期常量，
                                      * 真写错了在 Rc_Triggers_Create() 那边一眼能看出来
                                      * (6 个指针会全是 NULL)。 */

Edge_trigger_continuous* Edge_Trigger_Continuous_Create(Edge_continuous_type type,
    int16_t threshold_low, int16_t threshold_high) {
    /* 两个参数错原本是分开的两条(各自带一句 printf_log)，合并成一条是因为去掉日志后
     * 两个分支体一样了，留着会被 clang-tidy 报 "Repeated branch body"。 */
    if ((type != cont_pos && type != cont_neg) || threshold_low >= threshold_high) {
        return NULL;
    }

    /* 源工程是 RT_MALLOC(RT-Thread 的堆)，本工程没有 RTOS 内存池 —— 和
     * libs/controller 的 create_controller() 一样直接 malloc。顺手把源工程漏掉的
     * NULL 判断补上：不补的话下面那行 memset 会直接跳空指针。 */
    Edge_trigger_continuous* obj =
        (Edge_trigger_continuous*)malloc(sizeof(Edge_trigger_continuous));
    if (obj == NULL) {
        return NULL;
    }
    memset(obj, 0, sizeof(Edge_trigger_continuous));

    obj->type = type;
    obj->threshold_high = threshold_high;
    obj->threshold_low = threshold_low;

    const int32_t width = threshold_high - threshold_low;
    switch (type) {
    case cont_pos:
        obj->value = threshold_low - width;
        break;
    case cont_neg:
        obj->value = threshold_high + width;
        break;
    }
    obj->flag = 0;
    obj->triggered = 0;
    return obj;
}
/**
 * @brief 更新连续边沿触发器的状态，检测边沿触发并维护触发状态
 * 
 * 该函数实现了带滞回的边沿检测算法：
 * - cont_pos模式：检测上升沿（从低于高阈值变为高于等于高阈值）
 * - cont_neg模式：检测下降沿（从高于低阈值变为低于等于低阈值）
 * 
 * 运行逻辑：
 * 1. 更新值（value = new_value）
 * 2. 根据触发类型执行不同的边沿检测逻辑：
 *    
 *    cont_pos（正向触发）：
 *    - 边沿检测：当前值≥高阈值，且上一次更新后triggered==0时，设置flag=1
 *    - 状态重置：当前值≤低阈值时，清除flag=0
 *    - 触发状态：值≥高阈值时triggered=1，值≤低阈值时triggered=0,低阈值<值<高阈值时保持不变
 *    
 *    cont_neg（负向触发）：  
 *    - 边沿检测：当前值≤低阈值，且上一次更新后triggered==0时，设置flag=1
 *    - 状态重置：当前值≥高阈值时，清除flag=0
 *    - 触发状态：值≤低阈值时triggered=1，值≥高阈值时triggered=0,低阈值<值<高阈值时保持不变
 * 
 * @param obj 连续边沿触发器对象指针
 * @param new_value 新的输入值
 * 
 * @note flag用于边沿检测（一次性触发标志），triggered用于状态维护（持续状态）
 * @note 滞回机制防止在阈值附近的噪声导致频繁触发
 */
void Etc_Update(Edge_trigger_continuous* obj, int16_t new_value) {
    obj->value = new_value;

    switch (obj->type) {
    case cont_pos:
        if (obj->value >= obj->threshold_high && obj->triggered == 0) {
            obj->flag = 1;
        } else if (obj->value <= obj->threshold_low) {
            obj->flag = 0;
        }

        if (obj->value >= obj->threshold_high) {
            obj->triggered = 1;
        } else if (obj->value <= obj->threshold_low) {
            obj->triggered = 0;
        }
        break;
    case cont_neg:
        if (obj->value <= obj->threshold_low && obj->triggered == 0) {
            obj->flag = 1;
        } else if (obj->value >= obj->threshold_high) {
            obj->flag = 0;
        }

        if (obj->value <= obj->threshold_low) {
            obj->triggered = 1;
        } else if (obj->value >= obj->threshold_high) {
            obj->triggered = 0;
        }
        break;
    }
}
/**
 * @brief 查看当前是否超过阈值,但不获取触发标志位
 * 
 * @param obj 
 * @return uint8_t 0或1,表示是否超过阈值
 */
uint8_t Etc_Peek(Edge_trigger_continuous* obj) { return obj->triggered; }
/**
 * @brief 获取并清除触发标志位
 * @note 意义是避免在更新频率低于检查标志位频率时多次执行变化沿逻辑
 * 
 * @param obj 
 * @return uint8_t 0或1,表示是否获取到触发标志位
 */
uint8_t Etc_Get(Edge_trigger_continuous* obj) {
    uint8_t temp = obj->flag;
    obj->flag = 0;
    return temp;
}