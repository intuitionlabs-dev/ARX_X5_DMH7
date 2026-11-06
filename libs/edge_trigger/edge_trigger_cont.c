#include "edge_trigger_cont.h"
#include <stdlib.h>
#include <string.h>
  /* 非法触发类型或阈值顺序返回 NULL。 */

Edge_trigger_continuous* Edge_Trigger_Continuous_Create(Edge_continuous_type type,
    int16_t threshold_low, int16_t threshold_high) {
    if ((type != cont_pos && type != cont_neg) || threshold_low >= threshold_high) {
        return NULL;
    }

    /* 分配并初始化触发器对象，分配失败返回 NULL。 */
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
/* 按双阈值滞回更新持续状态及边沿标志；正向越上阈值触发，负向越下阈值触发。 */
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
/* 读取持续触发状态。 */
uint8_t Etc_Peek(Edge_trigger_continuous* obj) { return obj->triggered; }
/* 读取并清除一次性边沿标志。 */
uint8_t Etc_Get(Edge_trigger_continuous* obj) {
    uint8_t temp = obj->flag;
    obj->flag = 0;
    return temp;
}
