#ifndef H_MONITOR_H
#define H_MONITOR_H
#include <stdint.h>
typedef void (*lost_callback)(void* data);

#pragma pack(1)
typedef struct monitor_item_t {
    int reload_count;
    int count;
    lost_callback callback;
    void (*reset)(struct monitor_item_t* item);
    void* data;
} monitor_item;
#pragma pack()

void Monitor_Init(void);
/* count 的单位是 Monitor_Loop() 的调用周期，**不是毫秒**：注册点按
 * "想要的超时 / Monitor_Loop() 的周期" 给值。改 Monitor_Loop() 的频率，
 * 所有注册点的 count 都要同步改(周期写在 StartMonitorTask()，Core/Src/freertos.c)。 */
monitor_item* Monitor_Register(lost_callback callback, int count, void* callback_data);
uint8_t is_Offline(monitor_item* obj);
void Monitor_Loop(void);

#endif
