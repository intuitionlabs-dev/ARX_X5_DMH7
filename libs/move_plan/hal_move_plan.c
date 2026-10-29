#include "hal_move_plan.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "move_plan.h"
#include "bsp_supervise.h" /* FPS_t / FrameRateStatistics() */

/* 统计规划线程调用频率。 */
static FPS_t FPS_Moveplan;

/* 静态分配规划线程栈和任务控制块。 */
osThreadId_t Moveplan_TaskHandle;
uint32_t Moveplan_TaskBuffer[256];
StaticTask_t Moveplan_TaskControlBlock;
const osThreadAttr_t Moveplan_Task_attributes = {
    .name = "Moveplan_Task",
    .cb_mem = &Moveplan_TaskControlBlock,
    .cb_size = sizeof(Moveplan_TaskControlBlock),
    .stack_mem = &Moveplan_TaskBuffer[0],
    .stack_size = sizeof(Moveplan_TaskBuffer),
    /* 规划线程优先级为 AboveNormal。 */
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static void start_moveplan_task(void *argument) {
    (void)argument;
    /* 按 500 Hz 调度规划更新。 */
    portTickType currentTimeMoveplan;
    currentTimeMoveplan = xTaskGetTickCount();
    for (;;) {
        FrameRateStatistics(&FPS_Moveplan);
        Move_Plan_Update();
        /* 延时已过期时重置周期基准，保持后续按 2 ms 周期调度。 */
        if (xTaskDelayUntil(&currentTimeMoveplan, 2) == pdFALSE) {
            currentTimeMoveplan = xTaskGetTickCount();   /* 周期延时失效时以当前时刻重新对齐。 */
        }
    }
}

void HAL_Moveplan_Thread_Create(void) {
    /* 调度器启动前初始化动态规划列表，再创建规划线程。 */
    Move_Plan_Init();
    Moveplan_TaskHandle =
        osThreadNew(start_moveplan_task, NULL, &Moveplan_Task_attributes);
}
