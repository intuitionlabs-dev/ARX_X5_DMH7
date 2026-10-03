#include "hal_move_plan.h"

/* 源工程里这个文件被 `#if _HAL_MOVEPLAN_ENABLE` 包着，宏没定义时整个文件只剩一个空壳函数。
 * 本工程没有这个宏，条件恒为 0，所以把那对条件编译删掉，让它直接生效。
 * 源工程用的 osal_*（osal/osal.h）没移植，这里换成 CMSIS-RTOS2 / FreeRTOS，
 * 和 Core/Src/freertos.c 里其它任务的写法保持一致。 */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "move_plan.h"     /* Move_Plan_Init() / Move_Plan_Update() */
#include "bsp_supervise.h" /* FPS_t / FrameRateStatistics() */

/* 任务在不在跑，JLink 直接读它（FPS 应该在 500 左右，即 2ms 一拍） */
static FPS_t FPS_Moveplan;

/* 静态栈 + 静态 TCB，不占 FreeRTOS heap。
 * 本工程 configTOTAL_HEAP_SIZE 只有 15360，其它任务也都是静态栈。 */
osThreadId_t Moveplan_TaskHandle;
uint32_t Moveplan_TaskBuffer[256];
StaticTask_t Moveplan_TaskControlBlock;
const osThreadAttr_t Moveplan_Task_attributes = {
    .name = "Moveplan_Task",
    .cb_mem = &Moveplan_TaskControlBlock,
    .cb_size = sizeof(Moveplan_TaskControlBlock),
    .stack_mem = &Moveplan_TaskBuffer[0],
    .stack_size = sizeof(Moveplan_TaskBuffer),
    /* 源工程给的是 40（= osPriorityHigh），但本工程 CmdTask/MotorTask 已经是 High。
     * 规划器慢一拍只是 setpoint 旧 2ms，抢了控制任务却会直接掉控制周期，所以压到
     * AboveNormal：高于 defaultTask，低于 CmdTask/MotorTask/MonitorTask。 */
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static void start_moveplan_task(void *argument) {
    (void)argument;
    uint32_t tick = osKernelGetTickCount();
    /* Infinite loop */
    for (;;) {
        FrameRateStatistics(&FPS_Moveplan);
        Move_Plan_Update();
        tick += 2U;
        osDelayUntil(tick);
    }
}

void HAL_Moveplan_Thread_Create(void) {
    /* Move_Plan_Init() 里是 cvector_create() → malloc()。放在这里（建任务之前、
     * 调度器启动之前）调用，单线程、没有 malloc 竞态。
     * 不初始化的话 move_plan_list / move_plan_temp_list 都是 NULL，
     * Move_Plan_Update() 会在第一行直接 return，规划器等于没接。 */
    Move_Plan_Init();
    Moveplan_TaskHandle =
        osThreadNew(start_moveplan_task, NULL, &Moveplan_Task_attributes);
}
