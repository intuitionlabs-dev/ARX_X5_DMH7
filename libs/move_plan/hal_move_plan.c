#include "hal_move_plan.h"

#if _HAL_MOVEPLAN_ENABLE
#include "move_plan.h"
#include "bsp_supervise.h"
#include "osal/osal.h"

static FPS_t FPS_Moveplan;
static osal_task_t Moveplan_TaskHandle;
static void start_moveplan_task(void *argument) {
	osal_tick_t tick = osal_get_tick();
	/* Infinite loop */
	for (;;) {
		FrameRateStatistics(&FPS_Moveplan);
		Move_Plan_Update();
		osal_tick_delay_until(&tick, 2);
    }
}

void HAL_Moveplan_Thread_Create(void) {
    Moveplan_TaskHandle = osal_task_create(&Moveplan_TaskHandle, 
        start_moveplan_task, 128 * 4, 40, "Moveplan_Task");
}
#else
void HAL_Moveplan_Thread_Create(void){}
#endif