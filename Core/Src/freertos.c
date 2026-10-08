/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DM_motor.h"
#include "ENCOS_motor.h"
#include "monitor.h"
#include "can_loopback_test.h"
#include "uart_loopback_test.h"
#include "cmd.h"
#include "arm.h"
#include "gimbal_arm.h"
#include "gimbal_cmd.h"
#include "arm_zero.h"
#include "hal_move_plan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// FPS
typedef struct  {
    FPS_t cmd;
    FPS_t motor;
    FPS_t monitor;
} Freertos_fps;
Freertos_fps freertos_fps;
#if TEST_CAN_LOOPBACK_ENABLE
/* Definitions for CanLoopbackTask */
osThreadId_t CanLoopbackTaskHandle;
uint32_t CanLoopbackTaskBuffer[ 512 ];
osStaticThreadDef_t CanLoopbackTaskControlBlock;
const osThreadAttr_t CanLoopbackTask_attributes = {
  .name = "CanLoopbackTask",
  .cb_mem = &CanLoopbackTaskControlBlock,
  .cb_size = sizeof(CanLoopbackTaskControlBlock),
  .stack_mem = &CanLoopbackTaskBuffer[0],
  .stack_size = sizeof(CanLoopbackTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
#endif

#if TEST_UART_LOOPBACK_ENABLE
/* Definitions for UartLoopbackTask */
osThreadId_t UartLoopbackTaskHandle;
uint32_t UartLoopbackTaskBuffer[ 512 ];
osStaticThreadDef_t UartLoopbackTaskControlBlock;
const osThreadAttr_t UartLoopbackTask_attributes = {
  .name = "UartLoopbackTask",
  .cb_mem = &UartLoopbackTaskControlBlock,
  .cb_size = sizeof(UartLoopbackTaskControlBlock),
  .stack_mem = &UartLoopbackTaskBuffer[0],
  .stack_size = sizeof(UartLoopbackTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
#endif
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
uint32_t MonitorTaskBuffer[ 128 ];
osStaticThreadDef_t MonitorTaskControlBlock;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .cb_mem = &MonitorTaskControlBlock,
  .cb_size = sizeof(MonitorTaskControlBlock),
  .stack_mem = &MonitorTaskBuffer[0],
  .stack_size = sizeof(MonitorTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
uint32_t MotorTaskBuffer[ 1024 ];
osStaticThreadDef_t MotorTaskControlBlock;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .cb_mem = &MotorTaskControlBlock,
  .cb_size = sizeof(MotorTaskControlBlock),
  .stack_mem = &MotorTaskBuffer[0],
  .stack_size = sizeof(MotorTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CmdTask */
osThreadId_t CmdTaskHandle;
uint32_t CmdTaskBuffer[ 4096 ];
osStaticThreadDef_t CmdTaskControlBlock;
const osThreadAttr_t CmdTask_attributes = {
  .name = "CmdTask",
  .cb_mem = &CmdTaskControlBlock,
  .cb_size = sizeof(CmdTaskControlBlock),
  .stack_mem = &CmdTaskBuffer[0],
  .stack_size = sizeof(CmdTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#if TEST_CAN_LOOPBACK_ENABLE
void StartCanLoopbackTask(void *argument);
#endif
#if TEST_UART_LOOPBACK_ENABLE
void StartUartLoopbackTask(void *argument);
#endif
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMonitorTask(void *argument);
void StartMotorTask(void *argument);
void StartCmdTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of CmdTask */
  CmdTaskHandle = osThreadNew(StartCmdTask, NULL, &CmdTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#if TEST_CAN_LOOPBACK_ENABLE
  /* creation of CanLoopbackTask */
  CanLoopbackTaskHandle = osThreadNew(StartCanLoopbackTask, NULL, &CanLoopbackTask_attributes);
#endif
#if TEST_UART_LOOPBACK_ENABLE
  /* creation of UartLoopbackTask */
  UartLoopbackTaskHandle = osThreadNew(StartUartLoopbackTask, NULL, &UartLoopbackTask_attributes);
#endif
  HAL_Moveplan_Thread_Create();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop：把 CPU 让出来，1 ms 一拍。 */
    portTickType currentTimeDefault;
    currentTimeDefault = xTaskGetTickCount();
    for (;;) {
        if (xTaskDelayUntil(&currentTimeDefault, 1) == pdFALSE) {
            currentTimeDefault = xTaskGetTickCount();   /* 这一拍没睡成：重新对齐到"现在" */
        }
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the MonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTask */
    /* Infinite loop。100 Hz —— count 一次减 1 就是 10 ms。
     * 这个频率和每个 monitor 注册时给的 count 是绑死的：改这里必须同步改所有
     * Monitor_Register 的 count，否则所有掉线判定的时间一起变(见 monitor.h)。 */
    portTickType currentTimeMonitor;
    currentTimeMonitor = xTaskGetTickCount();
    for (;;) {
        FrameRateStatistics(&freertos_fps.monitor);
        Monitor_Loop();
        /* FreeRTOS 的 xTaskDelayUntil 在"该醒的时刻已经过去"时**不阻塞**，却照样把
         * prev 推进一个周期(task.h 原文：A task will not be delayed if the next
         * expected wake time is in the past)。不接住返回值，任务就会一直以循环体
         * 自身的耗时为空转周期、再也回不来 —— 详见 StartMotorTask 里的实测记录。 */
        if (xTaskDelayUntil(&currentTimeMonitor, 10) == pdFALSE) {
            currentTimeMonitor = xTaskGetTickCount();   /* 这一拍没睡成：重新对齐到"现在" */
        }
    }
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
    portTickType currentTimeMotor;
    currentTimeMotor = xTaskGetTickCount();
    for (;;) {
        
        FrameRateStatistics(&freertos_fps.motor);
        /* 设零状态机(App/arm_zero.c)。它必须和下面那段发送循环待在同一个任务里：
         * 两个驱动都要求自己的 API 由调用者串行化，而且"设零期间一条常规控制帧都不发"
         * 这个门只能关在发送循环里 —— 设零帧和常规控制帧是互斥的
         * (尤其达妙那次周期性使能帧，会把状态机刚发过的"全部失能"直接抵消)。
         * 先 Loop() 再判 Busy()：进入设零状态的那一拍就已经把门关上了。
         * **两台一起推进**：各台的状态块是独立的(App/inc/arm.h 的 Arm_zero)，两台同构、
         * 零点表一样、跑的是同一个流程，所以同一拍进、同一拍出。
         * 门是按**臂**关的(下面循环里的 Arm_Zero_Busy(g_arm[bus].gimbal))：原来是
         * "整条总线"一把闸住，双臂之后那样会让 A 台设零捎带着把 B 台也停了。 */
        uint8_t bus, round;
        Arm_Zero_Loop(g_arm[ARM_A].gimbal);
        Arm_Zero_Loop(g_arm[ARM_B].gimbal);
        /* 两个驱动都是"一次一帧"的步进形式(见 DM_motor.h / ENCOS_motor.h)：下面这个
         * 轮转循环每轮给**每条总线各发一帧**，然后**等一次** 200us 最小帧间隔 —— 这正是
         * 用户指定的"两路交替配对、一个来回等一次"。
         *
         * 为什么不是"每发一帧等一次"：两路各等一次会把每路的有效帧率砍半。为什么不是
         * "发完所有帧再等"：BSP_CAN_Send 连续背靠背灌 FIFO，200us 那一条(SpeedCat 适配
         * 器的脾气)就没意义了。
         *
         * 轮数上限 CAN_TX_MAX_FRAMES_PER_TICK(=4) 是怎么来的：8 拍一个周期里，每路
         * j1 发 8 次、j2/j3 各 4 次、j4/j5 各 4 次、j6/夹爪各 2 次 ⇒ 16 帧/8 拍，即
         * **每拍每路峰值 3 帧、均值 2 帧**，而两条总线同相位(分组门控只读 os 时基、
         * 与总线无关)，帧数永远相等。3 帧 + 每 75ms 那次补发使能帧的 1 帧 = 4。
         * 所以最坏一拍等 4×200us = 800us，和只有 CAN1 时是同一个量级；第二路是搭同一轮
         * 的车，不额外付等待。 */
        DMmotor_Send_Begin();
        ENCOSmotor_Send_Begin();
        /* "本拍属于哪个组"**整拍只锁存一次**。必须在这里锁，不能等到驱动里去现读
         * osKernelGetTickCount()：下面这个轮转循环一拍要跨 600~800us(每轮等一次 200us)，
         * 毫秒边沿会落在循环中间，现读的话同一拍里的电机会拿到两个相位 —— 排在轮转
         * 后面的那台被"下一组的门"整拍挡掉。实测症状：j1 掩码 0x55(500 Hz) 却只有 250 Hz，
         * 而排在它前面的 j4/j5 是准的。 */
        BSP_CAN_TxGroupLatch();
        for (round = 0U; round < (uint8_t)CAN_TX_MAX_FRAMES_PER_TICK; round++) {
            uint8_t sent = 0U;
            for (bus = 0U; bus < (uint8_t)ARM_NUM; bus++) {
                /* 这台臂在设零 → **它那条总线**关门。必须按臂判：原来是"整条总线"一把闸住
                 * 两个循环，双臂之后那样会让 A 台设零捎带着把 B 台也停了。 */
                if (Arm_Zero_Busy(g_arm[bus].gimbal)) continue;
                /* 两条 if **不能写成并列的**：continue 让这条总线这一轮就此打住，保证
                 * "每路每轮最多 1 帧"。写成并列的话达妙和 ENCOS 会在同一轮各出一帧，
                 * 那一轮出的是 2 帧、却只等了一次间隔。代价是达妙优先于 ENCOS ——
                 * 无所谓，每台电机每拍最多只发一次。 */
                if (DMmotor_Send_Step(bus)) { sent = 1U; continue; }
                if (ENCOSmotor_Send_Step(bus)) { sent = 1U; }
            }
            if (!sent) break;              /* 这一拍两条总线都没事干了 */
            BSP_CAN_TxGapWait();           /* 一轮之后才等这一次 */
        }
        if (xTaskDelayUntil(&currentTimeMotor, 1) == pdFALSE) {
            currentTimeMotor = xTaskGetTickCount();   /* 这一拍没睡成：重新对齐到"现在" */
        }
    }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartCmdTask */
/**
* @brief Function implementing the CmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdTask */
void StartCmdTask(void *argument)
{
  /* USER CODE BEGIN StartCmdTask */
  /* 六轴机械臂云台：建 7 个达妙电机 + 控制器。
   * 必须在任务里建、不能放 main() 的 USER CODE 2 —— Gimbal_Create() 栈上要
   * ~4 KB(joint_controller_config[ARM_MOTOR_NUM])，主栈 _Min_Stack_Size 只有 2 KB；
   * 本任务栈是 CmdTaskBuffer(16 KB)。依赖的 BSP_CAN_Init()/DMmotor_Driver_Init()
   * 在 main.c 里、调度器启动前就跑完了。失败(内存不够)时是 NULL，Cmd_Loop() 判空跳过。 */
  /* 两台臂的驱动 + 命令层都挂在 g_arm[] 上(App/inc/arm.h)。
   * arm_id == CAN 索引：ARM_A(0) 走 CAN1，ARM_B(1) 走 CAN2。两台**完全同构** ——
   * 同一套型号/量程/PID/零点表(Gimbal_Create 的 can_index 是唯一的差别)。
   *
   * **顺序建**：Gimbal_Create() 栈上要 ~4 KB(见上面那段)，循环里逐台建的话峰值仍然
   * 是 4 KB。别改成"先建两个再配"，那样栈会叠两倍。
   *
   * 命令层必须**接在驱动后面**：Gimbal_Cmd_Create(&g_arm[A]) 要读 arm->gimbal(它拿它
   * 去读电机反馈)。所以这两句的顺序不能反、也不能拆到两个循环里。
   *
   * g_arm[].id 不在这里写 —— Robot_State_Init()(main.c 的 USER CODE 2，调度器启动前)
   * 已经把 id 逐个填好了，那比这里早，收包中断一进来就是对的。 */
  for (uint8_t i = 0U; i < (uint8_t)ARM_NUM; i++) {
      g_arm[i].gimbal = Gimbal_Create(i);
      g_arm[i].gimbal_cmd = Gimbal_Cmd_Create(&g_arm[i]);
  }
    /* Infinite loop。500 Hz。 */
    portTickType currentTimeCmd;
    currentTimeCmd = xTaskGetTickCount();
    for (;;) {
        FrameRateStatistics(&freertos_fps.cmd);
        Cmd_Loop();
        /* FreeRTOS 的 xTaskDelayUntil 在"该醒的时刻已经过去"时**不阻塞**，却照样把
         * prev 推进一个周期(task.h 原文：A task will not be delayed if the next
         * expected wake time is in the past)。不接住返回值，任务就会一直以循环体
         * 自身的耗时为空转周期、再也回不来 —— 详见 StartMotorTask 里的实测记录。 */
        if (xTaskDelayUntil(&currentTimeCmd, 2) == pdFALSE) {
            currentTimeCmd = xTaskGetTickCount();   /* 这一拍没睡成：重新对齐到"现在" */
        }
    }
  /* USER CODE END StartCmdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#if TEST_CAN_LOOPBACK_ENABLE

/* CanLoopbackTest_Init() 在 main.c 的 USER CODE BEGIN 2 里调用，在调度器启动前。 */
void StartCanLoopbackTask(void *argument)
{
  (void)argument;
    /* 回环测试线程(TEST_CAN_LOOPBACK_ENABLE 才编译)。 */
    portTickType currentTimeCanLb;
    currentTimeCanLb = xTaskGetTickCount();
    for (;;) {
        CanLoopbackTest_Loop();
        /* FreeRTOS 的 xTaskDelayUntil 在"该醒的时刻已经过去"时**不阻塞**，却照样把
         * prev 推进一个周期(task.h 原文：A task will not be delayed if the next
         * expected wake time is in the past)。不接住返回值，任务就会一直以循环体
         * 自身的耗时为空转周期、再也回不来 —— 详见 StartMotorTask 里的实测记录。 */
        if (xTaskDelayUntil(&currentTimeCanLb, 1) == pdFALSE) {
            currentTimeCanLb = xTaskGetTickCount();   /* 这一拍没睡成：重新对齐到"现在" */
        }
    }
}
#endif

#if TEST_UART_LOOPBACK_ENABLE
/* UartLoopbackTest_Init() 在 main.c 的 USER CODE BEGIN 2 里调用，在调度器启动前。 */
void StartUartLoopbackTask(void *argument)
{
  (void)argument;
    /* 回环测试线程(TEST_UART_LOOPBACK_ENABLE 才编译)。 */
    portTickType currentTimeUartLb;
    currentTimeUartLb = xTaskGetTickCount();
    for (;;) {
        UartLoopbackTest_Loop();
        /* FreeRTOS 的 xTaskDelayUntil 在"该醒的时刻已经过去"时**不阻塞**，却照样把
         * prev 推进一个周期(task.h 原文：A task will not be delayed if the next
         * expected wake time is in the past)。不接住返回值，任务就会一直以循环体
         * 自身的耗时为空转周期、再也回不来 —— 详见 StartMotorTask 里的实测记录。 */
        if (xTaskDelayUntil(&currentTimeUartLb, 1) == pdFALSE) {
            currentTimeUartLb = xTaskGetTickCount();   /* 这一拍没睡成：重新对齐到"现在" */
        }
    }
}
#endif

/* USER CODE END Application */

