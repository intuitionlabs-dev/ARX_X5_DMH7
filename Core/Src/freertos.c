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
#include "monitor.h"
#include "can_loopback_test.h"
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartCanLoopbackTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMonitorTask(void *argument);
void StartMotorTask(void *argument);

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* creation of CanLoopbackTask */
  CanLoopbackTaskHandle = osThreadNew(StartCanLoopbackTask, NULL, &CanLoopbackTask_attributes);
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
  /* Infinite loop */
  uint32_t tick = osKernelGetTickCount();
  for(;;)
  {
    tick += 1U;
    osDelayUntil(tick);
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
  /* Infinite loop */
  uint32_t tick = osKernelGetTickCount();
  for(;;)
  {
    Monitor_Loop();
    tick += 1U;
    osDelayUntil(tick);
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
  /* BSP_CAN_Init() / DMmotor_Driver_Init() / Monitor_Init() live in
   * main.c USER CODE BEGIN 2, before the scheduler starts. */
  /* Infinite loop */
  uint32_t tick = osKernelGetTickCount();
  for(;;)
  {
    DMmotor_Calc_Send();
    tick += 1U;
    osDelayUntil(tick);
  }
  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* CanLoopbackTest_Init() 在 main.c 的 USER CODE BEGIN 2 里调用，在调度器启动前。 */
void StartCanLoopbackTask(void *argument)
{
  (void)argument;
  uint32_t tick = osKernelGetTickCount();
  for(;;)
  {
    CanLoopbackTest_Loop();
    tick += 1U;
    osDelayUntil(tick);
  }
}
/* USER CODE END Application */

