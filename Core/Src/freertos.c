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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
#include "can_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dashLedTask */
osThreadId_t dashLedTaskHandle;
const osThreadAttr_t dashLedTask_attributes = {
  .name = "dashLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for watchDogTask */
osThreadId_t watchDogTaskHandle;
const osThreadAttr_t watchDogTask_attributes = {
  .name = "watchDogTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for canTxTask */
osThreadId_t canTxTaskHandle;
const osThreadAttr_t canTxTask_attributes = {
  .name = "canTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxTask */
osThreadId_t canRxTaskHandle;
const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for btDumpTask */
osThreadId_t btDumpTaskHandle;
const osThreadAttr_t btDumpTask_attributes = {
  .name = "btDumpTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vcuStateTask */
osThreadId_t vcuStateTaskHandle;
const osThreadAttr_t vcuStateTask_attributes = {
  .name = "vcuStateTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mcHrtbeatTask */
osThreadId_t mcHrtbeatTaskHandle;
const osThreadAttr_t mcHrtbeatTask_attributes = {
  .name = "mcHrtbeatTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for acuHrtbeatTask */
osThreadId_t acuHrtbeatTaskHandle;
const osThreadAttr_t acuHrtbeatTask_attributes = {
  .name = "acuHrtbeatTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for appsProcTask */
osThreadId_t appsProcTaskHandle;
const osThreadAttr_t appsProcTask_attributes = {
  .name = "appsProcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mcCanCommsTask */
osThreadId_t mcCanCommsTaskHandle;
const osThreadAttr_t mcCanCommsTask_attributes = {
  .name = "mcCanCommsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for acuCanCommsTask */
osThreadId_t acuCanCommsTaskHandle;
const osThreadAttr_t acuCanCommsTask_attributes = {
  .name = "acuCanCommsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxPacketQueue */
osMessageQueueId_t canRxPacketQueueHandle;
const osMessageQueueAttr_t canRxPacketQueue_attributes = {
  .name = "canRxPacketQueue"
};
/* Definitions for canTxPacketQueue */
osMessageQueueId_t canTxPacketQueueHandle;
const osMessageQueueAttr_t canTxPacketQueue_attributes = {
  .name = "canTxPacketQueue"
};
/* Definitions for ackCarStateQueue */
osMessageQueueId_t ackCarStateQueueHandle;
const osMessageQueueAttr_t ackCarStateQueue_attributes = {
  .name = "ackCarStateQueue"
};
/* Definitions for mcCanCommsQueue */
osMessageQueueId_t mcCanCommsQueueHandle;
const osMessageQueueAttr_t mcCanCommsQueue_attributes = {
  .name = "mcCanCommsQueue"
};
/* Definitions for acuCanCommsQueue */
osMessageQueueId_t acuCanCommsQueueHandle;
const osMessageQueueAttr_t acuCanCommsQueue_attributes = {
  .name = "acuCanCommsQueue"
};
/* Definitions for iwdgEventGroup */
osEventFlagsId_t iwdgEventGroupHandle;
const osEventFlagsAttr_t iwdgEventGroup_attributes = {
  .name = "iwdgEventGroup"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartDashboardLedTask(void *argument);
extern void StartWatchDogTask(void *argument);
extern void StartCanTxTask(void *argument);
extern void StartCanRxTask(void *argument);
extern void StartBluetoothDumpTask(void *argument);
extern void StartVcuStateTask(void *argument);
extern void StartMcHeartbeatTask(void *argument);
extern void StartAcuHeartbeatTask(void *argument);
extern void StartAppsProcessTask(void *argument);
extern void StartMcCanCommsTask(void *argument);
extern void StartAcuCanCommsTask(void *argument);

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

  /* Create the queue(s) */
  /* creation of canRxPacketQueue */
  canRxPacketQueueHandle = osMessageQueueNew (64, sizeof(CAN_RxPacketTypeDef), &canRxPacketQueue_attributes);

  /* creation of canTxPacketQueue */
  canTxPacketQueueHandle = osMessageQueueNew (64, sizeof(CAN_TxPacketTypeDef), &canTxPacketQueue_attributes);

  /* creation of ackCarStateQueue */
  ackCarStateQueueHandle = osMessageQueueNew (32, sizeof(uint8_t), &ackCarStateQueue_attributes);

  /* creation of mcCanCommsQueue */
  mcCanCommsQueueHandle = osMessageQueueNew (16, sizeof(CAN_RxPacketTypeDef), &mcCanCommsQueue_attributes);

  /* creation of acuCanCommsQueue */
  acuCanCommsQueueHandle = osMessageQueueNew (16, sizeof(CAN_RxPacketTypeDef), &acuCanCommsQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, (void*) DEFAULT_TASK_ENABLED, &defaultTask_attributes);

  /* creation of dashLedTask */
  dashLedTaskHandle = osThreadNew(StartDashboardLedTask, (void*) DASH_LED_TASK_ENABLED, &dashLedTask_attributes);

  /* creation of watchDogTask */
  watchDogTaskHandle = osThreadNew(StartWatchDogTask, (void*) WATCH_DOG_TASK_ENABLED, &watchDogTask_attributes);

  /* creation of canTxTask */
  canTxTaskHandle = osThreadNew(StartCanTxTask, (void*) CAN_TX_TASK_ENABLED, &canTxTask_attributes);

  /* creation of canRxTask */
  canRxTaskHandle = osThreadNew(StartCanRxTask, (void*) CAN_RX_TASK_ENABLED, &canRxTask_attributes);

  /* creation of btDumpTask */
  btDumpTaskHandle = osThreadNew(StartBluetoothDumpTask, (void*) BT_DUMP_TASK_ENABLED, &btDumpTask_attributes);

  /* creation of vcuStateTask */
  vcuStateTaskHandle = osThreadNew(StartVcuStateTask, (void*) VCU_STATE_TASK_ENABLED, &vcuStateTask_attributes);

  /* creation of mcHrtbeatTask */
  mcHrtbeatTaskHandle = osThreadNew(StartMcHeartbeatTask, (void*) MC_HRTBEAT_TASK_ENABLED, &mcHrtbeatTask_attributes);

  /* creation of acuHrtbeatTask */
  acuHrtbeatTaskHandle = osThreadNew(StartAcuHeartbeatTask, (void*) ACU_HRTBEAT_TASK_ENABLED, &acuHrtbeatTask_attributes);

  /* creation of appsProcTask */
  appsProcTaskHandle = osThreadNew(StartAppsProcessTask, (void*) APPS_PROC_TASK_ENABLED, &appsProcTask_attributes);

  /* creation of mcCanCommsTask */
  mcCanCommsTaskHandle = osThreadNew(StartMcCanCommsTask, (void*) MC_CAN_COMMS_TASK_ENABLED, &mcCanCommsTask_attributes);

  /* creation of acuCanCommsTask */
  acuCanCommsTaskHandle = osThreadNew(StartAcuCanCommsTask, (void*) ACU_CAN_COMMS_TASK_ENABLED, &acuCanCommsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of iwdgEventGroup */
  iwdgEventGroupHandle = osEventFlagsNew(&iwdgEventGroup_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadExit();
    }

  /* Infinite loop */
  for(;;)
  {
      kickWatchdogBit(osThreadGetId());
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

