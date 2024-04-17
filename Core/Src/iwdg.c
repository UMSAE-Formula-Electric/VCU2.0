/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
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
#include "iwdg.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h"
#include <stdbool.h>
#include "event_groups.h"
#include "freertos_task_handles.h"

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = IWDG_RELOAD_PERIOD;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */
/*
 * startFromWD
 *
 * @return returns true if the system started due to a watchdog reset otherwise false
 *
 * @brief used for checking on reset if the system is starting from a watchdog reset
 */
bool startFromIWDG() {

    bool startFromWD = false; //true if we started from watchdog reset

    /* Check if the system has resumed from IWDG reset */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {

        startFromWD = true;

        /* Clear reset flags */
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }

    return startFromWD;
}

TaskInfo* getTaskInfos(size_t* count) {
    static TaskInfo taskInfos[] = {
            {&defaultTask_attributes, DEFAULT_TASK_ENABLED},
            {&dashLedTask_attributes, DASH_LED_TASK_ENABLED},
            {&watchDogTask_attributes, WATCH_DOG_TASK_ENABLED},
            {&canTxTask_attributes, CAN_TX_TASK_ENABLED},
            {&canRxTask_attributes, CAN_RX_TASK_ENABLED},
            {&canRxTask_attributes, BT_DUMP_TASK_ENABLED},
            {&vcuStateTask_attributes, VCU_STATE_TASK_ENABLED},
            {&mcHrtbeatTask_attributes, MC_HRTBEAT_TASK_ENABLED},
            {&acuHrtbeatTask_attributes, ACU_HRTBEAT_TASK_ENABLED},
            {&brakeProcTask_attributes, BRAKE_PROC_TASK_ENABLED},
            {&appsProcTask_attributes, APPS_PROC_TASK_ENABLED}
    };

    *count = sizeof(taskInfos) / sizeof(TaskInfo);
    return taskInfos;
}

TaskBit_t getTaskBit(const void *taskHandle) {
    size_t taskCount;
    TaskInfo* taskInfos = getTaskInfos(&taskCount);
    for (TaskBit_t taskBit = 0; taskBit < taskCount; taskBit++) {
        if (taskHandle == xTaskGetHandle(taskInfos[taskBit].task_attributes->name)) {
            return taskBit;
        }
    }
    return NUM_TASKS;
}

void kickWatchdogBit(osThreadId_t taskHandle) {
    TaskBit_t bitPosition = getTaskBit(taskHandle);
    xEventGroupSetBits(iwdgEventGroupHandle, (1 << bitPosition));
}

bool isTaskActive(TaskBit_t taskBit, TaskInfo *taskInfos, size_t taskCount) {
    if (taskBit < taskCount) {
        return taskInfos[taskBit].isTaskActive == 1;
    }

    return false;
}

bool isTaskReady(TaskBit_t bitPosition, EventBits_t taskBits) {
    return (taskBits & (1 << bitPosition)) != 0;
}

bool areAllActiveTasksReady() {
    size_t taskCount;
    TaskBit_t currTaskBit;
    osThreadId_t currTask;
    TaskInfo* taskInfos = getTaskInfos(&taskCount);
    EventBits_t taskBits = xEventGroupGetBits(iwdgEventGroupHandle);

    for (int taskBit = 0; taskBit < taskCount; taskBit++) {
        currTask = xTaskGetHandle(taskInfos[taskBit].task_attributes->name);
        currTaskBit = getTaskBit(currTask);
        if (isTaskActive(currTaskBit, taskInfos, taskCount) && !isTaskReady(currTaskBit, taskBits)) {
            return false;
        }
    }

    return true;
}

void StartWatchDogTask(void *argument) {
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

    MX_IWDG_Init();
    bool allActiveTasksReady;

    for(;;) {
        kickWatchdogBit(osThreadGetId());
        allActiveTasksReady = areAllActiveTasksReady();                               // Check if all active tasks have set their bits in iwdgEventGroup
        if (allActiveTasksReady) {                                                    // If all tasks have set their bits before IWDG_RELOAD_PERIOD, refresh the watchdog timer
            HAL_IWDG_Refresh(&hiwdg);
            xEventGroupClearBits(iwdgEventGroupHandle, 0xFFFFFFFF);
        }

        osDelay(IWDG_RELOAD_PERIOD);                                       // Delay for IWDG_RELOAD_PERIOD
    }
}

/* USER CODE END 1 */
