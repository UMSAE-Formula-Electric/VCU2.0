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
#include "error_handler.h"
#include "logger.h"
#include "freertos_task_handles.h"

static EventGroupHandle_t wdEvGroup = NULL;	//watchdog event group

void iwdgTask(void * pvParameters);
static void initWdEventGroup();
void kickIWDG();
static uint8_t initIWDGHardware();
void wdErrorHandler();

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

//  initWdEventGroup();

  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */

/*
 * wd_criticalTaskKick
 *
 * @brief this function is used for allowing critical tasks to notify the watchdog that they have not hanged yet
 *
 * @WD_CRITICALTASK the critical task notifying the watchdog that it has not hung yet
 *
 */
void wd_criticalTaskKick(enum WD_CRITICALTASK task){
    configASSERT(wdEvGroup != NULL);
    xEventGroupSetBits(wdEvGroup, 1<<task);
}

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

/*
 * wdErrorHandler
 *
 *@brief this function tries to fix the watchdog error handler
 */
void wdErrorHandler() {
    logMessage("Failed to create IWDG task", true);
}

bool areAllActiveTasksReady(const TaskInfo *taskInfos, size_t taskCount) {
    EventBits_t bits;
    bool allTasksReady = true;
    for (int i = 0; i < taskCount; i++) {                                   // Iterate over the taskInfos[] array
        if (taskInfos[i].isTaskActive == 1) {                               // Only check the bit of the task if its TASK_ENABLED value is 1
            bits = xEventGroupGetBits(iwdgEventGroupHandle);
            if ((bits & (1 << i)) == 0) {                                   // Check if the corresponding bit for the current task is set in iwdgEventGroup
                allTasksReady = false;                                      // If the bit for the current task is not set, update allTasksReady and break the loop
            }
        }
    }
    return allTasksReady;
}

void StartWatchDogTask(void *argument) {
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        return;
    }

    bool allTasksReady;
    size_t taskCount;
    TaskInfo taskInfos[] = {
            {defaultTaskHandle, DEFAULT_TASK_ENABLED},
            {dashLedTaskHandle, DASH_LED_TASK_ENABLED},
            {watchDogTaskHandle, WATCH_DOG_TASK_ENABLED},
            {canTxTaskHandle, CAN_TX_TASK_ENABLED},
            {canRxTaskHandle, CAN_RX_TASK_ENABLED},
            {btDumpTaskHandle, BT_DUMP_TASK_ENABLED},
            {vcuStateTaskHandle, VCU_STATE_TASK_ENABLED},
            {mcHrtbeatTaskHandle, MC_HRTBEAT_TASK_ENABLED},
            {acuHrtbeatTaskHandle, ACU_HRTBEAT_TASK_ENABLED},
            {brakeProcTaskHandle, BRAKE_PROC_TASK_ENABLED},
            {appsProcTaskHandle, APPS_PROC_TASK_ENABLED}
    };

    for(;;) {
        taskCount = sizeof(taskInfos) / sizeof(TaskInfo);
        allTasksReady = areAllActiveTasksReady(taskInfos, taskCount);

        if (allTasksReady) {                                                    // If all tasks have set their bits before IWDG_RELOAD_PERIOD, refresh the watchdog timer
            HAL_IWDG_Refresh(&hiwdg);
        }

        osDelay(IWDG_RELOAD_PERIOD);                                       // Delay for IWDG_RELOAD_PERIOD
    }
}

/* USER CODE END 1 */
