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
  hiwdg.Init.Reload = 500;
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
 * initWdEventGroup
 *
 * @brief this function sets up the watchdog event group. This event group is used for monitoring critical tasks
 */
static void initWdEventGroup(){
    configASSERT(wdEvGroup == NULL);
    wdEvGroup = xEventGroupCreate();

    /* Was the event group created successfully? */
    if( wdEvGroup == NULL )
    {
        /* The event group was not created because there was insufficient FreeRTOS heap available. */
        log_and_handle_error(IWDG_ERROR, wdErrorHandler);
    }
}

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

/*
 * wdTask
 *
 * @brief RTOS task for periodically kicking the watchdog
 */
//TODO Fix WatchDog
void StartWatchDogTask(void *argument) {
    while (1) {

        //kick the watchdog
        HAL_IWDG_Refresh(&hiwdg);

        //testIWDGReset(); //comment this out when not testing

        //wait a bit then wait for syncing
        HAL_Delay(WDPERIOD);
        //((1<<wd_NumCriticalTasks)-1) this will give us all the bits for all the critical tasks
        xEventGroupWaitBits(wdEvGroup, ((1<<wd_NumCriticalTasks)-1), pdTRUE, pdTRUE, 500);//wait for all the critical tasks to kick
    }
}

/* USER CODE END 1 */
