/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.h
  * @brief   This file contains all the function prototypes for
  *          the iwdg.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IWDG_H__
#define __IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_IWDG_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct {
    osThreadId_t taskHandle;
    uint8_t isTaskActive;
} TaskInfo;

enum WD_CRITICALTASK{
    wd_APPS_CTask = 0,
    wd_BRAKE_CTASK,
    wd_STARTUP_CTask,
    wd_NumCriticalTasks
};

uint8_t initIWDG();
void testIWDGReset();
void testIWDGResetting();
bool startFromIWDG();
void wd_criticalTaskKick(enum WD_CRITICALTASK task);
extern osEventFlagsId_t iwdgEventGroupHandle;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_H__ */

