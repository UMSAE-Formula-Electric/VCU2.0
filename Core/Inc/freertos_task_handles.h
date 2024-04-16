/*
 * freertos_task_handles.h
 *
 *  Created on: Apr 14, 2024
 *      Author: tonyz
 */

#ifndef INC_FREERTOS_TASK_HANDLES_H_
#define INC_FREERTOS_TASK_HANDLES_H_

#include "cmsis_os.h"

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t dashLedTaskHandle;
extern osThreadId_t watchDogTaskHandle;
extern osThreadId_t canTxTaskHandle;
extern osThreadId_t canRxTaskHandle;
extern osThreadId_t btDumpTaskHandle;
extern osThreadId_t vcuStateTaskHandle;
extern osThreadId_t mcHrtbeatTaskHandle;
extern osThreadId_t acuHrtbeatTaskHandle;
extern osThreadId_t brakeProcTaskHandle;
extern osThreadId_t appsProcTaskHandle;

extern const osThreadAttr_t defaultTask_attributes;
extern const osThreadAttr_t dashLedTask_attributes;
extern const osThreadAttr_t watchDogTask_attributes;
extern const osThreadAttr_t canTxTask_attributes;
extern const osThreadAttr_t canRxTask_attributes;
extern const osThreadAttr_t btDumpTask_attributes;
extern const osThreadAttr_t vcuStateTask_attributes;
extern const osThreadAttr_t mcHrtbeatTask_attributes;
extern const osThreadAttr_t acuHrtbeatTask_attributes;
extern const osThreadAttr_t brakeProcTask_attributes;
extern const osThreadAttr_t appsProcTask_attributes;

#endif /* INC_FREERTOS_TASK_HANDLES_H_ */
