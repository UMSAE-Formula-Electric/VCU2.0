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

#endif /* INC_FREERTOS_TASK_HANDLES_H_ */
