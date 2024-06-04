/*
 * mc_comms_handler.c
 *
 *  Created on: Apr 27, 2024
 *      Author: tonyz
 */


#include "FreeRTOS.h"
#include "heartbeat.h"
#include "iwdg.h"

/*
 *
 *
 * @Brief: this function updates the heartbeat task to the presence of the
 * motor controller
 */
void notify_mc_heartbeat_task() {
    TaskHandle_t task = NULL;
    task = get_mc_heartbeat_task_handle();
    if (task != NULL) {
        xTaskNotify(task, 0, eNoAction);
        osThreadFlagsSet(task, 0x01);
    }
}

void StartMcCanCommsTask(void *argument) {
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

    for (;;) {
        kickWatchdogBit(osThreadGetId());
    }
}
