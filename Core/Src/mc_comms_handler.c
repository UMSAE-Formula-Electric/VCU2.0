/*
 * mc_comms_handler.c
 *
 *  Created on: Apr 27, 2024
 *      Author: tonyz
 */


#include "FreeRTOS.h"
#include "heartbeat.h"

/*
 *
 *
 * @Brief: this function updates the heartbeat task to the presence of the
 * motor controller
 */
void update_heartbeat() {
    TaskHandle_t task = NULL;
    task = heartbeat_MC_get_task();
    if (task != NULL) {
        xTaskNotify(task, 0, eNoAction);
        osThreadFlagsSet(task, 0x01);
    }
}
