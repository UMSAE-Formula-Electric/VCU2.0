/*
 * vcu_errors.c
 *
 *  Created on: Apr 21, 2024
 *      Author: owenzonneveld
 */

#include "cmsis_os.h"


#include "vcu_errors.h"
#include "errors.h"
#include "can.h"
#include "logger.h"

void StartErrorLogTask(void *argument)
{
  for(;;)
  {
    LogPacket logPacket[8];

    osStatus_t result = osMessageQueueGet(
            errorLogQueueHandle,
            logPacket,
            0,
            0
    );

    if (result == osOK && CAN_TX_TASK_ENABLED) {
        sendCan(&hcan1, (uint8_t*) logPacket, 8, CAN_VCU_TO_SCU_LOG_ID, CAN_NO_RTR, CAN_NO_EXT);
    }
    else {
        /* TODO write a function that creates a string from LogPacket
         * and then use sendToUsart() */
    }
  }
}
