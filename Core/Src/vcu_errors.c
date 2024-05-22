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

void StartErrorLogTask(void *argument)
{
  for(;;)
  {
    uint8_t queue_data[8];

    osStatus_t result = osMessageQueueGet(
    		errorLogQueueHandle,
			queue_data,
			0,
			0
    );

    // Ignore the message if there was an issue
    if (result != osOK) {
    	continue;
    }
  }
}
