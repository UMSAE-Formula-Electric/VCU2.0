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

// The Handle to our logging queue
extern osMessageQueueId_t errorLogQueueHandle;


void StartErrorLogTask(void *argument)
{
  /* USER CODE BEGIN StartErrorLogTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);

    uint8_t queue_data[8];

    // Pull from Queue and log it
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

    // Send to CAN Bus
    //return sendCan(&hcan1, queue_data, 8, CAN_LOG_ERROR_ID, CAN_NO_EXT, CAN_NO_EXT) == 0;
    return sendCan(&hcan1, queue_data, 8, 0, CAN_NO_EXT, CAN_NO_EXT) == 0;
  }
  /* USER CODE END StartErrorLogTask */
}


//bool push_msg_to_queue()

/*
// Logging functions
bool log_error(ERR_TABLE error_type, DataType data_type, uint8_t *data) {
	LogLevel log_level = LOG_Error;	// Error

	// Send a CAN message
	//Send the chunk of the message over CAN
	uint8_t data_length = datatype_to_size(data_type);
	uint8_t msg_length = 3 + data_length;
	uint8_t msg_data[8];

	msg_data[0] = (uint8_t)log_level;
	msg_data[1] = error_type;
	msg_data[2] = data_type;

	for (int i = 0; i < data_length; i++) {
		msg_data[i + 3] = data[i];
	}

	osStatus_t result = osMessageQueuePut(
			errorLogQueueHandle,
			msg_data,
			0,
			0
	);

	return result == osOK;
};

bool log_warning(WARN_TABLE warning_type, DataType data_type, uint8_t* data) {
	LogLevel log_level = LOG_Warning;	// Error

	// Send a CAN message
	//Send the chunk of the message over CAN
	uint8_t data_length = datatype_to_size(data_type);
	uint8_t msg_length = 3 + data_length;
	uint8_t msg_data[8];

	msg_data[0] = (uint8_t)log_level;
	msg_data[1] = warning_type;
	msg_data[2] = data_type;

	for (int i = 0; i < data_length; i++) {
		msg_data[i + 3] = data[i];
	}

	osStatus_t result = osMessageQueuePut(
			errorLogQueueHandle,
			msg_data,
			0,
			0
	);

	return result == osOK;
};

bool log_info(INFO_TABLE info_type, DataType data_type, uint8_t* data) {
	LogLevel log_level = LOG_Info;	// Error

	// Send a CAN message
	//Send the chunk of the message over CAN
	uint8_t data_length = datatype_to_size(data_type);
	uint8_t msg_length = 3 + data_length;
	uint8_t msg_data[8];

	msg_data[0] = (uint8_t)log_level;
	msg_data[1] = info_type;
	msg_data[2] = data_type;

	for (int i = 0; i < data_length; i++) {
		msg_data[i + 3] = data[i];
	}

	osStatus_t result = osMessageQueuePut(
			errorLogQueueHandle,
			msg_data,
			0,
			0
	);

	return result == osOK;
};
*/
