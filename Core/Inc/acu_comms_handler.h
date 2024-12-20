#ifndef ABC_COMMS_HANDLER_H
#define ABC_COMMS_HANDLER_H

#include <stdbool.h>
#include "car_state.h"
#include "can.h"
#include "can_utils.h"

extern osMessageQueueId_t ackCarStateQueueHandle;

void processAcuToVcuCanIdRxData(const uint8_t *RxData);
void set_ACU_State(enum CAR_STATE new_state);
void send_ACU_mesg(enum STARTUP_STATUS_NOTIFY_MSG msg);
void send_ACU_mesg_data(enum STARTUP_STATUS_NOTIFY_MSG msg_id, uint8_t data_len, uint8_t * msg_data);

#endif
