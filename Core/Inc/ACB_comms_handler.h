#ifndef ABC_COMMS_HANDLER_H
#define ABC_COMMS_HANDLER_H

#include <stdbool.h>
#include "car_state.h"
#include "can.h"

void processVcuCanIdRxData(const uint8_t *RxData);
void set_ACB_State(enum CAR_STATE new_state);
void send_ACB_mesg(enum ACB_TO_CAN_MSG msg);
void send_ACB_mesg_data(enum ACB_TO_CAN_MSG msg_id, uint8_t data_len, uint8_t * msg_data);
bool acb_comms_init();

#endif
