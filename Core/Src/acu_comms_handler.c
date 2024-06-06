#include <dashboard_mgmt.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "acu_comms_handler.h"
#include "can.h"
#include "car_state.h"
#include "vcu_startup.h"
#include "logger.h"

#include "heartbeat.h"

#include "string.h"
#include "stdio.h"
#include "bt_protocol.h"
#include "can_utils.h"

static void notify_startup_task(enum startup_notify_value notify_val);
static void notify_acu_heartbeat_task(HeartbeatNotify_t notify_val);

void processAcuToVcuCanIdRxData(const uint8_t *RxData) {
    //TODO VCU#32 INFO Processing ACB CAN message
    char strBuff[50]; //buffer for making 'nice' logs

    if (RxData[0] == CAN_ACB_TSA_ACK) {
        notify_startup_task(ACU_TSA_ACK);
    } else if (RxData[0] == CAN_ACB_TSA_NACK) {
        notify_startup_task(ACU_TSA_NACK);
    } else if (RxData[0] == CAN_ACB_RTD_ACK) {
        notify_startup_task(ACU_RTD_ACK);
    } else if (RxData[0] == CAN_ACB_RTD_NACK) {
        notify_startup_task(ACU_RTD_NACK);
    } else if (RxData[0] == CAN_GO_IDLE_REQ) {
        notify_startup_task(GO_IDLE_REQ_FROM_ACU);
    } else if (RxData[0] == CAN_NO_SAFETY_LOOP_SET) {
        btLogIndicator(true, SAFETY_LOOP);
    } else if (RxData[0] == CAN_NO_SAFETY_LOOP_CLEAR) {
        btLogIndicator(false, SAFETY_LOOP);
    } else if (RxData[0] == CAN_AIR_WELD_SET) {
    } else if (RxData[0] == CAN_HEARTBEAT_REQUEST) {
        // Do nothing
    } else if (RxData[0] == CAN_HEARTBEAT_RESPONSE) {
        notify_acu_heartbeat_task(HEARTBEAT_RESPONSE_NOTIFY);
    } else if (RxData[0] == CAN_BATTERY_VOLTAGE_REQUEST) {
        //lv_battery_handle_voltage_request(); from 2018 car
    } else {
        //build up string for marking unexpected can message
        //TODO VCU#32 ERROR/INFO unexpected ACB notification
        sprintf(strBuff, "Unexpected ACU can notification: %s\r\n", RxData);
        logMessage(strBuff, false);
    }
}

void set_ACU_State(enum CAR_STATE new_state){
	uint8_t data = (uint8_t) new_state;
	sendCan(&hcan1, &data, 1, CAN_VCU_SET_ACB_STATE_ID, CAN_RTR_DATA, CAN_NO_EXT);
}

void send_ACU_mesg(enum STARTUP_STATUS_NOTIFY_MSG msg){
	uint8_t data = (uint8_t)msg;
	sendCan(&hcan1, &data, 1, CAN_VCU_TO_ACU_ID, CAN_RTR_DATA, CAN_NO_EXT);
}

void send_ACU_mesg_data(enum STARTUP_STATUS_NOTIFY_MSG msg_id, uint8_t data_len, uint8_t * msg_data){
	uint8_t data[8] = {0};
	data[0] = msg_id;
	memcpy(&data[1], msg_data, data_len);
	sendCan(&hcan1, data, data_len+1, CAN_ACU_CAN_ID, CAN_RTR_DATA, CAN_NO_EXT);
}

void notify_startup_task(enum startup_notify_value notify_val){
    osMessageQueuePut(ackCarStateQueueHandle, &notify_val, 0, 0);
}

void notify_acu_heartbeat_task(HeartbeatNotify_t notify_val){
	osThreadId_t task = NULL;
	task = get_acu_heartbeat_task_handle();
	if(task != NULL){
		xTaskNotify( task, notify_val, eSetValueWithOverwrite);
	}
}


void StartAcuCanCommsTask(void *argument){
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadExit();
    }
}
