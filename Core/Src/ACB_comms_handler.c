#include <dashboard_mgmt.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ACB_comms_handler.h"
#include "can.h"
#include "car_state.h"
#include "vcu_startup.h"
#include "logger.h"

#include "heartbeat.h"

#include "string.h"
#include "stdio.h"
#include "bt_protocol.h"

extern QueueHandle_t ACB_VCU_CAN_Queue;

static void notify_startup_task(enum startup_notify_value notify_val);
static void notify_heartbeat_task(HeartbeatNotify_t notify_val);

// TODO Create separate queues for different functionalities
void process_ACB_CAN_packets(void * pvParameters){
    CAN_RxHeaderTypeDef packetToProcess;
    uint8_t RxData[8];
	BaseType_t ret;

	for(;;){
		//TODO CAN Revisit
		ret = xQueueReceive(ACB_VCU_CAN_Queue,&packetToProcess, portMAX_DELAY);
		if(ret == pdPASS){
			if(packetToProcess.StdId ==  CAN_VCU_CAN_ID){
                processAcuToVcuCanIdRxData(RxData);
            }
		}
	}
	vTaskDelete(NULL);
}

void processAcuToVcuCanIdRxData(const uint8_t *RxData) {
    char strBuff[50]; //buffer for making 'nice' logs

    if (RxData[0] == CAN_ACB_TSA_ACK) {
        notify_startup_task(ACB_TSA_ACK);
    } else if (RxData[0] == CAN_ACB_TSA_NACK) {
        notify_startup_task(ACB_TSA_NACK);
    } else if (RxData[0] == CAN_ACB_RTD_ACK) {
        notify_startup_task(ACB_RTD_ACK);
    } else if (RxData[0] == CAN_ACB_RTD_NACK) {
        notify_startup_task(ACB_RTD_NACK);
    } else if (RxData[0] == CAN_GO_IDLE_REQ) {
        notify_startup_task(GO_IDLE_REQ_FROM_ACB);
    } else if (RxData[0] == CAN_NO_SAFETY_LOOP_SET) {
        btLogIndicator(true, SAFETY_LOOP);
        led_mgmt_set_error(DASH_SAFETY_LOOP_OPEN_ACB);
    } else if (RxData[0] == CAN_NO_SAFETY_LOOP_CLEAR) {
        btLogIndicator(false, SAFETY_LOOP);
        led_mgmt_clear_error(DASH_SAFETY_LOOP_OPEN_ACB);
    } else if (RxData[0] == CAN_AIR_WELD_SET) {
        led_mgmt_set_error(DASH_AIR_WELD);
    } else if (RxData[0] == CAN_HEARTBEAT_REQUEST) {
        // Do nothing
    } else if (RxData[0] == CAN_HEARTBEAT_RESPONSE) {
        notify_heartbeat_task(HEARTBEAT_RESPONSE_NOTIFY);
    } else if (RxData[0] == CAN_BATTERY_VOLTAGE_REQUEST) {
        //lv_battery_handle_voltage_request(); from 2018 car
    } else {
        //build up string for marking unexpected can message
        sprintf(strBuff, "Unexpected ACB can notification: %s\r\n", RxData);
        logMessage(strBuff, false);
    }
}

void set_ACB_State(enum CAR_STATE new_state){
	uint8_t data = (uint8_t) new_state;
	sendCan(&hcan1, &data, 1, CAN_VCU_SET_ACB_STATE_ID, CAN_RTR_DATA, CAN_NO_EXT);
	//wait for acknowledge?
}

void send_ACB_mesg(enum ACB_TO_CAN_MSG msg){
	uint8_t data = (uint8_t)msg;
	sendCan(&hcan1, &data, 1, CAN_VCU_TO_ACU_ID, CAN_RTR_DATA, CAN_NO_EXT);
}

void send_ACB_mesg_data(enum ACB_TO_CAN_MSG msg_id, uint8_t data_len, uint8_t * msg_data){
	uint8_t data[8] = {0};
	data[0] = msg_id;
	memcpy(&data[1], msg_data, data_len);
	sendCan(&hcan1, data, data_len+1, CAN_ACU_CAN_ID, CAN_RTR_DATA, CAN_NO_EXT);
}

void notify_startup_task(enum startup_notify_value notify_val){
	TaskHandle_t startupTask = NULL;
	startupTask = get_startup_task();
	if(startupTask != NULL){
		xTaskNotify( startupTask, notify_val, eSetValueWithOverwrite);
	}
}

void notify_heartbeat_task(HeartbeatNotify_t notify_val){
	osThreadId_t task = NULL;
	task = heartbeat_ACU_get_task();
	if(task != NULL){
		xTaskNotify( task, notify_val, eSetValueWithOverwrite);
	}
}
