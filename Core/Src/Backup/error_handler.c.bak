#include "error_handler.h"
#include "logger.h"
#include "stdlib.h"
#include "stdint.h"

/*
 * SEE HEADER FILE FOR INSTUCTIONS
 */

#define ERR_MAX_CAN_FAIL 3

static uint8_t can1_fail_count = 0;
static uint8_t can2_fail_count = 0;
static uint8_t mc_rx_fail_count = 0;
static uint8_t q_full_fail_count = 0;
static uint8_t q_empty_fail_count = 0;

void log_and_handle_error(error_case_t error, void (*errorHandlerFcn)(void)){
	if(errorHandlerFcn != NULL){
		errorHandlerFcn();
	}
	switch(error){
	case ERROR_HEARTBEAT_DEAD:
		//set the led state
		//log to the sd card
		logMessage("ERROR: Failed To Recieve Heartbeat", false);
		//setLedState(Pretty colours)
		break;
	case ERROR_AIR_WELD:
		//set the led state
		//log to the sd card
		logMessage("ERROR: AIR WELDED CLOSED!!!", false);
		//setLedState(Sad colours)
		break;
	case ERROR_ACB_SAFETY_LOOP_OPEN:
		//set the led state
		//log to the sd card
		logMessage("ERROR: ACB Saftey Loop Open", false);
		//setLedState(Sad colours)
		break;
	case ERROR_AIR_FAIL_TO_CLOSE:
		logMessage("ERROR: Failed to close AIRS", false);
		//setLedState(Sad colours)
		break;
	case ERROR_PRECHAGE_FAIL:
		logMessage("ERROR: Precharge failed", false);
		//setLedState(Sad colours)
		break;
	case IWDG_ERROR:
		//set the led state
		//log to sd card
		logMessage("ERROR: Init IWDG", false);
		break;
	case BAD_PEDAL_STRUCT:
		//in case a bad pointer is found in a pedal state struct

		logMessage("ERROR: Bad Pedal Struct", false);
		break;
	case ERROR_CAN_ONE_TX_FAIL:
		//set the led state
		//log to the sd card
		if(can1_fail_count < ERR_MAX_CAN_FAIL){
			logMessage("ERROR: CAN 1 TX Fail", false);
			can1_fail_count++;
		}
		break;
	case ERROR_CAN_TWO_TX_FAIL:
		//set the led state
		//log to the sd card
		if(can2_fail_count < ERR_MAX_CAN_FAIL){
			logMessage("ERROR: CAN 2 TX Fail", false);
			can2_fail_count++;
		}
		break;
	case ERROR_MC_CAN_RX_FAIL:
		//set the led state
		//log to the sd card
		if(mc_rx_fail_count < ERR_MAX_CAN_FAIL){
			logMessage("ERROR: MC CAN RX Fail", false);
			mc_rx_fail_count++;
		}
		break;
	case ERROR_Q_FULL:
		//set the led state
		//log to the sd card
		if(q_full_fail_count < ERR_MAX_CAN_FAIL){
			logMessage("ERROR: RTOS Queue Full", false);
			q_full_fail_count++;
		}
		break;
	case ERROR_Q_EMPTY:
			//set the led state
			//log to the sd card
			if(q_empty_fail_count < ERR_MAX_CAN_FAIL){
				logMessage("ERROR: RTOS Queue Empty", false);
				q_empty_fail_count++;
			}
			break;
	default:
		logMessage("Unknown Error", false);
		//setLedState(Other Pretty colours)
	}
}
