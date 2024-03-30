#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

/*
 * HOW TO UPDATED:
 * 1. Add your new error to ERROR_CASE enum (before NUMBER_OF_ERRORS), keep it organized
 * 2. Add a switch case for you error in log_and_handle_error
 * 3. The errorHandlerFcn will automatically be called before the error is logged
 * NOTE: The errorHandlerFcn must be a fuctions that takes void input and returns void
 * NOTE: Log messages are limited to 100 characters (defined by LOG_MESSAGE_LENGTH in sd.h)
 */


typedef enum ERROR_CASE{
	ERROR_AIR_WELD = 0,
	ERROR_HEARTBEAT_DEAD,
	ERROR_ACB_SAFETY_LOOP_OPEN,
	ERROR_AIR_FAIL_TO_CLOSE,
	ERROR_PRECHAGE_FAIL,
	LE_TEST_ERROR,
	IWDG_ERROR,
	BAD_PEDAL_STRUCT,
	ERROR_CAN_ONE_TX_FAIL,
	ERROR_CAN_TWO_TX_FAIL,
	ERROR_MC_CAN_RX_FAIL,
	ERROR_Q_FULL,
	ERROR_Q_EMPTY,
	//add new errors here
	NUMBER_OF_ERRORS
}error_case_t;

void log_and_handle_error(error_case_t error, void (*errorHandlerFcn)(void));

#endif
