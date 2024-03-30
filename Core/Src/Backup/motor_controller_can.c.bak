/*
 * Created on Jan 14 2019
 * Created by Martin Rickey
 *
 */

#include "motor_controller_can.h"
#include "can.h"
#include "logger.h"
#include "error_handler.h"
#include "global_board_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "heartbeat.h"

//busvoltage at 333V, 90% of full bus voltage is ~300
//meausred using bench top power supply powered to 270V, then added 100 margin
#define BUS_VOLTAGE_90_PER_LIMIT 3200 //[11000] reduced, was 12600
#define TR_MAX_RPM 6000

#ifdef VCU
static int8_t mc_direction = 1;
static int8_t mc_enable_inverter = 0;
static int8_t mc_enable_discharge = 0;
static int16_t mc_torque_limit = 0;
static int16_t mc_torque = 0;
static int16_t mc_speed = 0;
#endif

static int16_t mc_igbtA_temp = 0;
static int16_t mc_igbtB_temp = 0;
static int16_t mc_igbtC_temp = 0;
static int16_t mc_motor_temp = 0;

#ifdef ACB
static int16_t bus_voltage = 0;
static int16_t bus_current = 0;
static int16_t mc_currentA = 0;
static int16_t mc_currentB = 0;
static int16_t mc_currentC = 0;
static int16_t mc_rpm = 0;
static int16_t mc_output_voltage = 0;
static int16_t mc_torque_command = 0;
static int16_t mc_torque_feedback = 0;
static int16_t mc_vd = 0;
static int16_t mc_vq = 0;
#endif

#ifdef VCU
// state machine for motor controller
static mc_state_t motor_controller_state = MC_DISABLED;

static void update_heartbeat();

#endif

static void enableRegReading(uint8_t reg, uint8_t freq);

#ifdef VCU //only VCU keeps track of mc state machine
/*
 * @brief  Disabled the motor controller when there is not throttle input
 *		  re-enables when there is throttle input. Saves power, the documentation tells us to (E-DS-NDrive.pdf pg23 freecoasting on)
 *		  and it reduces spaceship noises
 * @param  mc_trottle_val: the current throittle value to send to the MC (adter processing!!!!)
 * @retval none
 */
void UpdateMCState(int16_t mc_trottle_val) {
	switch (motor_controller_state) {
	case MC_DISABLED:
		if (mc_trottle_val < 0) {
			//neg means go
			EnableMC();
			motor_controller_state = MC_ENABLED;
			logMessage("MC: Enable MC\n", false);
		}
		break;
	case MC_ENABLED:
		if (mc_trottle_val == 0) {
			//soft disable MC to coast
			DisableMC();
			motor_controller_state = MC_DISABLED;
			logMessage("MC: Disable MC\n", false);
		}
	}
}
#endif

/*
 * GETTERS
 */

float mc_getAverageIGBTTemp() {
	return (mc_igbtA_temp + mc_igbtB_temp + mc_igbtC_temp) / 30;
}

float mc_getIGBTATemp() {
	return mc_igbtA_temp / 10;
}

float mc_getIGBTBTemp() {
	return mc_igbtA_temp / 10;
}

float mc_getIGBTCTemp() {
	return mc_igbtA_temp / 10;
}

float mc_getMotorTemp() {
	return mc_motor_temp / 10;
}

float mc_getBusVoltage() {
	return bus_voltage / 10;
}

float mc_getBusCurrent() {
	return bus_current * 10;
}

float mc_getIGBTACurrent() {
	return mc_currentA / 10;
}

float mc_getIGBTBCurrent() {
	return mc_currentB / 10;
}

float mc_getIGBTCCurrent() {
	return mc_currentC / 10;
}

int mc_getRPM() {
	return mc_rpm;
}

float mc_getOutputVoltage() {
	return mc_output_voltage / 10;
}

float mc_getCommandedTorque() {
	return mc_torque_command / 10;
}

float mc_getFeedbackTorque() {
	return mc_torque_feedback / 10;
}

float mc_get_vd() {
	return mc_vd / 10;
}

float mc_get_vq() {
	return mc_vq / 10;
}

/**
 * SETTERS
 */

#ifdef VCU
/**
 * @brief Function to set torque limit.
 * @param setTorque: Value to set torque limit to in N-m, must be between -3276.8 to +3276.7, set to 0 to use EEPROM setting
 * Send value times 10 per documentation
 */
void mc_set_torque_limit(int setTorque) {
	mc_torque_limit = (int16_t) (setTorque * 10);
	mc_send_command_msg(TORQUE_MODE);
}

/**
 * @brief Function to set torque.
 * @param setTorque: Value to set torque to in N-m, must be between -3276.8 to +3276.7 (Send value times 10)
 * Send value times 10 per documentation
 */
void mc_set_torque(int setTorque) {
	mc_torque = (int16_t) (setTorque * 10);
	mc_send_command_msg(TORQUE_MODE);
}

/**
 * @brief Function to set speed.
 * @param setTorque: Value to set torque to in RPM, must be between -32768 to +32767
 */
void mc_set_speed(int setSpeed) {
	mc_speed = (int16_t) (setSpeed);
}

/**
 * @brief Function to set direction.
 * @param setDirection: 0x00 is "reverse", 0x01 is "forward"
 */
void mc_set_direction(uint8_t setDirection) {
	mc_direction = setDirection;
}

/**
 * @brief Function to enable or disable inverter.
 * @param setEnable: 0x00 is disabled, 0x01 is enabled
 */
void mc_set_inverter_enable(uint8_t setEnable) {
	mc_enable_inverter = setEnable;
}

/**
 * @brief Function to enable or disable inverter discharge.
 * @param setEnable: 0x00 is disabled, 0x01 is enabled
 */
void mc_set_inverter_discharge(uint8_t setEnable) {
	mc_enable_discharge = setEnable;
}
#endif

/*
 * isMCBusCharged
 *
 * @Brief: This function checks if the MC dc bus is charged. It is intended
 * to be used as a check before closing the AIRs
 *
 * @Return BUS_CHARGED or BUS_DISCHARGED
 */
uint8_t isMCBusCharged() {
	uint8_t isCharged = BUS_DISCHARGED;
	if (bus_voltage > BUS_VOLTAGE_90_PER_LIMIT) {
		isCharged = BUS_CHARGED;
	}
	return isCharged;
}

/**
 * PACKET PROCESSING
 */

void mc_process_temp1_can(uint8_t * data) {
	mc_igbtA_temp = (data[1] << 8) | data[0];
	mc_igbtB_temp = (data[3] << 8) | data[2];
	mc_igbtC_temp = (data[5] << 8) | data[4];
}

void mc_process_temp3_can(uint8_t * data) {
	mc_motor_temp = (data[5] << 8) | data[4];
}

void fixFaults() {
	uint8_t len = 8;
	uint8_t data[len];
	uint8_t dest = 0xC1;

	uint8_t ret = 0;

	data[0] = 20;
	data[1] = 0;
	data[2] = 1;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;

	ret = sendCan(CAN1, data, len, dest, CAN_NO_RTR, CAN_NO_EXT);
	if (ret != 0) {
		//can error, log it
		log_and_handle_error(ERROR_CAN_ONE_TX_FAIL, NULL);
		logMessage("MC: Failed to send MC command CAN packet\n", false); //should be critical??
	}
}

void mc_process_faults(uint8_t * inData) {
	if((inData[7] & 64) || 1) {
		//resolver fault
		DisableMC();
		sendTorque(0);
		uint8_t len = 8;
		uint8_t data[len];
		uint8_t dest = 0xC1;

		uint8_t ret = 0;

		data[0] = 20;
		data[1] = 0;
		data[2] = 1;
		data[3] = 0;
		data[4] = 0;
		data[5] = 0;
		data[6] = 0;
		data[7] = 0;

		ret = sendCan(CAN1, data, len, dest, CAN_NO_RTR, CAN_NO_EXT);
		if (ret != 0) {
			//can error, log it
			log_and_handle_error(ERROR_CAN_ONE_TX_FAIL, NULL);
			logMessage("MC: Failed to send MC command CAN packet\n", false); //should be critical??
		}
		sendTorque(0);
		vTaskDelay(pdMS_TO_TICKS(100));
		EnableMC();
		//sendTorque(0);
		return;

	}
}

void mc_process_fast_can(uint8_t * data) {
#ifdef VCU
	update_heartbeat();
#endif
	mc_torque_command = (data[1] << 8) | data[0];
	mc_torque_feedback = (data[3] << 8) | data[2];
	mc_rpm = (data[5] << 8) | data[4];
	bus_voltage = (data[7] << 8) | data[6];
	logSensor((float) bus_voltage / 10, MC_BUS_VOLTAGE_LOG);
	logSensor((float) (mc_rpm * 117.97) / 5500, MC_ACUAL_SPEED_REG_LOG);
}

#ifdef ACB


void mc_process_volt_can(uint8_t * data) {
	bus_voltage = (data[1] << 8) | data[0];
	mc_output_voltage = (data[3] << 8) | data[2];
	mc_vd = (data[5] << 8) | data[4];
	mc_vq = (data[7] << 8) | data[6];
}

void mc_process_motor_can(uint8_t * data) {
	mc_rpm = (data[3] << 8) | data[2];
}

#endif

void mc_process_current_can(uint8_t * data) {
	mc_currentA = (data[1] << 8) | data[0];
	mc_currentB = (data[3] << 8) | data[2];
	mc_currentC = (data[5] << 8) | data[4];
	bus_current = (data[7] << 8) | data[6];
	logSensor((float) bus_current, MC_I_ACTUAL_LOG);
}



#ifdef VCU
/**
 *	Sent whenever a torque request is read by the APPS
 */
void mc_send_command_msg(uint8_t mode) {
	uint8_t len = 8; // DLC MUST be 8 for command message, this is the sendCan bug
	uint8_t data[len];
	uint8_t dest = MC_COMMAND_MSG;

	uint8_t ret = 0;

	//Torque Mode
	if (mode == TORQUE_MODE) {
		data[0] = mc_torque & 0xFF;
		data[1] = (mc_torque >> 8) & 0xFF;
		data[2] = 0x00;
		data[3] = 0x00;
		data[4] = mc_direction;
		data[5] = (mc_enable_inverter);
		data[6] = 0x00;
		data[7] = 0x00;
	}
	//Speed Mode, Set 0x04 bit in data[5] to denote.
	//Probably not used in epbr22
	else if (mode == SPEED_MODE) {
		data[0] = 0x00;
		data[1] = 0x00;
		data[2] = mc_speed & 0xFF;
		data[3] = (mc_speed >> 8) & 0xFF;
		data[4] = mc_direction;
		data[5] = (mc_enable_inverter) | (mc_enable_discharge << 1) | 0x04;
		data[6] = mc_torque_limit & 0xFF;
		data[7] = (mc_torque_limit >> 8) & 0xFF;
	}

	ret = sendCan(CAN1, data, len, dest, CAN_NO_RTR, CAN_NO_EXT);
	if (ret != 0) {
		//can error, log it
		log_and_handle_error(ERROR_CAN_ONE_TX_FAIL, NULL);
		logMessage("MC: Failed to send MC command CAN packet\n", false); //should be critical??
	}
	return;

}

void sendTorque(int16_t torque) {
	uint8_t len = 8; // DLC MUST be 8 for command message, this is the sendCan bug
		uint8_t data[len];
		uint8_t dest = 0xC0;

		uint8_t ret = 0;

		data[0] = torque & 0xFF;
		data[1] = (torque >> 8) & 0xFF;
		data[2] = 0x00;
		data[3] = 0x00;
		data[4] = mc_direction;
		data[5] = mc_enable_inverter;
		data[6] = 0x00;
		data[7] = 0x00;

		ret = sendCan(CAN1, data, len, dest, CAN_NO_RTR, CAN_NO_EXT);
		if (ret != 0) {
			//can error, log it
			log_and_handle_error(ERROR_CAN_ONE_TX_FAIL, NULL);
			logMessage("MC: Failed to send MC command CAN packet\n", false); //should be critical??
		}
		return;
}

/**
 * @brief A method for sending a parameter command message to the motor controller
 *
 * @param param_address: the address for the specific kind of command message (Page 39)
 * @param RW: Read/Write. 0x1 -> write, 0x0 -> read.
 * @param Data: data to be sent, length must be 2 bytes. Data[0] goes to byte 4, Data[1] goes to byte 5.
 */
void mc_send_param_command_message(uint8_t param_address, uint8_t RW,
		uint8_t * Data) {
	uint8_t len = 8;
	uint8_t data[8];
	uint8_t dest = MC_PARAM_COMMAND_MSG;
	uint8_t ret = 0;

	//Referencing page 19, disabling the periodic can messages we want
	data[0] = param_address;
	data[1] = 0x00;
	data[2] = RW;
	data[3] = 0x00; 	//Reserved
	data[4] = Data[1];
	data[5] = Data[2];
	data[6] = 0x00;		//Reserved
	data[7] = 0x00;		//Reserved

	//ret = sendCan(CAN1, data, len, dest, CAN_NO_RTR, CAN_NO_EXT);
	//if (ret != 0) {
		//can error, log it

		//log_and_handle_error(ERROR_CAN_ONE_TX_FAIL, NULL);
		//logMessage("MC: Failed to send parameter MC CAN packet\n", false);
	//}
}

/**
 * PARAM COMMAND MESSAGES
 */

void sendZeroTorque() {
	mc_set_torque(0);
}

void mc_enable_broadcast_msgs() {
	uint8_t data[2];
	data[0] = MC_ENABLE_BYTE_4;
	data[1] = MC_ENABLE_BYTE_5;
	mc_send_param_command_message(CAN_MC_ACTIVE_MESSAGES, MC_COMMAND_WRITE,
			data);
}

void mc_disable_broadcast_msgs() {
	uint8_t data[2];

	data[0] = 0x0;
	data[1] = 0x0;

	mc_send_param_command_message(CAN_MC_ACTIVE_MESSAGES, MC_COMMAND_WRITE,
			data);
}

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

void EnableMC() {
	mc_enable_inverter = 1;
}

void DisableMC() {
	mc_enable_inverter = 0;
}

#endif
