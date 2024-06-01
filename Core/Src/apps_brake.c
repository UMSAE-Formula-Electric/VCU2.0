/*
 * programmed by Austin Shaski
 * 		edited 2018 Martin Rickey
 *
 * This file contains the code for polling the
 *
 */

#include <dashboard_mgmt.h>
#include "pedal_encoder.h"
#include "apps_brake.h"
#include "motor_controller_can_utils.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "adc.h"
#include "can.h"
#include "logger.h"
#include "iwdg.h"
#include "gpio.h"
#include "car_state.h"
#include <stdlib.h>
#include <string.h>
#include "vcu_startup.h"
#include "global_board_config.h"
#include "bt_protocol.h"

#define APPS_REQ_FREQ 200 //[Hz] frequency of polling loop for APPS
#define BRAKE_REQ_FREQ 100 //[Hz] frequency of polling loop for BRAKE PEDAL
#define MIN_TORQUE_REQUESTABLE 0
#define MAX_TORQUE_REQUESTABLE 2000

#define BYPASS_SAFETY	0
#define BYPASS_BRAKE	1
#define BYPASS_RTD		1

#define APPS_LOW_END 	300
#define APPS_HIGH_END 	600

#define APPS_TWO_FOOT_VAL 		400
#define APPS_TWO_FOOT_RELEASE 	310

#define PEDAL_TWO_FOOT_PERCENT 			0.25	// EV2.4.1 amount you can press apps while brake is depressed before stopping current [rule about two foot driving]
#define PEDAL_TWO_FOOT_RELEASE_PERCENT 	0.05	// Amount to get out of two foot pressed state
#define BRAKE_PRESS_PERCENT 			0.1 	//amount to press the brake before it is considered actuated
#define BRAKE_PRESS_TWOFOOT_PERCENT 	0.20 	//amount to press brake before it is considered pressed for
//THROTTLE VAL 2 IS INVERTED
//static uint32_t Sensor_DMABase[4]; // dereferencing Mem0BasePtr1 will give the value stored at its address at time of dereference!(Shit uses DMA! Whew!)

enum BRAKE_STATE readDigitalBrakeState() { return HAL_GPIO_ReadPin(BPS_GPIO_Port, BPS_Pin); }

//int16_t convertThrottleforMC(uint16_t value, pedal_state_t * state);
void adjust_for_power_limit(uint16_t * throttleRequest);
static void handleImpossiblilty();
void sendTorqueWithFaultFixing(int16_t);
bool twoFootRulePassed(long, pedal_state_t*);
bool EV2_4_check(uint16_t apps1, uint16_t brake1, pedal_state_t * apps_state,
		pedal_state_t * brake_state);

static uint32_t current_max_power = TR_MAX_POWER; //update based on data from AMS

static pedal_state_t brake; //Brake pedal position sensor / brake sensor
static pedal_state_t apps; //Accelerator pedal position sensor / throttle sensor

int16_t mapPedalPressToMotorTorque(uint16_t lowPedalPress) {
    if (lowPedalPress <= APPS_LOW_MIN) {
        return MIN_TORQUE_REQUESTABLE;
    } else if (lowPedalPress >= APPS_LOW_MAX) {
        return MAX_TORQUE_REQUESTABLE;
    }
    double torque = ((double)(lowPedalPress - APPS_LOW_MIN) * (MAX_TORQUE_REQUESTABLE - MIN_TORQUE_REQUESTABLE)) / (APPS_LOW_MAX - APPS_LOW_MIN) + MIN_TORQUE_REQUESTABLE;

    return (int16_t)(torque + 0.5);
}

void InitalizeApps(float gain, uint16_t low_zero, uint16_t low_min, uint16_t low_max,
					uint16_t high_zero, uint16_t high_min, uint16_t high_max){

	//setup apps state
	strcpy(apps.name, "Apps");
	apps.possibility = PEDAL_POSSIBLE;

	apps.gone_count = 0;
	apps.found_Count = 0;
	apps.impos_count = 0;
	apps.possible_count = 0;

	apps.impos_limit = (APPS_REQ_FREQ / 10); //100ms limit max (T.6.2.4) //TODO check

	apps.low_zero 	= low_zero;
	apps.low_min 	= low_min;
	apps.low_max 	= low_max;

	apps.high_zero	= high_zero;
	apps.high_min 	= high_min;
	apps.high_max 	= high_max;

	apps.gain = gain;
}

void InitializeBrake(float gain, uint16_t low_zero, uint16_t low_min, uint16_t low_max,
						uint16_t high_zero, uint16_t high_min, uint16_t high_max){

	//setup brake state
	strcpy(brake.name, "Brake");
	brake.possibility = PEDAL_POSSIBLE;

	apps.gone_count = 0;
	apps.found_Count = 0;
	apps.impos_count = 0;
	apps.possible_count = 0;

	apps.impos_limit = (BRAKE_REQ_FREQ / 10); //100ms limit max (T.6.2.4) //TODO check

	apps.low_zero 	= low_zero;
	apps.low_min 	= low_min;
	apps.low_max 	= low_max;

	apps.high_zero	= high_zero;
	apps.high_min 	= high_min;
	apps.high_max 	= high_max;

	apps.gain = gain;
}

/*
 * updateAPPSVals
 *
 * This is the main task for updating and handling the throttle. This task checks for impossibility of the apps and if the apps is present
 */
void StartAppsProcessTask(void *argument) {
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

	int16_t mc_apps_val;

	uint16_t apps_low = 0;
	uint16_t apps_high = 0;
    enum BRAKE_STATE brake1;

	//setup apps state
	InitalizeApps(APPS_GAIN, APPS_LOW_ZERO, APPS_LOW_MIN, APPS_LOW_MAX,
					APPS_HIGH_ZERO, APPS_HIGH_MIN, APPS_HIGH_MAX);

	for (;;) {
        kickWatchdogBit(osThreadGetId());

		//low pass filters to increase noise rejection
		apps_low = 0.5 * ADC_get_val(ADC_APPS1) + 0.5 * apps_low;
        apps_high = 0.5 * ADC_get_val(ADC_APPS2) + 0.5 * apps_high;
		brake1 = ADC_get_val(ADC_BPS);
        brake1 = readDigitalBrakeState();

		mc_apps_val = mapPedalPressToMotorTorque(apps_low);

		if(detectImpossibilty(apps_high, apps_low, brake1)){
			handleImpossiblilty();
		}
		else{
			sendTorqueWithFaultFixing(mc_apps_val);
		}

		osThreadYield();
	}
}

bool detectImpossibilty(uint16_t high_val, uint16_t low_val, uint16_t brake_val){
	bool res = true;

	if (!pedalValid(high_val, low_val, &apps) && false) {
	    btLogIndicator(true, THROTTLE_ERROR);
	} else {
		if (get_car_state() == READY_TO_DRIVE || BYPASS_RTD) {
			if (BYPASS_SAFETY) {
				res = false;
			} else {
				if (!rule_10percent_pedal_travel_apps_agreement(high_val, low_val, &apps)) {
	                btLogIndicator(true, THROTTLE_ERROR);
				} else {
					if (BYPASS_BRAKE) {
						res = false;
					} else {
						if (detectBrake() && twoFootRulePassed(high_val, &apps)) {
							btLogIndicator(false, THROTTLE_ERROR);
							res = false;
						}
					}
				}
			}
		} else {
			determineError(high_val, low_val, brake_val);
		}
	}

	return res;
}

void determineError(uint16_t high_val, uint16_t low_val, uint16_t brake_val){
	if (pedalValid(high_val, low_val, &apps)) {
		if (detectBrake()) {
			if(twoFootRulePassed(high_val, &apps)) {
				btLogIndicator(false, THROTTLE_ERROR); //all good, just not rtd
			} else {
				btLogIndicator(true, THROTTLE_ERROR); //brake offline
			}
		} else {
			btLogIndicator(true, THROTTLE_ERROR); //two foot fail
		}
	} else {
		btLogIndicator(true, THROTTLE_ERROR); //pedal sensor doesnt agree
	}
}


void sendTorqueWithFaultFixing(int16_t torque) {
	if (torque < 10) {
		DisableMC();
		sendTorque(0);
		fixFaults();
	} else {
		EnableMC();
		mc_send_command_msg(TORQUE_MODE);
		sendTorque(torque);
	}
}

bool twoFootRulePassed(long appsVal, pedal_state_t * pedalState) {
	if (pedalState->two_foot_flag == true) {
		if (brakePressed()) {
			pedalState->twoFootCount = 10;
		} else {
			if (appsVal < APPS_TWO_FOOT_RELEASE) {
				pedalState->twoFootCount--;
			} else {
				pedalState->twoFootCount = 10;
			}
		}
	} else if (appsVal > APPS_TWO_FOOT_VAL && brakePressed()) {
		pedalState->twoFootCount++;
	}

	if (pedalState->twoFootCount >= 10) {
		pedalState->two_foot_flag = true;
	} else if (pedalState->twoFootCount <= 0) {
		pedalState->two_foot_flag = false;
		pedalState->twoFootCount = 0;
	}

	if (pedalState->two_foot_flag == false) {
		return true;
	} else {
		return false;
	}
}

/*
 * updateBrakeVals
 *
 * This task is used for detecting the plausibility of the brake sensor
 */
void StartBrakeProcessTask(void *argument) {
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

	uint16_t brake1 = 0;
	uint16_t brake2 = 0;

	//setup brake state
	InitializeBrake(BRAKE_GAIN, BRAKE_LOW_ZERO, BRAKE_LOW_MIN, BRAKE_LOW_MAX, BRAKE_HIGH_ZERO, BRAKE_HIGH_MIN, BRAKE_HIGH_MAX);

	//task infinite loop
	for (;;) {
        kickWatchdogBit(osThreadGetId());

		//log brake sensors
        btLogSensor(ADC_get_val(ADC_BPS), BRAKE_1);
//		btLogSensor(ADC_get_val(ADC_BRK2), BRAKE_2);

		brake1 = ADC_get_val(ADC_BPS);
//		brake2 = ADC_get_val(ADC_BRK2);

		//kick wathcdog to make sure this doesn't hang
//		wd_criticalTaskKick(wd_BRAKE_CTASK);

		osThreadYield();
//		vTaskDelay(pdMS_TO_TICKS(1000/BRAKE_REQ_FREQ));             //TODO Revise task delay
	}
}

/*
 * This function handles the situation when the throttle is in an impossible state due to be being broken
 *
 */
void handleImpossiblilty() {
	sendTorqueWithFaultFixing(0);
}

//TODO CleanUp
/*
 * This function initializes the tasks for polling the APPS rotary encoder and polling
 * the brake rotary encoder.
 *
 */
//void start_brake_apps_tasks() {
//	bool created = false;
//
//	//create apps polling task
//	created = TaskManagerCreate(&update_apps_vals, &xTask_APPS_PROCESS);
//	if (!created) {
//		//log failed to start task
//		logMessage("APPS: Failed to create APPS_PROC task", false);
//	}
//
//	//create brake polling task
//	created = TaskManagerCreate(&update_brake_vals, &xTask_BRAKE_PROCESS);
//	if (!created) {
//		//log failed to start task
//		logMessage("BRAKE: Failed to create BRAKE_PROC task", false);
//	}
//}

//TODO MATH Check
/*
 * adjust_for_power_limit
 *
 * @Brief: This function adjusts the requested power to make sure that we don't pull more the 80kW from the motor.
 *
 * @Note: irrelevant as our power plant doesn't come close to 80kW with a single motor configuration
 */
//void adjust_for_power_limit(uint16_t * throttleRequest) {
//	float calculatedMaxTorque;
//	if (get_curr_angular_speed() == 0) {
//		//no speed reading or stopped
//		calculatedMaxTorque = TR_MAX_TORQUE_OUTPUT;
//	} else if (current_max_power
//			/ get_curr_angular_speed() > TR_MAX_TORQUE_OUTPUT) {
//		//limited by max torque of output not power
//		calculatedMaxTorque = TR_MAX_TORQUE_OUTPUT;
//
//	} else {
//		//limited by power output
//		calculatedMaxTorque = current_max_power / get_curr_angular_speed();
//	}
//	*throttleRequest = *(throttleRequest)
//			* calculatedMaxTorque/ TR_MAX_TORQUE_OUTPUT;
//}

/*
 * This function is used to get the current apps value
 *
 */
uint16_t get_apps() {
	return ADC_get_val(ADC_APPS1);
}

/*
 * This function is used to get the current break pedal position value
 *
 */
uint16_t get_brake() {
	return ADC_get_val(ADC_BPS);
}

/*
 * This function determines if the break is pressed and possible
 *
 */
bool get_brake_press() {
	bool is_pressed = false;
	if (brake.possibility == PEDAL_POSSIBLE) {
		if (brake.high_min
				+ BRAKE_PRESS_PERCENT * (brake.high_max - brake.high_min)
				< get_brake()) {
			is_pressed = true;
		}
	}
	return is_pressed;
}

bool brakePressed() {
	if (ADC_get_val(ADC_BPS) > 950) {
		return true;
	} else {
		return false;
	}
}

bool detectBrake() {
	long brakeVal = ADC_get_val(ADC_BPS);

	if (brakeVal > 1300 || brakeVal < 600) {
		return false;
	} else {
		return true;
	}
}

/*
 * EV2_4_check
 *
 * @Brief: This function determines if the break and apps are currently violating ev2.4 with both pedals being pressed.
 * Does not return true until throttle has been reduced to safe level
 *
 * @Param apps1 the high sensor from the apps
 * @Param brake1 the high sensor from the brake
 * @Param apps_state: state object for apps
 * @Param brake_state: state object for brake
 *
 *@return: returns true if the driver is not two foot driving
 */
bool EV2_4_check(uint16_t apps1, uint16_t brake1, pedal_state_t * apps_state,
		pedal_state_t * brake_state) {
	bool check_pass = false;

	if (apps1
			> (PEDAL_TWO_FOOT_PERCENT
					* (apps_state->high_max - apps_state->high_min)
					+ apps_state->high_min)
			&& brake1
					> (brake_state->high_min
							+ BRAKE_PRESS_TWOFOOT_PERCENT
									* (brake_state->high_max
											- brake_state->high_min))) {
		sendTorqueWithFaultFixing(0);	//stop sending torque values braking
		apps_state->two_foot_flag = true;
	} else {
		//check to see if driver needs to back off apps due to two foot driving
		if (apps_state->two_foot_flag) {
			//check to see if the apps has been backed off
			if (apps1
					< (PEDAL_TWO_FOOT_RELEASE_PERCENT
							* (apps_state->high_max - apps_state->high_min)
							+ apps_state->high_min)) {
				//the pedal has been released sufficiently
				check_pass = true;
				apps_state->two_foot_flag = false;
			} else
				sendTorqueWithFaultFixing(0);//redundant as the motor controller is already off
		} else {
			//no two foot driving and everything is good send value to motor controller
			check_pass = true;
		}
	}
	return check_pass;
}

