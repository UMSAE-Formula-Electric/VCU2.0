/*
 * programmed by Austin Shaski
 * 		edited 2018 Martin Rickey
 *
 * This file contains the code for polling the
 *
 */

#include "pedal_encoder.h"
#include "apps_brake.h"
#include "motor_controller_can_utils.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "logger.h"
#include "iwdg.h"
#include "car_state.h"
#include "vcu_startup.h"
#include "bt_protocol.h"
#include "heartbeat.h"

#define APPS_REQ_FREQ_HZ 100 //[Hz] frequency of polling loop for APPS
#define BRAKE_REQ_FREQ_HZ 100 //[Hz] frequency of polling loop for BRAKE PEDAL

#define MIN_TORQUE_REQUESTABLE 0
#define MAX_TORQUE_REQUESTABLE 2400

#define BYPASS_SAFETY	0
#define BYPASS_BRAKE	0
#define BYPASS_APPS		0
#define BYPASS_RTD		0

#define PEDAL_TWO_FOOT_PRESS_PERCENT    0.25	// EV2.4.1 amount you can press apps while brake is depressed before stopping current [rule about two foot driving]
#define PEDAL_TWO_FOOT_RELEASE_PERCENT 	0.05	// Amount to get out of two foot pressed state
#define APPS_LOW_RELEASE_PEDAL_TRAVEL ((APPS_LOW_MAX - APPS_LOW_MIN) * PEDAL_TWO_FOOT_RELEASE_PERCENT)
#define APPS_HIGH_RELEASE_PEDAL_TRAVEL ((APPS_HIGH_MAX - APPS_HIGH_MIN) * PEDAL_TWO_FOOT_RELEASE_PERCENT)
#define APPS_LOW_PRESS_PEDAL_TRAVEL ((APPS_LOW_MAX - APPS_LOW_MIN) * PEDAL_TWO_FOOT_PRESS_PERCENT)
#define APPS_HIGH_PRESS_PEDAL_TRAVEL ((APPS_HIGH_MAX - APPS_HIGH_MIN) * PEDAL_TWO_FOOT_PRESS_PERCENT)

enum BRAKE_STATE readDigitalBrakeState() { return (enum BRAKE_STATE) HAL_GPIO_ReadPin(BPS_GPIO_Port, BPS_Pin); }

static uint32_t current_max_power = TR_MAX_POWER; //update based on data from AMS

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

void InitializeApps(float gain, uint16_t low_zero, uint16_t low_min, uint16_t low_max,
                    uint16_t high_zero, uint16_t high_min, uint16_t high_max){

	apps.possibility = PEDAL_POSSIBLE;

	apps.gone_count = 0;
	apps.found_Count = 0;
	apps.impos_count = 0;
	apps.possible_count = 0;

	apps.impos_limit = (APPS_REQ_FREQ_HZ / 10); //100ms limit max (T.6.2.4) //TODO check

	apps.low_zero 	= low_zero;
	apps.low_min 	= low_min;
	apps.low_max 	= low_max;

	apps.high_zero	= high_zero;
	apps.high_min 	= high_min;
	apps.high_max 	= high_max;

    apps.two_foot_high_count = 0;
    apps.two_foot_low_count = 0;
    apps.two_foot_high_flag = false;
    apps.two_foot_low_flag = false;

	apps.gain = gain;
}

bool twoFootRulePassedPerPedal(uint16_t appsVal, uint16_t twoFootPressVal, uint16_t twoFootReleaseVal, uint16_t* twoFootCount, bool* twoFootFlag) {
    if ((*twoFootFlag) == true) {
        if (brakePressed()) {
            *twoFootCount = 10;
        } else {
            if (appsVal < twoFootReleaseVal) {
                (*twoFootCount)--;
            } else {
                *twoFootCount = 10;
            }
        }
    } else if (appsVal > twoFootPressVal && brakePressed()) {
        (*twoFootCount)++;
    }

    *twoFootFlag = ((*twoFootCount) >= 10);
    if ((*twoFootCount) <= 0) {
        *twoFootFlag = false;
        *twoFootCount = 0;
    }

    return !(*twoFootFlag);
}

bool twoFootRulePassed(uint16_t high_val, uint16_t low_val) {
    return twoFootRulePassedPerPedal(high_val, APPS_HIGH_PRESS_PEDAL_TRAVEL, APPS_HIGH_RELEASE_PEDAL_TRAVEL, &apps.two_foot_high_count, &apps.two_foot_high_flag) &&
           twoFootRulePassedPerPedal(low_val, APPS_LOW_PRESS_PEDAL_TRAVEL, APPS_LOW_RELEASE_PEDAL_TRAVEL, &apps.two_foot_low_count, &apps.two_foot_low_flag);
}

void readAccelPedals(uint16_t *apps_low, uint16_t *apps_high) {
    uint16_t temp_apps_low = ADC_get_val(ADC_APPS_LOW);
    uint16_t temp_apps_high = ADC_get_val(ADC_APPS_HIGH);
    if (temp_apps_low != INVALID_ADC_READING) {
        (*apps_low) = (uint16_t) ((0.5 * temp_apps_low) + (0.5 * (*apps_low)));
    }
    if (temp_apps_high != INVALID_ADC_READING) {
        (*apps_high) = (uint16_t) ((0.5 * temp_apps_high) + (0.5 * (*apps_high)));
    }
}

bool checkPedalsImplausibility(uint16_t high_val, uint16_t low_val){
	bool res = true;

    if (BYPASS_RTD || get_car_state() == READY_TO_DRIVE) {
        if (BYPASS_SAFETY || read_saftey_loop()) {
            if (BYPASS_BRAKE || twoFootRulePassed(high_val, low_val)) {
                if (BYPASS_APPS || rule_10percent_pedal_travel_apps_agreement(high_val, low_val, &apps)) {
                    res = false;
                }
                else {
                    btLogIndicator(false, THROTTLE_ERROR);
                }
            } else {
                btLogIndicator(true, THROTTLE_ERROR);
            }
        }
    }

	return res;
}

void sendTorqueWithFaultFixing(int16_t torque) {
    if ((get_mc_heartbeat_state() != HEARTBEAT_PRESENT) ||  (get_car_state() != READY_TO_DRIVE) || brakePressed() || torque < 10) {
        DisableMC();
        sendTorque(0);
        fixFaults();
    } else {
        EnableMC();
        mc_send_command_msg(TORQUE_MODE);
        sendTorque(torque);
    }
}

/*
 * This function handles the situation when the throttle is in an impossible state due to be being broken
 *
 */
void handleImplausibility() {
	sendTorqueWithFaultFixing(0);
}

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

bool brakePressed() {
    return readDigitalBrakeState() == BRAKE_PRESSED;
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
//bool EV2_4_check(uint16_t apps1, uint16_t brake1, pedal_state_t * apps_state,
//		pedal_state_t * brake_state) {
//	bool check_pass = false;
//
//	if (apps1
//			> (PEDAL_TWO_FOOT_PERCENT
//					* (apps_state->high_max - apps_state->high_min)
//					+ apps_state->high_min)
//			&& brake1
//					> (brake_state->high_min
//							+ BRAKE_PRESS_TWOFOOT_PERCENT
//									* (brake_state->high_max
//											- brake_state->high_min))) {
//		sendTorqueWithFaultFixing(0);	//stop sending torque values braking
//		apps_state->two_foot_flag = true;
//	} else {
//		//check to see if driver needs to back off apps due to two foot driving
//		if (apps_state->two_foot_flag) {
//			//check to see if the apps has been backed off
//			if (apps1
//					< (PEDAL_TWO_FOOT_RELEASE_PERCENT
//							* (apps_state->high_max - apps_state->high_min)
//							+ apps_state->high_min)) {
//				//the pedal has been released sufficiently
//				check_pass = true;
//				apps_state->two_foot_flag = false;
//			} else
//				sendTorqueWithFaultFixing(0);//redundant as the motor controller is already off
//		} else {
//			//no two foot driving and everything is good send value to motor controller
//			check_pass = true;
//		}
//	}
//	return check_pass;
//}

/*
 * StartAppsProcessTask
 *
 * This is the main task for updating and handling the throttle. This task checks for impossibility of the apps and if the apps is present
 */
void StartAppsProcessTask(void *argument) {
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadExit();
    }

    int16_t mc_apps_val;
    uint16_t apps_low = 0;
    uint16_t apps_high = 0;

    //setup apps state
    InitializeApps(APPS_GAIN, APPS_LOW_ZERO, APPS_LOW_MIN, APPS_LOW_MAX,
                   APPS_HIGH_ZERO, APPS_HIGH_MIN, APPS_HIGH_MAX);

    for (;;) {
        kickWatchdogBit(osThreadGetId());

        //low pass filters to increase noise rejection
        readAccelPedals(&apps_low, &apps_high);

        mc_apps_val = mapPedalPressToMotorTorque(apps_low);

        if(checkPedalsImplausibility(apps_high, apps_low)){
            handleImplausibility();
        }
        else{
             sendTorqueWithFaultFixing(mc_apps_val);
        }

        osDelay(pdMS_TO_TICKS(1000 / APPS_REQ_FREQ_HZ));
    }
}

/*
 * StartBrakeProcessTask
 *
 * This task is used for detecting the plausibility of the brake sensor
 */
void StartBrakeProcessTask(void *argument) {
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadExit();
    }

    enum BRAKE_STATE brake1;

    for (;;) {
        kickWatchdogBit(osThreadGetId());

        brake1 = readDigitalBrakeState();
        btLogSensor(brake1, BRAKE_1);

        osDelay(pdMS_TO_TICKS(1000 / BRAKE_REQ_FREQ_HZ));
    }
}

