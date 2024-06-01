/*
 * dashboard_mgmt.c
 *
 *  Created on: Mar 9, 2024
 *      Author: tonyz
 */

#include "dashboard_mgmt.h"
#include "lv_battery_tap.h"
#include "logger.h"
#include "vcu_startup.h"
#include "motor_controller_can_utils.h"
#include "iwdg.h"
#include "bt_protocol.h"

static bool dash_state_flags[DASH_NUM_LED_STATES] = {false};


void hmi_mgmt_task(void * pvParameters);

#define DASH_LED_TASK_DELAY_MS 100

void led_mgmt_clear_error(dash_led_state_t state){
	dash_state_flags[state] = false;
}

bool led_mgmt_check_error(dash_led_state_t state){
	return dash_state_flags[state];
}

//TODO CleanUp
/**
  * @brief  Starts the can message recieve and processing task
  * @retval 0 on success, 1 if failed to start tasks(probably out of rtos heap mem.)
  */
//bool led_mgmt_init() {
//	bool created = false;
//	//initalize leds in pushbuttons
//	dash_leds_init();
//
//	created = TaskManagerCreate(&hmi_mgmt_task, &xTask_LED_MGMT);
//	return created;
//}

/*
 * led_mgmt_task
 *
 * @Bried: This task manages the leds on the TSA and RTD buttons. Certain colours and flashing
 * are associated with different faults
 */
void StartDashboardLedTask(void *argument){
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

	dash_led_state_t prev_state = DASH_NO_ERROR;
	dash_led_state_t cur_state = DASH_NO_ERROR;

	for(;;){
        kickWatchdogBit(osThreadGetId());

		cur_state = DASH_NO_ERROR;
		//determine highest priority error
		for(uint8_t i = 0; i < DASH_NUM_LED_STATES; i++){
			if(true == dash_state_flags[i]){
				cur_state = i;
			}
		}

		//check battery voltage
		if(lv_battery_voltage() < 13){
			//low battery situation
            btLogIndicator(true, LOW_BATTERY);
			//go_idle();
		}
		else{
            btLogIndicator(false, LOW_BATTERY);
		}

		//check saftey loop
		if(read_saftey_loop() && !dash_state_flags[DASH_SAFETY_LOOP_OPEN_ACU] ){
			//btLogIndicator(false, SAFETY_LOOP);
		}
		else{
			//btLogIndicator(true, SAFETY_LOOP);
		}

		//overheat warnings
		if(mc_getAverageIGBTTemp() > 21000 || mc_getMotorTemp() > 14500){
            btLogIndicator(true, GENERAL_WARNING);
		}
		else{
            btLogIndicator(false, GENERAL_WARNING);
		}

		//overall error
			//actual IGBT overheat,
		if(mc_getAverageIGBTTemp() > 23000){
            btLogIndicator(true, GENERAL_ERROR);
		}
		else{
            btLogIndicator(false, GENERAL_ERROR);
		}
//		if(MC_getIGBTTemp() > 21000 || MC_getMotorTTemp() > 14500){
//			btLogIndicator(true, GENERAL_WARNING);
//		}
//		else{
//			btLogIndicator(false, GENERAL_WARNING);
//		}
//
//		//overall error
//			//actual IGBT overheat,
//		if(MC_getIGBTTemp() > 23000){
//			btLogIndicator(true, GENERAL_ERROR);
//		}
//		else{
//			btLogIndicator(false, GENERAL_ERROR);
//		}
        osDelay(pdMS_TO_TICKS(DASH_LED_TASK_DELAY_MS));
    }
}
