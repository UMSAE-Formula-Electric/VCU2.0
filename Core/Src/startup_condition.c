/*
 * startup_condition.c
 *
 *  Created on: Dec 27, 2018
 *      Author: Martin Rickey
 */

#include "startup_condition.h"
#include "logger.h"
#include "iwdg.h"
#include "global_board_config.h"
#include "dashboard_leds.h"

/*
 * checkStartupCondition
 *
 * @Brief This method checks under what condition the processor started and logs it
 * @Note: This function makes logs to the sd card so sdInitialize should already have been called
 */
void checkStartupCondition(){
	//check startup condition
	if(startFromIWDG()){
        sendToUsart("IWDG reset", false);
		vcu_debug_led(REDLED, true);//turn on red debug led
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET){//check for power on reset
        sendToUsart("Power on reset", false);
	}
	else if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET){
        sendToUsart("Pin reset", false);
	}
	else if(__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET){
        sendToUsart("Brown out reset", false);
	}
	else if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET){
        sendToUsart("Software reset", false);
	}
	else if(__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET){
        sendToUsart("Low power reset", false);
	}
	else if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET){
        sendToUsart("Window watchdog reset", false);
	}
	else{
        sendToUsart("Unknown startup condition", false);
	}

	/* Clear reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
