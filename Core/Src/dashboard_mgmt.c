/*
 * dashboard_mgmt.c
 *
 *  Created on: Mar 9, 2024
 *      Author: tonyz
 */

#include "dashboard_mgmt.h"
#include "vcu_startup.h"
#include "motor_controller_can_utils.h"
#include "iwdg.h"
#include "dashboard_leds.h"

#define DASH_LED_TASK_DELAY_MS 100

volatile dash_led_state_t dashLedsState = DASH_NO_STATE;

void dash_set_state(dash_led_state_t state){
    dashLedsState = state;
}

dash_led_state_t dash_get_state(){
    return dashLedsState;
}

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

    dash_led_state_t previousState = DASH_NO_STATE;

	for(;;){
        kickWatchdogBit(osThreadGetId());

        if(previousState != dashLedsState){
            // Clear the LEDs
            dash_clear_all_leds();
            previousState = dashLedsState;
        }

        switch(dashLedsState){
            case DASH_VCU_IDLE_ACU_IDLE:
                // VCU and ACU idle: TSA LED flashing cyan
                dash_flash_tsa_teal();
                break;
            case DASH_VCU_TSA_ACU_IDLE:
                // VCU tsa and ACU waiting for ack: TSA LED solid blue
                dash_set_tsa_teal();
                break;
            case DASH_VCU_TSA_ACU_TSA:
                // VCU and ACU tsa: TSA LED solid green and RTD LED flashing cyan
                dash_set_tsa_green();
                dash_flash_rtd_teal();
                break;
            case DASH_VCU_RTD_ACU_TSA:
                // VCU rtd and ACU waiting for ack: TSA LED solid green and RTD solid blue
                dash_set_tsa_green();
                dash_set_rtd_teal();
                break;
            case DASH_VCU_RTD_ACU_RTD:
                // VCU and ACU rta: Both LEDs green
                dash_set_tsa_green();
                dash_set_rtd_green();
                break;
            default:
                break;
        }
        osDelay(pdMS_TO_TICKS(DASH_LED_TASK_DELAY_MS));
    }
}
