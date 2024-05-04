/*
 * dashboard_mgmt.h
 *
 *  Created on: Mar 9, 2024
 *      Author: tonyz
 */

#ifndef INC_DASHBOARD_MGMT_H_
#define INC_DASHBOARD_MGMT_H_


#include "main.h"
#include "stdbool.h"
#include "stm32f4xx.h"

typedef enum {
    DASH_NO_ERROR = 0,
    DASH_SAFETY_LOOP_OPEN_ACU,
    DASH_SAFETY_LOOP_OPEN_VCU,
    DASH_NO_BRAKE,
    DASH_NO_THROTTLE,
    DASH_NO_ACU,
    DASH_AIR_WELD,
    DASH_NUM_LED_STATES
} dash_led_state_t;


bool hmi_mgmt_init();

void led_mgmt_set_error(dash_led_state_t state);
void led_mgmt_clear_error(dash_led_state_t state);
bool led_mgmt_check_error(dash_led_state_t state);
bool led_mgmt_init();

#endif /* INC_DASHBOARD_MGMT_H_ */
