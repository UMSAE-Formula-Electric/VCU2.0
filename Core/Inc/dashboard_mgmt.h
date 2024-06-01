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
    DASH_NO_STATE = 0,
    DASH_VCU_IDLE_ACU_IDLE,
    DASH_VCU_TSA_ACU_IDLE,
    DASH_VCU_TSA_ACU_TSA,
    DASH_VCU_RTD_ACU_TSA,
    DASH_VCU_RTD_ACU_RTD,
} dash_led_state_t;

void dash_set_state(dash_led_state_t state);
dash_led_state_t dash_get_state();

#endif /* INC_DASHBOARD_MGMT_H_ */
