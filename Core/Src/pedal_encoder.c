/*
 * pedal_encoder.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Martin Rickey and Austin Shaski
 *
 *This module holds generic functions for processing the pedal encoders.
 */
#include "pedal_encoder.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "logger.h"
#include "error_handler.h"

void bad_pedal_struct_err_handler();

#define PEDAL_AGREEMENT_PERCENT 0.10

/*
 * This function looks to see if the throttle is there. This function
 * only makes sense with the Throttles that have inverted gains and not
 * the half gain style
 *
 *	throttle_1
 */
bool detectPedal(uint16_t petal_1, uint16_t petal_2, pedal_state_t * state) {

	bool hasPetal = false; 	//true if we etect a pedal
	const uint8_t BUFFLEN = 40; //length of buffer
	char buff[BUFFLEN];			//string buffer

	//check that we dont get a bad pointer
	if (state == NULL || state->name == NULL) {
		log_and_handle_error(BAD_PEDAL_STRUCT, &bad_pedal_struct_err_handler);
	}
	else {

		if(petal_1 < 100 && petal_2 < 100) {
			return true;
		}

		if(abs((2 * petal_1) - petal_2) <= (1.2 * petal_1)) {
			state->gone_count = 0;
		} else {
			state->gone_count++;
		}

		if(state->gone_count > 20) {
			return false;
		} else {
			return true;
		}




		hasPetal = true;

		//no longer accurate
		if (state->possibility == PEDAL_POSSIBLE) {
			//checking for impossible state
			if (petal_1 < HIGH_SENSE_OFFSET || petal_2 < LOW_SENSE_OFFSET) {
				state->gone_count++;
			} else {
				state->gone_count = 0;
			}
			if (state->gone_count >= state->impos_limit) {
				hasPetal = false;
				state->gone_count = 0;
				state->found_Count = 0;
				state->possibility = PEDAL_IMPOSSIBLE;
				//handleImpossiblilty(); this should be called somewhere else

//				//check that name exists and no buffer overflow will happen
//				if (state->name != NULL && strlen(state->name) < BUFFLEN - strlen("PEDAL:  Not Detected")) {
//					sprintf(buff, "PEDAL: %s Not Detected", state->name);
//				} else {
//					sprintf(buff, "PEDAL: Not Detected");
//				}
//				logMessage(buff, false);
			}
		}
		else
		{
			//trying to recover from impossible state
			hasPetal = false;
			if (petal_1 >= HIGH_SENSE_OFFSET && petal_2 >= LOW_SENSE_OFFSET) {
				state->found_Count++;
			} else {
				state->found_Count = 0;
			}
			if (state->found_Count >= state->impos_limit) {
				hasPetal = false;
				state->gone_count = 0;
				state->found_Count = 0;
				state->possibility = PEDAL_POSSIBLE;

				//check that name exists and no buffer overflow will happen
				if (state->name != NULL && strlen(state->name) < BUFFLEN - strlen("PETAL:  Not Detected")) {
					sprintf(buff, "PETAL: %s Re-detected", state->name);
				} else {
					sprintf(buff, "PETAL: Re-detected");
				}
				logMessage(buff, false);
			}
		}
	}

	return hasPetal;
}

#ifdef OLD_PEDAL_SENSOR
/*
 * This function checks for implausiblity of the apps or brake petal. This function
 * expects the x pattern for the dual sensors where ones output goes up and the
 * others goes down.
 *
 * returns true if the throttle sensors agree otherwise false
 */
bool throttleAgreement_936(uint16_t throttle_1, uint16_t throttle_2,
		pedal_state_t * state) {


	bool agrees = true;
	uint16_t normalized_throttle_2;
	uint16_t throttle_error_max;
	uint16_t throttle_error_min;

	//static uint16_t impos_count = 0;
	//static uint16_t possible_count = 0;

	if (PEDAL_MAX_ENCODER_VAL < throttle_2) {
		normalized_throttle_2 = 0;
	} else {
		normalized_throttle_2 = PEDAL_MAX_ENCODER_VAL - throttle_2;
	}
	if (throttle_1 < TR_MAX_ERROR) {
		throttle_error_min = 0;
	} else {
		throttle_error_min = throttle_1 - TR_MAX_ERROR;
	}

	//no overflows here unless TR_MAX_ERROR >> whole throttle range
	throttle_error_max = throttle_1 + TR_MAX_ERROR;
	//check overflow anyway	//cant overflow on high end because only using 12 bits of 16bit int, watch lowend

	if (throttle_error_max < throttle_1) {
		//throttle overflow
		logMessage("THROTTLE: Overflow", false);
		return false;
	}
	if (state->possibility == PEDAL_POSSIBLE) {
		if (normalized_throttle_2 >= throttle_error_min && normalized_throttle_2 < throttle_error_max) {
			state->impos_count = 0;
		} else {
			state->impos_count++;
		}
		if (state->impos_count < state->impos_limit) {
			agrees = true;
		} else {
			agrees = false;
			//state->impos_count = 0;
			state->possible_count = 0;
			state->possibility = PEDAL_IMPOSSIBLE;
			//handleImpossiblilty();
			logMessage("THROTTLE: Throttle Disagreement", false);
		}
	} else {
		agrees = false;
		/*
		//try to recover from impossible state, ambiguous in rules
		if (normalized_throttle_2 >= throttle_error_min && normalized_throttle_2 < throttle_error_max) {
			state->possible_count++;
		} else {
			state->possible_count = 0;
		}
		if (state->possible_count < state->impos_limit) {
			agrees = false;
		} else {
			agrees = true;
			state->impos_count = 0;
			state->possible_count = 0;

			state->possibility = PEDAL_POSSIBLE;
			logMessage("THROTTLE: Throttle Sensors have reached an agreement", false);
		}
		*/
	}
	return agrees;
}
#endif



/*
 * This function checks for implausibility of the apps and brake pedal.
 * This function expects two offset slopes for sensor ranges. It checks that these two slopes agree within 10%.
 * Rule T.4.2.4 in 2024 FSAE Rules v1
 *
 * Note these ranges are scaled by a factor of 3.3/5
 *
 * throttle_2: 0.25 - 2.25
 * throttle_1: 0.5 - 4.5
 * returns true if the throttle sensors agree otherwise false
 */
bool rule_10percent_pedal_travel_apps_agreement(uint16_t sens_high, uint16_t sens_low, pedal_state_t * state)
{
    uint16_t sens_high_zero_offset = state->high_zero;
    uint16_t sens_low_zero_offset = state->low_zero;

    uint16_t sens_high_range = state->high_max - sens_high_zero_offset;
    uint16_t sens_low_range = state->low_max - sens_low_zero_offset;

    int32_t normalized_sens_1 = (sens_high - sens_high_zero_offset) * sens_low_range;
    int32_t normalized_sens_2 = (int32_t) ((state->gain) * ((float) ((sens_low - sens_low_zero_offset) * sens_high_range)));
    int32_t agreement_range_size = (int32_t) ((state->gain) * ((float) (sens_high_range * sens_low_range)) * PEDAL_AGREEMENT_PERCENT);

    bool within_range = abs(normalized_sens_1 - normalized_sens_2) < agreement_range_size;

    if (state->possibility == PEDAL_POSSIBLE)
    {
        state->impos_count = within_range ? 0 : state->impos_count + 1;
        if(state->impos_count >= state->impos_limit){
            state->possibility = PEDAL_IMPOSSIBLE;
            logMessage("APPS: Sensor Disagreement", false);
        }
        else {
            within_range = true;
        }
    }
    else
    {
        state->possible_count = within_range ? state->possible_count + 1 : 0;
        if (state->possible_count >= state->impos_limit) {
            state->impos_count = 0;
            state->possible_count = 0;
            state->possibility = PEDAL_POSSIBLE;
            logMessage("APPS: Sensors have reached an agreement", false);
        }
        else {
            within_range = false;
        }
    }

    return within_range;
}

/*
 * Error handler for bad pedal struct
 */
void bad_pedal_struct_err_handler() {
	//do nothing :(
    //TODO VCU#32 ERROR Bad pedal struct thing
}
