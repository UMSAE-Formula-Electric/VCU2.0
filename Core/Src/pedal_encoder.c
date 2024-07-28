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

#include "logger.h"

#define PEDAL_AGREEMENT_PERCENT 0.10

/*
 * pedalValid
 *
 * PURPOSE: determine if the pedal is in a valid range
 *
 * INPUTS:
 * 		sens_high:	Pedal sensor high gain output
 * 		sens_low:	Pedal sensor low gain output
 * 		state:		APPS pedal state struct
 *
 * 	RETURNS:
 * 		boolean value indicating whether the pedal is in a valid state
 */
bool pedalValid(uint16_t sens_high, uint16_t sens_low, pedal_state_t * state) {
	bool hasPedals = false; 	//true if we etect a pedal
    if((sens_high >= state->high_min && sens_low >= state->low_min)
       && (sens_high <= state->high_max && sens_low <= state->low_max)) { hasPedals = true; }
	return hasPedals;
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
 * sens_high: 	high gain petal sensor ADC reading
 * sens_low: 	low gain petal sensor ADC reading
 *
 * RETURN true if the throttle sensors agree within 10% otherwise false
 */
bool rule_10percent_pedal_travel_apps_agreement(uint16_t sens_high, uint16_t sens_low, pedal_state_t * state)
{
    // Calculate the expected sens_high value from sens_low
    double expected_sens_high = (1.8759f * (float)sens_low) - 192.03f;

    // Calculate the 10% range for the expected sens_high value
    double agreement_range_min = expected_sens_high * (1.0f - PEDAL_AGREEMENT_PERCENT);
    double agreement_range_max = expected_sens_high * (1.0f + PEDAL_AGREEMENT_PERCENT);

    // Check if the actual sens_high value falls within this range
    bool within_range = (sens_high >= agreement_range_min) && (sens_high <= agreement_range_max);

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
