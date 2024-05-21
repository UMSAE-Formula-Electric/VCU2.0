
/*
 * pedal_encoder.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Martin Rickey
 *
 *      This module holds generic functions for holding the state of a pedal sensor.
 */

#ifndef PEDAL_ENCODER_H_
#define PEDAL_ENCODER_H_

#include <stdint.h>
#include <stdbool.h>

//#define IMPOS_COUNT_LIMIT (uint16_t)(APPS_REQ_FREQ / 10) //100ms limit max (EV2.3.5), This should be made more modular
#define TR_GROUND 0 // disabled ground check while no mechanical offset
#define HIGH_SENSE_OFFSET 75
#define LOW_SENSE_OFFSET 75


//used for software deadbanding and calcuation of percent difference
//#define PEDAL_THROTTLE_MAX_ENCODER_VAL 0xfff //2845
//#define PEDAL_THROTTLE_MIN_ENCODER_VAL 0 //2050

//used for software deadbanding
//#define PEDAL_BRAKE_MAX_ENCODER_VAL 0xfff //1450
//#define PEDAL_BRAKE_MIN_ENCODER_VAL 0 //1275

//#define PEDAL_BRAKE_PRESS_MIN_ENCODER_VAL 1400

//#define PEDAL_AGREEMENT_PERCENT 0.1 //0.10

//#define TR_THROTTLE_MAX_ERROR ((PEDAL_THROTTLE_MAX_ENCODER_VAL - PEDAL_THROTTLE_MIN_ENCODER_VAL) * PEDAL_AGREEMENT_PERCENT)
//#define TR_BRAKE_MAX_ERROR ((PEDAL_BRAKE_MAX_ENCODER_VAL - PEDAL_BRAKE_MIN_ENCODER_VAL) * PEDAL_AGREEMENT_PERCENT)


//for EV2.4.1 APPS / Brake Pedal Plausibility Check
//#define PEDAL_TWO_FOOT_MAX ((PEDAL_THROTTLE_MAX_ENCODER_VAL - PEDAL_THROTTLE_MIN_ENCODER_VAL) / 4 + PEDAL_THROTTLE_MIN_ENCODER_VAL)		//25%
//#define PEDAL_TWO_FOOT_RELEASE ((PEDAL_THROTTLE_MAX_ENCODER_VAL - PEDAL_THROTTLE_MIN_ENCODER_VAL) / 20 + PEDAL_THROTTLE_MIN_ENCODER_VAL)	//5%

// plausability of pedal state
typedef enum {
	PEDAL_POSSIBLE = 0,
	PEDAL_IMPOSSIBLE
} pb_state_t;

/*
 * this struct contains the state of a single petal
 */
typedef struct{
	pb_state_t possibility;	//indicates if the petal is in a plausable state
	uint16_t gone_count;	//used to handle missing sensor count of how many times throttle wasn't detected in a row
	uint16_t found_Count;	//used to handle missing sensor
	uint16_t impos_count;
	uint16_t possible_count;
	uint16_t impos_limit;	//max number of times a sensor can be polled with impossible values before its considered impossible
	uint16_t low_min;	//min val of low sensor
	uint16_t low_max;	//max val of low sensor
	uint16_t high_min;	//min val of high sensor
	uint16_t high_max;	//max val of high sensor
	uint16_t low_zero;	//offset of zero of low sensor [should be zero *according* to datasheet]
	uint16_t high_zero;	//offset of high sensor
	uint16_t twoFootCount;
	float gain; //gain of sensor
	bool two_foot_flag; //only relevant on apps, indicates if there has been two feet pressed eventS
	char name[10];
}pedal_state_t;


bool rule_10percent_pedal_travel_apps_agreement(uint16_t sens_high, uint16_t sens_low, pedal_state_t * state);
bool sensAgreement_936(uint16_t throttle_1, uint16_t throttle_2, pedal_state_t * state);
bool detectPedal(uint16_t petal_1, uint16_t petal_2, pedal_state_t * state);


#endif /* PEDAL_ENCODER_H_ */

