#include "../../Firmware-Library/Inc/car_state.h"

static enum CAR_STATE current_state= IDLE;

void set_car_state(enum CAR_STATE new_state){
	if(new_state < NUM_CAR_STATES){
		current_state = new_state;
	}
}

enum CAR_STATE get_car_state(){
	enum CAR_STATE state;
	state = current_state;
	return state;
}
