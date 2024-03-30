#ifndef NEWVCU_CAR_STATE_H
#define NEWVCU_CAR_STATE_H

enum CAR_STATE{
    IDLE = 0,
    TRACTIVE_SYSTEM_ACTIVE,
    READY_TO_DRIVE,
    ERROR_STATE,
    CHARGING_SYSTEM_ACTIVE,
    NUM_CAR_STATES

};

void set_car_state(enum CAR_STATE new_state);
enum CAR_STATE get_car_state();

#endif //NEWVCU_CAR_STATE_H
