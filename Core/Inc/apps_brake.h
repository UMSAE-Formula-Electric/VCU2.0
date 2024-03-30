#ifndef APPS_BREAK_H
#define APPS_BREAK_H


#include <stdint.h>
#include <stdbool.h>

#define TR_MAX_MC_VALUE 0x7FFF
#define TR_MIN_MC_VALUE 0

#define TR_AVG_FREQ 200

#define TR_SLEEP_DELAY 1000/TR_REQ_FREQ //delay in ms

//wow everyones so salty in makes me sad :(   <-  You are the most saltiest one
#define DEADBAND_OFFSET 100

#define TR_MAX_TORQUE_ARBITRARY (-32000)

#define TR_MIN_TORQUE 0

#define TR_MAX_TORQUE_OUTPUT 240 // Nm]
#define TR_MAX_POWER 80000 //kWatts

//#define TR_BREAK_PRESSED_LIMIT 1350 //was 2000
//*******************
#define LOW_TORQUE_DIV 4
//*******************


void start_brake_apps_tasks();
void apps_brake_init();
uint16_t get_apps();

uint16_t get_brake();
bool get_brake_press();
bool brakePressed();
bool detectBrake();
#endif
