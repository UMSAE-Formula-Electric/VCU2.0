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

// APPS characteristics
#define APPS_GAIN		1.70

#define APPS_LOW_ZERO	4
#define APPS_LOW_MIN	4
#define APPS_LOW_MAX	288

#define APPS_HIGH_ZERO	8
#define APPS_HIGH_MIN	8
#define APPS_HIGH_MAX	490

// Brake characteristics
#define BRAKE_GAIN		1.74

#define BRAKE_LOW_ZERO	0
#define BRAKE_LOW_MIN	125
#define BRAKE_LOW_MAX	330

#define BRAKE_HIGH_ZERO	0
#define BRAKE_HIGH_MIN	259
#define BRAKE_HIGH_MAX	574

//#define TR_BREAK_PRESSED_LIMIT 1350 //was 2000
//*******************
#define LOW_TORQUE_DIV 4
//*******************


void start_brake_apps_tasks();
void apps_brake_init();
bool detectImpossibilty(uint16_t high_val, uint16_t low_val, uint16_t brake_val);
void determineError(uint16_t high_val, uint16_t low_val, uint16_t brake_val);
uint16_t get_apps();

uint16_t get_brake();
bool get_brake_press();
bool brakePressed();
bool detectBrake();
#endif
