#ifndef APPS_BREAK_H
#define APPS_BREAK_H


#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"

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

#define APPS_LOW_ZERO	15
#define APPS_LOW_MIN	37
#define APPS_LOW_MAX	280

#define APPS_HIGH_ZERO	18
#define APPS_HIGH_MIN	122
#define APPS_HIGH_MAX	480

// Brake characteristics
#define BRAKE_GAIN		1.74

#define BRAKE_LOW_ZERO	0
#define BRAKE_LOW_MIN	125
#define BRAKE_LOW_MAX	330

#define BRAKE_HIGH_ZERO	0
#define BRAKE_HIGH_MIN	259
#define BRAKE_HIGH_MAX	574

enum BRAKE_STATE{
    BRAKE_RELEASED = GPIO_PIN_SET,
    BRAKE_PRESSED = GPIO_PIN_RESET,
};

//#define TR_BREAK_PRESSED_LIMIT 1350 //was 2000
//*******************
#define LOW_TORQUE_DIV 4
//*******************

bool checkPedalsImplausibility(uint16_t high_val, uint16_t low_val);
void determineError(uint16_t high_val, uint16_t low_val, uint16_t brake_val);

bool brakePressed();
bool detectBrake();
#endif
