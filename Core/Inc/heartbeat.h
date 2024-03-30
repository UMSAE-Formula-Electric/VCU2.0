#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

typedef enum {
	HEARTBEAT_REQUEST_NOTIFY = 0,
	HEARTBEAT_RESPONSE_NOTIFY
} heatbeat_notif_vals_t;

typedef enum {
	HEARTBEAT_NONE = 0, //have not received anything from ACB/MC yet
	HEARTBEAT_LOST, //similar to none except that we previously had acb/mc signal and lost it
	HEARTBEAT_PRESENT
} heatbeat_state_t;

bool heartbeat_init();
osThreadId_t heartbeat_ACU_get_task();
osThreadId_t heartbeat_MC_get_task();
heatbeat_state_t get_acu_heartbeat_State();
heatbeat_state_t get_mc_heartbeat_State();
extern osThreadId_t mcHrtbeatTaskHandle;
extern osThreadId_t acuHrtbeatTaskHandle;

#endif
