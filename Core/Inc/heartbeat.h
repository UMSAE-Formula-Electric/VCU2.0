#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "heartbeat_utils.h"

bool heartbeat_init();
osThreadId_t heartbeat_ACU_get_task();
osThreadId_t heartbeat_MC_get_task();
HeartbeatState_t get_acu_heartbeat_State();
HeartbeatState_t get_mc_heartbeat_State();

#endif
