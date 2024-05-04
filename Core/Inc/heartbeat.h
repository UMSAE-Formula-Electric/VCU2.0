#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "heartbeat_utils.h"

bool heartbeat_init();
osThreadId_t get_acu_heartbeat_task_handle();
osThreadId_t get_mc_heartbeat_task_handle();
HeartbeatState_t get_acu_heartbeat_state();
HeartbeatState_t get_mc_heartbeat_state();

#endif
