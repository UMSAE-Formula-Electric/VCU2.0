#include <dashboard_mgmt.h>
#include "heartbeat.h"
#include "FreeRTOS.h"
#include "task.h"
#include "logger.h"
#include "ACB_comms_handler.h"
#include "freertos_task_handles.h"
#include "iwdg.h"
#include "bt_protocol.h"

#define HEARTBEAT_TASK_DELAY_MS     100
#define HEARTBEAT_MAX_MISSES		50
//Max number of times we can miss a heartbeat notification

//file level variables
static HeartbeatState_t acu_connection_state = HEARTBEAT_NONE;
static HeartbeatState_t mc_connection_state;

void updateAcuStateLedsAndIndicators() {
    if(acu_connection_state == HEARTBEAT_PRESENT){
      //heartbeat all good
        btLogIndicator(false, NO_ACB);
        led_mgmt_clear_error(DASH_NO_ACB);
    }
    else{
      //heartbeat sadness
        btLogIndicator(true, NO_ACB);
        led_mgmt_set_error(DASH_NO_ACB);
    }
}

/*
 * heartbeat_master_task
 *
 * @Brief: This task is used to check if the ACU is reachable as well as send
 * heartbeat messages to the ACU
 */
void StartAcuHeartbeatTask(void *argument){
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

	BaseType_t retRTOS;
    HeartbeatNotify_t acuNotification = 0;
	uint8_t misses = 0; //indicates how many cycles we have gone without detecting ACU

	for(;;){
        kickWatchdogBit(osThreadGetId());

		//send heartbeat message to ACU
        send_ACU_mesg(CAN_HEARTBEAT_REQUEST);
        logMessage("Heartbeat: Sent heartbeat request to ACU\r\n", true);

		//Check if ACU has sent a message
		retRTOS = xTaskNotifyWait(0x00, 0x00, (uint32_t*) &acuNotification, pdMS_TO_TICKS(HEARTBEAT_TASK_DELAY_MS));

		//check if the ACU responded
		if(retRTOS == pdTRUE && acuNotification == HEARTBEAT_RESPONSE_NOTIFY){
            // Received notification from ACU
            misses = 0; // Reset misses counter
            logMessage(acu_connection_state == HEARTBEAT_LOST ? "Heartbeat: ACU re-connection\r\n" : "Heartbeat: Heartbeat received from the ACU\r\n", true);
            acu_connection_state = HEARTBEAT_PRESENT; // Set state
		}
		else{
            // Did not receive notification from ACU
            if(++misses > HEARTBEAT_MAX_MISSES){
                // Lost ACU
                logMessage(acu_connection_state == HEARTBEAT_PRESENT ? "Heartbeat: Lost Connection with ACU\r\n" : "Heartbeat: Could not connect with ACU\r\n", true);
                acu_connection_state = HEARTBEAT_LOST;
            }
		}

		//do dash leds and indicators
        updateAcuStateLedsAndIndicators();

        osThreadYield();
	}
}

/*
 * heartbeat_master_task
 *
 * @Brief: This task is used to check if the Motor Controller is reachable as well as send
 * heartbeat messages to the Motor Controller
 */
void StartMcHeartbeatTask(void *argument){
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

  BaseType_t retRTOS;
  uint32_t ulNotifiedValue = 0;
  mc_connection_state = HEARTBEAT_NONE;
  uint8_t misses = 0; //indicates how many cycles we have gone without detecting ACU

  for(;;){
      kickWatchdogBit(osThreadGetId());

    //Check if MC has sent a message
    retRTOS = xTaskNotifyWait(0x00,0x00, &ulNotifiedValue, HEARTBEAT_TASK_DELAY_MS);
    if(retRTOS == pdPASS){
      misses = 0;

      //log connection
      if(mc_connection_state == HEARTBEAT_NONE){
        logMessage("HEARTBEAT: Connected with MC", false);
      }
      else if(mc_connection_state == HEARTBEAT_LOST){
        logMessage("HEARTBEAT: Reconnected with MC", false);
      }

      //set connection state
      mc_connection_state = HEARTBEAT_PRESENT;

    }
    else{
      if(misses < HEARTBEAT_MAX_MISSES){
        misses++;
      }
      else if(misses >= HEARTBEAT_MAX_MISSES){

        //log loss of mc if was just lost
        if(mc_connection_state == HEARTBEAT_PRESENT){
          logMessage("HEARTBEAT: LOST MC", false);
        }


        if(mc_connection_state == HEARTBEAT_NONE){
          mc_connection_state = HEARTBEAT_NONE; //still haven't connected yet
        }
        else{
          //was previously connected to MC
          mc_connection_state = HEARTBEAT_LOST;
        }
      }


    }

      osDelay(IWDG_RELOAD_PERIOD / 2);                                   // Delay for half IWDG_RELOAD_PERIOD
  }
}

/*
 * get_acu_hearbeat_State
 *
 * @Brief: This method is used to get the current state of heartbeat
 */
HeartbeatState_t get_acu_heartbeat_state(){
	return acu_connection_state;
}

/*
 * get_mc_hearbeat_State
 *
 * @Brief: This method is used to get the current state of heartbeat
 */
HeartbeatState_t get_mc_heartbeat_state(){
  return mc_connection_state;
}

osThreadId_t get_acu_heartbeat_task_handle(){
    return acuHrtbeatTaskHandle;
}

osThreadId_t get_mc_heartbeat_task_handle(){
    return mcHrtbeatTaskHandle;
}

