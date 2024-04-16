#include <dashboard_mgmt.h>
#include "heartbeat.h"
#include "FreeRTOS.h"
#include "task.h"
#include "logger.h"
#include "ACB_comms_handler.h"
#include "freertos_task_handles.h"
#include "iwdg.h"

#define HEARTBEAT_TASK_DELAY_MS     100
#define HEARTBEAT_MAX_MISSES		10
//Max number of times we can miss a heartbeat notification

//file level variables
static heatbeat_state_t acb_connection_state;
static heatbeat_state_t mc_connection_state;
void heartbeat_master_task_ACU(void * pvParameters);
void heartbeat_master_task_MC(void * pvParameters);

//TODO CleanUP
//This is now handled by freertos.c
///**
//  * @brief  Starts the heartbeat master task
//  * @retval 0 on success, 1 if failed to start tasks(probably out of rtos heap mem.)
//  */
//bool heartbeat_init() {
//	bool created = false;
//
//	created = TaskManagerCreate(&heartbeat_master_task_ACU, &xTask_Heartbeat_ACU);
//	if(created){
//	  created = TaskManagerCreate(&heartbeat_master_task_MC, &xTask_Heartbeat_MC);
//	}
//
//	if(!created){
//		logMessage("Failed to create heartbeat task", true);
//	}
//
//	return created;
//}

/*
 * heartbeat_master_task
 *
 * @Brief: This task is used to check if the ACB is reachable as well as send
 * heartbeat messages to the ACB
 */
void StartAcuHeartbeatTask(void *argument){
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        return;
    }

	BaseType_t retRTOS;
	uint32_t ulNotifiedValue = 0;
	acb_connection_state = HEARTBEAT_NONE;
	uint8_t misses = 0; //indicates how many cycles we have gone without detecting ACB

	for(;;){
        kickWatchdogBit(osThreadGetId());

		//send heartbeat message to ACB
		send_ACB_mesg(CAN_HEARTBEAT_REQUEST);

		//Check if ACB has sent a message
		retRTOS = xTaskNotifyWait(0x00,0x00, &ulNotifiedValue, HEARTBEAT_TASK_DELAY_MS);

		//check if the ACB responded
		if(retRTOS == pdTRUE && ulNotifiedValue == HEARTBEAT_RESPONSE_NOTIFY){
			//received notification from ACB

			//reset misses counter
			misses = 0;

			//check if we have previously lost the ACB
			if(acb_connection_state == HEARTBEAT_LOST){
				logMessage("Heartbeat: ACB re-connection", true);
			}
			else if(acb_connection_state == HEARTBEAT_NONE){
				//connecting with acb for first time
				logMessage("Heartbeat: Made connection with ACB", true);
			}

			//set state
			acb_connection_state = HEARTBEAT_PRESENT;
		}
		else{
			//did not receive notification from ACB
			if(acb_connection_state == HEARTBEAT_PRESENT){

				if(misses < HEARTBEAT_MAX_MISSES){
					//losing ACB
					misses++;

					//just for safety
					acb_connection_state = HEARTBEAT_PRESENT;
				}
				else{
					//lost ACB
					acb_connection_state = HEARTBEAT_LOST;
					logMessage("Heartbeat: Lost Connection with ACB", true);
				}
			}
			else if(acb_connection_state == HEARTBEAT_NONE ){
				if(misses < HEARTBEAT_MAX_MISSES){
					//losing ACB
					misses++;

					//just for safety
					acb_connection_state = HEARTBEAT_NONE;

				}
				else{
					//lost ACB
					acb_connection_state = HEARTBEAT_NONE;
					logMessage("Heartbeat: Could not connect with ACB", true);
				}
			}
			else{
				//acb_connection_state = HEARTBEAT_LOST_ACB;
				led_mgmt_set_error(DASH_NO_ACB);
			}
		}


		//do dash leds and indicators
		if(get_mc_heartbeat_State() == HEARTBEAT_PRESENT && get_acu_heartbeat_State()==HEARTBEAT_PRESENT){
		  //heartbeat all good
      logIndicator(false, NO_ACB);
      led_mgmt_clear_error(DASH_NO_ACB);
		}
		else{
		  //heartbeat sadness
		  led_mgmt_set_error(DASH_NO_ACB);
		  logIndicator(true, NO_ACB);
		}

        osDelay(IWDG_RELOAD_PERIOD / 2);                                   // Delay for half IWDG_RELOAD_PERIOD
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
        return;
    }

  BaseType_t retRTOS;
  uint32_t ulNotifiedValue = 0;
  mc_connection_state = HEARTBEAT_NONE;
  uint8_t misses = 0; //indicates how many cycles we have gone without detecting ACB

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
heatbeat_state_t get_acu_heartbeat_State(){
	return acb_connection_state;
}

/*
 * get_mc_hearbeat_State
 *
 * @Brief: This method is used to get the current state of heartbeat
 */
heatbeat_state_t get_mc_heartbeat_State(){
  return mc_connection_state;
}

osThreadId_t heartbeat_ACU_get_task(){
    return acuHrtbeatTaskHandle;
}

osThreadId_t heartbeat_MC_get_task(){
    return mcHrtbeatTaskHandle;
}

