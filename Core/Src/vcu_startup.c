/*
 * vcu startup task
 *
 * Programmed by: Multiple people (Austin, Martin)
 *
 * This task is used to handle the state changes of the car. This task handles the transitions
 * from IDLE to Tractive system active (TSA) to Ready to Drive (RTD)
 */

#include <dashboard_mgmt.h>
#include "apps_brake.h"
#include "ACB_comms_handler.h"
#include "can.h"
#include "car_state.h"
#include "vcu_startup.h"
#include "logger.h"
#include "motor_controller_can_utils.h"
#include "dashboard_leds.h"
#include "iwdg.h"
#include "heartbeat.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stdio.h"
#include "freertos_task_handles.h"

static void fail_pulse();
bool isButtonPressed(GPIO_TypeDef* port, uint16_t pin);

//timing
#define TSA_ACK_TIMEOUT 8000 	//[ms] timeout for receiving acknowledgment from ACB when going TSA
#define RTD_ACK_TIMEOUT 8000 	//[ms] timeout for receiving acknowledgment from ACB when going RTD
#define FAIL_FLASH_LEN 1000		//[ms] length of warning flash on failed arm
#define STARTUP_TASK_DELAY 100	//[ms] how often the startup task is executed
#define MC_STARTUP_DELAY 1000	//[ms] delay used to wait for the motor controller

#define HOON_MODE 0

#define DISABLE_HEARTBEAT_CHECK 0
#define DISABLE_SAFETY_LOOP_CHECK 1
#define DISABLE_BRAKE_CHECK 0
#define DISABLE_ACU_ACK 0

//TODO CleanUp
/**
 * @brief  Starts the can message recieve and processing task
 * @retval 0 on success, 1 if failed to start tasks(probably out of rtos heap mem.)
 */
//bool startup_Task_start(){
//	bool created = false;
//
//	created = TaskManagerCreate(&VCU_startup_Task, &xTask_VCU_Startup);
//
//	if(!created){
//		logMessage("Failed to create startup task", true);
//	}
//
//	return created;
//}

/**
 * @brief  handle the startup routine
 * @retval never return from a freeRTOS task, kills task if infinite task ends
 */
void StartVcuStateTask(void *argument){
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

	//Keep user led on to simulate LV key is on
	BaseType_t retRTOS = 0;
	uint32_t ulNotifiedValue;
	char strBuff[40]; //buffer for making 'nice' logs
	enum CAR_STATE state;

	for(;;){
        kickWatchdogBit(osThreadGetId());
		state = get_car_state();
		switch(state){
		case IDLE:
			retRTOS = xTaskNotifyWait(0x00,0x00, &ulNotifiedValue, 0);
			if(retRTOS == pdTRUE){
				sprintf(strBuff, "Received ACB notification: %lu", ulNotifiedValue);
				logMessage(strBuff,false);
			}

			//Go TSA procedure
			if(read_saftey_loop()) {
				//Safety loop closed
				if(isButtonPressed(TSA_BTN_GPIO_Port, TSA_BTN_Pin)) {
					//Dash button pressed
					dash_set_tsa_teal();
					if(brakePressed() || DISABLE_BRAKE_CHECK) {
						//Brake pressed
						if(checkHeartbeat()) {
							//Heartbeats are valid
							goTSA();
							retRTOS = xTaskNotifyWait(0x00,0x00, &ulNotifiedValue, TSA_ACK_TIMEOUT + MC_STARTUP_DELAY);
							if(ulNotifiedValue != ACB_TSA_ACK){
								//ACU did not ACK
								go_idle();
								logMessage("ACB failed to ack TSA Request", true);
								fail_pulse();
							} else {
								logMessage("Went TSA!", false);
								dash_set_tsa_green();
							}
						} //Heartbeats not valid
					} //Brake not pressed
				} //Dash button not pressed
			} //Safety loop open

			if(get_car_state() != IDLE) {
				fail_pulse();
				go_idle();
			}

			break;
		case TRACTIVE_SYSTEM_ACTIVE:
			if(read_saftey_loop) {
				if(isButtonPressed(RTD_BTN_GPIO_Port, RTD_BTN_Pin)){
					if(checkHeartbeat()){
						dash_set_rtd_teal();
						if(brakePressed() || DISABLE_BRAKE_CHECK){
							retRTOS = xTaskNotifyWait(0x00,0x00, &ulNotifiedValue, RTD_ACK_TIMEOUT);
							EnableMC();
							if(retRTOS != pdPASS || ulNotifiedValue != ACB_RTD_ACK){
								logMessage("ACB failed to ack RTD Request", false);
								go_idle();
								fail_pulse();
							}
							else{
								dash_set_rtd_green();
								mc_set_inverter_enable(1);
								logMessage("Went RTD!", false);
							}
						} 	//Brake not pressed
					} 		//Heartbeats not valid
				} 			//Button not pressed
			} else {		//Safety loop open
				go_idle();
			}

			if(isButtonPressed(TSA_BTN_GPIO_Port, TSA_BTN_Pin)) {
				go_idle();
			}

			retRTOS = xTaskNotifyWait(0x00,0x00, &ulNotifiedValue, 0);
			if(retRTOS == pdTRUE && ulNotifiedValue == GO_IDLE_REQ_FROM_ACB){
				logMessage("ACB request IDLE state change", true);
				go_idle();
			}

			break;
		case READY_TO_DRIVE:
			EnableMC();
			if(isButtonPressed(RTD_BTN_GPIO_Port, RTD_BTN_Pin) || isButtonPressed(TSA_BTN_GPIO_Port, TSA_BTN_Pin)){
				set_ACB_State(IDLE);
				go_idle();
				logMessage("RTD or VCU Button Pressed, going IDLE", false);
			}

			retRTOS = xTaskNotifyWait(0x00,0x00, &ulNotifiedValue, 0);
			if(retRTOS == pdTRUE && ulNotifiedValue == GO_IDLE_REQ_FROM_ACB){
				logMessage("ACB request IDLE state change", true);
				go_idle();
			}

			if(!DISABLE_HEARTBEAT_CHECK) {
				if(checkHeartbeat()){//make sure we have ACB heartbeat
					logMessage("Going Idle due to lack of ACB", true);
					go_idle();
				}
			}
			break;
		default:
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(STARTUP_TASK_DELAY));                  //TODO Revise task delay
	}
	logMessage("Error exiting from startup task", true);
	vTaskDelete( NULL );
}

/*
 * set_saftey_loop_state
 *
 * @Brief: This function is used for setting the VCU control of the safety loop.
 * @Param: state should be one of LOOP_CLOSE or LOOP_OPEN
 * Software control of shutdown circuit has been removed for 2020 vehicle
 */
//void set_saftey_loop_state(uint8_t state){
//	GPIO_WriteBit(VCU_SHUTDOWN_CTRL_PORT, VCU_SHUTDOWN_CTRL_PIN , state);
//}

int checkHeartbeat() {
	if(DISABLE_HEARTBEAT_CHECK) return true;

	if(get_acu_heartbeat_state() == HEARTBEAT_PRESENT){
		if(get_mc_heartbeat_state() == HEARTBEAT_PRESENT) {
			return true;
		}
	}
	return false;
}

/*
 * read_saftey_loop
 *
 * @Brief: This method checks the satus of the saftey loop at the VCU
 *
 * @Return: returns true if there is voltage on the safety loop at the VCU
 * otherwise returns false
 */
bool read_saftey_loop(){
	int state;


	if(HAL_GPIO_ReadPin(SAFETY_LOOP_GPIO_Port, SAFETY_LOOP_Pin) == GPIO_PIN_SET){
		led_mgmt_clear_error(DASH_SAFETY_LOOP_OPEN_VCU);
		state = true;
	}
	else{
		led_mgmt_set_error(DASH_SAFETY_LOOP_OPEN_VCU);
		state = false;
	}

	return (state || DISABLE_SAFETY_LOOP_CHECK);
}

/**
 * @brief  Returns the handle for the ACB task
 * @retval
 */
TaskHandle_t get_startup_task(){
	return vcuStateTaskHandle;
}

/*
 * go_idle
 *
 * @Brief: This function is used to bring the entire car into the idle state
 */
void go_idle(){
	dash_clear_all_leds();
	DisableMC();
	mc_set_inverter_enable(0);
	set_car_state(IDLE);
	set_ACB_State(IDLE);
}

void goTSA() {
	set_car_state(TRACTIVE_SYSTEM_ACTIVE);
	set_ACB_State(TRACTIVE_SYSTEM_ACTIVE);
	DisableMC();
	mc_enable_broadcast_msgs();
}

void goRTD() {
	dash_set_rtd_teal();
	set_car_state(READY_TO_DRIVE);
	set_ACB_State(READY_TO_DRIVE);
}

/*
 * fail_pulse
 *
 * @Brief: This function is used to flash the dash lights to indicate that something has gone wrong.
 * This function delays and must be called from a running task
 */
static void fail_pulse(){
	go_idle();
	dash_set_rtd_blue();
	dash_set_tsa_blue();
	vTaskDelay(pdMS_TO_TICKS(FAIL_FLASH_LEN));
	dash_clear_all_leds();
	DisableMC();
}

/*
 * isButtonPressed
 * @Brief: This function is used to check if a button is pressed
 * @Param: port is the GPIO port of the button
 * @Param: pin is the GPIO pin of the button
 * @Return: returns true if the button is pressed, otherwise returns false
 */
bool isButtonPressed(GPIO_TypeDef* port, uint16_t pin){
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}
