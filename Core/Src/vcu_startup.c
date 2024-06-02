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
#include "acu_comms_handler.h"
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
#define FIRST_ACK_TIMEOUT_MS 1500
#define TSA_ACK_TIMEOUT_MS 8000 	//[ms] timeout for receiving acknowledgment from ACU when going TSA
#define RTD_ACK_TIMEOUT_MS 8000 	//[ms] timeout for receiving acknowledgment from ACU when going RTD

#define MC_STARTUP_DELAY_MS 1000	//[ms] delay used to wait for the motor controller
#define STARTUP_TASK_DELAY_MS 25

#define DISABLE_HEARTBEAT_CHECK 0
#define DISABLE_SAFETY_LOOP_CHECK 0
#define DISABLE_BRAKE_CHECK 0
#define DISABLE_ACU_ACK 0

void goTsaProcedure(uint8_t *vcuStateTaskNotification) {
    osStatus_t retRTOS = 0;

    osMessageQueueGet(ackCarStateQueueHandle, vcuStateTaskNotification, 0, pdMS_TO_TICKS(2));
    if(read_saftey_loop() || DISABLE_SAFETY_LOOP_CHECK) {
        if(isButtonPressed(TSA_BTN_GPIO_Port, TSA_BTN_Pin) && (brakePressed() || DISABLE_BRAKE_CHECK) && (checkHeartbeat() || DISABLE_HEARTBEAT_CHECK)) {
            goTSA();
            retRTOS = osMessageQueueGet(ackCarStateQueueHandle, vcuStateTaskNotification, 0, pdMS_TO_TICKS(FIRST_ACK_TIMEOUT_MS));
            if(retRTOS != osOK || (*vcuStateTaskNotification) != ACU_TSA_ACK){
            	go_idle();
            	logMessage("ACU failed to ack TSA Request", true);
            	fail_pulse();
            	return;
            }
            dash_set_state(DASH_VCU_TSA_ACU_IDLE);
            retRTOS = osMessageQueueGet(ackCarStateQueueHandle, vcuStateTaskNotification, 0, pdMS_TO_TICKS(TSA_ACK_TIMEOUT_MS + MC_STARTUP_DELAY_MS));
            if(retRTOS != osOK || (*vcuStateTaskNotification) != ACU_TSA_ACK){
                go_idle();
                logMessage("ACU failed to ack TSA Request", true);
                fail_pulse();
            } else {
                dash_set_state(DASH_VCU_TSA_ACU_TSA);
                logMessage("Went TSA!", false);
            }
        }
    }
}

void goRtdProcedure(uint8_t *vcuStateTaskNotification) {
    osStatus_t retRTOS = 0;

    if(read_saftey_loop() || DISABLE_SAFETY_LOOP_CHECK) {
        if(isButtonPressed(RTD_BTN_GPIO_Port, RTD_BTN_Pin) && (checkHeartbeat() || DISABLE_HEARTBEAT_CHECK) && (brakePressed() || DISABLE_BRAKE_CHECK)) {
            goRTD();
            dash_set_state(DASH_VCU_RTD_ACU_TSA);
            retRTOS = osMessageQueueGet(ackCarStateQueueHandle, vcuStateTaskNotification, 0, pdMS_TO_TICKS(RTD_ACK_TIMEOUT_MS));
            if(retRTOS != osOK || (*vcuStateTaskNotification) != ACU_RTD_ACK){
                logMessage("ACU failed to ack RTD Request", false);
                go_idle();
                fail_pulse();
            } else {
                dash_set_state(DASH_VCU_RTD_ACU_RTD);
                EnableMC();
                mc_set_inverter_enable(1);
                logMessage("Went RTD!", false);
            }
        }
    } else {
        go_idle();
    }

    if(isButtonPressed(TSA_BTN_GPIO_Port, TSA_BTN_Pin)) {
        go_idle();
    }

    retRTOS = osMessageQueueGet(ackCarStateQueueHandle, vcuStateTaskNotification, 0, 0);
    if(retRTOS == osOK && (*vcuStateTaskNotification) == GO_IDLE_REQ_FROM_ACU){
        logMessage("ACU request IDLE state change", true);
        go_idle();
    }
}

void rtdStateProcedure(uint8_t *vcuStateTaskNotification) {
    osStatus_t retRTOS = 0;

    EnableMC();
    if(isButtonPressed(RTD_BTN_GPIO_Port, RTD_BTN_Pin) || isButtonPressed(TSA_BTN_GPIO_Port, TSA_BTN_Pin)){
        set_ACU_State(IDLE);
        go_idle();
        logMessage("RTD or VCU Button Pressed, going IDLE", false);
    }

    retRTOS = osMessageQueueGet(ackCarStateQueueHandle, vcuStateTaskNotification, 0, 0);
    if(retRTOS == osOK && (*vcuStateTaskNotification) == GO_IDLE_REQ_FROM_ACU){
        logMessage("ACU request IDLE state change", true);
        go_idle();
    }

    if(!DISABLE_HEARTBEAT_CHECK || checkHeartbeat()){//make sure we have ACU heartbeat
        //TODO VCU#32 ERROR Going idle because ACB hasn't sent a heart beat
        logMessage("Going Idle due to lack of ACU", true);
        go_idle();
    }
}

/**
 * set_safety_loop_state
 *
 * @brief This function is used for setting the VCU control of the safety loop.
 * @param state should be one of LOOP_CLOSE or LOOP_OPEN
 * @return void
 */
void set_safety_loop_state(enum safetyLoopState state){
    GPIO_PinState pinState = (state == SAFETY_LOOP_CLOSED) ? GPIO_PIN_RESET : GPIO_PIN_SET;
	HAL_GPIO_WritePin(SAFETY_LOOP_CTL_GPIO_Port, SAFETY_LOOP_CTL_Pin , pinState);
}

int checkHeartbeat() {
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


	if(HAL_GPIO_ReadPin(SAFETY_LOOP_TAP_GPIO_Port, SAFETY_LOOP_TAP_Pin) == GPIO_PIN_SET){
		state = true;
	}
	else{
        //TODO VCU#32 ERROR Safety loop open [hardware fault]A
		state = false;
	}

	return state;
}

/**
 * @brief  Returns the handle for the ACU task
 * @retval
 */
TaskHandle_t get_startup_task(){
	return vcuStateTaskHandle;
}

/**
 * @Brief: This function is used to bring the entire car into the idle state
 */
void go_idle(){
    //TODO VCU#32 INFO Going idle
    dash_set_state(DASH_VCU_IDLE_ACU_IDLE);
	DisableMC();
	mc_set_inverter_enable(0);
	set_car_state(IDLE);
    set_ACU_State(IDLE);
}

void goTSA() {
	set_car_state(TRACTIVE_SYSTEM_ACTIVE);
    set_ACU_State(TRACTIVE_SYSTEM_ACTIVE);
	DisableMC();
	mc_enable_broadcast_msgs();
}

void goRTD() {
	dash_set_rtd_teal();
	set_car_state(READY_TO_DRIVE);
    set_ACU_State(READY_TO_DRIVE);
}

/*
 * fail_pulse
 *
 * @Brief: This function is used to flash the dash lights to indicate that something has gone wrong.
 * This function delays and must be called from a running task
 */
static void fail_pulse(){
    //TODO VCU#32 ERROR pulse failed
    dash_set_state(DASH_FAIL_PULSE);
    go_idle();
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

/**
 * @brief  handle the startup routine
 * @retval never return from a freeRTOS task, kills task if infinite task ends
 */
void StartVcuStateTask(void *argument){
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

    uint8_t ulNotifiedValue;
    enum CAR_STATE state;

    for(;;){
        kickWatchdogBit(osThreadGetId());

        //TODO VCU#32 Car state changed
        state = get_car_state();
        set_ACU_State(state);

        switch(state){
            case IDLE:
                dash_set_state(DASH_VCU_IDLE_ACU_IDLE);
                goTsaProcedure(&ulNotifiedValue);
                break;
            case TRACTIVE_SYSTEM_ACTIVE:
                //TODO VCU#32 INFO Tractive system active Ready to drive procedure begun
                goRtdProcedure(&ulNotifiedValue);
                break;
            case READY_TO_DRIVE:
                rtdStateProcedure(&ulNotifiedValue);
                break;
            default:
                break;
        }
        osDelay(pdMS_TO_TICKS(STARTUP_TASK_DELAY_MS));
    }
}
