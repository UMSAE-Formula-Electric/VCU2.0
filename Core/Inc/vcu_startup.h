#ifndef VCU_STARTUP_H
#define VCU_STARTUP_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#define LOOP_CLOSE 1
#define LOOP_OPEN 0

void startup_Init();
void go_idle();
int checkHeartbeat();
void goTSA();
void goRTD();
bool startup_Task_start();
TaskHandle_t get_startup_task();


enum startup_notify_value{
	ACU_TSA_NACK = 0,
	ACU_TSA_ACK,
	ACU_RTD_NACK,
	ACU_RTD_ACK,
	GO_IDLE_REQ_FROM_ACU
};

void set_saftey_loop_state(uint8_t state); //use this for blocking the safety loop in scary conditions
bool read_saftey_loop();

#endif
