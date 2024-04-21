#ifndef VCU_STARTUP_H
#define VCU_STARTUP_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#define LOOP_CLOSE 1
#define LOOP_OPEN 0
extern const osThreadAttr_t vcuStateTask_attributes;
void startup_Init();
void go_idle();
int checkHeartbeat();
void goTSA();
void goRTD();
bool startup_Task_start();
TaskHandle_t get_startup_task();


enum startup_notify_value{
	ACB_TSA_NACK = 0,
	ACB_TSA_ACK,
	ACB_RTD_NACK,
	ACB_RTD_ACK,
	GO_IDLE_REQ_FROM_ACB
};

void set_saftey_loop_state(uint8_t state); //use this for blocking the safety loop in scary conditions
bool read_saftey_loop();

#endif
