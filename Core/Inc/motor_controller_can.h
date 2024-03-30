

#ifndef _MOTOR_CONTROLLER_CAN_H
#define _MOTOR_CONTROLLER_CAN_H

#include "stdint.h"

#define TORQUE_MODE			0
#define SPEED_MODE			1

#define BUS_DISCHARGED 		0
#define BUS_CHARGED    		1

#define MC_COMMAND_READ 	0
#define MC_COMMAND_WRITE 	1

#define MC_COMMAND_MSG			 0x0C0
#define MC_PARAM_COMMAND_MSG	 0x0C1

#define MC_ENABLE_BYTE_4 		 0b11100101
#define MC_ENABLE_BYTE_5		 0b00000000

#define PEAK_TORQUE			230	//Peak torque for EMRAX 228 motor

static int16_t bus_voltage;
static int16_t bus_current;
static int16_t mc_currentA;
static int16_t mc_currentB;
static int16_t mc_currentC;
static int16_t mc_igbtA_temp;
static int16_t mc_igbtB_temp;
static int16_t mc_igbtC_temp;
static int16_t mc_motor_temp;
static int16_t mc_rpm;
static int16_t mc_output_voltage;
static int16_t mc_torque_command;
static int16_t mc_torque_feedback;
static int16_t mc_vd;
static int16_t mc_vq;

static int8_t  mc_direction;
static int8_t  mc_enable_inverter;
static int8_t  mc_enable_discharge;
static int16_t mc_torque_limit;
static int16_t mc_torque;
static int16_t mc_speed;

float mc_getBusVoltage();
float mc_getBusCurrent();
float mc_getIGBTACurrent();
float mc_getIGBTBCurrent();
float mc_getIGBTCCurrent();
float mc_getAverageIGBTTemp();
float mc_getIGBTATemp();
float mc_getIGBTBTemp();
float mc_getIGBTCTemp();
float mc_getMotorTemp();
int mc_getRPM();
float mc_getOutputVoltage();
float mc_getCommandedTorque();
float mc_getFeedbackTorque();
float mc_get_vd();
float mc_get_vq();

void mc_set_torque_limit(int setTorque);
void mc_set_torque(int setTorque);
void mc_set_speed(int setSpeed);
void mc_set_direction(uint8_t setDirection);
void mc_set_inverter_enable(uint8_t setEnable);
void mc_set_inverter_discharge(uint8_t setEnable);
uint8_t isMCBusCharged();

void mc_process_fast_can(uint8_t * data);
void mc_process_temp1_can(uint8_t * data);
void mc_process_temp3_can(uint8_t * data);
void mc_process_volt_can(uint8_t * data);
void mc_process_motor_can(uint8_t * data);
void mc_process_current_can(uint8_t * data);

void sendTorque(int16_t);

void mc_send_command_msg(uint8_t mode);
void mc_send_param_command_message(uint8_t param_address, uint8_t RW, uint8_t * Data);

void mc_enable_broadcast_msgs();
void mc_disable_broadcast_msgs();

typedef enum{
	MC_DISABLED = 0,
	MC_ENABLED
}mc_state_t;

void UpdateMCState(int16_t mc_trottle_val);
void EnableMC();
void DisableMC();

void fixFaults();
#endif
