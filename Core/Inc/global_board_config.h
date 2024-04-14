#ifndef GLOBAL_BOARD_CONFIG_H
#define GLOBAL_BOARD_CONFIG_H

#define VCU
#define PROFILER_ENABLE	//enables the run time stats profiller
#define HEARTBEAT_ENABLE
//#define BTRECEIVE //enables the usart3 receive interrupt
#define STLINK_SERVER_ENABLE

//toggles indicators on dash app
//#define BT_TESTING

//disables the sending of torque values to the motor controller
//#define MOTOR_TESTING

//enables austins led hmi
//#define LED_HMI_EN

//bypass break checks in RTD
#define BREAK_BYPASS
//Bypass all the other USART except BT
//#define BT_PASS_THROUGH

#endif
