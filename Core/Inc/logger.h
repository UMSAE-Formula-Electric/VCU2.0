#ifndef LOGGER_H_
#define LOGGER_H_

#define VCU_LOG_MSG_LEN 32

#include <stdbool.h>

#include "global_board_config.h"

extern bool LOGGING_INITIALIZED;

/*	Typedef'd enumerator for sensors
*		Final value, NUM_OF_SENSORS returns total number of sensors
*		--ALWAYS HAVE NUM_OF_SENSORS AS THE LAST ENUM VALUE--
*		--IF YOU ADD A SENSOR, DON'T FORGET TO ADD TO THE SENSORNAMES ARRAY--
*/
typedef enum {
	TR_SENS1, 					// 01
	TR_SENS2, 					// 02
	TR_REQUEST, 				// 03
	MC_ACUAL_SPEED_REG_LOG, 	// 04
	MC_ACUAL_SPEED_REG_LOG100,	// 05
	MC_N_CMD_LOG,				// 06
	MC_N_CMD_LOG100,			// 07
	MC_N_CMD_RAMP_LOG,			// 08
	MC_N_CMD_RAMP_LOG100,		// 09
	MC_N_ERROR_LOG,				// 10
	MC_N_ERROR_LOG100,			// 11
	MC_I_CMD_LOG,				// 12
	MC_I_CMD_RAMP_LOG,			// 13
	MC_I_ACTUAL_LOG,			// 14
	MC_V_D_LOG,					// 15
	MC_V_Q_LOG,					// 16
	MC_V_OUT_LOG,				// 17
	MC_T_MOTOR_LOG,				// 18
	MC_T_IGBT_LOG,				// 19
	MC_T_AIR_LOG,				// 20
	MC_DIG_TORQUE_LOG,			// 21
	MC_BUS_VOLTAGE_LOG,			// 22
	MC_BUS_VOLTAGE_LOG100,		// 23
	FR_WHEEL_SPEED,				// 24
	FL_WHEEL_SPEED,				// 25
	BRAKE_1,					// 26
	BRAKE_2,					// 27
	MC_RPM,                     // 28
	BNO055_ACCEL_X,             // 29
	BNO055_ACCEL_Y,             // 30
	BNO055_ACCEL_Z,             // 31
	BNO055_GYRO_X,              // 32
	BNO055_GYRO_Y,                // 33
	BNO055_GYRO_Z,                // 34
	VBATT,							//35
	T_IC,							//36
	NUM_OF_SENSORS				// 37 (36)
} SENSOR;

/* Typedef'd enumerator for indicators
 * 		Final value, NUM_OF_INDICATORS returns total number of indicators
 */
typedef enum {
	GENERAL_ERROR,				// 00
	GENERAL_WARNING,			// 01
	LOW_BATTERY,				// 02
	NO_ACB,						// 03
	THROTTLE_ERROR,				// 04
	SAFETY_LOOP,				// 05
	MESSAGE,					//06
	NUM_OF_INDICATORS			// 07
} INDICATOR;


//Sensor-Related
extern float data_sensors[NUM_OF_SENSORS];
extern char *data_ids_sd[NUM_OF_SENSORS+1];
extern char *data_ids_bt[NUM_OF_SENSORS+NUM_OF_INDICATORS];

/**
 * BT ERROR STATES:
 *
 * 0x00: No Error
 * 0x01: Failed to Create BT RTOS Task for Dumping
 * 0x02: Invalid Data Entry Type
 **/

extern char BT_ERROR_STATE;

/**
 * SD ERROR STATES:
 *
 * 0x00: No Error
 * 0x01: Mounting Error (No SD card inserted?)
 * 0x02: Failed to Create Folders
 * 0x03: Failed to Create/Open Files
 * 0x04: Failed to Seek to End of Files
 * 0x05: Failed to Dump Sensor Headers
 * 0x06: Failed to Create Diagnostics Logging Queue
 * 0x07: Failed to Create SD RTOS Task for Dumping
 * 0x08: Failed to Get Free Space Available
 * 0x09: Not Enough Free Space Available
 **/

extern char SD_ERROR_STATE;

bool logInitialize();
bool logTerminate();
void logIndicator(bool value, INDICATOR indc);
void logMessage(char *data, bool critical);
void logSensor(float value, SENSOR sens);
void logErrorMessage(char *data, INDICATOR indc);
void enableVCULogging();
void nullTerminate(char *str);

#endif
