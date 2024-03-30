#include "logger.h"
#include "bt_protocol.h"
#include <string.h>

//Stores the current state of the logger initialization
bool LOGGING_INITIALIZED = false;

//Holds all the sensor values
float data_sensors[NUM_OF_SENSORS] = {0.0f/0.0f};

//Holds the VCUs message sent over CAN (Used by the ACB to decompile the VCUs message)
char VCU_msg[VCU_LOG_MSG_LEN];
int VCU_msgLen = 0;

static int VCU_loggingReady = 0;

/*
 * logInitialize()
 *
 * Initializes the SD card and readies the Bluetooth packet
 */
bool logInitialize() {
	//Ensures it isn't already initialized, and initializes the SD and BT
	if(!LOGGING_INITIALIZED) {
		//On success, set the bool flag and return true
		LOGGING_INITIALIZED = true;
        if(!btInitialize()) {
            logMessage("Failed to initialize BT", false);
        }
		return true;
	}
	//Return false on any init failures
	return false;
}

/*
 * logTerminate()
 *
 * Closes the SD card/stops logging
 */
bool logTerminate() {
	//Ensures it has been initialized already, and terminates the SD and BT
	if(LOGGING_INITIALIZED) {
		if(!btTerminate()) {
			return false;
		}
		//Reset the bool flag, return true
		LOGGING_INITIALIZED = false;
		return true;
	}
	//Return false on any term failures
	return false;
}

/*
 * logIndicator(bool value, INDICATOR indc)
 *
 * Log a indicator value to be added to the Bluetooth packet
 *
 * value = Bool value of the indicator
 * indc = An indicator that has been defined in the enum typedef in logger.h
 */
void logIndicator(bool value, INDICATOR indc) {
	if(LOGGING_INITIALIZED) {
		btUpdateData((void *)&value, NUM_OF_SENSORS+indc);
	}
}

/**
 * Enables the VCU to start the logging process
 */
void enableVCULogging() {
	VCU_loggingReady = 1;
}


/*
 * logMessage(char *data, bool critical)
 *
 * Log a diagnostics message, by sending it to the ACB in 8 byte chunks
 *
 * data = Char array (String) that contains the message
 * critical = Boolean flag on if the message is critical, bypassing the log buffer
 */
void logMessage(char *data, bool critical) {
	if (VCU_loggingReady && LOGGING_INITIALIZED) {
		int sliceAmount = strlen(data - 1) / 8 + 1;

		int letterCounter = 0;
		int exit = 0;
		char slicedMesg[8];
		int i = 0;

		//Loop for each slice, breaking it down into chunks of 8
		for (int slice = 0; slice < sliceAmount && !exit; slice++) {
			//Go through 1 of the chunks
			for (i = 0; i < 8 && !exit; i++) {
				exit = (data[letterCounter] == '\0');
				slicedMesg[i] = data[letterCounter];
				letterCounter++;
			}
			//Send the chunk of the message over CAN
			//sendCan(CAN1, slicedMesg, i, CAN_VCU_CAN_ID, CAN_NO_EXT, CAN_NO_EXT);
		}
	}
}

/*
 * logErrorMessage(char *data, INDICATOR indc)
 *
 * Log a diagnostics message to the SD card
 *
 * data = Char array (String) that contains the message
 * critical = Boolean flag on if the message is critical, bypassing the log buffer
 */
void logErrorMessage(char *data, INDICATOR indc){
	if(LOGGING_INITIALIZED) {
		btUpdateData(data, NUM_OF_SENSORS + indc);
	}
}

/*
 * logSensor(float value, SENSOR sens)
 *
 * Log a sensor value to the SD card and Bluetooth packet
 *
 * value = Float value of the sensor
 * sens = A sensor that has been defined in the enum typedef in logger.h
 */
void logSensor(float value, SENSOR sens) {
	if(LOGGING_INITIALIZED) {
        btUpdateData((void *)&value, sens);
	}
}
