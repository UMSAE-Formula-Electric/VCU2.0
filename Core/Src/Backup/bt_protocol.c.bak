#include "bt_protocol.h"
#include "motor_controller_can.h"
#include "car_state.h"
#include "logger.h"
#include <stdio.h>
#include "string.h"
#include "stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"

//Variables for BT status/errors
static bool BT_INITIALIZED = false;
char BT_ERROR_STATE = 0x00;

//Prototypes for Static/Private Methods
static void buildPacket();
static void initPacket();
static void setStartFlag();
static void setAcknowledgement();
static void setHeader();
static void setExpansion();
static void setChecksum();
static void setEndFlag();
static void updateSequence();

//Declaration of Static/Private Variables
static uint16_t curr_sequence = 0;
static PACKET packet = {0};

//FreeRTOS Variables
TickType_t BT_DUMP_DELAY = pdMS_TO_TICKS(250);
TickType_t LONG_BT_DUMP_DELAY = pdMS_TO_TICKS(1000);
TickType_t BT_MUTEX_TIMEOUT = pdMS_TO_TICKS(200);
SemaphoreHandle_t packetMutex;

//DATA IDs
char *data_ids_bt[NUM_OF_SENSORS+NUM_OF_INDICATORS] = {
	//Sensors
	"TS1",	// 01
	"TS2",	// 02
	"TRR",	// 03
	"SPD",	// 04
	"SPP",	// 05
	"NCM",	// 06
	"NCP",	// 07
	"NCR",	// 08
	"CRP",	// 09
	"ERR",	// 10
	"ERP",	// 11
	"ICM",	// 12
	"ICR",	// 13
	"IAC",	// 14
	"VVD",	// 15
	"VVQ",	// 16
	"VVO",	// 17
	"MTE",	// 18
	"ITE",	// 19
	"ATE",	// 20
	"TCM",	// 21
	"BUV",	// 22
	"BVP",	// 23
	"FRW",	// 24
	"FLW",	// 25
	"BE1",	// 26
	"BE2",	// 27
	"MRP",  // 28
	"BAX",  // 29
	"BAY",  // 30
	"BAZ",  // 31
	"BGX",  // 32
	"BGY",  // 33
	"BGZ",  // 34
	"VBT",  // 35
	"ICT",  // 36
	//Indicators
	"IER",	// 37
	"IWA",	// 38
	"LBA",	// 39
	"NOA",	// 40
	"TER",	// 41
	"SLO",	// 42
	"MSG"	//43
};

//DATA TYPES
DATATYPE data_types_bt[NUM_OF_SENSORS+NUM_OF_INDICATORS] = {
	//Sensors
	BT_DT_FLOAT,	// 01
	BT_DT_FLOAT,	// 02
	BT_DT_FLOAT,	// 03
	BT_DT_FLOAT,	// 04
	BT_DT_FLOAT,	// 05
	BT_DT_FLOAT,	// 06
	BT_DT_FLOAT,	// 07
	BT_DT_FLOAT,	// 08
	BT_DT_FLOAT,	// 09
	BT_DT_FLOAT,	// 10
	BT_DT_FLOAT,	// 11
	BT_DT_FLOAT,	// 12
	BT_DT_FLOAT,	// 13
	BT_DT_FLOAT,	// 14
	BT_DT_FLOAT,	// 15
	BT_DT_FLOAT,	// 16
	BT_DT_FLOAT,	// 17
	BT_DT_FLOAT,	// 18
	BT_DT_FLOAT,	// 19
	BT_DT_FLOAT,	// 20
	BT_DT_FLOAT,	// 21
	BT_DT_FLOAT,	// 22
	BT_DT_FLOAT,	// 23
	BT_DT_FLOAT,	// 24
	BT_DT_FLOAT,	// 25
	BT_DT_FLOAT,	// 26
	BT_DT_FLOAT,	// 27
	BT_DT_FLOAT,	// 28
	BT_DT_FLOAT,	// 29
	BT_DT_FLOAT,	// 30
	BT_DT_FLOAT,	// 31
	BT_DT_FLOAT,	// 32
	BT_DT_FLOAT,	// 33
	BT_DT_FLOAT,	// 34
	BT_DT_FLOAT,	// 35
	BT_DT_FLOAT,	// 36
	//Indicators
	BT_DT_BOOLEAN,	// 37
	BT_DT_BOOLEAN,	// 38
	BT_DT_BOOLEAN,	// 39
	BT_DT_BOOLEAN,	// 40
	BT_DT_BOOLEAN,	// 41
	BT_DT_BOOLEAN,	// 42
	BT_DT_STRING      //43
};
//DATA SCALARS
int data_scalars_bt[NUM_OF_SENSORS+NUM_OF_INDICATORS] = {
	//Sensors
	0,	// 01
	0,	// 02
	0,	// 03
	0,	// 04
	0,	// 05
	0,	// 06
	0,	// 07
	0,	// 08
	0,	// 09
	0,	// 10
	0,	// 11
	0,	// 12
	0,	// 13
	0,	// 14
	0,	// 15
	0,	// 16
	0,	// 17
	0,	// 18
	0,	// 19
	0,	// 20
	0,	// 21
	0,	// 22
	0,	// 23
	0,	// 24
	0,	// 25
	0,	// 26
	0,	// 27
	0,	// 28
	0,	// 29
	0,	// 30
	0,	// 31
	0,	// 32
	0,	// 33
	0,	// 34
	0,	// 35
	0,	// 36
	//Indicators
	0,	// 37
	0,	// 38
	0,	// 39
	0,	// 40
	0,	// 41
	0,	// 42
	0 //43 // add a comma here later to test
};

/**
 * btInitialize()
 *
 * 		Initializes the packet and starts the Bluetooth task
 */
bool btInitialize() {
	//Check if BT has already been initialized
	if(!BT_INITIALIZED) {
        initPacket();
        //Create the mutex
        packetMutex = xSemaphoreCreateMutex();
        //Set the init flag and error state
        BT_ERROR_STATE = 0x00;
        BT_INITIALIZED = true;
        return true;
	}
	return false;
}

/**
 * btTerminate()
 *
 * 		Terminates/stops the Bluetooth dumping task
 */
bool btTerminate() {
	//Ensure BT is initialized before trying to terminate
	if(BT_INITIALIZED) {
		//Delete the RTOS task
        osThreadTerminate(btDumpTaskHandle);
		//Delete the RTOS mutexes
		if(packetMutex != NULL) {
			vSemaphoreDelete(packetMutex);
		}
		//Set the init flag, return
		BT_INITIALIZED = false;
		return true;
	}
	return false;
}

/**
 * btSendPacket()
 *
 * 		Builds and sends the packet via USART
 */
void btSendPacket() {
	if(BT_INITIALIZED && BT_ERROR_STATE == 0x00) {
		//Try to take the semaphore to update the packet memory location
		if(xSemaphoreTake(packetMutex, BT_MUTEX_TIMEOUT) == pdPASS) {
			updateSequence();
            //TODO Bluetooth
//			usartBTSend((char*)&packet);
			xSemaphoreGive(packetMutex);
		}
	}
}

/**
 * btUpdateData()
 *
 * 		Updates an individual data member's value
 */
bool btUpdateData(void *value, SENSOR name) {
	//Try to take the semaphore to update the packet memory location
	if( 1){//xSemaphoreTake(packetMutex, BT_MUTEX_TIMEOUT) == pdPASS) {
		int size = 51; //I thibk //used to be 11
		//Convert packet type back to an integer
		int packet_type = packet.data[name].type[0] - '0';
		//Char array for storing final value
		char value_buff[55] = {0};  // usually 15
		//Convert value to char depending on type using sprintf, place in value_buff
		if(packet_type == BT_DT_BYTE) {
			snprintf(value_buff, size, "%10c",*((char *)value));
		} else if(packet_type == BT_DT_SHORT) {
			snprintf(value_buff, size, "%010hi", *((short *)value));
		} else if(packet_type == BT_DT_INT) {
			snprintf(value_buff, size, "%010d", *((int *)value));
		} else if(packet_type == BT_DT_LONG) {
			snprintf(value_buff, size, "%010li", *((long *)value));
		} else if(packet_type == BT_DT_FLOAT) {
			snprintf(value_buff, size, "%010li", (uint32_t)*((float *)value));
		} else if(packet_type == BT_DT_DOUBLE) {
			snprintf(value_buff, size, "%010f", *((double *)value));
		} else if(packet_type == BT_DT_CHAR) {
			value_buff[0] = *((char *)value);
		} else if(packet_type == BT_DT_STRING) {
			snprintf(value_buff, size, "%10s", (char *)value);
		} else if(packet_type == BT_DT_BOOLEAN) {
			//Checks if value is true, and sets value accordingly
			if(*((bool *)value)) {
				value_buff[0] = (char)BT_VALUE_TRUE;
			} else {
				value_buff[0] = (char)BT_VALUE_FALSE;
			}
		} else {
			//Somehow has an invalid datatype, set the error state and return
			BT_ERROR_STATE = 0x02;
			return false;
		}
		//Copies the computed character array back into the packet and returns
		memcpy(packet.data[name].value,value_buff,sizeof(packet.data[name].value));
		//xSemaphoreGive(packetMutex);
	}
	return true;
}

/**
 * initPacket()
 * 
 * 		Initializes the packet structure
 */
static void initPacket() {
	//Build/set the other packet structure parts
	buildPacket();
	//Build the data entries part of the packet
	for(int i=0; i<(NUM_OF_SENSORS+NUM_OF_INDICATORS); i++) {
		//Mem copies the ID into struct
		memcpy(packet.data[i].id,data_ids_bt[i],sizeof(packet.data[i].id));

		//Fetches type from array, converts to string, mem copies the type into struct
		char type_buff[sizeof(packet.data[i].type)+1];
		itoa(data_types_bt[i],type_buff,10);
		memcpy(packet.data[i].type,type_buff,sizeof(packet.data[i].type));

		//Fetches scalar from array, converts to string, mem copies the scalar into struct
		char scalar_buff[sizeof(packet.data[i].scalar)+1];
		sprintf(scalar_buff, "%02d",data_scalars_bt[i]);
		memcpy(packet.data[i].scalar,scalar_buff,sizeof(packet.data[i].scalar));

		//Set the value to zero
		memset(packet.data[i].value,'\0',sizeof(packet.data[i].value));
	}
}

/**
 * buildPacket()
 *
 * 		Builds the base packet to send
 */
static void buildPacket() {
	//Set individual sections of the packet
	setStartFlag();
	setAcknowledgement();
	setHeader();
	setExpansion();
	setChecksum();
	setEndFlag();
	updateSequence();
}

/**
 * setStartFlag()
 *
 * 		Sets the start flag in the packet struct
 */
static void setStartFlag() {
	packet.start = BT_START_FLAG;
}

/**
 * setAcknowledgement()
 *
 * 		Sets the acknowledgment in the packet struct
 */
static void setAcknowledgement() {
	memset(packet.ack,'\0',sizeof(packet.ack));
}

/**
 * setHeader()
 *
 * 		Sets the header in the packet struct
 */
static void setHeader() {
	memset(packet.header,'\0',sizeof(packet.header));
}

/**
 * setExpansion()
 *
 * 		Sets the expansion in the packet struct
 */
static void setExpansion() {
	memset(packet.expan,'\0',sizeof(packet.expan));
}

/**
 * setChecksum()
 *
 * 		Sets the checksum in the packet struct
 */
static void setChecksum() {
	memset(packet.checksum,'\0',sizeof(packet.checksum));
}

/**
 * setEndFlag()
 *
 * 		Sets the end flag in the packet struct
 */
static void setEndFlag() {
	packet.end = BT_END_FLAG;
}

/**
 * updateSequence()
 *
 * 		Updates the static 4-byte character buffer
 * 		Overflows at 10000, modulo back to 0
 * 		Stores the current sequence value in the packet structure
 */
static void updateSequence() {
	//Increment the current sequence value
	curr_sequence++;
	curr_sequence = curr_sequence%10000;
	//Fill temporary buffer of FIVE bytes
	char seq_buff[sizeof(packet.sequence)+1];
	sprintf(seq_buff, "%04d", curr_sequence);
	//Copy first FOUR bytes (no null terminator) back to packet structure
	strncpy(packet.sequence,seq_buff,4); 
}

/*
 *	FreeRTOS Tasks:
 *
 *		btDumpingTask(void *pvParameters): Timed task to dump the packet to the Bluetooth USART channel
 */


void StartBluetoothDumpTask(void *argument) {
	for(;;) {
		//Dump, Wait
		btSendPacket();
		logSensor((float)0 -1,MC_ACUAL_SPEED_REG_LOG);
		logSensor(mc_getBusCurrent(),MC_I_ACTUAL_LOG);
		logSensor((float)0,MC_BUS_VOLTAGE_LOG);

		if(get_car_state() == READY_TO_DRIVE) {
			vTaskDelay(BT_DUMP_DELAY);
		} else {
			vTaskDelay(LONG_BT_DUMP_DELAY);
		}
	}
}
