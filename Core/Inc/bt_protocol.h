#ifndef BT_PROTOCOL_H_
#define BT_PROTOCOL_H_

#include "logger.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "logger.h"

//Protocol start and end flags
#define BT_START_FLAG 0x01
#define BT_END_FLAG 0x17

//Boolean values
#define BT_VALUE_TRUE 0x01
#define BT_VALUE_FALSE 0x00

//Configure the Bluetooth module
typedef enum BLUETOOTHMODE{
    BT_APPLICATION = 110,
    BT_TEST = 010,
    BT_WRITE = 001
}BTMODE;

typedef enum Datatype {
	BT_DT_BYTE,		//0
	BT_DT_SHORT,	//1
	BT_DT_INT,		//2
	BT_DT_LONG,		//3
	BT_DT_FLOAT,	//4
	BT_DT_DOUBLE,	//5
	BT_DT_CHAR,		//6
	BT_DT_STRING,	//7
	BT_DT_BOOLEAN	//8
} DATATYPE;

//Struct for each data entry, 16 bytes each
typedef struct Data {
	char id[3];
	char type[1];
	char scalar[2];
	char value[50];
} DATA;

//Struct for the entire packet, 21 + (x6*(NUM_OF_SENSORS+NUM_OF_INDICATORS)) bytes
typedef struct Packet {
	char start;
	char sequence[4];
	char ack[1];
	char header[4];
	DATA data[NUM_OF_SENSORS+NUM_OF_INDICATORS];
	char expan[5];
	char checksum[5];
	char end;
} PACKET;

//Prototypes
bool btInitialize();
bool btTerminate();
bool btUpdateData(void *value, SENSOR name);
void btSendPacket();
void btDumpingTask(void *pvParameters);
extern osThreadId_t btDumpTaskHandle;
extern const osThreadAttr_t btDumpTask_attributes;
void btLogIndicator(bool value, INDICATOR indc);
void btLogErrorMessage(char *data, INDICATOR indc);
void btLogSensor(float value, SENSOR sens);
void btModeConfig(BTMODE mode);

#endif
