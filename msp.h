#pragma once

#define MSP_QUEUE_SIZE 10
#define MSP_IN_BUF_SIZE 70
#define MSP_OUT_BUF_SIZE 255
#define MSP_COMMAND_OFFSET 5

const char MSP_PREAMBLE = "$M";

enum MSP_CMD {
	MSP_API_VERSION 			 = 1,	// out message
	MSP_FC_VARIANT 				 = 2,	// out message
	MSP_FC_VERSION				 = 3,	// out message
	MSP_BOARD_INFO				 = 4,	// out message
	MSP_BUILD_INFO				 = 5,	// out message
	
	MSP_NAME					 = 10,	// out message
	MSP_SET_NAME				 = 11,	// in message
	
	MSP_MODE_RANGES				 = 34,	// out message
	MSP_SET_MODE_RANGE			 = 35,	// in message
	
	MSP_FEATURE					 = 36,	// out message
	MSP_SET_FEATURE				 = 37,	// in message
	
	MSP_BOARD_ALIGNMENT 		 = 38,	// out message
	MSP_SET_BOARD_ALIGNMENT		 = 39,	// in message
	
	MSP_CURRENT_METER_CONFIG	 = 40,	// out message
	MSP_SET_CURRENT_MEGER_CONFIG = 41,	// in message
	
	MSP_MIXER					 = 42,	// out message
	MSP_SET_MIXER				 = 43,	// in message
	
	MSP_RX_CONFIG				 = 44,	// out message
	MSP_SET_RX_CONFIG			 = 45,	// in message
	
	MSP_LED_COLORS				 = 46,	// out message
	MSP_SET_LED_COLORS			 = 47,	// in message
	
	MSP_LED_STRIP_CONFIG		 = 48,	// out message
	MSP_SET_LED_STRIP_CONFIG	 = 49,	// in message
	
	MSP_RSSI_CONFIG				 = 50,	// out message
	MSP_SET_RSSI_CONFIG			 = 51,	// in message
	
	MSP_ADJUSTMENT_RANGES		 = 52,	// out message
	MSP_SET_ADJUSTMENT_RANGE	 = 53,	// in message
	
	MSP_REBOOT					 = 68,	// in message
	
	MSP_DATAFLASH_SUMMARY		 = 70,	// out message
	MSP_DATAFLASH_READ			 = 71,	// out message
	MSP_DATAFLASH_ERASE			 = 72,	// in message
	
	MSP_LOOP_TIME				 = 73,	// out message
	MSP_SET_LOOP_TIME			 = 74,	// in message
	
	MSP_FAILSAFE_CONFIG			 = 75,	// out message
	MSP_SET_FAILSAFE_CONFIG		 = 76,	// in message
	
	MSP_RXFAIL_CONFIG			 = 77,	// out message
	MSP_SET_RXFAIL_CONFIG		 = 78,	// in message
	
	MSP_SDCARD_SUMMARY			 = 79,	// out message
	
	MSP_BLACKBOX_CONFIG			 = 80,	// out message
	MSP_SET_BLACKBOX_CONFIG		 = 81,	// in message
	
	MSP_TRANSPONDER_CONFIG		 = 82,	// out message
	MSP_SET_TRANSPONDER_CONFIG	 = 83,	// in message
	
	MSP_OSD_CHAR_READ			 = 86,	// out message
	MSP_OSD_CHAR_WRITE			 = 87,	// in message
	
	MSP_VTX_CONFIG				 = 88,	// out message
	MSP_SET_VTX_CONFIG			 = 89,	// in message
	
	//
	// Multiwii original MSP commands
	//
	MSP_STATUS					 = 101,	// out message
	MSP_RAW_IMU					 = 102,	// out message
	MSP_SERVO					 = 103,	// out message
	MSP_MOTOR					 = 104,	// out message
	MSP_RC						 = 105,	// out message
	MSP_RAW_GPS					 = 106,	// out message
	MSP_COMP_GPS				 = 107,	// out message
	MSP_ATTITUDE				 = 108,	// out message
	MSP_ALTITUDE				 = 109,	// out message
	MSP_ANALOG					 = 110,	// out message
	MSP_RC_TUNING				 = 111,	// out message
	MSP_PID						 = 112,	// out message
	MSP_BOX						 = 113, // out message
	MSP_MISC					 = 114,	// out message
	MSP_MOTOR_PINS				 = 115,	// out message
	MSP_BOXNAMES				 = 116,	// out message
	MSP_PIDNAMES				 = 117,	// out message
	MSP_WP						 = 118,	// out message
	MSP_BOXIDS					 = 119,	// out message
	MSP_SERVO_CONFIGURATIONS	 = 120,	// out message
	MSP_NAV_STATUS				 = 121,	// out message
	MSP_NAV_CONFIG				 = 122,	// out message
	MSP_3D						 = 124,	// out message
	MSP_RC_DEADBAND				 = 125,	// out message
	MSP_SENSOR_ALIGNMENT		 = 126,	// out message
	MSP_LED_STRIP_MODECOLOR		 = 127,	// out message
	MSP_VOLTAGE_METERS			 = 128,	// out message
	MSP_BATTERY_STATES			 = 130,	// out message
	MSP_PILOT					 = 131,	// out message
	
	MSP_SET_RAW_RC				 = 200,	// in message
	MSP_SET_RAW_GPS				 = 201, // in message
	MSP_SET_PID					 = 202, // in message
	MSP_SET_BOX					 = 203, // in message
	MSP_SET_RC_TUNING			 = 204, // in message
	MSP_ACC_CALIBRATION			 = 205, // in message
	MSP_MAG_CALIBRATION			 = 206, // in message
	MSP_SET_MISC				 = 207, // in message
	MSP_RESET_CONF				 = 208, // in message
	MSP_SET_WP					 = 209, // in message
	MSP_SELCT_SETTING			 = 210, // in message
	MSP_SET_HEAD				 = 211, // in message
	MSP_SET_SERVO_CONFIGURATION	 = 212, // in message
	MSP_SET_MOTOR				 = 214, // in message
	MSP_SET_NAV_CONFIG			 = 215, // in message
	MSP_SET_3D					 = 217, // in message
	MSP_SET_RC_DEADBAND			 = 218, // in message
	MSP_SET_RESET_CURR_PID		 = 219, // in message
	MSP_SET_SENSOR_ALIGNMENT	 = 220, // in message
	MSP_SET_LED_STRIP_MODECOLOR	 = 221, // in message
	MSP_SET_PILOT				 = 222 // in message
};

enum Status {
	IDLE,
	HEADER_START,
	HEADER_M,
	HEADER_DIR,
	HEADER_SIZE,
	CMD,
	DATA
};

typedef struct _COMMANDDATA {
	uint8_t command;
	uint8_t size;
	uint8_t data[MSP_IN_BUF_SIZE];
} CommandData;

// private
bool FLAG_CommandReceived;

uint8_t CommandQueue[MSP_QUEUE_SIZE][MSP_IN_BUF_SIZE];

enum Status status;
uint8_t queueCurser;
uint8_t cDataCurser;
uint8_t bufCurser;

uint8_t calcChecksum(uint8_t, uint8_t, const uint8_t*);
bool verifyChecksum(uint8_t, uint8_t, const uint8_t*, uint8_t);

// public
CommandData cData[MSP_QUEUE_SIZE];

void mspSendCmd;(uint8_t);
void mspSendCmdData(uint8_t, size_t, const uint8_t*);
void mspAddQueue(size_t, const uint8_t*);
void mspWrite();
void mspReceiveCmd(char);
CommandData mspRetrieveCMD();
int mspAvailable();

uint8_t parseDataUint8(uint8_t);
int8_t parseDataInt8(uint8_t);
uint16_t parseDataUint16(uint8_t, uint8_t);
int16_t parseDataInt16(uint8_t, uint8_t);