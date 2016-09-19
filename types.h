/*
 * types.h
 *
 * Created: 2016-09-20 오전 12:08:09
 *  Author: SeoJu
 */ 

#pragma  once

/*
 * Enum
 */
enum RC_CHANS {
	ROLL, PITCH, YAW, THROTTLE,
	AUX1, AUX2, AUX3 , AUX4 , AUX5 , AUX6 , AUX7,
	AUX8, AUX9, AUX10, AUX11, AUX12, AUX13, AUX14
};

enum ATT {
	ANGX, ANGY, HEADING
};

enum GPS {
	LAT, LON
};

/*
 * 전역 타입
 */
typedef struct _MSP_RC_T {
	uint16_t rcData[18];
} msp_rc_t;

typedef struct _MSP_GPS_T {
	uint8_t fix;
	uint8_t num_sat;
	uint32_t gpsData[2];
	uint16_t altitude;
	uint16_t speed;
	uint16_t ground_course;
} msp_gps_t;

typedef struct _MSP_ATT_T {
	uint16_t attData[3];
} msp_att_t;