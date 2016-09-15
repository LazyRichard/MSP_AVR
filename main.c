/*
 * NAZE32_AVR.c
 *
 * Created: 2016-09-14 오후 6:44:23
 * Author : SeoJu
 */ 
#define F_CPU 16000000UL // 16MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "msp.h"

#define INPUT 0
#define OUTPUT 1

#define NUM_CH 6

#define TCNT2_BASE_1US 0xFE  // Clk/8
#define TCNT2_BASE_10US 0xEC // Clk/8
#define TCNT2_BASE_1MS 0xF0	 // Clk/1024

/*
 * 디버그 관련
 */
#define DEBUG_SYSTEMSTATUS

/*
 * Enum
 */
enum RC_CHANS {
	ROLL     = 0,
	PITCH    = 1,
	YAW      = 2,
	THROTTLE = 3,
	AUX1     = 4,
	AUX2     = 5,
	AUX3     = 6,
	AUX4     = 7
};

/*
 * 전역 상수
 */
const uint8_t TCNT2_Value = TCNT2_BASE_1MS;

/*
 * 전역 구조체
 */
typedef struct _MSP_RC_T {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t throttle;
	uint16_t aux1;
	uint16_t aux2;
	uint16_t aux3;
	uint16_t aux4;
} msp_rc_t;
typedef struct _MSP_GPS_T {
	uint8_t fix;
	uint8_t num_sat;
	uint32_t lat;
	uint32_t lon;
	uint16_t altitude;
	uint16_t speed;
	uint16_t ground_course;
} msp_gps_t;

typedef struct _MSP_ATT_T {
	uint16_t angx;
	uint16_t angy;
	uint16_t heading;
} msp_att_t;

/*
 * 전역 변수
 */
// Ext. 인터럽트 관련
volatile bool FLAG_ExtIntRisingEdge[NUM_CH];
volatile bool FLAG_ExtIntReceivedPWM[NUM_CH];

// MSP 데이터 관련
//                   ROLL  PITCH YAW  THRO  AUX1  AUX2  AUX3  AUX4
uint16_t RawRC[8] = {1500, 1500, 1500, 880, 1500, 1500, 1500, 1500};
msp_rc_t MspRC_FC = { .roll     = 1500,
					  .pitch    = 1500,
					  .yaw      = 1500,
					  .throttle = 880,
					  .aux1     = 1500,
					  .aux2     = 1500,
					  .aux3     = 1500,
					  .aux4     = 1500 };
msp_gps_t MspGPS;
msp_att_t MspATT;

// 시간 관련
unsigned long CurrTime = 0;

unsigned long PrevTimeDebug = 0;
unsigned long PrevTimeRC = 0;
unsigned long PrevTimeGPS = 0;
unsigned long PrevTimeUsartWriteCH0 = 0;
unsigned long PrevTimeUsartWriteCH1 = 0;

uint16_t ExtIntRisingTCNT1[NUM_CH];
uint16_t ExtIntFallingTCNT1[NUM_CH];

// USART 관련
volatile bool FLAG_SerialReceived[2];

unsigned char SerialChar[2];

// MSP 관련
CommandData cData;

/*
 * 함수 선언
 */
int usartRxCharCh0();
int usartTxCharCh0(char, FILE*);
int usartRxCharCh1();
int usartTxCharCh1(char, FILE*);

bool timeDiff(unsigned long*, unsigned long);

void serialEvent();
void serialEvent1();

/*
 * 파일 스트림
 */
FILE *fpStdio;
FILE *fpFC;

void setup() {
	/*
	 * 스트림 할당
	 */
	fpStdio = fdevopen(usartTxCharCh0, usartRxCharCh0); // stdout, stdin, stderr
	fpFC = fdevopen(usartTxCharCh1, usartRxCharCh1);

	/*
	 * Timer/Counter 2 1ms (system time)
	 */
	//TCCR2 = (1 << CS21); // Clk / 8
	TCCR2 = (1 << CS20) |  // Clk / 1024
			(1 << CS22);
	TCNT2 = TCNT2_Value;   // Setup initial value 1us
	TIMSK |= (1 << TOIE2); // Grant timer/counter 2 interrupt
	
	/*
	 * 16bit Timer/Counter 1 1us w/o interrupt
	 */
	TCCR1B |= (1 << CS11);   // Clk / 8
	//TIMSK |= (1 << TOIE1); // no interrupt

	/*
	 * Serial 0
	 */ 
	UCSR0A = 0x00;
	UCSR0B |= (1 << RXCIE0) | // Grant RX complete interrupt
			  (1 << RXEN0) |  // RX/TX enable
			  (1 << TXEN0);
	UCSR0C |= (1 << UCSZ00) | // 8bit
			  (1 << UCSZ01);
	//UBRR0H = 0x00;			  // Baudrate: 9,600
	//UBRR0L = 0x67;
	UBRR0H = 0x00;			  // Baudrate: 115,200
	UBRR0L = 0x08;
	
	DDRE &= ~(1 << DDE0);
	DDRE |= (1 << DDE1);

	/*
	 * Serial 1
	 */
	UCSR1A = 0x00;
	UCSR1B |= (1 << RXCIE1) | // Grant RX complete interrupt
			  (1 << RXEN1) |  // RX/TX enable
			  (1 << TXEN1);
	UCSR1C |= (1 << UCSZ10) | // 8bit
			  (1 << UCSZ11);
	UBRR1H = 0x00;		      // Baudrate: 115,200
	UBRR1L = 0x08;

	DDRD &= ~(1 << DDD2);
	DDRD |= (1 << DDD3);

	/*
	 * External Interrupts
	 */
	EICRA |= (1 << ISC00) | // Ext.Int 0 w rising edge
			 (1 << ISC01) |
			 (1 << ISC10) | // Ext.Int 1 w rising edge
			 (1 << ISC11);
	EICRB |= (1 << ISC40) | // Ext.Int 4 w rising edge
			 (1 << ISC41) |
			 (1 << ISC50) | // Ext.Int 5 w rising edge
			 (1 << ISC51) |
			 (1 << ISC60) | // Ext.Int 6 w rising edge
			 (1 << ISC61) |
			 (1 << ISC70) | // Ext.Int 7 w rising edge
			 (1 << ISC71);

	DDRD &= ~(1 << DDD0) |
			~(1 << DDD1);
	DDRE &= ~(1 << DDE4) |
			~(1 << DDE5) |
			~(1 << DDE6) |
			~(1 << DDE7);


	// Grant individual external interrupts
	EIMSK |= (1 << INT0) |
			 (1 << INT1) |
			 (1 << INT4) |
			 (1 << INT5) |
			 (1 << INT6) |
			 (1 << INT7);
}

int main() {
	setup();
	
	printf_P(PSTR("Start\r\n"));

	_delay_ms(50);

	// Grant global interrupt
	sei();
	
	while(true) {
		
		if (timeDiff(&PrevTimeDebug, 1000)) {
			printf_P(PSTR("SystemStatus "));
			printf("%lu", CurrTime);
			printf_P(PSTR("=====\r\n"));
			#ifdef DEBUG_SYSTEMSTATUS
			printf_P(PSTR(" R: ")); printf("%" PRIu16, RawRC[ROLL]);
			printf_P(PSTR(" P: ")); printf("%" PRIu16, RawRC[PITCH]);
			printf_P(PSTR(" Y: ")); printf("%" PRIu16, RawRC[YAW]);
			printf_P(PSTR(" T: ")); printf("%" PRIu16, RawRC[THROTTLE]);
			printf_P(PSTR(" A1: ")); printf("%" PRIu16, RawRC[AUX1]);
			printf_P(PSTR(" A2: ")); printf("%" PRIu16, RawRC[AUX2]); printf_P(PSTR("\r\n"));
			#endif
		}

		for(uint8_t i = 0; i < NUM_CH; i++) {
			if(FLAG_ExtIntReceivedPWM[i]) {
				FLAG_ExtIntReceivedPWM[i] = false;
				int16_t diffExtIntTCNT1 = ExtIntFallingTCNT1[i] - ExtIntRisingTCNT1[i];

				// 동기를 맞추지 못한 경우
				if (diffExtIntTCNT1 > 4000) {
					continue;
				// 순방향
				} else if (diffExtIntTCNT1 > 0) {
					RawRC[i] = diffExtIntTCNT1 - 1500;
				// 역방향
				} else {
					RawRC[i] = (0xFFFF + diffExtIntTCNT1) - 1500;
				}
			}
		}

		// write msp command to usart1
		mspWrite(usartTxCharCh1);

		if(mspAvailable()) {
			cData = mspRetrieveCMD();

			switch(cData.command) {
			case MSP_RC:
				MspRC_FC.roll = parseDataUint16(cData.data[0], cData.data[1]);
				MspRC_FC.pitch = parseDataUint16(cData.data[2], cData.data[3]);
				MspRC_FC.yaw = parseDataUint16(cData.data[4], cData.data[5]);
				MspRC_FC.throttle = parseDataUint16(cData.data[6], cData.data[7]);
				MspRC_FC.aux1 = parseDataUint16(cData.data[8], cData.data[9]);
				MspRC_FC.aux2 = parseDataUint16(cData.data[10], cData.data[11]);
				MspRC_FC.aux3 = parseDataUint16(cData.data[12], cData.data[13]);
				MspRC_FC.aux4 = parseDataUint16(cData.data[14], cData.data[15]);

				break;
			case MSP_RAW_GPS:
				MspGPS.fix = parseDataUint8(cData.data[0]);
				MspGPS.num_sat = parseDataUint8(cData.data[1]);
				MspGPS.lat = parseDataUint32(cData.data[2], cData.data[3], cData.data[4]);
				MspGPS.lon = parseDataUint32(cData.data[5], cData.data[6], cData.data[7]);
				MspGPS.altitude = parseDataUint16(cData.data[8], cData.data[9]);
				MspGPS.speed = parseDataUint16(cData.data[10], cData.data[11]);
				MspGPS.ground_course = parseDataUint16(cData.data[12], cData.data[13]);

				printf_P(PSTR(" GPS Fix: ")); printf("%" PRIu8, MspGPS.fix);
				printf_P(PSTR(" numSat: ")); printf("%" PRIu8, MspGPS.num_sat);
				printf_P(PSTR(" lat: ")); printf("%" PRIu32, MspGPS.lat);
				printf_P(PSTR(" lon: ")); printf("%" PRIu32, MspGPS.lon);

				break;
			default:
				printf_P(PSTR("CMD: ")); printf("%d", cData.command);
				printf_P(PSTR(" size: ")); printf("%d", cData.size);
				printf_P(PSTR(" data: "));
				for(uint8_t i = 0; i < cData.size; i++) {
					printf("%d ", cData.data[i]);
				}
				printf_P(PSTR("\r\n"));

				break;
			}
		}

		if(timeDiff(&PrevTimeRC, 50)) {
			uint8_t rcData[] = {RawRC[ROLL], RawRC[ROLL] >> 8, RawRC[PITCH], RawRC[PITCH] >> 8,
							    RawRC[YAW], RawRC[YAW] >> 8, RawRC[AUX1], RawRC[AUX1] >> 8,
								RawRC[AUX2], RawRC[AUX2] >> 8, RawRC[AUX3], RawRC[AUX3] >> 8,
								RawRC[AUX4], RawRC[AUX4] >> 8 };

			// Apply receiver RC to FC
			mspSendCmdData(MSP_SET_RAW_RC,sizeof(rcData) / sizeof(*rcData), rcData);
			// Request RC information from FC
			mspSendCmd(MSP_RC);
		}

		if(timeDiff(&PrevTimeGPS, 75)) {
			mspSendCmd(MSP_RAW_GPS);
		}

		serialEvent();
		serialEvent1();
	}

	return 0;
}

/**
 * @brief Serial event from debug console
 */
void serialEvent() {
	if(FLAG_SerialReceived[0]) {
		FLAG_SerialReceived[0] = false;

		switch(SerialChar[0]) {
		case 'a':
			mspSendCmd(MSP_API_VERSION);
			break;
		case 's':
			mspSendCmd(MSP_FC_VARIANT);
			break;
		case 'd':
			mspSendCmd(MSP_FC_VERSION);
			break;
		default:
			printf("%c", SerialChar[0]);
			break;
		}
	}
}

/**
 * @brief Serial event from msp
 */
void serialEvent1() {
	if(FLAG_SerialReceived[1]) {
		FLAG_SerialReceived[1] = false;


		switch(SerialChar[1]) {
		default:
			mspReceiveCmd(SerialChar[1]);
			printf("%c", SerialChar[1]); printf(".");
			break;
		}
	}
}

/**
 * @brief Timer/Counter 2 overflow ISR
 */
ISR(TIMER2_OVF_vect) {
	TCNT2 = TCNT2_Value;
	CurrTime++;
}

/**
 * @brief USART0 RX complete ISR
 */
ISR(USART0_RX_vect) {
	FLAG_SerialReceived[0] = true;

	SerialChar[0] = usartRxCharCh0();
}

/**
 * @brief USART1 RX complete ISR
 */
 ISR(USART1_RX_vect) {
	 FLAG_SerialReceived[1] = true;

	SerialChar[1] = usartRxCharCh0();

	mspReceiveCmd(SerialChar[1]);
 }
 
/**
 * @brief External interrupt 0 ISR
 */
ISR(INT0_vect) {
	const uint8_t index = 0;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTCNT1[index] = TCNT1;
		// 센싱 모드 Rising edge > Falling edge
		EICRA &= ~(1 << ISC00);
	} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTCNT1[index] = TCNT1;
		// 센싱 모드 Falling edge > Rising edge
		EICRA |= (1 << ISC00);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
}

/**
 * @brief External interrupt 1 ISR
 */
ISR(INT1_vect) {
	const uint8_t index = 1;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTCNT1[index] = TCNT1;
		// 센싱 모드 Rising edge > Falling edge
		EICRA &= ~(1 << ISC10);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTCNT1[index] = TCNT1;
		// 센싱 모드 Falling edge > Rising edge
		EICRA |= (1 << ISC10);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
}

/**
 * @brief External interrupt 4 ISR
 */
ISR(INT4_vect) {
	const uint8_t index = 2;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTCNT1[index] = TCNT1;
		// 센싱 모드 Rising edge > Falling edge
		EICRB &= ~(1 << ISC40);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTCNT1[index] = TCNT1;
		// 센싱 모드 Falling edge > Rising edge
		EICRB |= (1 << ISC40);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
}

/**
 * @brief External interrupt 5 ISR
 */
ISR(INT5_vect) {
	const uint8_t index = 3;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTCNT1[index] = TCNT1;
		// 센싱 모드 Rising edge > Falling edge
		EICRB &= ~(1 << ISC50);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTCNT1[index] = TCNT1;
		// 센싱 모드 Falling edge > Rising edge
		EICRB |= (1 << ISC50);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
}

/**
 * @brief External interrupt 6 ISR
 */
ISR(INT6_vect) {
	const uint8_t index = 4;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTCNT1[index] = TCNT1;
		// 센싱 모드 Rising edge > Falling edge
		EICRB &= ~(1 << ISC60);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTCNT1[index] = TCNT1;
		// 센싱 모드 Falling edge > Rising edge
		EICRB |= (1 << ISC60);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
}

/**
 * @brief External interrupt 7 ISR
 */
ISR(INT7_vect) {	
	const uint8_t index = 5;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTCNT1[index] = TCNT1;
		// 센싱 모드 Rising edge > Falling edge
		EICRB &= ~(1 << ISC70);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTCNT1[index] = TCNT1;
		// 센싱 모드 Falling edge > Rising edge
		EICRB |= (1 << ISC70);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
}


/**
 * @brief USART0 read char
 */
int usartRxCharCh0() {
	while(!(UCSR0A & (1 << RXC0)));
	
	UCSR0A &= ~(1 << RXC0);

	return UDR0;
}

/**
 * @brief USART0 write char
 */
int usartTxCharCh0(char ch, FILE *fp) {
	while (!(UCSR0A & (1 << UDRE0))) {
		// Write timeout
		if(timeDiff(&PrevTimeUsartWriteCH0, 500))
			break;
	}
	
	UDR0 = ch;

	return 0;
}

/**
 * @brief USART1 read char
 */
int usartRxCharCh1() {
	while(!(UCSR1A & (1 << RXC1)));

	return UDR1;
}

/**
 * @brief USART1 write char
 */
int usartTxCharCh1(char ch, FILE *fp) {
	while (!(UCSR1A & (1 << UDRE1))) {
		// Write timeout
		if(timeDiff(&PrevTimeUsartWriteCH1, 500))
			break;
	}
	
	UDR1 = ch;

	return 0;
}

/**
 * @brief 현재 시간과 이전 시간과의 차이를 비교해 원하는 시간 차가 나왔을 때,
 *        true를 반환함.
 */
bool timeDiff(unsigned long *PrevTime, unsigned long desiredTime) {
	if((CurrTime - *PrevTime) > desiredTime) {
		*PrevTime = CurrTime;

		return true;
	}
	
	return false;
}