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
#include "types.h"

#define INPUT 0
#define OUTPUT 1

#define NUM_CH 6

#define TCNT2_BASE_1US 0xFE  // Clk/8
#define TCNT2_BASE_10US 0xEC // Clk/8
#define TCNT2_BASE_1MS 0xF0	 // Clk/1024

#define MAX_TIME_COUNT F_CPU >> 1

#define SERIAL_RX_BUFFER_SIZE 64

#define RC_MIN 1000
#define RC_MAX 2000
#define RC_MID 1500

/*
 * 디버그 관련
 */
//#define DEBUG_SYSTEMSTATUS
#define DEBUG_RECEIVERRC
#define DEBUG_AVRFLIGHTRC
#define DEBUG_FCRCINFO
//#define DEBUG_WRITERC
#define DEBUG_GPSINFO
#define DEBUG_ATTINFO
//#define DEBUG_EXTINT

/*
 * 전역 상수
 */
const uint8_t TCNT2_Value = TCNT2_BASE_1MS;

/*
 * 전역 변수
 */
// Ext. 인터럽트 관련
volatile bool FLAG_ExtIntRisingEdge[NUM_CH];
volatile bool FLAG_ExtIntReceivedPWM[NUM_CH];

// MSP 데이터 관련
//                   ROLL  PITCH YAW  THRO  AUX1  AUX2  AUX3  AUX4
//uint16_t RawRC[8] = {1500, 1500, 1500, 880, 1500, 1500, 1500, 1500};
msp_rc_t MspRC_FC;
msp_rc_t MspRC_MCU;
msp_gps_t MspGPS;
msp_att_t MspATT;

// AVR Flight 관련
volatile bool AvrFlight = false;
volatile uint8_t RcOverride;
msp_rc_t MspRC_AvrFlight = {
//             ROLL  PITCH YAW  THRO  AUX1  AUX2  AUX3  AUX4  AUX5  AUX6  AUX7  AUX8  AUX9  AUX10 AUX11 AUX12 AUX13 AUX14
	.rcData = {1500, 1500, 1500, 800, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}
};

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
typedef struct _hardwareSerial_t {
	uint8_t _rx_buffer[SERIAL_RX_BUFFER_SIZE];

	uint8_t _rx_buffer_head;
	uint8_t _rx_buffer_tail;
} HardwareSerial_t;

HardwareSerial_t HwSerial0 = {
	._rx_buffer_head = 0,
	._rx_buffer_tail = 0,
};

HardwareSerial_t HwSerial1 = {
	._rx_buffer_head = 0,
	._rx_buffer_tail = 0,
};

// MSP 관련
CommandData cData;

/*
 * 함수 선언
 */
int usartTxCharCh0(char, FILE*);
int usartTxCharCh1(char, FILE*);

bool timeDiff(unsigned long*, unsigned long);
bool getABit(uint8_t x, int n);

int serialAvailable(const HardwareSerial_t*);
int serialRead(HardwareSerial_t*);

void serialEvent();
void serialEvent1();

void avrFlight_arm();
void avrFlight_disarm();
bool avrFlight_status();

void writeRC(msp_rc_t*, msp_rc_t*, uint8_t);

/*
 * 파일 스트림
 */
FILE *fpStdio;

void setup() {
	/*
	 * 스트림 할당
	 */
	fpStdio = fdevopen(usartTxCharCh0, NULL); // stdout, stdin, stderr

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
	//TIMSK |= (1 << TOIE1); // block interrupt for T/C1

	/*
	 * Serial 0
	 */
	 UCSR0A |= (1 << U2X);    // Double the USARTn transmission speed
	 UCSR0B |= (1 << RXCIE) | // Grant RX complete interrupt
			   (1 << RXEN) |  // RX/TX enable
			   (1 << TXEN);
	 UCSR0C |= (1 << UCSZ00) | // 8bit
			   (1 << UCSZ01);
	 UBRR0H = 0x00;			   // Baudrate: 38,400
	 UBRR0L = 0x33;
	 
	 DDRE &= ~(1 << DDE0);
	 DDRE |= (1 << DDE1);

	/*
	 * Serial 1
	 */
	UCSR1A |= (1 << U2X);	  // Double the USARTn transmission speed
	UCSR1B |= (1 << RXCIE) |  // Grant RX complete interrupt
			  (1 << RXEN) |   // RX/TX enable
			  (1 << TXEN);
	UCSR1C |= (1 << UCSZ10) | // 8bit
			  (1 << UCSZ11);
	UBRR1H = 0x00;		      // Baudrate: 38,400
	UBRR1L = 0x33;

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
		
		/************************************************************************/
		/* PRINT SYSTEM STATUS                                                  */
		/************************************************************************/
		if (timeDiff(&PrevTimeDebug, 1000)) {
			printf_P(PSTR("SystemStatus "));
			printf("%lu", CurrTime);
			printf_P(PSTR("===== AVRFlight: "));
			printf("%d\r\n", avrFlight_status());
			#ifdef DEBUG_RECEIVERRC
			printf_P(PSTR(" R: ")); printf("%" PRIu16, MspRC_MCU.rcData[ROLL]);
			printf_P(PSTR(" P: ")); printf("%" PRIu16, MspRC_MCU.rcData[PITCH]);
			printf_P(PSTR(" Y: ")); printf("%" PRIu16, MspRC_MCU.rcData[YAW]);
			printf_P(PSTR(" T: ")); printf("%" PRIu16, MspRC_MCU.rcData[THROTTLE]);
			printf_P(PSTR(" A1: ")); printf("%" PRIu16, MspRC_MCU.rcData[AUX1]);
			printf_P(PSTR(" A2: ")); printf("%" PRIu16, MspRC_MCU.rcData[AUX2]); printf_P(PSTR("\r\n"));
			#endif

			#ifdef DEBUG_AVRFLIGHTRC
			printf_P(PSTR("AvrFlight RC - "));
			printf_P(PSTR(" R: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[ROLL]);
			printf_P(PSTR(" P: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[PITCH]);
			printf_P(PSTR(" Y: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[YAW]);
			printf_P(PSTR(" T: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[THROTTLE]);
			printf_P(PSTR(" A1: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX1]);
			printf_P(PSTR(" A2: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX2]);
			printf_P(PSTR(" A3: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX3]);
			printf_P(PSTR(" A4: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX4]);
			printf_P(PSTR(" A5: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX5]); printf_P(PSTR("\r\n"));
			printf_P(PSTR(" A6: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX6]);
			printf_P(PSTR(" A7: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX7]);
			printf_P(PSTR(" A8: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX8]);
			printf_P(PSTR(" A9: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX9]);
			printf_P(PSTR(" A10: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX10]);
			printf_P(PSTR(" A11: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX11]);
			printf_P(PSTR(" A12: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX12]);
			printf_P(PSTR(" A13: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX13]);
			printf_P(PSTR(" A14: ")); printf("%" PRIu16, MspRC_AvrFlight.rcData[AUX14]); printf_P(PSTR("\r\n"));
			#endif
		}

		/************************************************************************/
		/* PROCESSING RC DATA FROM RECEIVER                                     */
		/************************************************************************/
		for(uint8_t i = 0; i < NUM_CH; i++) {
			if(FLAG_ExtIntReceivedPWM[i]) {
				FLAG_ExtIntReceivedPWM[i] = false;
				int16_t diffExtIntTCNT1 = ExtIntFallingTCNT1[i] - ExtIntRisingTCNT1[i];

				// 동기를 맞추지 못한 경우
				if (diffExtIntTCNT1 > 4000) {
					continue;
				// 순방향
				} else if (diffExtIntTCNT1 > 0) {
					MspRC_MCU.rcData[i] = diffExtIntTCNT1 / 2;
				// 역방향
				} else {
					MspRC_MCU.rcData[i] = (0xFFFF + diffExtIntTCNT1) / 2;
				}
			}
		}

		// write msp command to usart1
		mspWrite(usartTxCharCh1);

		/************************************************************************/
		/* PROCESSING DATA FROM FC                                              */
		/************************************************************************/
		if(mspAvailable()) {
			cData = mspRetrieveCMD();

			switch(cData.command) {
			case MSP_RC:
				for (uint8_t i = 0; i < cData.size; i++) {
					MspRC_FC.rcData[i] = parseDataUint16(cData.data[2 * i], cData.data[(2 * i) + 1]);
				}

				#ifdef DEBUG_FCRCINFO
				printf_P(PSTR("FC RC - "));
				printf_P(PSTR(" R: ")); printf("%" PRIu16, MspRC_FC.rcData[ROLL]);
				printf_P(PSTR(" P: ")); printf("%" PRIu16, MspRC_FC.rcData[PITCH]);
				printf_P(PSTR(" Y: ")); printf("%" PRIu16, MspRC_FC.rcData[YAW]);
				printf_P(PSTR(" T: ")); printf("%" PRIu16, MspRC_FC.rcData[THROTTLE]);
				printf_P(PSTR(" A1: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX1]);
				printf_P(PSTR(" A2: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX2]);
				printf_P(PSTR(" A3: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX3]);
				printf_P(PSTR(" A4: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX4]); 
				printf_P(PSTR(" A5: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX5]); printf_P(PSTR("\r\n"));
				printf_P(PSTR(" A6: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX6]);
				printf_P(PSTR(" A7: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX7]);
				printf_P(PSTR(" A8: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX8]);
				printf_P(PSTR(" A9: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX9]);
				printf_P(PSTR(" A10: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX10]);
				printf_P(PSTR(" A11: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX11]);
				printf_P(PSTR(" A12: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX12]);
				printf_P(PSTR(" A13: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX13]);
				printf_P(PSTR(" A14: ")); printf("%" PRIu16, MspRC_FC.rcData[AUX14]); printf_P(PSTR("\r\n"));
				#endif

				break;
			case MSP_RAW_GPS:
				MspGPS.fix = parseDataUint8(cData.data[0]);
				MspGPS.num_sat = parseDataUint8(cData.data[1]);
				MspGPS.gpsData[LAT] = parseDataUint32(cData.data[2], cData.data[3], cData.data[4]);
				MspGPS.gpsData[LON] = parseDataUint32(cData.data[5], cData.data[6], cData.data[7]);
				MspGPS.altitude = parseDataUint16(cData.data[8], cData.data[9]);
				MspGPS.speed = parseDataUint16(cData.data[10], cData.data[11]);
				MspGPS.ground_course = parseDataUint16(cData.data[12], cData.data[13]);

				#ifdef DEBUG_GPSINFO
				printf_P(PSTR(" GPS Fix: ")); printf("%" PRIu8, MspGPS.fix);
				printf_P(PSTR(" numSat: ")); printf("%" PRIu8, MspGPS.num_sat);
				printf_P(PSTR(" lat: ")); printf("%" PRIu32, MspGPS.gpsData[LAT]);
				printf_P(PSTR(" lon: ")); printf("%" PRIu32, MspGPS.gpsData[LON]); printf_P(PSTR("\r\n"));
				#endif

				break;
			case MSP_ATTITUDE:
				MspATT.attData[ANGX] = parseDataUint16(cData.data[0], cData.data[1]);
				MspATT.attData[ANGY] = parseDataUint16(cData.data[2], cData.data[3]);
				MspATT.attData[HEADING] = parseDataUint16(cData.data[4], cData.data[5]);

				#ifdef DEBUG_ATTINFO
				printf_P(PSTR(" ATT ANGX: ")); printf("%" PRIu16, MspATT.attData[ANGX]);
				printf_P(PSTR(" ANGY: ")); printf("%", PRIu16, MspATT.attData[ANGY]);
				printf_P(PSTR(" HEADING: ")); printf("%", PRIu16, MspATT.attData[HEADING]); printf_P(PSTR("\r\n"));
				#endif

				break;
			case MSP_SET_RAW_RC:
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

		/************************************************************************/
		/* AVR Flight                                                           */
		/************************************************************************/
		if (MspRC_MCU.rcData[AUX2] > RC_MID)
		{
			avrFlight_arm();

			RcOverride = //(1 << ROLL) |
						 (1 << PITCH);
						 //(1 << THROTTLE) |
						 //(1 << YAW) |
						 //(1 << AUX1) |
						 //(1 << AUX3) |
						 //(1 << AUX4);
		} else {
			avrFlight_disarm();
		}

		/************************************************************************/
		/* WRITE RC DATA                                                        */
		/************************************************************************/
		if(timeDiff(&PrevTimeRC, 75)) {
			writeRC(&MspRC_MCU, &MspRC_AvrFlight, RcOverride);
			// Request RC information from FC
			// mspSendCmd(MSP_RC);
		}
		
		/************************************************************************/
		/* REQUEST GPS DATA                                                     */
		/************************************************************************/
		if(timeDiff(&PrevTimeGPS, 500)) {
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
	char ch;

	if (serialAvailable(&HwSerial0)) {
		ch = serialRead(&HwSerial0);

		switch (ch) {
		case 'a':
			mspSendCmd(MSP_API_VERSION);
			break;
		case 's':
			mspSendCmd(MSP_FC_VARIANT);
			break;
		case 'd':
			mspSendCmd(MSP_FC_VERSION);
			break;
		case 'f':
			mspSendCmd(MSP_RC);
			break;
		case 'g':
			mspSendCmd(MSP_ATTITUDE);
			break;
		default:
			printf(" %c", ch);
			break;	
		}
	}
}

/**
 * @brief Serial event from msp
 */
void serialEvent1() {
	char ch;

	if (serialAvailable(&HwSerial1)) {
		ch = serialRead(&HwSerial1);

		switch (ch) {
		default:
			mspReceiveCmd(ch);

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
	// If no parity error
	if (!(UCSR0A & (1 << UPE)))
	{
		unsigned char c = UDR0;

		uint8_t i = (HwSerial0._rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;

		if (i != HwSerial0._rx_buffer_tail) {
			HwSerial0._rx_buffer[HwSerial0._rx_buffer_head] = c;
			HwSerial0._rx_buffer_head = i;
		}
		} else {
		UDR0;
	}
}

/**
 * @brief USART1 RX complete ISR
 */
 ISR(USART1_RX_vect) {
	// If no parity error
	if (!(UCSR1A & (1 << UPE)))
	{
		unsigned char c = UDR1;

		uint8_t i = (HwSerial1._rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;

		if (i != HwSerial1._rx_buffer_tail) {
			HwSerial1._rx_buffer[HwSerial1._rx_buffer_head] = c;
			HwSerial1._rx_buffer_head = i;
		}
	} else {
		UDR1;
	}
 }
 
/**
 * @brief External interrupt 0 ISR
 */
ISR(INT0_vect) {
	#ifdef DEBUG_EXTINT
	printf_P(PSTR("ExtInterrupt0\r\n"));
	#endif

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
	#ifdef DEBUG_EXTINT
	printf_P(PSTR("ExtInterrupt1\r\n"));
	#endif

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
	#ifdef DEBUG_EXTINT
	printf_P(PSTR("ExtInterrupt4\r\n"));
	#endif

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
	#ifdef DEBUG_EXTINT
	printf_P(PSTR("ExtInterrupt5\r\n"));
	#endif

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
	#ifdef DEBUG_EXTINT
	printf_P(PSTR("ExtInterrupt6\r\n"));
	#endif

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
	#ifdef DEBUG_EXTINT
	printf_P(PSTR("ExtInterrupt7\r\n"));
	#endif

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
 * @brief USART0 write char
 */
int usartTxCharCh0(char ch, FILE *fp) {
	uint32_t count = 0;

	while (!(UCSR0A & (1 << UDRE))) {
		// write timeout
		if (count++ > MAX_TIME_COUNT)
		return -1;
	}

	UDR0 = ch;

	return 0;
}

/**
 * @brief USART1 write char
 */
int usartTxCharCh1(char ch, FILE *fp) {
	uint32_t count = 0;

	while (!(UCSR1A & (1 << UDRE))) {
		// write timeout
		if (count++ > MAX_TIME_COUNT)
		return -1;
	}

	UDR1 = ch;

	return 0;
}

/**
 * @brief 버퍼에 읽을 문자가 있는지 여부 확인
 */
int serialAvailable(const HardwareSerial_t *hwSerial) {
	return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + hwSerial->_rx_buffer_head - hwSerial->_rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
}

/**
 * @brief 버퍼에 있는 문자 읽음
 */
int serialRead(HardwareSerial_t *hwSerial) {
	if(hwSerial->_rx_buffer_head == hwSerial->_rx_buffer_tail) {
		return -1;
	} else {
		unsigned char c = hwSerial->_rx_buffer[hwSerial->_rx_buffer_tail];

		hwSerial->_rx_buffer_tail = (hwSerial->_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;

		return c;
	}
}

void writeRC(msp_rc_t* mcu_rc, msp_rc_t* avrFlight_rc, uint8_t rcOverride) {
	/*uint8_t rcData[36];

	for (int i = 0; i < 8; i++) {
		rcData[2 * i] = (avrFlight_status() && (rcOverride & (1 << ROLL))) ? (mcu_rc->rcData[i]) : (avrFlight_rc->rcData[i]);
		rcData[2 * i] = (avrFlight_status() && (rcOverride & (1 << ROLL))) ? (mcu_rc->rcData[i] >> 8) : (avrFlight_rc->rcData[i] >> 8);
	} */
	bool avrFlightStatus = avrFlight_status();

	uint8_t rcData[] = {
		(avrFlightStatus && getABit(rcOverride, ROLL)) ? (avrFlight_rc->rcData[ROLL]) : (mcu_rc->rcData[ROLL]),
		(avrFlightStatus && getABit(rcOverride, ROLL)) ? (avrFlight_rc->rcData[ROLL] >> 8) : (mcu_rc->rcData[ROLL] >> 8),
		(avrFlightStatus && getABit(rcOverride, PITCH)) ? (avrFlight_rc->rcData[PITCH]) : (mcu_rc->rcData[PITCH]),
		(avrFlightStatus && getABit(rcOverride, PITCH)) ? (avrFlight_rc->rcData[PITCH] >> 8) : (mcu_rc->rcData[PITCH] >> 8),
		(avrFlightStatus && getABit(rcOverride, THROTTLE)) ? (avrFlight_rc->rcData[THROTTLE]) : (mcu_rc->rcData[THROTTLE]),
		(avrFlightStatus && getABit(rcOverride, THROTTLE)) ? (avrFlight_rc->rcData[THROTTLE] >> 8) : (mcu_rc->rcData[THROTTLE] >> 8),
		(avrFlightStatus && getABit(rcOverride, YAW)) ? (avrFlight_rc->rcData[YAW]) : (mcu_rc->rcData[YAW]),
		(avrFlightStatus && getABit(rcOverride, YAW)) ? (avrFlight_rc->rcData[YAW] >> 8) : (mcu_rc->rcData[YAW] >> 8),
		(avrFlightStatus && getABit(rcOverride, AUX1)) ? (avrFlight_rc->rcData[AUX1]) : (mcu_rc->rcData[AUX1]),
		(avrFlightStatus && getABit(rcOverride, AUX1)) ? (avrFlight_rc->rcData[AUX1] >> 8) : (mcu_rc->rcData[AUX1] >> 8),
		(avrFlightStatus && getABit(rcOverride, AUX2)) ? (avrFlight_rc->rcData[AUX2]) : (mcu_rc->rcData[AUX2]),
		(avrFlightStatus && getABit(rcOverride, AUX2)) ? (avrFlight_rc->rcData[AUX2] >> 8) : (mcu_rc->rcData[AUX2] >> 8),
		(avrFlight_rc->rcData[AUX3]), (avrFlight_rc->rcData[AUX3] >> 8),
		(avrFlight_rc->rcData[AUX4]), (avrFlight_rc->rcData[AUX4] >> 8),
		(avrFlight_rc->rcData[AUX5]), (avrFlight_rc->rcData[AUX5] >> 8),
		(avrFlight_rc->rcData[AUX6]), (avrFlight_rc->rcData[AUX6] >> 8),
		(avrFlight_rc->rcData[AUX7]), (avrFlight_rc->rcData[AUX7] >> 8),
		(avrFlight_rc->rcData[AUX8]), (avrFlight_rc->rcData[AUX8] >> 8),
		(avrFlight_rc->rcData[AUX9]), (avrFlight_rc->rcData[AUX9] >> 8),
		(avrFlight_rc->rcData[AUX10]), (avrFlight_rc->rcData[AUX10] >> 8),
		(avrFlight_rc->rcData[AUX11]), (avrFlight_rc->rcData[AUX11] >> 8),
		(avrFlight_rc->rcData[AUX12]), (avrFlight_rc->rcData[AUX12] >> 8),
		(avrFlight_rc->rcData[AUX13]), (avrFlight_rc->rcData[AUX13] >> 8),
		(avrFlight_rc->rcData[AUX14]), (avrFlight_rc->rcData[AUX14] >> 8)
	};

	#ifdef DEBUG_WRITERC
	printf_P(PSTR(" WRITE RC - "));
	for (uint8_t i = 0; i < (sizeof(rcData) / sizeof(*rcData) / 2); i++) {
		printf(" %d", parseDataUint16(rcData[2 * i], rcData[(2 * i) + 1]));
	}
	printf_P(PSTR("\r\n"));
	#endif

	mspSendCmdData(MSP_SET_RAW_RC, (sizeof(rcData) / sizeof(*rcData)), rcData);
}

void avrFlight_arm() {
	AvrFlight = true;

	// Clear all RC override bits for safety
	RcOverride = 0;
}

void avrFlight_disarm() {
	AvrFlight = false;
}

bool avrFlight_status() {
	return AvrFlight;
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

bool getABit(uint8_t x, int n) {
	return ((x >> n) & 0x01);
}