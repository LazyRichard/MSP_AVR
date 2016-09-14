/*
 * NAZE32_AVR.c
 *
 * Created: 2016-09-14 오후 6:44:23
 * Author : SeoJu
 */ 
#define F_CPU 16000000UL // 16MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#define INPUT 0
#define OUTPUT 1

#define TCNT2_1US 2

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
 * 전역 변수
 */
// Ext. 인터럽트 관련
bool FLAG_ExtIntRisingEdge[8];
bool FLAG_ExtIntReceivedPWM[8];

uint16_t RawRC[8];

// 시간 관련
unsigned long CurrTime = 0;

unsigned long ExtIntRisingTime[8];
unsigned long ExtIntFallingTime[8];

unsigned long PrevTimeDebug = 0;

// USART 관련
bool FLAG_SerialReceived[2];

unsigned char SerialChar[2];

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
 * 스트림 지정
 */
FILE *fpStdio;
FILE *fpFC;

void setup() {
	fpStdio = fdevopen(usartTxCharCh0, usartRxCharCh0);
	fpFC = fdevopen(usartTxCharCh1, usartRxCharCh1);

	/*
	 * Timer/Counter 2 1us
	 */
	TCCR2 = (1 << CS21);   // Clk / 8
	TCNT2 = TCNT2_1US;      // Setup initial value 1us
	TIMSK |= (1 << TOIE2); // Grant timer/counter 2 interrupt
	
	/*
	 * Serial 0
	 */ 
	UCSR0A = 0x00;
	UCSR0B |= (1 << RXCIE0) | // Grant RX complete interrupt
			  (1 << RXEN0) |  // RX/TX enable
			  (1 << TXEN0);
	UCSR0C |= (1 << UCSZ00) | // 8bit
			  (1 << UCSZ01);
	UBRR0H = 0x00;			  // Baudrate: 9600
	UBRR0L = 0x67;
	
	/*
	 * Serial 1
	 */
	UCSR1A = 0x00;
	UCSR1B |= (1 << RXCIE1) | // Grant RX complete interrupt
			  (1 << RXEN1) |  // RX/TX enable
			  (1 << TXEN1);
	UCSR1C |= (1 << UCSZ10) | // 8bit
			  (1 << UCSZ11);
	UBRR1H = 0x00;		      // Baudrate: 9600
	UBRR1L = 0x67;

	DDRE &= ~(1 << DDE0);
	DDRE |= (1 << DDE1);

	/*
	 * External Interrupts
	 */
	EICRA |= (1 << ISC00) | // Ext.Int 0 w rising edge
			 (1 << ISC01) |
			 (1 << ISC10) | // Ext.Int 1 w rising edge
			 (1 << ISC11) |
			 (1 << ISC20) | // Ext.Int 2 w rising edge
			 (1 << ISC21) |
			 (1 << ISC30) | // Ext.Int 3 w rising edge
			 (1 << ISC31);
	EICRB |= (1 << ISC40) | // Ext.Int 4 w rising edge
			 (1 << ISC41) |
			 (1 << ISC50) | // Ext.Int 5 w rising edge
			 (1 << ISC51) |
			 (1 << ISC60) | // Ext.Int 6 w rising edge
			 (1 << ISC61);

	DDRD &= ~(1 << DDD0) |
			~(1 << DDD1) |
			~(1 << DDD2) |
			~(1 << DDD3);
	DDRE &= ~(1 << DDE4) |
			~(1 << DDE5) |
			~(1 << DDE6);

	// Grant individual external interrupts
	EIMSK |= (1 << INT0) |
			 (1 << INT1) |
			 (1 << INT2) |
			 (1 << INT3) |
			 (1 << INT4) |
			 (1 << INT5) |
			 (1 << INT6);
	
	// Grant global interrupt
	sei();
}

int main() {
	setup();
	
	_delay_ms(50);
	
	printf("Start\r\n");
	
	while(true) {
		
		#ifdef DEBUG_SYSTEMSTATUS
		if (timeDiff(&PrevTimeDebug, 500)) {
			printf("SystemStatus %lu =====\r\n", CurrTime);
			printf(" ROLL : %" PRIu16 "\r\n", RawRC[0]);
			printf(" PICTH: %" PRIu16 "\r\n", RawRC[1]);
			printf(" YAW  : %" PRIu16 "\r\n", RawRC[2]);
			printf(" AUX1 : %" PRIu16 "\r\n", RawRC[3]);
			printf(" AUX2 : %" PRIu16 "\r\n", RawRC[4]);
			printf(" AUX3 : %" PRIu16 "\r\n", RawRC[5]);
			printf(" AUX4 : %" PRIu16 "\r\n", RawRC[6]);
		}
		#endif

		for(size_t i = 0; i < (sizeof(FLAG_ExtIntReceivedPWM) / sizeof(*FLAG_ExtIntReceivedPWM)); i++) {
			if(FLAG_ExtIntReceivedPWM[i]) {
				unsigned long diffExtIntTime = ExtIntFallingTime[i] - ExtIntRisingTime[i];

				if (diffExtIntTime > 0) {
					RawRC[i] = diffExtIntTime;
				} else {
					RawRC[i] = -1 * diffExtIntTime;
				}
			
			}
		}
		serialEvent();
		serialEvent1();
	}

	return 0;
}

void serialEvent() {
	if(FLAG_SerialReceived[0]) {
		switch(SerialChar[0]) {
		default:
			break;
		}
	}
}

void serialEvent1() {
	return;
}

/**
 * @brief Timer/Counter 2 overflow ISR
 */
ISR(TIMER2_OVF_vect) {
	cli();
	
	TCNT2 = TCNT2_1US;
	CurrTime++;
	
	sei();
}

/**
 * @brief USART0 RX complete ISR
 */
ISR(USART0_RX_vect) {
	cli();
	 
	SerialChar[0] = usartRxCharCh0();
	FLAG_SerialReceived[0] = true;
	
	sei();
}

/**
 * @brief USART0 data register empty ISR
 */
ISR(USART0_UDRE_vect) {
	
}

/**
 * @brief USART1 RX complete ISR
 */
 ISR(USART1_RX_vect) {
	 
 }
 
 /**
  * @brief USART1 data register empty ISR
  */
 ISR(USART1_UDRE_vect) {
	 
 }
 
/**
 * @brief External interrupt 0 ISR
 */
ISR(INT0_vect) {
	cli();
	
	const uint8_t index = 0;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTime[index] = CurrTime;
		// 센싱 모드 Rising edge > Falling edge
		EICRA &= ~(1 << ISC00);
	} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTime[index] = CurrTime;
		// 센싱 모드 Falling edge > Rising edge
		EICRA |= (1 << ISC00);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
	
	sei();
}

/**
 * @brief External interrupt 1 ISR
 */
ISR(INT1_vect) {
	cli();
	
	const uint8_t index = 1;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTime[index] = CurrTime;
		// 센싱 모드 Rising edge > Falling edge
		EICRA &= ~(1 << ISC10);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTime[index] = CurrTime;
		// 센싱 모드 Falling edge > Rising edge
		EICRA |= (1 << ISC10);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
	
	sei();
}

/**
 * @brief External interrupt 2 ISR
 */
ISR(INT2_vect) {
	cli();
	
	const uint8_t index = 2;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTime[index] = CurrTime;
		// 센싱 모드 Rising edge > Falling edge
		EICRA &= ~(1 << ISC20);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTime[index] = CurrTime;
		// 센싱 모드 Falling edge > Rising edge
		EICRA |= (1 << ISC20);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
	
	sei();
}

/**
 * @brief External interrupt 3 ISR
 */
ISR(INT3_vect) {
	cli();
	
	const uint8_t index = 3;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTime[index] = CurrTime;
		// 센싱 모드 Rising edge > Falling edge
		EICRA &= ~(1 << ISC30);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTime[index] = CurrTime;
		// 센싱 모드 Falling edge > Rising edge
		EICRA |= (1 << ISC30);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
	
	sei();
}

/**
 * @brief External interrupt 4 ISR
 */
ISR(INT4_vect) {
	cli();
	
	const uint8_t index = 4;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTime[index] = CurrTime;
		// 센싱 모드 Rising edge > Falling edge
		EICRB &= ~(1 << ISC40);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTime[index] = CurrTime;
		// 센싱 모드 Falling edge > Rising edge
		EICRB |= (1 << ISC40);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
	
	sei();
}

/**
 * @brief External interrupt 5 ISR
 */
ISR(INT5_vect) {
	cli();
	
	const uint8_t index = 5;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTime[index] = CurrTime;
		// 센싱 모드 Rising edge > Falling edge
		EICRB &= ~(1 << ISC50);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTime[index] = CurrTime;
		// 센싱 모드 Falling edge > Rising edge
		EICRB |= (1 << ISC50);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
	
	sei();
}

/**
 * @brief External interrupt 6 ISR
 */
ISR(INT6_vect) {
	cli();
	
	const uint8_t index = 6;

	if(!FLAG_ExtIntRisingEdge[index]) {
		FLAG_ExtIntRisingEdge[index] = true;
		
		ExtIntRisingTime[index] = CurrTime;
		// 센싱 모드 Rising edge > Falling edge
		EICRB &= ~(1 << ISC60);
		} else {
		FLAG_ExtIntRisingEdge[index] = false;
		
		ExtIntFallingTime[index] = CurrTime;
		// 센싱 모드 Falling edge > Rising edge
		EICRB |= (1 << ISC60);
		
		FLAG_ExtIntReceivedPWM[index] = true;
	}
	
	sei();
}

/**
 * @brief USART0 read char
 */
int usartRxCharCh0() {
	while((UCSR0A & (1 << RXC0)) == 0) {
		return UDR0;
	}
	
	return 0;	
}

/**
 * @brief USART0 write char
 */
int usartTxCharCh0(char ch, FILE *fp) {
	while (!(UCSR0A & (1 << UDRE0)));
	
	UDR0 = ch;

	return 0;
}

/**
 * @brief USART1 read char
 */
int usartRxCharCh1() {
	while((UCSR1A & (1 << RXC1)) == 0) {
		return UDR1;
	}
	
	return 0;
}

/**
 * @brief USART1 write char
 */
int usartTxCharCh1(char ch, FILE *fp) {
	while (!(UCSR1A & (1 << UDRE1)));
	
	UDR1 = ch;

	return 0;
}

bool timeDiff(unsigned long *PrevTime, unsigned long desiredTime) {
	if((CurrTime - *PrevTime) > (desiredTime * 10)) {
		*PrevTime = CurrTime;

		return true;
	}
	
	return false;
}