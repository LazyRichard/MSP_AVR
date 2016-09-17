#include "msp.h"

//#define DEBUG_MSP
//#define DEBUG_MSP_SIMPLE
//#define DEBUG_MSP_INPUT
//#define DEBUG_MSP_INPUT_SIMPLE


volatile bool FLAG_CommandReceived = false;

uint8_t CommandQueue[MSP_QUEUE_SIZE][MSP_OUT_BUF_SIZE];
CommandData cData[MSP_QUEUE_SIZE];

enum Status status = IDLE;
volatile uint8_t queueCursor = 0;
volatile uint8_t cDataCursor = 0;
volatile uint8_t bufCursor = 0;

// private
uint8_t calcChecksum(uint8_t, uint8_t, const uint8_t*);
bool verifyChecksum(uint8_t, uint8_t, const uint8_t*, uint8_t);

/**
 * @brief 데이터가 없는 단순 msp 명령의 전송 예약
 */
void mspSendCmd(uint8_t cmd) {
	#ifdef DEBUG_MSP_SIMPLE
	printf_P(PSTR("mspSendCmd\r\n"));
	#endif

	mspSendCmdData(cmd, 0, 0);	
}

/**
 * @brief 데이터가 있는 msp 명령의 전송 예약
 */
void mspSendCmdData(uint8_t cmd, size_t size, const uint8_t *data) {
	uint8_t dataBuff[MSP_OUT_BUF_SIZE];
	size_t cursor = 0;

	dataBuff[cursor++] = '$';
	dataBuff[cursor++] = 'M';
	dataBuff[cursor++] = '<';
	dataBuff[cursor++] = (uint8_t)size;
	dataBuff[cursor++] = cmd;
	
	for(size_t i = 0; i < size; i++) {
		dataBuff[cursor++] = *(data + i);
	}

	dataBuff[cursor++] = calcChecksum(cmd, (uint8_t)size, data);

	#ifdef DEBUG_MSP
	printf_P(PSTR("dataBuff: "));
	for(size_t i = 0; i < cursor; i++) {
		printf(" %d", dataBuff[i]);
	}
	printf("\r\n");
	#endif

	mspAddQueue(cursor, dataBuff);
}

/**
 * @brief msp 명령을 전송 큐에 저장
 */
void mspAddQueue(size_t size, const uint8_t *data) {
	#ifdef DEBUG_MSP	
	printf_P(PSTR("mspAddQueue:"));
	#endif

	for(size_t i = 0; i < size; i++) {
		CommandQueue[queueCursor][i] = *(data + i);
		
		#ifdef DEBUG_MSP	
		printf(" %d", *(data + i));
		#endif
	}

	#ifdef DEBUG_MSP
	printf_P(PSTR("\r\n"));
	#endif

	queueCursor = (queueCursor + 1) % MSP_QUEUE_SIZE;
}

/**
 * @brief 전송 큐에 있는 명령들을 시리얼에 출력하는 함수
 */
void mspWrite(int (*fPtr)(char, FILE*)) {
	if(queueCursor) {
		queueCursor--;

		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("Command Write:"));
		printf("%d ", MSP_COMMAND_OFFSET + CommandQueue[queueCursor][3]);
		#endif

		for (int i = 0; i < (MSP_COMMAND_OFFSET + CommandQueue[queueCursor][3]); i++) {
			#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
			printf(" %d", CommandQueue[queueCursor][i]);
			#endif

			fPtr(CommandQueue[queueCursor][i], NULL);
		}

		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf("\r\n");
		#endif
	}
}

/**
 * @brief USART를 통해 입력 받은 msp 데이터 처리
 */
void mspReceiveCmd(char ch) {
	switch(status) {
	case IDLE:
		#ifdef DEBUG_MSP_INPUT
		printf_P(PSTR("MSPReceive IDLE\r\n"));
		#endif

		status = (ch == '$') ? HEADER_START : IDLE;
		
		break;
	case HEADER_START:
		#ifdef DEBUG_MSP_INPUT
		printf_P(PSTR("MSPReceive HEADER_START\r\n"));
		#endif

		status = (ch == 'M') ? HEADER_M : IDLE;
		
		break;
	case HEADER_M:
		#ifdef DEBUG_MSP_INPUT
		printf_P(PSTR("MSPReceive HEADER_M\r\n"));
		#endif

		status = (ch == '>') ? HEADER_DIR : IDLE;
		
		break;
	case HEADER_DIR:
		#ifdef DEBUG_MSP_INPUT
		printf_P(PSTR("MSPReceive HEADER_DIR\r\n"));
		#endif

		cData[cDataCursor].size = ch;
		status = HEADER_SIZE;
		
		if(cData[cDataCursor].size > MSP_IN_BUF_SIZE)
			status = IDLE;
		
		break;
	case HEADER_SIZE:
		#ifdef DEBUG_MSP_INPUT
		printf_P(PSTR("MSPReceive HEADER_SIZE\r\n"));
		#endif

		cData[cDataCursor].command = ch;
		bufCursor = 0;
		status = CMD;
		
		break;
	case CMD:
		#ifdef DEBUG_MSP_INPUT
		printf_P(PSTR("MSPReceive CMD: "));
		printf("%d\r\n", ch);
		#endif

		if (bufCursor > cData[cDataCursor].size - 1) {
			status = DATA;
		} else {
			cData[cDataCursor].data[bufCursor++] = ch;
			break;
		}
		
	case DATA:
		#if defined(DEBUG_MSP_INPUT) || defined(DEBUG_MSP_INPUT_SIMPLE)
		printf_P(PSTR("MSPReceive DATA\r\n"));
		#endif

		if (verifyChecksum(cData[cDataCursor].command, cData[cDataCursor].size, cData[cDataCursor].data, ch)) {

			#if defined(DEBUG_MSP_INPUT) || defined(DEBUG_MSP_INPUT_SIMPLE)
			printf_P(PSTR("MSPReceive Data: "));
			printf("%d.%d.", cData[cDataCursor].command, cData[cDataCursor].size);
			for(uint8_t i = 0; i < cData[cDataCursor].size; i++) {
			printf("%d.", cData[cDataCursor].data[i]);
			}
			printf("\r\n");
			#endif

			cDataCursor++;
		} else {
			#if defined(DEBUG_MSP_INPUT) || defined(DEBUG_MSP_INPUT_SIMPLE)
			printf_P(PSTR("MSPReceive Verify checksum failed\r\n"));
			#endif
		}

		status = IDLE;
	}
}

/**
 * @brief 무결성이 확인된 msp 명령을 반환
 */
CommandData mspRetrieveCMD() {
	return (cDataCursor < 0) ? cData[0] : cData[--cDataCursor];
}

/**
 * @brief 입력 받은 msp 명령들이 존재하는지 반환
 */
int mspAvailable() {
	return cDataCursor;
}

/**
 * @brief 체크섬 계산 함수
 */
uint8_t calcChecksum(uint8_t cmd, uint8_t size, const uint8_t *data) {
	uint8_t checksum = 0;
	
	checksum ^= cmd;
	checksum ^= size;
	
	for (uint8_t i = 0; i < size; i++)
		checksum ^= *(data + i);

	return checksum;
}

/**
 * @brief 체크섬의 무결성을 확인하는 함수
 */
bool verifyChecksum(uint8_t cmd, uint8_t size, const uint8_t *data, uint8_t checksum) {
	return (calcChecksum(cmd, size, data) == checksum) ? true : false;
}

/************************************************************************/
/* 데이터 처리 함수                                                     */
/************************************************************************/
uint8_t parseDataUint8(uint8_t data) {
	return data;
}

int8_t parseDataInt8(uint8_t data) {
	return (int8_t)data;
}

uint16_t parseDataUint16(uint8_t data1, uint8_t data2) {
	return ((uint16_t)data2 << 8) + data1;
}

int16_t parseDataInt16(uint8_t data1, uint8_t data2) {
	return ((int16_t)data2 << 8) + data1;
}

uint32_t parseDataUint32(uint8_t data1, uint8_t data2, uint8_t data3) {
	return ((uint32_t)data3 << 16) + ((uint32_t)data2 << 8) + data1;
}

int32_t parseDataInt32(uint8_t data1, uint8_t data2, uint8_t data3) {
	return ((int32_t)data3 << 16) + ((int32_t)data2 << 8) + data1;
}