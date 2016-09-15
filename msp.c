#include "msp.h"

#define DEBUG_MSP
#define DEBUG_MSP_SIMPLE

volatile bool FLAG_CommandReceived = false;

uint8_t CommandQueue[MSP_QUEUE_SIZE][MSP_OUT_BUF_SIZE];
CommandData cData[MSP_QUEUE_SIZE];

enum Status status = IDLE;
volatile uint8_t queueCurser = 0;
volatile uint8_t cDataCurser = 0;
volatile uint8_t bufCurser = 0;

uint8_t calcChecksum(uint8_t, uint8_t, const uint8_t*);
bool verifyChecksum(uint8_t, uint8_t, const uint8_t*, uint8_t);

void mspSendCmd(uint8_t cmd) {
	printf_P(PSTR("mspSendCmd\r\n"));

	mspSendCmdData(cmd, 0, 0);	
}

void mspSendCmdData(uint8_t cmd, size_t size, const uint8_t *data) {
	uint8_t dataBuff[MSP_OUT_BUF_SIZE];
	size_t curser = 0;

	dataBuff[curser++] = '$';
	dataBuff[curser++] = 'M';
	dataBuff[curser++] = '<';
	dataBuff[curser++] = (uint8_t)size;
	dataBuff[curser++] = cmd;
	
	for(size_t i = 0; i < size; i++) {
		dataBuff[curser++] = *(data + i);
	}

	dataBuff[curser++] = calcChecksum(cmd, (uint8_t)size, data);

	#ifdef DEBUG_MSP
	printf_P(PSTR("dataBuff: "));
	for(size_t i = 0; i < curser; i++) {
		printf(" %d", dataBuff[i]);
	}
	printf("\r\n");
	#endif

	mspAddQueue(curser, dataBuff);
}

void mspAddQueue(size_t size, const uint8_t *data) {
	#ifdef DEBUG_MSP	
	printf_P(PSTR("mspAddQueue:"));
	#endif

	for(size_t i = 0; i < size; i++) {
		CommandQueue[queueCurser][i] = *(data + i);
		
		#ifdef DEBUG_MSP	
		printf(" %d", *(data + i));
		#endif
	}

	#ifdef DEBUG_MSP
	printf_P(PSTR("\r\n"));
	#endif

	queueCurser = (queueCurser + 1) % MSP_QUEUE_SIZE;
}

void mspWrite(int (*fPtr)(char, FILE*)) {
	if(queueCurser) {
		queueCurser--;

		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("Command Queue:"));
		#endif

		for (int i = 0; i < (MSP_COMMAND_OFFSET + CommandQueue[queueCurser][3]); i++) {
			fPtr(CommandQueue[queueCurser][i], 0);

			#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
			printf(" %d", CommandQueue[queueCurser][i]);
			#endif
		}

		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf("\r\n");
		#endif
	}
}

void mspReceiveCmd(char ch) {
	switch(status) {
	case IDLE:
		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("MSPReceive IDLE\r\n"));
		#endif

		status = (ch == '$') ? HEADER_START : IDLE;
		
		break;
	case HEADER_START:
		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("MSPReceive HEADER_START\r\n"));
		#endif

		status = (ch == 'M') ? HEADER_M : IDLE;
		
		break;
	case HEADER_M:
		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("MSPReceive HEADER_M\r\n"));
		#endif

		status = (ch == '>') ? HEADER_DIR : IDLE;
		
		break;
	case HEADER_DIR:
		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("MSPReceive HEADER_DIR\r\n"));
		#endif

		cData[cDataCurser].size = ch;
		status = HEADER_SIZE;
		
		if(cData[cDataCurser].size > MSP_IN_BUF_SIZE)
			status = IDLE;
		
		break;
	case HEADER_SIZE:
		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("MSPReceive HEADER_SIZE\r\n"));
		#endif

		cData[cDataCurser].command = ch;
		bufCurser = 0;
		status = CMD;
		
		break;
	case CMD:
		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("MSPReceive CMD\r\n"));
		#endif

		cData[cDataCurser].data[bufCurser++] = ch;
		
		if (bufCurser > cData[cDataCurser].size)
			status = DATA;
		
		break;
	case DATA:
		#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
		printf_P(PSTR("MSPReceive DATA\r\n"));
		#endif

		if (verifyChecksum(cData[cDataCurser].command, cData[cDataCurser].size, cData[cDataCurser].data, ch)) {

			#ifdef DEBUG_MSP
			printf_P(PSTR("MSPReceive Data: "));
			printf("%d.%d.", cData[cDataCurser].command, cData[cDataCurser].size);
			for(uint8_t i = 0; i < cData[cDataCurser].size; i++) {
			printf("%d.", cData[cDataCurser].data[i]);
			}
			printf("\r\n");
			#endif

			cDataCurser++;
			
			status = IDLE;
		} else {
			#if defined(DEBUG_MSP) || defined(DEBUG_MSP_SIMPLE)
			printf_P(PSTR("MSPReceive Verify checksum failed\r\n"));
			#endif

			status = IDLE;
		}
	}
}

CommandData mspRetrieveCMD() {
	return (cDataCurser < 0) ? cData[0] : cData[cDataCurser--];
}

int mspAvailable() {
	return cDataCurser;
}

uint8_t calcChecksum(uint8_t cmd, uint8_t size, const uint8_t *data) {
	uint8_t checksum = 0;
	
	checksum ^= cmd;
	checksum ^= size;
	
	for (uint8_t i = 0; i < size; i++)
		checksum ^= *(data + i);

	return checksum;
}

bool verifyChecksum(uint8_t cmd, uint8_t size, const uint8_t *data, uint8_t checksum) {
	return (calcChecksum(cmd, size, data) == checksum) ? true : false;
}