#include "msp.h"

volatile bool FLAG_CommandReceived = false;

uint8_t CommandQueue[MSP_QUEUE_SIZE][MSP_IN_BUF_SIZE];
CommandData cData[MSP_QUEUE_SIZE];

enum Status status = IDLE;
volatile uint8_t queueCurser = 0;
volatile uint8_t cDataCurser = 0;
volatile uint8_t bufCurser = 0;

uint8_t calcChecksum(uint8_t, uint8_t, const uint8_t*);
bool verifyChecksum(uint8_t, uint8_t, const uint8_t*, uint8_t);

void mspSendCmd(uint8_t cmd) {
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

	mspAddQueue(curser, dataBuff);
}

void mspAddQueue(size_t size, const uint8_t *data) {
	for(size_t i = 0; i < size; i++) {
		CommandQueue[queueCurser][i] = *(data + i);
	}

	queueCurser = (queueCurser + 1) % MSP_QUEUE_SIZE;
}

void mspWrite(int (*fPtr)(char, FILE*)) {
	if(queueCurser) {
		for (int i = 0; i < (MSP_COMMAND_OFFSET + CommandQueue[queueCurser][3]); i++) {
			fPtr(CommandQueue[queueCurser][i], 0);
			printf(" %d", CommandQueue[queueCurser][i]);
		}
		queueCurser--;
	}
}

void mspReceiveCmd(char ch) {
	switch(status) {
	case IDLE:
		status = (ch == '$') ? HEADER_START : IDLE;
		
		break;
	case HEADER_START:
		status = (ch == 'M') ? HEADER_M : IDLE;
		
		break;
	case HEADER_M:
		status = (ch == '>') ? HEADER_DIR : IDLE;
		
		break;
	case HEADER_DIR:
		cData[cDataCurser].size = ch;
		status = HEADER_SIZE;
		
		if(cData[cDataCurser].size > MSP_IN_BUF_SIZE)
			status = IDLE;
		
		break;
	case HEADER_SIZE:
		cData[cDataCurser].command = ch;
		bufCurser = 0;
		status = CMD;
		
		break;
	case CMD:
		cData[cDataCurser].data[bufCurser++] = ch;
		
		if (bufCurser > cData[cDataCurser].size)
			status = DATA;
		
		break;
	case DATA:
		if (verifyChecksum(cData[cDataCurser].command, cData[cDataCurser].size, cData[cDataCurser].data, ch)) {
			cDataCurser++;
			
			status = IDLE;
		} else {
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