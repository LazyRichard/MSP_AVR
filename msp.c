#include "msp.h"

void mspSendCmd(uint8_t cmd) {
	mspSendCmdData(cmd, 0, 0);	
}

void mspSendCmdData(uint8_t cmd, size_t size, const uint8_t *data) {
	uint8_t dataBuff[MSP_OUT_BUF_SIZE];
	uint8_t curser = 0;

	for(size_t i = 0; strnlen(MSP_PREAMBLE, MSP_OUT_BUF_SIZE); i++) {
		dataBuff[curser++] = *(data + i);
	}

	dataBuff[curser++] = '<';
	dataBuff[curser++] = (uint8_t)size;
	
	for(size_t i = 0; i < size; i++) {
		dataBuff[curser++] = *(data + i);
	}

	dataBuff[curser++] = calcChecksum(cmd, (uint8_t)size, &data);

	mspAddQueue((size_t)curser, dataBuff);
}

void mspAddQueue(size_t size, const uint8_t *data) {
	for(size_t i = 0; i < size; i++) {
		CommandQueue[queueCurser][i] = *(data + i);
	}

	queueCurser++;
}

void mspWrite() {
	for (int i = 0; i < (MSP_COMMAND_OFFSET + CommandQueue[queueCurser][3]); i++) {
		usartTxCharCh0(CommandQueue[queueCurser][i]);
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
		cData[cDataCurser].size;
		status = HEADER_SIZE;
		
		if(cData[cDataCurser].size > MSP_IN_BUF_SIZE)
			status = IDLE;
		
		break;
	case HEADER_SIZE;
		cData[cDataCurser].command;
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
	return (calcChecksum(cmd, size, *data) == checksum) ? true : false;
}