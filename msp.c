#include "msp.h"

void receiveCMD(char ch) {
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
		mspDataSize = ch;
		status = HEADER_SIZE;
		
		if(mspDataSize > MSP_IN_BUF_SIZE)
			status = IDLE;
		
		break;
	case HEADER_SIZE;
		mspCommand = ch;
		bufCurser = 0;
		status = CMD;
		
		break;
	case CMD:
		mspDataBuff[bufCurser++] = ch;
		
		if (bufCurser > mspDataSize)
			status = DATA;
		
		break;
	case DATA:
		if (verifyChecksum(mspCommand, mspDataSize, mspDataBuff, ch)) {
			FLAG_CommandReceived = true;
			
			status = IDLE;
		} else {
			status = IDLE;
		}
	}
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