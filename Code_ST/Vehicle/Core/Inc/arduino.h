/*
 * arduino.h
 *
 *  Created on: Oct 27, 2020
 *      Author: Dang Nam
 */

#ifndef INC_ARDUINO_H_
#define INC_ARDUINO_H_

#include "def_myself.h"
#include "ringbuffer.h"
#include "debug_cmd.h"

union ArrayToInt {
    uint8_t array[2];
    int32_t integer;
};
//ArrayToInt _arrayToInt;

union ArrayToLong {
    uint8_t array[4];
    int64_t integer;
};

//ArrayToLong _arrayToLong;


#define CMD_SET_FEATURE 108

//#pragma region AT_REGION_CRC_16
void InitChecksum();
uint16_t ComputeChecksum(uint8_t bytes[], uint32_t offset, uint32_t length);
uint16_t ComputeChecksum2(uint16_t crc, uint8_t b);
//#pragma endregion

#define SERIAL_RECEIVE_MAX_BYTES 64

//#pragma region AT_REGION_Command
#define CMD_SYNC_BYTE_1 0xA5 // 165
#define CMD_SYNC_BYTE_2 0x5A // 90
//#pragma endregion


//#pragma region AT_REGION_TCP_read_state
#define SRS_SYNC_BYTE_1 1
#define SRS_SYNC_BYTE_2 2

#define SRS_TCP_CMD_FLAG  3
#define SRS_DATA_LEN_PADDING 4 // 0 or 1

#define SRS_DATA_LEN_PLUS_PADDING_BYTE_0 10
#define SRS_DATA_LEN_PLUS_PADDING_BYTE_1 11
#define SRS_DATA_LEN_PLUS_PADDING_BYTE_2 12
#define SRS_DATA_LEN_PLUS_PADDING_BYTE_3 13

#define SRS_CHECKSUM_HEADER_BYTE_0 20
#define SRS_CHECKSUM_HEADER_BYTE_1 21

#define SRS_DATA 30

#define SRS_CHECKSUM_DATA_BYTE_0 40
#define SRS_CHECKSUM_DATA_BYTE_1 41
//#pragma endregion

//#pragma region AT_REGION_Segment
#define SEG_INDEX_CMD_BYTE 2
#define SEG_INDEX_DATA_PADDING_BYTE 3
#define SEG_INDEX_DATA_LEN_BYTE 4
#define SEG_INDEX_CHECKSUM_HEADER_BYTE 8
#define SEG_INDEX_DATA_BYTE 10
#define SEG_LEN_DATA_MAX 60
//#pragma endregion

// at command
#define CMD_MOVE			106
#define CMD_HAND			107
#define CMD_SET_FEATURE 	108
#define CMD_POSITION_INFO	114

// lenght data according at command
#define LENGHT_CMD_MOVE			3
#define LENGHT_CMD_HAND			3
#define LENGHT_CMD_SET_FEATURE 	11


/* Object Form */
typedef struct AtSerial
{
    // Public
    uint8_t _serialRecvBytes[SERIAL_RECEIVE_MAX_BYTES];
    uint8_t _tcpCommand;
    uint32_t _tcpDataIndex;
    uint32_t _tcpDataLength;
    uint32_t _segmentLength;

    uint8_t bufferWrite[SERIAL_RECEIVE_MAX_BYTES];
    uint32_t _sendByteCount;
    // private

    uint32_t _tcpDataPadding;
    uint32_t _tcpEndDataSegmentIndex;
    uint32_t _serialRecvBytesIndex;
    uint8_t _serialReadState;
    uint16_t _serialChecksum;
} AtSerial_t;


/* Function Prototype */
void 	AtSerial_Init(AtSerial_t *atSerial);
uint8_t AtSerial_ReadCommand(AtSerial_t  *atSerial, RingBuffer_t *ringbuff_rx);
void 	AtSerial_PrepareCommand(AtSerial_t *atSerial, uint8_t command, uint8_t data[], uint32_t offset, uint32_t lenght);

uint8_t	AtSerial_GetCommand(AtSerial_t *atSerial);
int32_t	AtSerial_GetData(AtSerial_t *atSerial, uint8_t *ptr_des);

enum_DebugCmd AtSerial_HaldleCommand(uint8_t at_cmd,
									uint8_t *data,
									int32_t lenght,
									mainTaskMail_t *mail);
void

#endif /* INC_ARDUINO_H_ */
