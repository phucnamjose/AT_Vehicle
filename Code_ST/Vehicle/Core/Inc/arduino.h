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

/* Parent command */
#define CMD_MOVE			106
#define CMD_HAND			107
#define CMD_SET_FEATURE 	108
#define CMD_POSITION_INFO	250
#define CMD_EMERGENCY_STOP  130
#define CMD_SWITCH_MODE  	131
#define CMD_AUTO_MOVE  		132
#define CMD_AUTO_HAND  		133
#define CMD_AUTO_ROTATE 	134
#define CMD_AUTO_START  	135
#define CMD_AUTO_STOP  		136
/* Child command */
// Mode
#define CMD_SWITCH_MODE_MANUAL  	1
#define CMD_SWITCH_MODE_AUTO  		2
// Auto hand
#define CMD_AUTO_HAND_NONE  		0
#define CMD_AUTO_HAND_RECTANGLE  	1
#define CMD_AUTO_HAND_CIRCLE  		2
// Feature
#define CMD_SET_FEATURE_MOVESPEED = 3;
#define CMD_SET_FEATURE_NHIET_DO_DO_AM 30
#define CMD_SET_FEATURE_VAT_CAN_1 31
#define CMD_SET_FEATURE_VAT_CAN_2 32
#define CMD_SET_FEATURE_VAT_CAN_3 33
#define CMD_SET_FEATURE_VAT_CAN_4 34
#define CMD_SET_FEATURE_VAT_CAN_5 35
#define CMD_SET_FEATURE_VAT_CAN_6 36
#define CMD_SET_FEATURE_COI_BAO 40
#define CMD_SET_FEATURE_CAM_BIEN_NGUOI_1 41
#define CMD_SET_FEATURE_CAM_BIEN_NGUOI_2 42
#define CMD_SET_FEATURE_CAM_BIEN_NGUOI_3 43
#define CMD_SET_FEATURE_CAM_BIEN_NGUOI_4 44
#define CMD_SET_FEATURE_QUAT_CAU_H 50
#define CMD_SET_FEATURE_RELAY_8_KENH_1 51
#define CMD_SET_FEATURE_RELAY_8_KENH_2 52
#define CMD_SET_FEATURE_RELAY_8_KENH_3 53
#define CMD_SET_FEATURE_RELAY_8_KENH_4 54
#define CMD_SET_FEATURE_RELAY_8_KENH_5 55
#define CMD_SET_FEATURE_RELAY_8_KENH_6 56
#define CMD_SET_FEATURE_RELAY_8_KENH_7 57
#define CMD_SET_FEATURE_RELAY_8_KENH_8 58
#define CMD_SET_FEATURE_DONG_CO_PHAN_LUC_PHUN 60
#define CMD_SET_FEATURE_RELAY_30A_1 61
#define CMD_SET_FEATURE_RELAY_30A_2 62
#define CMD_SET_FEATURE_DEN_BAO_1 71
#define CMD_SET_FEATURE_DEN_BAO_2 72
#define CMD_SET_FEATURE_DEN_BAO_3 73

// Lenght data according to command
#define LENGHT_CMD_MOVE				3
#define LENGHT_CMD_HAND				3
#define LENGHT_CMD_SET_FEATURE 		11
#define LENGHT_CMD_EMERGENCY_STOP 	2
#define LENGHT_CMD_SWITCH_MODE  	3
#define LENGHT_CMD_AUTO_MOVE  		38
#define LENGHT_CMD_AUTO_HAND  		1
#define LENGHT_CMD_AUTO_ROTATE 		2
#define LENGHT_CMD_AUTO_START  		2
#define LENGHT_CMD_AUTO_STOP  		2

// Exact name
#define RELAY_MOTION_POW	CMD_SET_FEATURE_RELAY_8_KENH_4


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


typedef enum enum_MessageClass_t
{
	MSG_WRONG = 0,
	MSG_ONLY_ME,
	MSG_FORWARD,
	MSG_MAIL_TO_MAINTASK
}enum_MessageClass_t;


typedef struct ValueUsing_t
{
	// Internal
	uint8_t		ls_step_up;
	uint8_t		ls_step_down_left;
	uint8_t		ls_step_down_right;
	float		angle_step_up;
	float		angle_step_down;
	// External read
	uint8_t		ls_object_1;
	uint8_t		ls_object_2;
	uint8_t		ls_object_3;
	uint8_t		ls_object_4;
	uint8_t		ls_object_5;
	uint8_t		ls_object_6;
	double		pos_X;
	double		pos_Y;
	double		pos_Z;
	double		pos_Yaw;
	uint32_t	pos_count;
	Position_t	position_info;
	float		speed_move;
	// External write
	uint8_t		relay_power_motion;
} ValueUsing_t;


/* Function Prototype */
void 	AtSerial_Init(AtSerial_t *atSerial);
uint8_t AtSerial_ReadCommand(AtSerial_t  *atSerial, RingBuffer_t *ringbuff_rx);
void 	AtSerial_PrepareCommand(AtSerial_t *atSerial,
								uint8_t command,
								uint8_t data[],
								uint32_t offset,
								uint32_t lenght);

uint8_t	AtSerial_GetCommand(AtSerial_t *atSerial);
int32_t	AtSerial_GetData(AtSerial_t *atSerial, uint8_t *ptr_des);

enum_MessageClass_t AtSerial_HaldleComputer(uint8_t at_cmd,
											uint8_t *data,
											int32_t lenght,
											mainTaskMail_t *mail);

enum_MessageClass_t AtSerial_HaldleArduino(uint8_t at_cmd,
											uint8_t *data,
											int32_t lenght,
											mainTaskMail_t *mail);

void		AtSerial_ReadFeatureArduino(uint8_t *data);
void		AtSerial_SetPowerMotion(uint8_t value);
void		AtSerial_RequestPosition(void);
void		AtSerial_ReportFinishTarget(uint8_t *data);
void		AtSerial_ReportSensor(uint8_t nb_of_sensor, uint8_t value);

uint32_t 	AtSerial_GetPositionCount(void);
void		AtSerial_GetPosition(Position_t *position);


#endif /* INC_ARDUINO_H_ */
